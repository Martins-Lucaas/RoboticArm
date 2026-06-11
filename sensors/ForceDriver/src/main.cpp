#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

// ======================================================
// WIFI — rede local do laboratório
// ======================================================
const char* ssid     = "Ender 3 V2 - coleta";
const char* password = "Biolabeb0608";

static const IPAddress LOCAL_IP (192, 168, 5, 105);
static const IPAddress GATEWAY  (192, 168, 5,   1);
static const IPAddress SUBNET   (255, 255, 255,  0);

#define DEST_IP  "192.168.5.255"
#define UDP_PORT 8080

WiFiUDP udp;

// ======================================================
// ADC — célula de carga via amplificador no GPIO 34
// ======================================================
#define ADC_PIN       34
#define OVERSAMPLE_N  64
#define TRIM_N        16   // descarta as 16 menores e 16 maiores leituras

// ======================================================
// FILTRO — 3 estágios contra spikes do ADC (pioram com
// WiFi ativo) e ruído de fundo:
//   1. média aparada das 64 leituras (remove outliers
//      dentro da rajada de oversampling)
//   2. mediana de 5 amostras de força (elimina spikes
//      isolados por completo, sem atrasar degraus reais)
//   3. EMA em cascata dupla (rolloff de 2ª ordem,
//      fc ≈ 0.6 Hz @ 50 Hz)
// ======================================================
const float ALPHA = 0.12f;

static float ema_stage1     = 0.0f;
static float force_filtered = 0.0f;

#define MEDIAN_N 5
static float median_buf[MEDIAN_N] = {0};
static int   median_idx           = 0;

static float median5(const float* buf)
{
    float s[MEDIAN_N];
    memcpy(s, buf, sizeof(s));
    for (int i = 1; i < MEDIAN_N; ++i) {
        float v = s[i];
        int j = i - 1;
        while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; --j; }
        s[j + 1] = v;
    }
    return s[MEDIAN_N / 2];
}

static void filter_reset(float value)
{
    ema_stage1     = value;
    force_filtered = value;
    for (int i = 0; i < MEDIAN_N; ++i) median_buf[i] = value;
}

// ======================================================
// DIVISOR DE TENSÃO DO AMPLIFICADOR
// ======================================================
const float R1 = 220000.0f;
const float R2 =  98600.0f;

// ======================================================
// CALIBRAÇÃO — slope calibrado em tração, válido para
// compressão pelo princípio de simetria da ponte de
// Wheatstone: mesma sensibilidade V/N nos dois sentidos.
//
// Positivo = tração  |  Negativo = compressão
// F = (v_sensor - v_zero) / SLOPE
// ======================================================
const float CALIB_SLOPE = 0.4490f;

// ======================================================
// TARA — zero de referência dinâmico
// Persiste no EEPROM (endereço 0, 4 bytes float).
// Comando serial: 'T' → captura tara com sensor em repouso
//                 'R' → reseta tara para 0.0 V
// ======================================================
#define EEPROM_SIZE        4
#define EEPROM_ADDR_VZERO  0

static float v_zero = 0.0f;

static void load_tare()
{
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(EEPROM_ADDR_VZERO, v_zero);
    // valor inválido (NaN ou fora de faixa física) → começa em 0
    if (isnan(v_zero) || v_zero < 0.0f || v_zero > 15.0f) v_zero = 0.0f;
    EEPROM.end();
    Serial.printf("Tara carregada: v_zero = %.4f V\n", v_zero);
}

static void save_tare(float v)
{
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(EEPROM_ADDR_VZERO, v);
    EEPROM.commit();
    EEPROM.end();
}

// ======================================================
// TEMPORIZAÇÃO — 50 Hz não-bloqueante
// ======================================================
#define SAMPLE_INTERVAL_MS 20
static uint32_t last_sample_ms = 0;

// ======================================================
// PAYLOAD UDP (little-endian, 8 bytes)
//   v_sensor      : tensão real do sensor (V)
//   force_filtered: força com sinal — positivo=tração,
//                   negativo=compressão (N)
// ======================================================
struct __attribute__((packed)) Payload {
    float v_sensor;
    float force_filtered;
};

static Payload packet;

// ======================================================
// WIFI — conexão / reconexão
// ======================================================
static void wifi_connect()
{
    WiFi.mode(WIFI_STA);
    WiFi.config(LOCAL_IP, GATEWAY, SUBNET);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    WiFi.begin(ssid, password);

    Serial.print("Conectando");
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi conectado!");
        Serial.print("IP ESP32: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nTimeout WiFi — tentando no próximo ciclo.");
    }
}

// ======================================================
// SETUP
// ======================================================
void setup()
{
    Serial.begin(115200);
    analogReadResolution(12);
    load_tare();
    wifi_connect();
    last_sample_ms = millis();
    Serial.println("Comandos: T = tarar | R = resetar tara");
}

// ======================================================
// LOOP — 50 Hz não-bloqueante
// ======================================================
void loop()
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WARN] WiFi desconectado — reconectando...");
        wifi_connect();
        last_sample_ms = millis();
        return;
    }

    uint32_t now = millis();
    if (now - last_sample_ms < SAMPLE_INTERVAL_MS) return;
    last_sample_ms += SAMPLE_INTERVAL_MS;

    // Oversampling com média aparada: ordena as leituras e
    // descarta os TRIM_N extremos de cada lado antes da média
    uint16_t samples[OVERSAMPLE_N];
    for (int i = 0; i < OVERSAMPLE_N; ++i) {
        uint16_t v = (uint16_t)analogRead(ADC_PIN);
        int j = i - 1;
        while (j >= 0 && samples[j] > v) { samples[j + 1] = samples[j]; --j; }
        samples[j + 1] = v;
    }
    uint32_t adc_acc = 0;
    for (int i = TRIM_N; i < OVERSAMPLE_N - TRIM_N; ++i) {
        adc_acc += samples[i];
    }
    float v_adc    = (adc_acc / (float)(OVERSAMPLE_N - 2 * TRIM_N) * 3.3f) / 4095.0f;
    float v_sensor = v_adc * ((R1 + R2) / R2);

    // Comandos seriais — lidos após leitura do ADC para
    // que 'T' capture a tensão recém-medida
    if (Serial.available()) {
        char cmd = (char)Serial.read();
        if (cmd == 'T' || cmd == 't') {
            v_zero = v_sensor;
            filter_reset(0.0f);
            save_tare(v_zero);
            Serial.printf("Tara definida: v_zero = %.4f V\n", v_zero);
        } else if (cmd == 'R' || cmd == 'r') {
            v_zero = 0.0f;
            filter_reset(0.0f);
            save_tare(v_zero);
            Serial.println("Tara resetada para 0.0 V");
        }
    }

    // Força com sinal: positivo = tração, negativo = compressão
    float force = (v_sensor - v_zero) / CALIB_SLOPE;

    // Estágio 2: mediana de 5 — spikes isolados não passam
    median_buf[median_idx] = force;
    median_idx = (median_idx + 1) % MEDIAN_N;
    float force_med = median5(median_buf);

    // Estágio 3: EMA em cascata dupla (passa-baixa de 2ª ordem)
    ema_stage1     = ALPHA * force_med  + (1.0f - ALPHA) * ema_stage1;
    force_filtered = ALPHA * ema_stage1 + (1.0f - ALPHA) * force_filtered;

    packet.v_sensor       = v_sensor;
    packet.force_filtered = force_filtered;

    udp.beginPacket(DEST_IP, UDP_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(Payload));
    udp.endPacket();

    Serial.printf("v_sensor: %.4f V | v_zero: %.4f V | Força: %+.3f N\n",
                  v_sensor, v_zero, force_filtered);
}
