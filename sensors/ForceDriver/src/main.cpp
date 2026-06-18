#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

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

// EMA dupla. α é definido em AMOSTRAS, então o fc em Hz escala com a taxa
// de amostragem — ao subir de 50→100 Hz, α cai de 0.20→0.10 para MANTER
// fc ≈ 1.1 Hz (cascata). α=0.10 @ 100 Hz → fc ≈ 1.08 Hz, atraso de grupo
// ~180 ms. Observação: dobrar a taxa NÃO reduz a latência — esta é fixada
// por fc, não por fs; mais taxa só dá mais pontos por segundo. Se quiser
// MENOS latência (custe ruído), suba α em vez de manter a banda.
const float ALPHA = 0.10f;

static float ema_stage1     = 0.0f;
static float volt_filtered  = 0.0f;
static bool  filter_seeded  = false;

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

// Inicializa todos os estágios na primeira leitura para que o
// filtro não precise convergir a partir de zero.
static void filter_seed(float value)
{
    ema_stage1    = value;
    volt_filtered = value;
    for (int i = 0; i < MEDIAN_N; ++i) median_buf[i] = value;
    filter_seeded = true;
}

// ======================================================
// DIVISOR DE TENSÃO + GANHO DO AMPLIFICADOR
// ======================================================
const float R1 = 220000.0f;
const float R2 =  100000.0f;

// ======================================================
// TEMPORIZAÇÃO — 100 Hz não-bloqueante
// ======================================================
#define SAMPLE_INTERVAL_MS 10
static uint32_t last_sample_ms = 0;

struct __attribute__((packed)) Payload {
    uint32_t seq;
    float    v_sensor;
};

static Payload  packet;
static uint32_t tx_seq = 0;

const uint32_t WIFI_RETRY_MS = 3000;
static uint32_t last_wifi_retry_ms = 0;

// Conexão inicial — só roda no setup(). Bloquear no boot é aceitável (não
// há amostragem ainda); na operação a reconexão é feita por wifi_kick().
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
        Serial.println("\nTimeout WiFi — reconectando em background.");
    }
    last_wifi_retry_ms = millis();
}

// Reconexão não-bloqueante: dispara um begin() throttled e retorna na hora.
static void wifi_kick()
{
    uint32_t now = millis();
    if (now - last_wifi_retry_ms < WIFI_RETRY_MS) return;
    last_wifi_retry_ms = now;
    Serial.println("[WARN] WiFi desconectado — tentando reconectar...");
    WiFi.begin(ssid, password);
}

// ======================================================
// SETUP
// ======================================================
void setup()
{
    Serial.begin(115200);
    analogReadResolution(12);
    wifi_connect();
    last_sample_ms = millis();
    Serial.println("ESP: enviando apenas tensão (V) — calibração na GUI.");
}

// ======================================================
// LOOP — 100 Hz não-bloqueante
// ======================================================
void loop()
{
    if (WiFi.status() != WL_CONNECTED) {
        wifi_kick();                 // não-bloqueante: tenta e segue
        last_sample_ms = millis();   // evita rajada ao religar
        return;
    }

    uint32_t now = millis();
    if (now - last_sample_ms < SAMPLE_INTERVAL_MS) return;
    if (now - last_sample_ms > 4 * SAMPLE_INTERVAL_MS) {
        last_sample_ms = now;
    } else {
        last_sample_ms += SAMPLE_INTERVAL_MS;
    }

    // Oversampling com média aparada: ordena as leituras e descarta os
    // TRIM_N extremos de cada lado antes da média. analogReadMilliVolts()
    // já entrega mV calibrados pela curva de fábrica (eFuse), linearizando
    // a não-linearidade do ADC do ESP32 — não é só 3.3/4095.
    uint16_t samples[OVERSAMPLE_N];   // mV (0..~3300), cabe em uint16
    for (int i = 0; i < OVERSAMPLE_N; ++i) {
        uint16_t v = (uint16_t)analogReadMilliVolts(ADC_PIN);
        int j = i - 1;
        while (j >= 0 && samples[j] > v) { samples[j + 1] = samples[j]; --j; }
        samples[j + 1] = v;
    }
    uint32_t mv_acc = 0;
    for (int i = TRIM_N; i < OVERSAMPLE_N - TRIM_N; ++i) {
        mv_acc += samples[i];
    }
    float v_adc    = (mv_acc / (float)(OVERSAMPLE_N - 2 * TRIM_N)) / 1000.0f;
    float v_sensor = v_adc * ((R1 + R2) / R2);

    if (!filter_seeded) filter_seed(v_sensor);

    // Estágio 2: mediana de 5 — spikes isolados não passam
    median_buf[median_idx] = v_sensor;
    median_idx = (median_idx + 1) % MEDIAN_N;
    float v_med = median5(median_buf);

    // Estágio 3: EMA em cascata dupla (passa-baixa de 2ª ordem)
    ema_stage1    = ALPHA * v_med      + (1.0f - ALPHA) * ema_stage1;
    volt_filtered = ALPHA * ema_stage1 + (1.0f - ALPHA) * volt_filtered;

    packet.seq      = tx_seq++;
    packet.v_sensor = volt_filtered;

    udp.beginPacket(DEST_IP, UDP_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(Payload));
    udp.endPacket();

    Serial.printf("v_sensor (filtrada): %.4f V\n", volt_filtered);
}
