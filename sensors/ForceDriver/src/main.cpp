#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ======================================================
// WIFI — rede local do laboratório
// ======================================================
const char* ssid     = "Ender 3 V2 - coleta";
const char* password = "Biolabeb0608";

// IP estático do ESP32 (fixo na subnet do laboratório)
static const IPAddress LOCAL_IP (192, 168, 5, 105);
static const IPAddress GATEWAY  (192, 168, 5,   1);
static const IPAddress SUBNET   (255, 255, 255,  0);

// Broadcast da subnet — qualquer PC em 192.168.5.x recebe
#define DEST_IP  "192.168.5.255"
#define UDP_PORT 8080

WiFiUDP udp;

// ======================================================
// ADC — célula de carga via amplificador no GPIO 34
// ======================================================
#define ADC_PIN       34
#define OVERSAMPLE_N  16        // 16 amostras → +6 dB SNR (~1 bit efetivo extra)

// ======================================================
// FILTRO EXPONENCIAL
// ======================================================
static float force_filtered = 0.0f;
const float  ALPHA          = 0.15f;

// ======================================================
// DIVISOR DE TENSÃO DO AMPLIFICADOR
// ======================================================
const float R1 = 220000.0f;
const float R2 =  98600.0f;

// ======================================================
// CALIBRAÇÃO PADRÃO — ajustada via GUI do touch_pack
// F = (v_sensor - INTERCEPT) / SLOPE
// ======================================================
const float CALIB_INTERCEPT = 0.0017f;
const float CALIB_SLOPE     = 0.4490f;

// ======================================================
// TEMPORIZAÇÃO — 50 Hz sem delay()
// millis() é não-bloqueante: WiFi stack e watchdog
// continuam a ser servidos entre os ticks.
// ======================================================
#define SAMPLE_INTERVAL_MS 20   // 1000 ms / 50 Hz
static uint32_t last_sample_ms = 0;

// ======================================================
// PAYLOAD UDP  (little-endian, 8 bytes)
//   v_sensor      : tensão real do sensor (V)
//   force_filtered: força estimada com calibração padrão (N)
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
    wifi_connect();
    last_sample_ms = millis();
}

// ======================================================
// LOOP — 50 Hz não-bloqueante
// ======================================================
void loop()
{
    // Reconexão automática se o WiFi cair
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WARN] WiFi desconectado — reconectando...");
        wifi_connect();
        last_sample_ms = millis();  // reset do timer após reconexão
        return;
    }

    // Aguarda o próximo tick de 20 ms sem bloquear o sistema
    uint32_t now = millis();
    if (now - last_sample_ms < SAMPLE_INTERVAL_MS) return;
    last_sample_ms += SAMPLE_INTERVAL_MS;  // mantém fase mesmo com jitter leve

    // Oversampling: 16 leituras acumuladas → reduz ruído de quantização
    uint32_t adc_acc = 0;
    for (int i = 0; i < OVERSAMPLE_N; ++i) {
        adc_acc += analogRead(ADC_PIN);
    }
    float v_adc = (adc_acc / (float)OVERSAMPLE_N * 3.3f) / 4095.0f;

    // Reconstrução da tensão real do sensor (divisor R1/R2)
    float v_sensor = v_adc * ((R1 + R2) / R2);

    // Força com calibração padrão
    float force = (v_sensor - CALIB_INTERCEPT) / CALIB_SLOPE;

    // Filtro exponencial de primeira ordem
    force_filtered = ALPHA * force + (1.0f - ALPHA) * force_filtered;

    // Monta e envia pacote UDP broadcast
    packet.v_sensor       = v_sensor;
    packet.force_filtered = force_filtered;

    udp.beginPacket(DEST_IP, UDP_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(Payload));
    udp.endPacket();

    Serial.printf("v_sensor: %.4f V | Força: %.3f N\n", v_sensor, force_filtered);
}
