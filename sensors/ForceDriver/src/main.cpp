#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ======================================================
// WIFI — rede local do laboratório
// ======================================================
const char* ssid     = "Ender 3 V2-coleta";
const char* password = "Biolabeb0608";

// IP do PC na rede acima — descubra com: ip addr show (Linux)
// Altere antes de gravar na ESP32
#define DEST_IP  "192.168.1.100"
#define UDP_PORT 8080

WiFiUDP udp;

// ======================================================
// ADC — célula de carga via amplificador no GPIO 34
// ======================================================
#define ADC_PIN 34

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
// PAYLOAD UDP
//   v_sensor      : tensão real do sensor (V) — usada pela GUI p/ calibração
//   force_filtered: força estimada com calibração padrão (N)
// ======================================================
struct __attribute__((packed)) Payload {
    float v_sensor;
    float force_filtered;
};

static Payload packet;

// ======================================================
// SETUP
// ======================================================
void setup()
{
    Serial.begin(115200);
    analogReadResolution(12);

    WiFi.begin(ssid, password);
    Serial.print("Conectando");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi conectado!");
    Serial.print("IP ESP32: ");
    Serial.println(WiFi.localIP());
}

// ======================================================
// LOOP — 50 Hz
// ======================================================
void loop()
{
    // Leitura ADC de 12 bits
    int adc_raw = analogRead(ADC_PIN);

    // Conversão ADC → tensão no pino (3.3 V, 12 bits)
    float v_adc = (adc_raw * 3.3f) / 4095.0f;

    // Reconstrução da tensão real do sensor (divisor R1/R2)
    float v_sensor = v_adc * ((R1 + R2) / R2);

    // Força com calibração padrão
    float force = (v_sensor - CALIB_INTERCEPT) / CALIB_SLOPE;

    // Filtro exponencial de primeira ordem
    force_filtered = ALPHA * force + (1.0f - ALPHA) * force_filtered;

    // Monta e envia pacote UDP
    packet.v_sensor       = v_sensor;
    packet.force_filtered = force_filtered;

    udp.beginPacket(DEST_IP, UDP_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(Payload));
    udp.endPacket();

    Serial.printf("v_sensor: %.4f V | Força: %.3f N\n", v_sensor, force_filtered);

    delay(20);  // 50 Hz
}
