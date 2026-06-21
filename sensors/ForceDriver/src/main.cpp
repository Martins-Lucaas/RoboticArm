#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// ======================================================
// WIFI — rede local do laboratório
// ======================================================
const char* ssid     = "Ender 3 V2 - coleta";
const char* password = "Biolabeb0608";

static const IPAddress LOCAL_IP (192, 168, 5, 105);
static const IPAddress GATEWAY  (192, 168, 5,   1);
static const IPAddress SUBNET   (255, 255, 255,  0);

static const IPAddress BCAST_IP (192, 168, 5, 255);
#define UDP_PORT 8080

// Auto-descoberta: o force_receiver manda um "hello" (tag 'FRCV') para
// DISCOVERY_PORT; gravamos o IP do remetente e passamos a enviar a telemetria
// por UNICAST de volta — o WiFi reconhece/retransmite unicast, então a perda
// (que era ~30% no broadcast) cai para perto de zero. Se nunca recebermos um
// hello (ou ele ficar obsoleto > HELLO_TIMEOUT_MS), caímos de volta no
// broadcast, então o sistema funciona mesmo antes da descoberta.
#define DISCOVERY_PORT     8090
#define DISCOVERY_MAGIC    "FRCV"
#define HELLO_TIMEOUT_MS   10000

WiFiUDP udp;
WiFiUDP udpRx;                       // socket de escuta do hello

static IPAddress g_dest_ip   = BCAST_IP;
static bool      g_have_dest = false;
static uint32_t  g_last_hello_ms = 0;

// ======================================================
// ADC — célula de carga via amplificador no GPIO 34
// ======================================================
#define ADC_PIN       34

// Oversampling CONTÍNUO com analogReadMilliVolts() (mV calibrados pela eFuse).
//
// NOTA: tentamos analogRead() cru, mas nesta placa/core ele retornava 0 (pino
// grudava no piso) — voltamos para analogReadMilliVolts(), que lê certo e
// ainda lineariza o ADC pela curva de fábrica. Ele devolve mV INTEIROS (passo
// de 1 mV), MAS acumulamos centenas de leituras por tick em ponto flutuante:
// como o ruído do ADC é > 1 LSB, ele funciona como dither e a média recupera
// frações de mV (resolução efetiva ~0.1-0.2 mV no pino → bem abaixo do passo
// de 1 mV). É a forma mais barata e confiável de aumentar a resolução.
static uint64_t adc_sum_mv = 0;   // soma de mV calibrados
static uint32_t adc_count  = 0;

// DIAGNÓSTICO: se 1, envia a tensão CRUA do pino (V, média do oversampling,
// SEM ganho/offset/mediana/EMA). Nesse modo o repouso aparece em ~0.142 V
// (= V_OFFSET / V_GAIN, o offset do amp visto DEPOIS do divisor), que é o
// "offset alto" observado na GUI. Em PRODUÇÃO (0) o firmware subtrai V_OFFSET
// e filtra, então v_sensor sai ~0 sem carga e usa a escala inteira (5 N em
// 1000 N ≈ 50 mV no domínio do amp, muito acima do passo do ADC). Só volte a
// 1 para diagnosticar o hardware.
#define DIAG_RAW  0

// EMA dupla. α é definido em AMOSTRAS, então o fc em Hz escala com a taxa
// de amostragem. O atraso de grupo da cascata vale ~2·(1−α)/α amostras a
// 100 Hz, então α controla diretamente a LATÊNCIA da leitura:
//   α=0.02 → ~0.9 s   (silencioso, mas lento demais)
//   α=0.05 → ~0.36 s
//   α=0.20 → ~0.08 s  (fc da cascata ~2.2 Hz)
// Histórico: 0.10→0.05 cortou ~metade do ruído (~70 g RMS) p/ palpação quase
// estática; depois 0.05→0.02 cortou mais ~40% (~45 g RMS). MAS em 2026-06-21,
// testando COMPRESSÃO dinâmica da célula, ~0.9 s ficou lento demais — o sensor
// precisa identificar a força aplicada na hora. Subi α p/ 0.20: atraso cai p/
// ~80 ms (resposta rápida), ao custo de mais ruído (~120–150 g RMS no nível
// atual de ~2 mV/kg no pino). O conserto de verdade p/ ruído é analógico (subir
// o SPAN e/ou remover o divisor → V_GAIN=1.0), não baixar α. Se for voltar a
// uso estático e quiser leitura mais limpa (custe latência), reduza α.
const float ALPHA = 0.20f;

static float ema_stage1     = 0.0f;
static float volt_filtered  = 0.0f;
static bool  filter_seeded  = false;

// Mediana de 5 (era 9): rejeita glitches isolados do ADC do ESP32 — o pré-
// filtro não-linear antes da EMA. Encurtada de 9→5 junto com α=0.20 para
// resposta rápida à compressão: a janela de 9 (~90 ms) atrasava a borda de
// subida da força; 5 (~50 ms) ainda derruba spikes de 1–2 amostras.
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
const float R1 = 221000.0f;
const float R2 =  98600.0f;

// Fator para reconstruir a saída do amplificador a partir da tensão lida no
// pino. COM o divisor montado: V_GAIN = (R1+R2)/R2 ≈ 3.2. ATENÇÃO: o divisor
// atenua o sinal ANTES do ADC, então joga resolução fora. Se você REMOVER o
// divisor (saída do amp direto no GPIO34, desde que ≤ 3.3 V no fundo de
// escala), troque V_GAIN para 1.0f — aí o ADC enxerga o sinal inteiro.
const float V_GAIN = (R1 + R2) / R2;

// Offset DC do amplificador em repouso (sem carga). Com a célula MK CSA/ZL-100
// a saída repousa em ~0.4544 V. Subtraímos para que a tensão enviada seja ~0
// sem força. Medido em produção (DIAG_RAW=0) sobrava +0.00446 V de resíduo no
// repouso → somado (0.4544 + 0.00446 = 0.45886). Re-aferido em 2026-06-21 com a
// célula em repouso (porta 8080): ainda sobravam +0.001416 V → somado de novo
// (0.45886 + 0.001416 = 0.460276), agora o repouso sai em 0. Se mudar de
// célula/amp ou houver deriva térmica, reajuste (a GUI ainda faz tare por cima).
const float V_OFFSET = 0.460276f;

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
    // Desliga o modem-sleep: por padrão o ESP32 dorme o rádio entre DTIMs do AP
    // e atrasa/derruba pacotes — a 100 Hz isso aparecia como perda de UDP de
    // 5–37% no force_receiver. Sem sleep o rádio fica sempre ativo (consome mais
    // ~80 mA, irrelevante com alimentação USB/bancada) e a perda cai muito.
    WiFi.setSleep(false);
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

// Atende o "hello" do force_receiver (auto-descoberta). Não-bloqueante: lê
// todos os datagramas pendentes na porta de descoberta e, se a tag bater,
// grava o IP do remetente como destino unicast da telemetria.
static void discovery_poll()
{
    int psize;
    while ((psize = udpRx.parsePacket()) > 0) {
        char buf[8] = {0};
        int n = udpRx.read((uint8_t*)buf, sizeof(buf) - 1);
        if (n >= (int)sizeof(DISCOVERY_MAGIC) - 1 &&
            strncmp(buf, DISCOVERY_MAGIC, sizeof(DISCOVERY_MAGIC) - 1) == 0) {
            IPAddress src = udpRx.remoteIP();
            if (!g_have_dest || src != g_dest_ip) {
                Serial.print("[DISC] receiver em ");
                Serial.print(src);
                Serial.println(" — telemetria por unicast.");
            }
            g_dest_ip       = src;
            g_have_dest     = true;
            g_last_hello_ms = millis();
        }
    }
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
// OTA — gravação pela rede (espota). A 1ª gravação ainda é por USB; depois
// é só `pio run -t upload` apontando para o IP do ESP. A senha evita que
// qualquer um na rede regrave o dispositivo.
// ======================================================
#define OTA_HOSTNAME  "forcedriver"
#define OTA_PASSWORD  "Biolabeb0608"

static void ota_setup()
{
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() {
        Serial.println("[OTA] início — pausando amostragem.");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] concluído — reiniciando.");
    });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
        Serial.printf("[OTA] %u%%\r", (p * 100) / t);
    });
    ArduinoOTA.onError([](ota_error_t e) {
        Serial.printf("[OTA] erro %u\n", e);
    });
    ArduinoOTA.begin();
}

// ======================================================
// SETUP
// ======================================================
void setup()
{
    Serial.begin(115200);
    analogReadResolution(12);
    analogSetPinAttenuation(ADC_PIN, ADC_11db);  // fundo de escala ~0..3.3 V
    wifi_connect();
    ota_setup();
    udpRx.begin(DISCOVERY_PORT);   // escuta o hello do force_receiver
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
        adc_sum_mv = 0; adc_count = 0;   // descarta acúmulo parcial
        return;
    }

    ArduinoOTA.handle();   // atende pedidos de gravação OTA (não-bloqueante)
    discovery_poll();      // aprende/renova o destino unicast (hello do receiver)

    // Acumula UMA leitura (mV calibrados) por passagem do loop(). Entre dois
    // ticks o loop roda centenas de vezes, então adc_count chega a ~150-300:
    // é o oversampling de fato, e é ele que dá a resolução sub-mV.
    adc_sum_mv += (uint32_t)analogReadMilliVolts(ADC_PIN);
    adc_count++;

    uint32_t now = millis();
    if (now - last_sample_ms < SAMPLE_INTERVAL_MS) return;
    if (now - last_sample_ms > 4 * SAMPLE_INTERVAL_MS) {
        last_sample_ms = now;
    } else {
        last_sample_ms += SAMPLE_INTERVAL_MS;
    }

    if (adc_count == 0) return;   // segurança: nada acumulado neste tick
    uint32_t n_used = adc_count;
    // Média FRACIONÁRIA dos mV (sub-mV pelo dither) → volts.
    float v_adc = (adc_sum_mv / (float)adc_count) / 1000.0f;
    adc_sum_mv = 0; adc_count = 0;

    float v_sensor = v_adc * V_GAIN - V_OFFSET;

    if (!filter_seeded) filter_seed(v_sensor);

    // Estágio 2: mediana de 5 — spikes isolados não passam
    median_buf[median_idx] = v_sensor;
    median_idx = (median_idx + 1) % MEDIAN_N;
    float v_med = median5(median_buf);

    // Estágio 3: EMA em cascata dupla (passa-baixa de 2ª ordem)
    ema_stage1    = ALPHA * v_med      + (1.0f - ALPHA) * ema_stage1;
    volt_filtered = ALPHA * ema_stage1 + (1.0f - ALPHA) * volt_filtered;

    packet.seq      = tx_seq++;
#if DIAG_RAW
    packet.v_sensor = v_adc;          // tensão CRUA do pino (sem nada)
#else
    packet.v_sensor = volt_filtered;
#endif

    // Destino: unicast para o receiver descoberto (WiFi retransmite → sem
    // perda); se o hello sumiu por > HELLO_TIMEOUT_MS, volta ao broadcast.
    bool fresh = g_have_dest && (now - g_last_hello_ms < HELLO_TIMEOUT_MS);
    IPAddress dst = fresh ? g_dest_ip : BCAST_IP;
    udp.beginPacket(dst, UDP_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(Payload));
    udp.endPacket();

    // Aviso de calibração do SPAN: o ADC do ESP32 fica não-linear/satura
    // acima de ~2.9 V (≈3600 contagens). Se você vir este aviso ao aplicar a
    // força máxima, ABAIXE o SPAN do MKTC-05 até a saída do condicionador dar
    // ~8 V no fundo de escala (≈2.5 V no pino) — aí fica na região linear.
    if (v_adc > 2.9f) {
        Serial.printf("[WARN] ADC perto do topo (%.3f V no pino) — sinal "
                      "alto, risco de saturar/não-linearizar.\n", v_adc);
    }

    Serial.printf("v_sensor (filtrada): %.6f V  | oversample N=%lu\n",
                  volt_filtered, (unsigned long)n_used);
}
