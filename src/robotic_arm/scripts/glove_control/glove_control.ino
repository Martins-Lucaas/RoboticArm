#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;

// Configuração do filtro complementar
const float ALPHA = 0.98; // Peso do giroscópio no filtro complementar
unsigned long lastSensorRead = 0; // Marca temporal da última leitura
const unsigned long SENSOR_INTERVAL = 10; // Intervalo entre leituras (ms)

// Estrutura para armazenar os dados dos sensores
struct SensorData {
  float pitch;
  float roll;
  float yaw;
};

// Função para inicializar o MPU6050
void initializeMPU6050() {
  if (!mpu.begin()) {
    Serial.println("Falha ao iniciar MPU6050. Verifique a conexão.");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// Função para calcular os ângulos com filtro complementar
SensorData calculateAngles(SensorData previousAngles, float dt) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Cálculo dos ângulos do acelerômetro
  float accPitch = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float accRoll = atan2(-a.acceleration.x, a.acceleration.z) * 180.0 / PI;

  // Atualização dos ângulos com filtro complementar
  SensorData newAngles;
  newAngles.pitch = ALPHA * (previousAngles.pitch + g.gyro.x * dt * 180.0 / PI) + (1 - ALPHA) * accPitch;
  newAngles.roll = ALPHA * (previousAngles.roll + g.gyro.y * dt * 180.0 / PI) + (1 - ALPHA) * accRoll;
  newAngles.yaw = previousAngles.yaw + g.gyro.z * dt * 180.0 / PI;

  // Normaliza yaw entre -180° e +180°
  if (newAngles.yaw > 180.0) newAngles.yaw -= 360.0;
  if (newAngles.yaw < -180.0) newAngles.yaw += 360.0;

  return newAngles;
}

// Função para enviar os dados via serial
void sendSensorData(SensorData angles) {
  Serial.print("P:"); Serial.print(angles.pitch);
  Serial.print(", R:"); Serial.print(angles.roll);
  Serial.print(", Y:"); Serial.println(angles.yaw);
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // Aguarda inicialização do monitor serial
  initializeMPU6050();
}

void loop() {
  unsigned long currentTime = millis();
  static SensorData angles = {0.0, 0.0, 0.0};

  // Verifica se é hora de ler os sensores
  if (currentTime - lastSensorRead >= SENSOR_INTERVAL) {
    float dt = (currentTime - lastSensorRead) / 1000.0; // Tempo em segundos
    lastSensorRead = currentTime;

    // Calcula os ângulos e envia os dados
    angles = calculateAngles(angles, dt);
    sendSensorData(angles);
  }
}
