#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_BNO08x.h>
#include <esp_now.h>
#include <WiFi.h>

#define I2C_SDA D2 
#define I2C_SCL D3 
#define SENSOR_PWR D0 

Adafruit_ADS1115 ads;
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

uint8_t broadcastAddress[] = {0xFC, 0xF5, 0xC4, 0x19, 0x92, 0x7C};


typedef struct struct_message {
  float cardiac;
  float fsr;
  float q_r; float q_i; float q_j; float q_k;
  float a_x; float a_y; float a_z; 
  float g_x; float g_y; float g_z; 
} struct_message;

struct_message telemetry;
esp_now_peer_info_t peerInfo;

unsigned long lastDataRead = 0;
const unsigned long TIMEOUT_MS = 1000; 

void setup(void) {
  Serial.begin(115200);
  delay(2000);
  
  pinMode(SENSOR_PWR, OUTPUT);
  digitalWrite(SENSOR_PWR, LOW); 

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { delay(1000); ESP.restart(); }

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){ delay(1000); ESP.restart(); }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); 
  Wire.setTimeOut(100); 

  if (!ads.begin(0x48, &Wire)) { delay(1000); ESP.restart(); }
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS); 

  if (!bno08x.begin_I2C(0x4B, &Wire)) { delay(1000); ESP.restart(); }

  digitalWrite(SENSOR_PWR, HIGH); 
  delay(100); 

  Wire.setClock(400000); 
  
  // --- WAKE UP ALL 3 ENGINES ---
  bno08x.enableReport(SH2_ROTATION_VECTOR, 10000); 
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000); 
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000); // WAKE UP GYRO
  
  lastDataRead = millis();
}

void loop(void) {
  if (millis() - lastDataRead > TIMEOUT_MS) { ESP.restart(); }

  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    lastDataRead = millis();

    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      telemetry.q_r = sensorValue.un.rotationVector.real;
      telemetry.q_i = sensorValue.un.rotationVector.i;
      telemetry.q_j = sensorValue.un.rotationVector.j;
      telemetry.q_k = sensorValue.un.rotationVector.k;

      telemetry.cardiac = ads.computeVolts(ads.readADC_SingleEnded(0)); 
      telemetry.fsr     = ads.computeVolts(ads.readADC_SingleEnded(1)); 

      esp_now_send(broadcastAddress, (uint8_t *) &telemetry, sizeof(telemetry));
      
    } 
    else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      telemetry.a_x = sensorValue.un.linearAcceleration.x;
      telemetry.a_y = sensorValue.un.linearAcceleration.y;
      telemetry.a_z = sensorValue.un.linearAcceleration.z;
    }
    else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      // Capture the Gyroscope data silently in the background
      telemetry.g_x = sensorValue.un.gyroscope.x;
      telemetry.g_y = sensorValue.un.gyroscope.y;
      telemetry.g_z = sensorValue.un.gyroscope.z;
    }
  }
}