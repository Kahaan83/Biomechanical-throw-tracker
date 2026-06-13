#include <esp_now.h>
#include <WiFi.h>

// MUST MATCH THE GLOVE EXACTLY
typedef struct struct_message {
  float cardiac;
  float fsr;
  float q_r; float q_i; float q_j; float q_k;
  float a_x; float a_y; float a_z; 
  float g_x; float g_y; float g_z; // NEW: Gyroscope
} struct_message;

struct_message myData;

unsigned long lastRecvTime = 0;
const unsigned long TIMEOUT_MS = 2000; 
bool isConnected = false;

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(myData)) {
    memcpy(&myData, incomingData, sizeof(myData));
    
    lastRecvTime = millis();
    isConnected = true;
    
    Serial.print("Cardiac_V:"); Serial.print(myData.cardiac, 3);
    Serial.print(", FSR_V:"); Serial.print(myData.fsr, 3);
    Serial.print(", Quat_R:"); Serial.print(myData.q_r, 3);
    Serial.print(", Quat_I:"); Serial.print(myData.q_i, 3);
    Serial.print(", Quat_J:"); Serial.print(myData.q_j, 3);
    Serial.print(", Quat_K:"); Serial.print(myData.q_k, 3);
    Serial.print(", Accel_X:"); Serial.print(myData.a_x, 2);
    Serial.print(", Accel_Y:"); Serial.print(myData.a_y, 2);
    Serial.print(", Accel_Z:"); Serial.print(myData.a_z, 2);
    Serial.print(", Gyro_X:"); Serial.print(myData.g_x, 2);
    Serial.print(", Gyro_Y:"); Serial.print(myData.g_y, 2);
    Serial.print(", Gyro_Z:"); Serial.println(myData.g_z, 2);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (isConnected && (millis() - lastRecvTime > TIMEOUT_MS)) {
    Serial.println("Cardiac_V:0.0, FSR_V:0.0, Quat_R:0.0, Quat_I:0.0, Quat_J:0.0, Quat_K:0.0, Accel_X:0.0, Accel_Y:0.0, Accel_Z:0.0, Gyro_X:0.0, Gyro_Y:0.0, Gyro_Z:0.0");
    isConnected = false; 
  }
}