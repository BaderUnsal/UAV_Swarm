#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// —— Configuration ——
// Serial speed to PC
static const uint32_t SERIAL_BAUD = 115200;

// Drone’s ESP-NOW MAC address (replace with your drone’s MAC)
uint8_t droneMAC[6] = { 0x00, 0x4B, 0x12, 0x3B, 0x7F, 0xA8 };

// Telemetry packet layout (must match your drone’s struct)
struct TelemetryData {
  float Desired_Roll;
  float Desired_Pitch;
  float Desired_Yaw;
  float Actual_Roll;
  float Actual_Pitch;
  float Actual_Yaw;
};

// —— Helpers ——
// Forward any pending PID commands from Serial → ESP-NOW
void handleSerialCommands() {
  const size_t CMD_SIZE = sizeof(int32_t) + sizeof(float);
  while (Serial.available() >= (int)CMD_SIZE) {
    int32_t param_index;
    float   new_value;
    // Read from USB-Serial
    Serial.readBytes((char*)&param_index, sizeof(param_index));
    Serial.readBytes((char*)&new_value,  sizeof(new_value));

    // Pack & send to drone
    uint8_t buf[CMD_SIZE];
    memcpy(buf, &param_index,        sizeof(param_index));
    memcpy(buf + sizeof(param_index),
           &new_value, sizeof(new_value));

    esp_err_t res = esp_now_send(droneMAC, buf, CMD_SIZE);
    Serial.printf("PID cmd idx=%d val=%.3f → %s\n",
                  param_index, new_value,
                  res == ESP_OK ? "OK" : "ERR");
  }
}

// Callback: invoked when telemetry arrives from drone
void onTelemetryRecv(const esp_now_recv_info_t *info,
                     const uint8_t *data, int len) {
  if (len != (int)sizeof(TelemetryData)) return;
  TelemetryData t;
  memcpy(&t, data, sizeof(t));
  // Print CSV: DesRoll,DesPitch,DesYaw,ActRoll,ActPitch,ActYaw
  Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                t.Desired_Roll,  t.Desired_Pitch,  t.Desired_Yaw,
                t.Actual_Roll,   t.Actual_Pitch,   t.Actual_Yaw);
}

// Optional: callback for send-status
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // could blink an LED or log success/failure here
}

void setup() {
  // USB-Serial to PC
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {}

  // ESP-NOW setup
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed"); 
    while (true);
  }

  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onTelemetryRecv);

  // Register drone as peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, droneMAC, 6);
  peer.channel = 0;       // match your channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add drone peer");
    while (true);
  }

  Serial.println("Ground station ready");
}

void loop() {
  // 1) Forward any GUI PID updates
  handleSerialCommands();

  // 2) Nothing else—telemetry prints via callback

  // small delay to avoid tight spin
}
