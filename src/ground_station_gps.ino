#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ——— Define the incoming data structure ———
// Must match exactly how the leader packed it!
typedef struct {
  float lat1;
  float lng1;
  float lat2;
  float lng2;
  float lat3;
  float lng3;
} SwarmData;

static const uint32_t SERIAL_BAUD = 115200;

// Drone’s ESP-NOW MAC address (replace with your drone’s MAC)
uint8_t droneMAC[6] = { 0x00, 0x4B, 0x12, 0x3B, 0x7F, 0xA8 };

// ——— ESP-NOW receive callback ———
void onSwarmDataReceived(const esp_now_recv_info_t* info,
                         const uint8_t* rawData, int len) {
  if (len != sizeof(SwarmData)) return;

  SwarmData data;
  memcpy(&data, rawData, sizeof(data));

  // Make a larger buffer
  static char buf[256];

  // snprintf returns the number of bytes **that would have been written**,
  // not counting the trailing null.
  if(data.lat1 !=0 && data.lat2 !=0 && data.lat3 !=0){
  int msgLen = snprintf(buf, sizeof(buf),
    "{\"drone1\":{\"lat\":%.6f,\"lng\":%.6f},"
    "\"drone2\":{\"lat\":%.6f,\"lng\":%.6f},"
    "\"drone3\":{\"lat\":%.6f,\"lng\":%.6f}}",
    data.lat1, data.lng1,
    data.lat2, data.lng2,
    data.lat3, data.lng3
  );
  
  // If msgLen is negative or >= sizeof(buf), we've had an error/truncation
  if (msgLen > 0 && msgLen < sizeof(buf)) {
    // Send exactly msgLen bytes, then CR+LF
    Serial.write((uint8_t*)buf, msgLen);
    Serial.write("\r\n", 2);
  } else {
    Serial.println(F("Error: JSON too long or snprintf failed"));
  }
  }
}



void setup() {
  // Start Serial for the Python GUI
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {}

  // Init WiFi & ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true);
  }

  // Register callback
  esp_now_register_recv_cb(onSwarmDataReceived);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, droneMAC, 6);
  peer.channel = 0;       // match your channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add drone peer");
    while (true);
  }
  
  //Serial.println("Ground Station ready, listening for swarm data...");
}

void loop() {
  // Nothing else to do—onSwarmDataReceived handles incoming packets.
}
