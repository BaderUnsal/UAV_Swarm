#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <TinyGPSPlus.h>

// ——— GPS parsing ———
static const uint32_t GPS_BAUD = 9600;        // your GPS’s baud
static const int      GPS_RX_PIN = 16;       // RX2 pin (to GPS TX)
static const int      GPS_TX_PIN = 17;       // TX2 pin (to GPS RX)
TinyGPSPlus gps;
HardwareSerial GPSser(1);  // use UART1

// ——— Packet format ———
typedef struct {
  float lat;
  float lng;
} GPSData;

// ——— Leader’s MAC ———
uint8_t leaderMAC[6] = { 0x00, 0x4B, 0x12, 0x3B, 0x7F, 0xA8 };  

// ——— Send-status callback ———
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("-> Sent to leader: %s\n", 
                status == ESP_OK ? "OK" : "FAIL");
}

void setup() {
  // Serial for debug
  Serial.begin(115200);
  delay(100);

  // GPS UART
  GPSser.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // WiFi/ESP-NOW init
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) { delay(1000); }
  }
  esp_now_register_send_cb(onDataSent);

  // Add leader as peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, leaderMAC, 6);
  peer.channel = 0;        
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add leader peer");
  }

  Serial.println("Follower ready, streaming GPS to leader...");
}

void loop() {
  // 1) Pump GPS serial into parser
  while (GPSser.available()) {
    gps.encode(GPSser.read());
  }

  // 2) On each valid, updated fix, send a packet
  if (gps.location.isUpdated() && gps.location.isValid()) {
    GPSData pkt = {
      (float)gps.location.lat(),
      (float)gps.location.lng()
    };
    Serial.println((float)gps.location.lat());
    Serial.println((float)gps.location.lng());
    
    esp_err_t res = esp_now_send(leaderMAC, 
                                 (uint8_t*)&pkt, 
                                 sizeof(pkt));
    delay(200); 
  }
}
