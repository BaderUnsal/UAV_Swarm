#include <SPI.h>
#include <ESP32Servo.h>
#include "ICM_20948.h"
#include <RunningMedian.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WiFiManager.h> 
#include <EasyBNO055_ESP.h>
#include "Wire.h"
#include <TinyGPSPlus.h>

// GPS RX/TX pins
static const int RXPin = 16;  
static const int TXPin = 17;  
static const uint32_t GPSBaud = 9600;



TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// ——— Incoming GPS struct from each drone ———
typedef struct {
  float lat;
  float lng;
} GPSData;

// ——— Hold the latest positions ———
float mlat=0, mlng=0;
float s1lat=0, s1lng=0;
float s2lat=0, s2lng=0;

// ——— MAC addresses of each drone ———
uint8_t drone2MAC[6] = { 0x00,0x4B,0x12,0x3A,0xA4,0xC4 };
uint8_t drone3MAC[6] = { 0x94,0x54,0xC5,0xE6,0xE2,0x08 };


uint8_t peerMAC[6] = { 0x34, 0x5F, 0x45, 0xAC, 0x09,0xC0};
// MAC of the ground-station ESP that will receive telemetry
uint8_t telemetryMAC[6] = {0x34,0x5f,0x45,0xab,0xe5,0xf0};
//34:5f:45:ab:e5:f0
struct TelemetryData {
  float Desired_Roll;
  float Desired_Pitch;
  float Desired_Yaw;
  float Actual_Roll;
  float Actual_Pitch;
  float Actual_Yaw;
};



typedef struct {
  float lat1, lng1;   // Master
  float lat2, lng2;   // Slave 1
  float lat3, lng3;   // Slave 2
} SwarmData;


TelemetryData telemetryData;

// Data structure for incoming data
struct DataPackage {
  int16_t Roll_Input;    // Desired roll angle (-30,30, scaled)
  int16_t Pitch_Input;   // Desired pitch angle (-30,30, scaled)
  int16_t Yaw_Input;     // Desired Yaw angl (-30,30, scaled)
  int16_t Alt_Input;     // Desired throttle (0-150, scaled)
  bool Armed_Status;     // 0 = Disarmed, 1 = Armed
};
DataPackage receivedData;

bool newPacket = false;
// Variables for commands 
double Desired_Roll = 0.0;
double Desired_Pitch = 0.0;
double Desired_Yaw = 0.0;
double Desired_Altitude = 0.0;
bool Armed = false;

// IMU readings
double Actual_Roll = 0.0;
double Actual_Pitch = 0.0;
double Actual_Yaw = 0.0;
double Actual_Altitude = 0.0;

// Motor setup
Servo Motor_FL, Motor_FR, Motor_BR, Motor_BL;  // Front-Left (3), Front-Right (1), Back-Right (4), Back-Left (2)
const int Motor_FL_Pin = 4;
const int Motor_FR_Pin = 32;
const int Motor_BR_Pin = 33;
const int Motor_BL_Pin = 2;

// IMU setup 
EasyBNO055_ESP bno;

// PID Control Variables
double Roll_PID_Output, Pitch_PID_Output, Yaw_PID_Output;
double Roll_Error_Previous, Pitch_Error_Previous, Yaw_Error_Previous;
double Roll_Integral, Pitch_Integral, Yaw_Integral;

// Constants for PID control
float Roll_KP = 7, Roll_KI = 0.1, Roll_KD = 1.5;
float Pitch_KP = 7, Pitch_KI = 0.1, Pitch_KD = 1.5;
float Yaw_KP = 6, Yaw_KI = 0.05, Yaw_KD = 0.0015;
double Integral_Limit = 25.0;
float Roll_offset =0, Pitch_offset = 0, Yaw_offset = 0; 
// Onboard LED pin
#define LED_PIN 0

// Maxsonar EZ2
const int pw_pin = 15;
const int arraysize = 9;
RunningMedian distanceFilter = RunningMedian(arraysize);

// Timing variables
const unsigned long loopInterval = 1000000 / 100;  
static unsigned long lastSignalTime = 0; 
int pid_int_count = 0;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin,TXPin);
  initESPNow();

  // Initialize onboard LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);


  // Initialize Maxsonar
  pinMode(pw_pin, INPUT);


  // Initialize IMU
  bno.start(&otherI2CUpdate);

  // Initialize Motors
  initMotors();

  // Blink LED 3 times to indicate setup completion
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  digitalWrite(LED_PIN, HIGH);  
}

void loop() {
  bool imu_correct = true;
  // Record the start time of the loop
  unsigned long loopStartTime = micros();
  unsigned long t0 = micros();
  unsigned long t1;
  unsigned long t2;
  unsigned long t3;
  unsigned long t4;
  unsigned long t5;


  // Read RF24 data
   if (newPacket) {
  newPacket = false;
  lastSignalTime = millis();
  }
   else if (millis() - lastSignalTime > 1000) {
       receivedData.Armed_Status = 0;
   }
    t1 = micros();

  // If disarmed, reset motor outputs and skip control loop
  if (!receivedData.Armed_Status) {
    resetMotors();
  } else {  
    // Update desired angles and throttle
    Desired_Roll = receivedData.Roll_Input;     // Roll (-30 to 30 degrees)
    Desired_Pitch = receivedData.Pitch_Input;   // Pitch (-30 to 30 degrees)
    Desired_Yaw = receivedData.Yaw_Input;       // Altitude (0 to 150 cm)
    Desired_Altitude = receivedData.Alt_Input;  // Altitude (0 to 150 cm)

    // Get IMU data
    updateIMUData();
    t2 = micros();
    if (isnan(Actual_Roll) || isnan(Actual_Pitch) || isnan(Actual_Yaw))
      imu_correct = false;
    else 
      imu_correct = true;
    if (imu_correct){
      // Get altitude
      Actual_Altitude = readDistance();

      // Perform PID Control
      performPIDControl();
      t3 = micros();

      // Mix motor signals and send commands
      motorMixer();
      t4 = micros();
    }
    
    
  }



  // Delay to maintain 100 Hz loop frequency
    
 getGPSCoordinates(mlat, mlng);
    sendSwarmData(mlat, mlng,
                s1lat, s1lng,
                s2lat, s2lng);
  // Serial.print("dt:");
  // Serial.println(dt);
  telemetryData.Desired_Roll = Desired_Roll;
  telemetryData.Desired_Pitch = Desired_Pitch;
  telemetryData.Desired_Yaw = Desired_Yaw;
  telemetryData.Actual_Roll = Actual_Roll;
  telemetryData.Actual_Pitch = Actual_Pitch;
  telemetryData.Actual_Yaw = Actual_Yaw;

  //Serial.println(Roll_KP);
  //if (!client.connected()) {
    //Serial.println("Disconnected! Reconnecting...");
  //  client.connect(serverIP, serverPort);
  //}

  //receive PID values
  //receivePIDValues();

 esp_now_send(telemetryMAC,
              (uint8_t*)&telemetryData,
              sizeof(telemetryData));
  // Send data via TCP
  //client.write((uint8_t*)&telemetryData, sizeof(telemetryData));
  while (micros() - loopStartTime < loopInterval);
  unsigned long loopElapsedTime = micros() - loopStartTime;

  /*Serial.print(" Radio read: ");
  Serial.print(t1-t0);
  Serial.print(" IMU read: ");
  Serial.print(t2-t1);
  Serial.print(" PID loop: ");
  Serial.print(t3-t2);
  Serial.print(" Motor mixer: ");
  Serial.println(t4-t3);
  Serial.print(" Delay: ");
  Serial.println(loopElapsedTime);
  */
}

void connectToWifi() {
  // Create an instance of WiFiManager
  WiFiManager wm;

  // Optional: Set a timeout for the WiFi configuration portal
  wm.setConfigPortalTimeout(180); // 3 minutes

  // Attempt to auto-connect to a saved WiFi network
  // If none exists, start an access point with the specified SSID
  if (!wm.autoConnect("MyWiFiManagerAP")) {
    //Serial.println("Failed to connect to WiFi and timeout reached. Restarting...");
    ESP.restart();
  }

  // Once connected, print the local IP address
  //Serial.println("Connected to WiFi!");
  //Serial.println("IP Address: " + WiFi.localIP().toString());
}


void initMotors() {
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  delay(1000);
  Motor_FL.attach(Motor_FL_Pin,1000,2000);
  delay(1000);
  Motor_FL.setPeriodHertz(500);
  delay(100);
  Motor_FR.attach(Motor_FR_Pin,1000,2000);
  delay(1000);
  Motor_FR.setPeriodHertz(500);
  delay(100);
  Motor_BR.attach(Motor_BR_Pin,1000,2000);
  delay(1000);
  Motor_BR.setPeriodHertz(500);
  delay(100);
  Motor_BL.attach(Motor_BL_Pin,1000,2000);
  delay(1000);
  Motor_BL.setPeriodHertz(500);
  delay(100);

  Motor_FL.writeMicroseconds(1000);
  Motor_FR.writeMicroseconds(1000);
  Motor_BR.writeMicroseconds(1000);
  Motor_BL.writeMicroseconds(1000);

  delay(2000);  // Allow ESCs to initialize
}

void resetMotors() {
  Motor_FL.writeMicroseconds(1000);
  Motor_FR.writeMicroseconds(1000);
  Motor_BR.writeMicroseconds(1000);
  Motor_BL.writeMicroseconds(1000);
}


int readDistance() {
  // Clear previous values
  distanceFilter.clear();

  // Read sensor values and add them to the RunningMedian object
  for (int i = 0; i < arraysize; i++) {
    // int distance = pulseIn(pw_pin, HIGH) / 58; // Convert to cm
    int distance = 0;
    distanceFilter.add(distance);
  }

  // Get the median value
  int medianDistance = distanceFilter.getMedian();

  // Print the median distance
  // Serial.print("The distance = ");
  // Serial.print(medianDistance);
  // Serial.println(" cm");

  // Return the median value
  return medianDistance;
}

void updateIMUData() {
  Actual_Yaw = fmod(bno.orientationZ, 360.0);
  Actual_Pitch = bno.orientationY;
  Actual_Roll = -bno.orientationX;
  
  if (Actual_Yaw > 180)
    Actual_Yaw = Actual_Yaw - 360;
  Actual_Yaw = -Actual_Yaw;
}

void performPIDControl() {
  // Roll PID
  double Roll_Error = Desired_Roll  - Actual_Roll + Roll_offset;
  Roll_Integral += (Roll_Error + Roll_Error_Previous);
  Roll_Integral = constrain(Roll_Integral, -Integral_Limit, Integral_Limit);
  if (abs(Roll_Error) <= 0 || isnan(Roll_Integral) || pid_int_count > 5) { Roll_Integral = 0; pid_int_count = 0; Roll_Error = 0;}
  double Roll_Derivative = (Roll_Error - Roll_Error_Previous);
  // double Roll_Derivative = imu.agmt.gyr.axes.y;
  Roll_Error_Previous = Roll_Error;
  Roll_PID_Output = (Roll_KP * Roll_Error) + Roll_KI * Roll_Integral + (Roll_KD * Roll_Derivative * 100);

  // Pitch PID
  double Pitch_Error = Desired_Pitch - Actual_Pitch + Pitch_offset;
  Pitch_Integral += (Pitch_Error + Pitch_Error_Previous);
  Pitch_Integral = constrain(Pitch_Integral, -Integral_Limit, Integral_Limit);
  if (Pitch_Error == 0 || isnan(Pitch_Integral)) { Pitch_Integral = 0; }
  double Pitch_Derivative = (Pitch_Error - Pitch_Error_Previous);
  Pitch_Error_Previous = Pitch_Error;
  Pitch_PID_Output = (Pitch_KP * Pitch_Error) + (Pitch_KI * Pitch_Integral) + (Pitch_KD * Pitch_Derivative * 100);

  // Yaw PID
  double Yaw_Error = Desired_Yaw - Actual_Yaw + Yaw_offset;
  double Yaw_Proportional = Yaw_Error;
  Yaw_Integral += (Yaw_Error + Yaw_Error_Previous);
  Yaw_Integral = constrain(Yaw_Integral, -Integral_Limit, Integral_Limit);
  if (Yaw_Error == 0 || isnan(Yaw_Integral)) { Yaw_Integral = 0; }
  double Yaw_Derivative = (Yaw_Error - Yaw_Error_Previous);
  Yaw_Error_Previous = Yaw_Error;
  Yaw_PID_Output = (Yaw_KP * Yaw_Proportional) + (Yaw_KI * Yaw_Integral) + (Yaw_KD * Yaw_Derivative * 100);


  // Serial.print(F("Roll PID: "));
  // Serial.print(Roll_PID_Output);
  // Serial.print(F(", Pitch PID: "));
  // Serial.print(Pitch_PID_Output);
  // Serial.print(F(", Yaw PID: "));
  // Serial.println(Yaw_PID_Output);
}

void motorMixer() {
  // Mix motor signals
  Desired_Altitude = map(Desired_Altitude, 0, 150, 1000, 2000);
  double Motor_FL_Signal = Desired_Altitude - Pitch_PID_Output + Roll_PID_Output - Yaw_PID_Output; 
  double Motor_FR_Signal = Desired_Altitude - Pitch_PID_Output - Roll_PID_Output + Yaw_PID_Output; 
  double Motor_BR_Signal = Desired_Altitude + Pitch_PID_Output - Roll_PID_Output - Yaw_PID_Output; 
  double Motor_BL_Signal = Desired_Altitude + Pitch_PID_Output + Roll_PID_Output + Yaw_PID_Output;

  // Convert to PWM
  Motor_FL.writeMicroseconds(constrain(Motor_FL_Signal, 1050, 2000));
  Motor_FR.writeMicroseconds(constrain(Motor_FR_Signal, 1050, 2000));
  Motor_BR.writeMicroseconds(constrain(Motor_BR_Signal, 1050, 2000));
  Motor_BL.writeMicroseconds(constrain(Motor_BL_Signal, 1050, 2000));

  // Serial.print(F("Motor_FL_Signal: "));
  // Serial.print(Motor_FL_Signal);
  // Serial.print(F(", Motor_FR_Signal: "));
  // Serial.print(Motor_FR_Signal);
  // Serial.print(F(", Motor_BR_Signal: "));
  // Serial.print(Motor_BR_Signal);
  // Serial.print(F(", Motor_BL_Signal: "));
  // Serial.println(Motor_BL_Signal);
}

/*void receivePIDValues() {
  if (client.available() >= sizeof(int) + sizeof(double)) {
      int param_index;
      float new_value;

      client.read((uint8_t*)&param_index, sizeof(int));
      client.read((uint8_t*)&new_value, sizeof(float));
      Serial.println(new_value);

      switch (param_index) {
          case 0: Roll_KP = new_value; break;
          case 1: Roll_KI = new_value; break;
          case 2: Roll_KD = new_value; break;
          case 3: Pitch_KP = new_value; break;
          case 4: Pitch_KI = new_value; break;
          case 5: Pitch_KD = new_value; break;
          case 6: Yaw_KP = new_value; break;
          case 7: Yaw_KI = new_value; break;
          case 8: Yaw_KD = new_value; break;
      }

      //Serial.printf("Updated PID: Index %d -> Value %.2f\n", param_index, new_value);
  }
}*/

void otherI2CUpdate(){
	//Serial.println("otherI2CUpdate");
}

///GPS obtaining coordinates function

bool getGPSCoordinates(float &latitude, float &longitude) {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);

    if (gps.location.isUpdated() && gps.location.isValid()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      return true;
    }
  }
  return false; // No valid coordinates found
}

// ——————————————————————————————————————————
// 1) Initialize ESP-NOW and register peer & callback:

void initESPNow() {
  // station mode only
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while(true);
  }
  // receive joystick & PID packets
  esp_now_register_recv_cb(onDataReceived);
  // (optional) track your own send status
  // esp_now_register_send_cb(onDataSent);

  // add joystick transmitter as a peer
  {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, peerMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }

  // add ground-station as peer for telemetry
  {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, telemetryMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }
  {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, drone2MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }
  {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, drone3MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }
}


// ——————————————————————————————————————————
// 2) This will fire whenever a packet arrives:

void sendSwarmData(float mlat, float mlng,
                   float s1lat, float s1lng,
                   float s2lat, float s2lng) {
  SwarmData pkt = { mlat, mlng, s1lat, s1lng, s2lat, s2lng };
  esp_now_send(telemetryMAC, (uint8_t*)&pkt, sizeof(pkt));
}


void onDataReceived(const esp_now_recv_info_t *info, 
                    const uint8_t *data, int len) {
  // 1) Joystick / PID‐command data?
  if (len == sizeof(DataPackage)) {
    memcpy(&receivedData, data, len);
    newPacket = true;
    // send back a 1-byte ACK so remote sees “Connected”
    uint8_t ack = 0x01;
    esp_now_send(info->src_addr, &ack, sizeof(ack));
    Serial.println("received joystick/command packet");
    return;
  }

  // 2) GPS update from a slave/master?
  if (len == sizeof(GPSData)) {
    GPSData gps;
    memcpy(&gps, data, sizeof(gps));

    // Figure out which drone sent it
    if (memcmp(info->src_addr, drone2MAC, 6) == 0) {
      s1lat = gps.lat;   s1lng = gps.lng;
    }
    else if (memcmp(info->src_addr, drone3MAC, 6) == 0) {
      s2lat = gps.lat;   s2lng = gps.lng;
    }
    else {
      // Unknown sender—ignore
      return;
    }

    // Broadcast the full swarm positions to the ground station
    sendSwarmData(mlat, mlng,
                  s1lat, s1lng,
                  s2lat, s2lng);
    Serial.println("rebroadcasted swarm packet");
  }
}


//   // 2) PID-tuning data? (4-byte int32 + 4-byte float)
//   else {
//     int32_t idx;
//     float   val;
//     memcpy(&idx, data,           sizeof(idx));
//     memcpy(&val, data + sizeof(idx), sizeof(val));
//     switch (idx) {
//       case 0: Roll_KP  = val; break;
//       case 1: Roll_KI  = val; break;
//       case 2: Roll_KD  = val; break;
//       case 3: Pitch_KP = val; break;
//       case 4: Pitch_KI = val; break;
//       case 5: Pitch_KD = val; break;
//       case 6: Yaw_KP   = val; break;
//       case 7: Yaw_KI   = val; break;
//       case 8: Yaw_KD   = val; break;
//       case 9: Roll_offset = val; break;
//       case 10: Pitch_offset = val; break;
//       case 11: Yaw_offset = val; break;
      
//     }
//     Serial.printf("PID[%d]=%.3f\n", idx, val);
//     Serial.println("received from ground station");
//   }
// }

