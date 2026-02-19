// ========== LIBRARIES ==========
#include <ArduinoJson.h>
#include <HCSR04.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// Hardware Serial 2 pins
#define RXD2 16 
#define TXD2 17 
#define RXRV 27 // blue
#define TXRV 26 // white
#define TRIG_PIN 13
#define ECHO_PIN 12


int32_t x_mm = 0; int32_t y_mm = 0; int32_t z_mm = 0; byte quality = 0;
float x_cmd = 0.0; float z_cmd = 0.0;
float posX; float posY; float posZ;

typedef struct {
  float x;
  float y;
  float z;
  float sonar_dist;
  int quality;
} RobotState_t;

typedef struct {
  float x_cmd;
  float z_cmd;
} RobotCommand;


// Variables
uint8_t peerMAC[6] = {0x28, 0x05, 0xA5, 0x27, 0x3A, 0xFC};
RobotState_t myRobotState;
RobotCommand incomingRobotCommand;
RobotCommand myCmdVel;

// uint8_t peerMAC[6] = {0x28, 0x05, 0xa5, 0x26, 0x33, 0x8C};

// ========== SETUP FUNCTION ==========
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXRV, TXRV);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  // Initialize ESP NOW
  setupNOW();
  // Start sonar sensor
  printBeaconStatus();

  HCSR04.begin(TRIG_PIN, ECHO_PIN);
}

// ========== MAIN LOOP ==========
void loop() {
  // Location data read
  // Send status request command 0x32 0x00
  Serial.println("\nSending position request...");
  ReadLocation();
  double* distances = HCSR04.measureDistanceCm();

  // Pretend these are your sensor readings
  myRobotState.x = float(x_mm)/1000;
  myRobotState.y = float(y_mm)/1000;
  myRobotState.z = float(z_mm)/1000;
  myRobotState.sonar_dist = distances[0];
  myRobotState.quality = int(quality);

  // Serial.print("x:"); Serial.println(myRobotState.x);
  // Serial.print("y:"); Serial.println(myRobotState.y);
  // Serial.print("z:"); Serial.println(myRobotState.z);
  // Serial.print("qf:"); Serial.println(myRobotState.quality);

  delay(100);
  // char msg[64];
  // snprintf(msg, sizeof(msg), "x=%2f,y=%2f,z=%2f,q=%hhu, d_s=%2f", posX, posY, posZ, quality, distances[0]);
  // esp_now_send(peerMAC, (uint8_t*)msg, strlen(msg));
  esp_now_send(peerMAC, (uint8_t *) &myRobotState, sizeof(myRobotState));

}

// ========== MAIN LOOP ==========
void writeCmdVel(double x, double z){
  // format {"T":13,"X":0.1,"Z":0.3}
  JsonDocument doc;
  doc["T"] = 13;
  doc["X"] = -x;
  doc["Z"] = -z;
  Serial.println("Json cmmand");
  serializeJson(doc, Serial);
  Serial1.println("\n");
}



