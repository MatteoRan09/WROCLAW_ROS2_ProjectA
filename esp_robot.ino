// ========== LIBRARIES ==========
#include <ArduinoJson.h>

// Hardware Serial 2 pins
#define RXD2 16
#define TXD2 17
#define RXRV 27
#define TXRV 26
#define TRIG_PIN 13
#define ECHO_PIN 12


int32_t x_mm = 0; int32_t y_mm = 0; int32_t z_mm = 0; byte quality = 0;
float x_cmd = 0.0; float z_cmd = 0.0;
float posX; float posY; float posZ;


// Variables
uint8_t peerMAC[6] = {0x28, 0x05, 0xa5, 0x27, 0x3A, 0xFC};

// ========== SETUP FUNCTION ==========
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXRV, TXRV);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Initialize ESP NOW
  setupNOW();
  // Start sonar sensor
  setupSonar();
  printBeaconStatus();
}

// ========== MAIN LOOP ==========
void loop() {
  // Location data read
  // Send status request command 0x32 0x00
  Serial.println("\nSending position request...");
  ReadLocation();
  double* distances = HCSR04. 
  posX = float(x_mm);
  posY = float(y_mm);
  posZ = float(z_mm);

  Serial.print("x:"); Serial.println(x_mm);
  Serial.print("y:"); Serial.println(y_mm);
  Serial.print("z:"); Serial.println(z_mm);
  Serial.print("qf:"); Serial.println(quality);

  delay(100);
  char msg[64];
  snprintf(msg, sizeof(msg), "x=%ld,y=%ld,z=%ld", posX, posY, posZ);
  esp_now_send(peerMAC, (uint8_t*)msg, strlen(msg));

  writeCmdVel(0.1, 0.0);
  delay(1000);
  writeCmdVel(0.0, 0.0);

}

// ========== MAIN LOOP ==========
void writeCmdVel(double x, double z){
  // format {"T":13,"X":0.1,"Z":0.3}
  JsonDocument doc;
  doc["T"] = 13;
  doc["X"] = x;
  doc["Z"] = z;
  serializeJson(doc, Serial1);
}



