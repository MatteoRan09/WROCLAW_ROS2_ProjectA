// ========== LIBRARIES ==========
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// ========== FUNCTION PROTOTYPES ==========

void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  char msg[len+1] = {0};
  memcpy(msg, (void*)incomingData, len);

  Serial.print("Received data: ");
  Serial.println((char*) msg);
}

void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) { 
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo){
  if(peerInfo == NULL){
    peerInfo = (esp_now_peer_info_t*)malloc(sizeof(esp_now_peer_info_t));
    memset(peerInfo, 0, sizeof(esp_now_peer_info_t));
  }
  memcpy(peerInfo->peer_addr, mac, 6);
  peerInfo->channel = 0;
  peerInfo->encrypt = false;
  return esp_now_add_peer(peerInfo);
}

// Variables
uint8_t peerMAC[6] = {0x28, 0x05, 0xA5, 0x27, 0x5E, 0x38};

// ========== SETUP FUNCTION ==========
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);;
  // Set the Wi-Fi channel (e.g., channel 1-13), should match between boards
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Register Callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  
  addPeer(peerMAC, NULL);
}

// ========== MAIN LOOP ==========
void loop() {

  delay(1000);
  char msg[64] = "x_cmd=1.0,z_cmd=0.0";
  esp_now_send(peerMAC, (uint8_t*)msg, strlen(msg));
  delay(500);
  char msg_1[64] = "x_cmd=0.0,z_cmd=0.0";
  esp_now_send(peerMAC, (uint8_t*)msg_1, strlen(msg));
}
