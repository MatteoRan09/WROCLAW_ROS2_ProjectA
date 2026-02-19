#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// ========== FUNCTION PROTOTYPES ==========
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo);

void setupNOW(){
  WiFi.begin();
  // Set the Wi-Fi channel (e.g., channel 1-13), should match between boards
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  esp_now_init();

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESPNOW setup complete");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  addPeer(peerMAC, NULL);
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

void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  char msg[len+1] = {0};
  memcpy(msg, (void*)incomingData, len);
  float items = sscanf(msg, "x_cmd=%ld,z_cmd=%ld", &x_cmd, &z_cmd);

  Serial.print("Received data: ");
  Serial.println((char*) msg);
}

void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) { 
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}