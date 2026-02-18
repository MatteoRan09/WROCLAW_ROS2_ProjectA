// ========== LIBRARIES ==========
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/float32.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// ========== Variables ==========
uint8_t peerMAC[6] = {0x28, 0x05, 0xA5, 0x27, 0x5E, 0x38};

// ROS Objects
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Mock Data Variable
float received_distance = 0.0;

typedef struct {
  float x;
  float y;
  float z;
  float sonar_dist; // Distance in cm
  int quality;
} RobotState_t;

typedef struct {
  float x_cmd;       // Position in meters
  float z_cmd;       // Rotation in radians
} RobotCommand_t;


RobotState_t myRobotData;


// ========== ESP-NOW ==========
// The Sender
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) { 
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// The Receiver
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  char msg[len+1] = {0};
  memcpy(msg, (void*)incomingData, len);

  Serial.print("Received data: ");
  Serial.println((char*) msg);
}

void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  // Check if the size matches the struct to avoid memory errors
  if (len != sizeof(RobotState_t)) return;

  // Translate raw bytes into the Struct
  memcpy(&myRobotData, incomingData, sizeof(RobotState_t));

  // Update the ROS message variable directly
  // Now the loop() will publish the REAL value, not the mock value
  received_distance = myRobotData.sonar_dist; 
}

// Add Peer ESP
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

// ========== ROS ==========
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_twist;

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // Extract values
  float x_cmd = msg->linear.x;
  float z_cmd = msg->angular.z;

  // Prepare ESP-NOW packet (Simple array for now)
  // [Linear, Angular]
  float command_packet[2] = {linear_x, angular_z};

  // Send to ESP-Robot
  esp_now_send(peerMAC, 
  (uint8_t *) &command_packet, // "Start reading bytes from this memory address."
  sizeof(command_packet));
}

// ========== SETUP FUNCTION ==========
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  set_microros_transports(); // Hooks up to USB Serial by default for ROS
  
  //Init Wifi
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station
  // Set the Wi-Fi channel (e.g., channel 1-13), should match between boards
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    // Serial.println("Error"); // CANNOT USE SERIAL PRINT HERE
    return;
  }

  //Register Callbacks for the ESP-Robot
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Add Subscriber to ROS
  rclc_subscription_init_default(
  &subscriber, // "Here is the address where I want you to build the subscriber."
  &node,       // "Here is the address of the node to attach it to."
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  "cmd_vel"
  );

  // Add to Executor , Increase handle count from 1 to 2
  rclc_executor_init(&executor, &support.context, 2, &allocator); 
  rclc_executor_add_subscription(&executor, &subscriber, &msg_twist, &subscription_callback, ON_NEW_DATA);
  
  addPeer(peerMAC, NULL);

  // ========== Micro ROS ==========
    // Init Micro-ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_bridge", "", &support);

  // Create Publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "sonar_dist"
  );
  
  // Create Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
}



}

// ========== MAIN LOOP ==========
void loop() {
  // MOCKING ESP1: Increment distance to see it changing in ROS
  received_distance += 0.1;
  if(received_distance > 50.0) received_distance = 0.0;

  Subscribe to ESP

  // Publish to ESP trough ESP-NOW
  delay(1000);  // Send data every 1000ms
  char msg[16] = "ESP Bridge";
  esp_now_send(peerMAC, (uint8_t*)msg, strlen(msg));

  // Publish to ROS
  msg.data = received_distance;
  rcl_publish(&publisher, &msg, NULL);

  // Spin (process callbacks)
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(100);
}