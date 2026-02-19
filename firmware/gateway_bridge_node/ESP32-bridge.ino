// ========== LIBRARIES ==========
#include <Arduino.h>
#include <math.h>

// ESP NOW 
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

//MICRO ROS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point32.h>

// NOT ME - ESP 1 reads sensors (x, y, z, sonar, quality), packs them into a struct, and sends via ESP-NOW to the Bridge.
// I receive the packet and publish it to ROS 2 topics (/odom or /sonar)
// NOT ME - ROS 2 Navigation calculates velocity commands (Twist message: linear X, angular Z).
// I need to subscribe to /cmd_vel to access x linear, z angular data from ROS
// Pack it into a struct and send it via ESP-NOW to ESP 1.
// NOT ME - ESP 1 receives the ESP-NOW packet, parses the velocities, and drives the motors.

// ========== FUNCTION PROTOTYPES esp ==========
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo);


// ========== CONFIG ==========
uint8_t peerMAC[6] = {0x28, 0x05, 0xA5, 0x27, 0x5E, 0x38};

// ========== STRUCTS ==========

typedef struct {
  float x;
  float y;
  float z;
  float sonar_dist;
  int quality;
} RobotState_t; // It's a type, the struct in not created yet

// Simple command packet to send to Robot
typedef struct {
  float linear_x;
  float angular_z;
} RobotCommand_t;

// Declaration of the structs I will receive
//"Reserve some memory space right now to hold these specific shapes of data."
RobotState_t incomingRobotState; // Comes from ESP1
RobotCommand_t outgoingCommand; // sent to ESP1

// ========== ROS OBJECTS ==========

rcl_publisher_t odom_publisher;      // Publisher for Position
geometry_msgs__msg__Point32 odom_msg; // The message container (x,y,z)

rcl_publisher_t publisher; 
std_msgs__msg__Float32 sonar_msg; // The message sent to PC

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg; // The message received from PC

// Scheduler - It manages the "to-do list." 
// It checks if new messages arrived and triggers callbacks to handle them.
rclc_executor_t executor;

// The Foundation - It holds the system status. It keeps the connection to the Agent alive, 
// manages the clock, and ensures the "plumbing" is working.
rclc_support_t support;

// Memory Manager -It tells micro-ROS how to grab RAM
rcl_allocator_t allocator;

// This IS the robot in the ROS world. It gives the ESP32 a name (e.g., "esp32_bridge")
// so other nodes know who is talking.
rcl_node_t node;

// Global variables
float current_sonar_dist = 0.0;
bool use_mock_data = true; // Will be set to FALSE when receiving data from ESP1

// ========== ESP-NOW CALLBACKS ==========

// On Data Sent (Confirmation)
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  // Intentionally empty. 
  // Cannot use Serial.print because Micro-ROS owns the Serial port.
  digitalWrite(2, HIGH);
}

// On Data Receive (From Robot)
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  if (len != sizeof(RobotState_t)) return; // Safety check
 
  //Go to the memory address incomingData. Grab exactly sizeof(RobotState_t) bytes from there. 
  //Paste them directly into the memory address of my variable incomingRobotState.
  memcpy(&incomingRobotState, incomingData, sizeof(RobotState_t));
  //float items = sscanf(msg, "x=%2f,y=%2f,z=%2f,q=%hhu,d_s=%2f", &odom_msg.x, &odom_msg.y, &odom_msg.z);
  
  // Update the global variable with REAL data later
  current_sonar_dist = incomingRobotState.sonar_dist;
  // Update the ROS Message DIRECTLY here (most efficient way)
  odom_msg.x = incomingRobotState.x;
  odom_msg.y = incomingRobotState.y;
  odom_msg.z = incomingRobotState.z;
  use_mock_data = false;
}

// ========== ROS CALLBACKS ==========

// Subscription Callback (From PC)
// When PC says "Move", we tell ESP-NOW "Move"
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // Pack the struct
  outgoingCommand.linear_x = msg->linear.x;
  outgoingCommand.angular_z = msg->angular.z;

  // Send via ESP-NOW to Robot
  esp_now_send(peerMAC, (uint8_t *) &outgoingCommand, sizeof(outgoingCommand));


  // char string1[] = "Hello"; 
  // esp_now_send(peerMAC, (uint8_t *) string1, strlen(string1));
}

// ========== SETUP ==========
void setup() {
  // Init Micro-ROS Transports (Takes over Serial)
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  // Init WiFi & ESP-NOW
  WiFi.begin();//WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    
    // Add Peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo)); // FIXED : Cleaning peerInfo before peer_addr
    memcpy(peerInfo.peer_addr, peerMAC, 6);
    peerInfo.channel = 0;  // has to match WiFi channel
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }


  set_microros_transports(); 
  // Init Micro-ROS Node
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_bridge", "", &support);

  // Create Publisher for Position (to PC)
  rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32),
    "/groupA/robot_pos" 
  );

  // Create Publisher for sonar (To PC)
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/groupA/sonar_dist"
  );

  // Create Subscriber for twist (From PC)
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/groupA/cmd_vel"
  );

  // Init Executor (Total handles = 1 subscriber)
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA);
}

// ========== LOOP ==========
void loop() {
  

  // --- MOCKING LOGIC (For Testing Only) ---
  if (use_mock_data) {
    // mock sonar
    current_sonar_dist += 0.5;
    if (current_sonar_dist > 50.0) current_sonar_dist = 0.0;

    // Mock Position (Circle path)
    // We use a static variable 'angle' so it remembers its value between loops
    static float angle = 0.0; 
    float radius = 2.0; // 2 meters radius

    // Calculate x and y based on the angle (basic trigonometry)
    odom_msg.x = radius * cos(angle);
    odom_msg.y = radius * sin(angle);
    odom_msg.z = 0.0; // Robot is on the ground

    // Increment angle for next loop (approx 1 full circle every 6 seconds at 100ms delay)
    angle += 0.1; 
    if (angle > 6.28) angle = 0.0; // Reset after 2*PI (360 degrees)
  }
  // ----------------------------------------
  
  // Publish current state to ROS
  sonar_msg.data = current_sonar_dist;
  rcl_publish(&publisher, &sonar_msg, NULL);

  // Publish Position to ROS
  rcl_publish(&odom_publisher, &odom_msg, NULL);

  // Process ROS callbacks (Check for new cmd_vel)
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // This is the subscribe command
  delay(100);
}