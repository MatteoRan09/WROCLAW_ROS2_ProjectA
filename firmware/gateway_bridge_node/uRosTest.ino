#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

// PIN DEFINITION
#define LED_PIN 2

// ROS Objects
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg_pub;
std_msgs__msg__Bool msg_sub;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ============================================================================
// CALLBACK: Runs when you send a Bool to /test_led
// ============================================================================
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  // Turn LED on/off based on data
  digitalWrite(LED_PIN, (msg->data) ? HIGH : LOW);  
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // 1. Setup Hardware
  set_microros_transports(); // Uses Serial (USB) by default
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  delay(2000);

  // 2. Init Micro-ROS
  allocator = rcl_get_default_allocator();

  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "test_node", "", &support);

  // 3. Create Publisher (Sends Int32)
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "test_counter");

  // 4. Create Subscriber (Receives Bool)
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "test_led");

  // 5. Create Executor (Manage 1 Pub + 1 Sub = 2 Handles? No, just 1 Sub needs management)
  // Actually, usually we add number of subscriptions + number of timers.
  // Here we have 1 subscription.
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  // 1. Update the data
  msg_pub.data++;

  // 2. Publish the data
  rcl_publish(&publisher, &msg_pub, NULL);

  // 3. Spin (Check for incoming data)
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  
  delay(100);
}
