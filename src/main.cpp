<<<<<<< HEAD
#include <Arduino.h>

// Buffer sizes BEFORE ros.h
#define INPUT_SIZE 1024
#define OUTPUT_SIZE 1024

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::String hello_msg;
ros::Publisher hello_pub("hello", &hello_msg);

std_msgs::Int32 counter_msg;
ros::Publisher counter_pub("counter", &counter_msg);

char hello_buffer[50];
int counter = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);  // USB CDC enumeration
  
  nh.initNode();
  delay(100);
  
  nh.advertise(hello_pub);
  nh.advertise(counter_pub);
}

void loop() {
  snprintf(hello_buffer, sizeof(hello_buffer), "Hello from STM32! Count: %d", counter);
  hello_msg.data = hello_buffer;
  
  hello_pub.publish(&hello_msg);
  
  counter_msg.data = counter;
  counter_pub.publish(&counter_msg);
  
  counter++;
  
  nh.spinOnce();
  delay(500);  // Publish at 2 Hz
=======
/**
 * Standalone ROS Serial Test for STM32
 * Simple hello world that publishes continuously
 */

#include <Arduino.h>

// Include ROS headers
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

// ============================================================================
// ROS Setup with Custom Buffer Sizes
// ============================================================================
// NodeHandle_<Hardware, MaxSubscribers, MaxPublishers, InputSize, OutputSize>
// Default is 512 bytes for input/output - we increase to 1024 for STM32 USB CDC
ros::NodeHandle_<ArduinoHardware, 25, 25, 1024, 1024> nh;

// Publisher for hello world message
std_msgs::String hello_msg;
ros::Publisher hello_pub("hello", &hello_msg);

// Publisher for counter
std_msgs::Int32 counter_msg;
ros::Publisher counter_pub("counter", &counter_msg);

// Message buffer
char hello_buffer[50];
int counter = 0;

// ============================================================================
// Setup
// ============================================================================
void setup() {
  // CRITICAL: Initialize Serial for USB CDC
  Serial.begin(115200);

  // CRITICAL: Wait for USB CDC to enumerate on STM32
  delay(1000);

  // Initialize ROS node
  nh.initNode();

  // Small delay after init
  delay(100);

  // Advertise publishers
  nh.advertise(hello_pub);
  nh.advertise(counter_pub);

  // Wait for connection
  unsigned long startTime = millis();
  while (!nh.connected() && (millis() - startTime < 5000)) {
    nh.spinOnce();
    delay(50);
  }

  if (nh.connected()) {
    nh.loginfo("STM32 ROS Test - Connected!");
  }
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
  // Create hello message
  snprintf(hello_buffer, sizeof(hello_buffer), "Hello from STM32! Count: %d", counter);
  hello_msg.data = hello_buffer;

  // Publish messages
  hello_pub.publish(&hello_msg);

  counter_msg.data = counter;
  counter_pub.publish(&counter_msg);

  // Increment counter
  counter++;

  // CRITICAL: Must call spinOnce to handle ROS communication
  nh.spinOnce();

  // Publish every 500ms (2 Hz)
  delay(500);
>>>>>>> 4d3ba0aa2a1200b599c8e6cb6f2195869214e379
}
