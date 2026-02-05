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
  // Longer delay ensures host-side driver is ready too
  delay(2000);

  // Initialize ROS node
  nh.initNode();

  // Small delay after init
  delay(100);

  // Advertise publishers
  nh.advertise(hello_pub);
  nh.advertise(counter_pub);

  // Don't try to connect here - let loop() handle it
  // The host might not be running rosserial yet
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
  // CRITICAL: Call spinOnce() frequently to handle ROS communication
  // MUST be called more frequently than SERIAL_MSG_TIMEOUT (20ms)
  nh.spinOnce();

  // Only publish when connected to avoid junk data during sync
  if (nh.connected()) {
    static unsigned long lastPublish = 0;
    unsigned long now = millis();

    // Publish every 500ms (2 Hz)
    if (now - lastPublish >= 500) {
      lastPublish = now;

      // Create and publish hello message
      snprintf(hello_buffer, sizeof(hello_buffer), "Hello from STM32! Count: %d", counter);
      hello_msg.data = hello_buffer;
      hello_pub.publish(&hello_msg);

      // Publish counter
      counter_msg.data = counter;
      counter_pub.publish(&counter_msg);

      // Increment counter
      counter++;
    }
  }

  // Short delay - spinOnce() is called every 10ms (100Hz)
  // This is MUCH faster than the 20ms SERIAL_MSG_TIMEOUT
  delay(10);
}
