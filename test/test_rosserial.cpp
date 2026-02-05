/**
 * Minimal rosserial test for STM32 USB CDC
 *
 * IMPORTANT: This file demonstrates the CORRECT way to use rosserial on STM32
 *
 * To test this:
 * 1. Rename src/main.cpp to src/main.cpp.bak
 * 2. Copy this file to src/main.cpp
 * 3. Upload to STM32
 * 4. Run: rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
 * 5. Check: rostopic echo /chatter
 */

#include <Arduino.h>

// ============================================================================
// CRITICAL: Define buffer sizes BEFORE including ros.h
// ============================================================================
// STM32 USB CDC needs larger buffers than the default 512 bytes
// Without this, you'll get "Unable to sync with device" errors
#ifndef INPUT_SIZE
  #define INPUT_SIZE 1024
#endif

#ifndef OUTPUT_SIZE
  #define OUTPUT_SIZE 1024
#endif

// Now include ROS headers
#include <ros.h>
#include <std_msgs/String.h>

// ROS node handle
ros::NodeHandle nh;

// Message and publisher
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup() {
  // CRITICAL: Initialize Serial for USB CDC
  // Even though rosserial will use it, we must call Serial.begin()
  Serial.begin(115200);

  // CRITICAL: Wait for USB CDC to enumerate on STM32
  // Without this delay, rosserial sync will fail
  delay(1000);

  // Initialize ROS node
  nh.initNode();

  // Wait a bit before advertising
  delay(100);

  // Advertise the publisher
  nh.advertise(chatter);

  // Wait for connection (optional, but helpful)
  unsigned long startTime = millis();
  while (!nh.connected() && (millis() - startTime < 5000)) {
    nh.spinOnce();
    delay(50);
  }

  if (nh.connected()) {
    nh.loginfo("STM32 Hello World connected!");
  }
}

void loop() {
  // Publish message
  str_msg.data = hello;
  chatter.publish(&str_msg);

  // CRITICAL: Must call spinOnce() regularly
  nh.spinOnce();

  // Wait 1 second
  delay(1000);
}
