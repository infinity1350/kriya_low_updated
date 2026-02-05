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
}
