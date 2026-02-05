# ROS Serial Troubleshooting Guide for STM32

## Why Your Sample Code Failed

Your "hello world" sample code failed because it was **missing critical STM32-specific configuration**:

### ❌ What Was Missing:

```cpp
// Your sample code:
#include <ros.h>          // ← WRONG ORDER!
#include <std_msgs/String.h>

ros::NodeHandle nh;       // ← Uses default 512-byte buffers

void setup() {
  nh.initNode();          // ← No Serial.begin() or delays!
  nh.advertise(chatter);
}
```

### ✅ What STM32 Needs:

```cpp
// Buffer sizes MUST be defined BEFORE including ros.h
#define INPUT_SIZE 1024
#define OUTPUT_SIZE 1024

#include <ros.h>          // ← Now ros.h uses 1024-byte buffers
#include <std_msgs/String.h>

ros::NodeHandle nh;

void setup() {
  Serial.begin(115200);   // ← MUST initialize USB CDC
  delay(1000);            // ← MUST wait for USB enumeration

  nh.initNode();
  delay(100);
  nh.advertise(chatter);
}
```

---

## Root Causes of "Unable to sync with device"

### 1. **Buffer Size Issue** (Most Common)

**Problem:**
- Default rosserial buffers: **512 bytes**
- STM32 USB CDC packets during sync: **600-800 bytes**
- Result: Buffer overflow → corrupted handshake → sync failure

**Solution:**
```cpp
// BEFORE including ros.h
#define INPUT_SIZE 1024
#define OUTPUT_SIZE 1024
```

**Why this matters:**
During the initial handshake, rosserial:
1. Sends topic list (multiple topics = large packet)
2. Sends service list
3. Negotiates message definitions

With multiple publishers/subscribers, these packets exceed 512 bytes.

---

### 2. **USB CDC Initialization Timing**

**Problem:**
- STM32 USB CDC needs ~500-1000ms to enumerate
- If rosserial starts too early → communication fails
- No error message, just silent failure

**Solution:**
```cpp
void setup() {
  Serial.begin(115200);
  delay(1000);  // ← Critical! Wait for USB CDC
  nh.initNode();
}
```

**What happens without delay:**
```
Time 0ms:     Serial.begin(115200) called
Time 10ms:    nh.initNode() tries to send
Time 10ms:    USB CDC not ready → packets lost
Time 15000ms: rosserial times out with "Unable to sync"
```

**With proper delay:**
```
Time 0ms:     Serial.begin(115200) called
Time 1000ms:  USB CDC fully enumerated ✓
Time 1000ms:  nh.initNode() sends packets ✓
Time 1100ms:  Sync successful ✓
```

---

### 3. **Serial.begin() Not Called**

**Problem:**
- Arduino examples assume Serial auto-initializes
- STM32 requires explicit `Serial.begin()` for USB CDC
- Without it, USB device never appears

**Solution:**
```cpp
void setup() {
  Serial.begin(115200);  // ← MUST be called explicitly
  delay(1000);
  nh.initNode();
}
```

---

## Complete Working Example

See `test/test_rosserial.cpp` for a minimal working example.

To test:
```bash
# 1. Backup your main.cpp
cd ~/Desktop/kriya_low_updated
mv src/main.cpp src/main.cpp.bak

# 2. Copy test file
cp test/test_rosserial.cpp src/main.cpp

# 3. Build and upload
pio run -t upload

# 4. Wait 3 seconds, then run rosserial
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200

# 5. In another terminal, check the topic
rostopic echo /chatter
```

Expected output:
```
data: "hello world!"
---
data: "hello world!"
---
```

---

## Step-by-Step Debugging

### Step 1: Verify USB Device Appears

```bash
# Before plugging STM32
ls /dev/ttyACM*

# Plug in STM32, wait 3 seconds
ls /dev/ttyACM*

# Should show: /dev/ttyACM0 (or similar)
```

**If no device appears:**
- USB CDC not enabled in firmware
- Check platformio.ini has USB flags
- Try different USB cable (data cable, not charge-only)
- Try different USB port

### Step 2: Check Serial Communication

```bash
# Send raw data to device
echo "test" > /dev/ttyACM0

# Should NOT show permission errors
# If permission denied:
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

### Step 3: Test with Screen/Minicom

```bash
# Connect with screen
screen /dev/ttyACM0 115200

# Press STM32 reset button
# You should NOT see any debug text (if DEBUG_MODE = 0)
# If you see text like "[BOOT] Starting...", DEBUG_MODE is still 1

# Exit screen: Ctrl+A, then K, then Y
```

### Step 4: Test rosserial with Verbose Output

```bash
# Run with Python verbose logging
python -u $(which rosserial_python) serial_node.py /dev/ttyACM0 _baud:=115200

# Watch for:
# - "Requesting topics..." (normal)
# - "Unable to sync" (problem)
# - "Setup publisher" (success!)
```

### Step 5: Reset Timing Test

```bash
# Terminal 1
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200

# Wait for "Requesting topics..."
# Then press RESET button on STM32
# This ensures proper timing synchronization
```

---

## Common Error Messages

### "Unable to sync with device"

**Causes:**
1. ❌ Buffer sizes too small (most common)
2. ❌ No delay after Serial.begin()
3. ❌ DEBUG_MODE = 1 (sending text instead of binary)
4. ❌ Wrong baud rate
5. ❌ USB cable issue

**Fix:**
```cpp
#define INPUT_SIZE 1024
#define OUTPUT_SIZE 1024
#include <ros.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  nh.initNode();
}
```

### "Lost sync with device"

**Causes:**
1. ❌ spinOnce() not called frequently enough
2. ❌ delay() too long in loop()
3. ❌ USB cable unplugged/loose

**Fix:**
```cpp
void loop() {
  // Do work...
  nh.spinOnce();  // ← MUST call regularly (every 10-50ms)
  delay(10);      // ← Keep delays short
}
```

### "Requested topic not published"

**Causes:**
1. ❌ Publisher not advertised before use
2. ❌ Wrong topic name

**Fix:**
```cpp
void setup() {
  nh.initNode();
  delay(100);
  nh.advertise(myPublisher);  // ← Must advertise in setup()
}
```

---

## Technical Details

### Why STM32 is Different from Arduino

**Arduino (e.g., Uno, Mega):**
- Hardware UART for USB (via FTDI chip)
- Auto-initializes with board power
- 512-byte buffers usually sufficient

**STM32 (e.g., F407, F411):**
- USB CDC (software USB stack)
- Requires explicit initialization
- Larger buffers needed due to USB packet overhead
- Enumeration delay required

### Buffer Size Calculation

```
Minimum buffer size = (number of topics * ~80 bytes) + overhead

Example with 5 publishers + 2 subscribers:
= 7 topics * 80 bytes = 560 bytes
= Minimum 600 bytes needed
= Use 1024 bytes for safety margin
```

### USB CDC vs Hardware UART

**USB CDC (STM32):**
- Pros: Built-in, no extra hardware
- Cons: Needs enumeration time, larger buffers
- Speed: Up to 12 Mbps

**Hardware UART (Arduino):**
- Pros: Simple, reliable, instant
- Cons: Requires USB-UART converter
- Speed: Typically 115200 baud

---

## Quick Reference

### Minimal Working Code for STM32

```cpp
#include <Arduino.h>
#define INPUT_SIZE 1024
#define OUTPUT_SIZE 1024
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String msg;
ros::Publisher pub("topic", &msg);

void setup() {
  Serial.begin(115200);
  delay(1000);
  nh.initNode();
  delay(100);
  nh.advertise(pub);
}

void loop() {
  msg.data = "test";
  pub.publish(&msg);
  nh.spinOnce();
  delay(100);
}
```

### rosserial Commands

```bash
# Start rosserial node
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200

# List topics
rostopic list

# Echo topic
rostopic echo /topic_name

# Check topic info
rostopic info /topic_name

# Publish to topic
rostopic pub /topic_name std_msgs/String "data: 'test'"
```

---

## Checklist Before Running rosserial

- [ ] platformio.ini has USB CDC flags
- [ ] `#define INPUT_SIZE 1024` BEFORE `#include <ros.h>`
- [ ] `#define OUTPUT_SIZE 1024` BEFORE `#include <ros.h>`
- [ ] `Serial.begin(115200)` in setup()
- [ ] `delay(1000)` after Serial.begin()
- [ ] `nh.initNode()` called in setup()
- [ ] `nh.advertise()` called for all publishers
- [ ] `nh.spinOnce()` called in loop() every 10-50ms
- [ ] DEBUG_MODE = 0 (no debug prints on Serial)
- [ ] USB cable is data cable (not charge-only)
- [ ] /dev/ttyACM0 exists and has correct permissions
- [ ] roscore is running
- [ ] rosserial_python is installed: `sudo apt install ros-$ROS_DISTRO-rosserial-python`

---

## Still Not Working?

### Check rosserial Version Compatibility

```bash
# Check rosserial version
rosversion rosserial_python

# Should be 0.8.0 or newer
# If older, update:
sudo apt update
sudo apt install --reinstall ros-$ROS_DISTRO-rosserial-python
```

### Check Arduino Library Version

In `platformio.ini`:
```ini
lib_deps =
    frankjoshua/Rosserial Arduino Library@^0.9.1
```

Should be version **0.9.1** or newer.

### Hardware Test

```cpp
// Test if USB CDC works at all
void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  Serial.println("Hello");
  delay(1000);
}
```

Upload this, then:
```bash
screen /dev/ttyACM0 115200
# Should see "Hello" every second
```

If this doesn't work, it's a hardware/USB CDC issue, not rosserial.

---

## Contact & Support

For more help:
- ROS Answers: https://answers.ros.org/
- rosserial GitHub: https://github.com/ros-drivers/rosserial
- STM32duino forum: https://www.stm32duino.com/

Common search terms:
- "STM32 rosserial unable to sync"
- "rosserial buffer size STM32"
- "STM32 USB CDC rosserial"
