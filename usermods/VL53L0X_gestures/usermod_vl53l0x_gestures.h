/*
 * That usermod implements support of simple hand gestures with VL53L0X sensor: on/off and brightness correction.
 * It can be useful for kitchen strips to avoid any touches.
 * - on/off - just swipe a hand below your sensor ("shortPressAction" is called and can be customized through WLED macros)
 * - brightness correction - keep your hand below sensor for 1 second to switch to "brightness" mode.
        Configure brightness by changing distance to the sensor (see parameters below for customization).
 *
 * Enabling this usermod:
 * 1. Attach VL53L0X sensor to i2c pins according to default pins for your board.
 * 2. Add `-D USERMOD_VL53L0X_GESTURES` to your build flags at platformio.ini (plaformio_override.ini) for needed environment.
 * In my case, for example: `build_flags = ${env.build_flags} -D USERMOD_VL53L0X_GESTURES`
 * 3. Add "pololu/VL53L0X" dependency below to `lib_deps` like this:
 * lib_deps = ${env.lib_deps}
 *     pololu/VL53L0X @ ^1.3.0
 */
#pragma once

#include <string>
#include <sstream>
#include <Wire.h>
#include <wled.h>
#include <FX.h>
// Pololu VL53L0X Time-of-Flight (ToF) sensor driver
// NOTE: Adafruit VL53L0X driver not used as it consumes more RAM than available on a D1 Mini
#include <VL53L0X.h>

#define BREAKBEAM_LOGIC_ENABLED

// Break sensor
// Sensor Alpha
// Sensor Beta
// Alpha triggers --> Keep on until Beta triggers OR 25 minute timeout

#ifndef VL53L0X_MAX_RANGE_MM
#define VL53L0X_MAX_RANGE_MM 230 // max height in millimeters to react for motions
#endif

#ifndef VL53L0X_MIN_RANGE_OFFSET
#define VL53L0X_MIN_RANGE_OFFSET 30 // minimal range in millimeters that sensor can detect. Used in long motions to correct brightness calculation.
#endif

#ifndef VL53L0X_DELAY_MS
#define VL53L0X_DELAY_MS 200 // how often to get data from sensor
#endif

#ifndef VL53L0X_LONG_MOTION_DELAY_MS
#define VL53L0X_LONG_MOTION_DELAY_MS 1000 // switch onto "long motion" action after this delay
#endif

#define SAMPLING_FREQUENCY_MS 100

#define LOG(x) \
  DEBUG_PRINTF("UM: %s\r\n", x)

class UsermodVL53L0XGestures : public Usermod
{
private:
  // Private class members. You can declare variables and functions only accessible to your usermod here
  bool enabled = true;

  unsigned long lastTime = 0;

  VL53L0X sensor1;
  VL53L0X sensor2;

  static const uint8_t sensorCount = 1;
  VL53L0X sensors[sensorCount] = {sensor1 };

  // The pin connected to the XSHUT pin of each sensor.
  // const uint8_t xshutPins[sensorCount] = {D5, D6};
  const uint8_t xshutPins[sensorCount] = {D6};

#ifndef BREAKBEAM_LOGIC_ENABLED
  bool wasMotionBefore = false;
  bool isLongMotion = false;
  unsigned long motionStartTime = 0;
#endif

  // True when user is between sensor1 and sensor2
  bool userWithinSensorRegion;
  unsigned long userEnteredSensorRegionTime = 0;
  uint8_t userCount = 0;

public:
  // strings to reduce flash memory usage (used more than twice)
  static const char _name[];
  static const char _enabled[];

  void setup() override;
  void loop() override;

  /*
   * Disable usermod
   */
  void disableUsermod(const char* errorMessage)
  {
    DEBUG_PRINTF("UM: Disabling usermod: %s\r\n", errorMessage);
    enabled = false;
  }

  /*
   * addToConfig() can be used to add custom persistent settings to the cfg.json file in the "um" (usermod) object.
   * It will be called by WLED when settings are actually saved (for example, LED settings are saved)
   * I highly recommend checking out the basics of ArduinoJson serialization and deserialization in order to use custom settings!
   */
  void addToConfig(JsonObject &root) override;

  /**
   * readFromConfig() is called before setup() to populate properties from values stored in cfg.json
   *
   * The function should return true if configuration was successfully loaded or false if there was no configuration.
   */
  bool readFromConfig(JsonObject &root) override;

  /*
   * getId() allows you to optionally give your V2 usermod an unique ID (please define it in const.h!).
   * This could be used in the future for the system to determine whether your usermod is installed.
   */
  uint16_t getId() override { return USERMOD_ID_VL53L0X; }

  /*
   * Use ntp time and prescribed sunrise/sunset timings to determine whether
   * it is day time.
   */
  static bool isDayTime();
};

void UsermodVL53L0XGestures::setup()
{
  // i2c pin configuration not set, cannot enable usermod
  if (i2c_scl < 0 || i2c_sda < 0)
  {
    disableUsermod("i2c clock/data not set, disabling usermod");
    return;
  }

  // Initialize break beam logic
  userWithinSensorRegion = false;

  // Setup multiple devices on the same i2c bus by assigning each
  // device a unique address.

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      // std::ostringstream stream;
      // stream << "Failed to detect and initialize sensor";
      // stream << i;
      // disableUsermod(stream.str().c_str());
      // return;
      DEBUG_PRINTLN(F("Failed to detect and initialize sensor "));
      while (1);
    }

    DEBUG_PRINTF("UM: sensor %d initialized\r\n", i);

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x2A + i);
    DEBUG_PRINTF("UM: sensor %d address: %d\r\n", i, sensors[i].getAddress());
    sensors[i].setMeasurementTimingBudget(SAMPLING_FREQUENCY_MS * 1000);
    sensors[i].startContinuous(50);
  }
}

void UsermodVL53L0XGestures::loop()
{
  if (!enabled || strip.isUpdating())
  {
    // Skip loop if usermod is disabled or LED strip is unavailable
    return;
  }

  if (millis() - lastTime > VL53L0X_DELAY_MS)
  {
    lastTime = millis();

    LOG("loop");

#ifdef BREAKBEAM_LOGIC_ENABLED
    int range1 = sensor1.readRangeSingleMillimeters();
    // int range2 = sensor2.readRangeSingleMillimeters();
    // int range1 = sensor1.readRangeContinuousMillimeters();
    // int range2 = sensor2.readRangeContinuousMillimeters();
    int range2 = 666;

    DEBUG_PRINTF("range1: %d ---- range2: %d\r\n", range1, range2);

    // User can walk from "Outside" to "Region", thereby breaking
    // the beam.  The user can walk in either direction.

    // Outside -----------------------------
    // Beam    +---------------------------+
    // Region  +++++++++++++++++++++++++=+++
    // Region  +++++++++++++++++++++++++=+++
    // Region  +++++++++++++++++++++++++=+++
    // Region  +++++++++++++++++++++++++=+++
    // Region  +++++++++++++++++++++++++=+++
    // Beam    +---------------------------+
    // Outside -----------------------------

    // TODO: ND - Handle the case where multiple users can be walking
    // in the same direction
    // TODO: ND - Handle the case where multiple users can be walking
    // in the opposite direction
    // TODO: Keep track of how many users are within the region
    // TODO: Keep track of users who backtrack, e.g. break one beam and regress through the same beam

    if (range1 < VL53L0X_MAX_RANGE_MM)
    {
      // User is passing sensor1
      // Either entering the sensor region or exiting it
      if (userWithinSensorRegion)
      {
        if (userCount == 1) {
          userWithinSensorRegion = false;
        }
        userCount--;
      }
      else
      {
        userWithinSensorRegion = true;
        userEnteredSensorRegionTime = millis();
        userCount++;
      }
    }

    if (range2 < VL53L0X_MAX_RANGE_MM)
    {
      // User is passing sensor2
      // Either entering the sensor region or exiting it
      if (userWithinSensorRegion)
      {
        if (userCount == 1) {
          userWithinSensorRegion = false;
        }
        userCount--;
      }
      else
      {
        userWithinSensorRegion = true;
        userEnteredSensorRegionTime = millis();
        userCount++;
      }
    }

    // Timeout a userWithinSensorRegion after 5 minutes
    if (millis() > (userEnteredSensorRegionTime + 5 * (60 * 1000)))
    {
      // Timed out
      userWithinSensorRegion = false;
    }

    if (userWithinSensorRegion)
    {
      bri = (uint8_t) 1.0 * 255;
    }
    else
    {
      if (isDayTime())
      {
         bri = 0;
      }
      else
      {
        // 10% brightness
        bri = (uint8_t) 0.10 * 255;
      }
    }
    stateUpdated(1);
    DEBUG_PRINTF("UM: brightness updated (userWithinSensorRegion = %d): %d\r\n", userWithinSensorRegion, bri);

#else
    for (uint8_t i = 0; i < sensorCount; i++)
    {
      int range = sensors[i].readRangeSingleMillimeters();
      DEBUG_PRINTF("range (%d): %d\r\n", i, range);
      if (i == 0)
      {
        if (range > VL53L0X_MAX_RANGE_MM)
        {
          bri = 0;
        }
        else
        {
          bri = (VL53L0X_MAX_RANGE_MM - max(range, VL53L0X_MIN_RANGE_OFFSET)) * 255 / (VL53L0X_MAX_RANGE_MM - VL53L0X_MIN_RANGE_OFFSET);
        }
        DEBUG_PRINTF("brightness updated (%d): %d\r\n", i, bri);
        stateUpdated(1);
      }
      if (sensors[i].timeoutOccurred())
      {
        DEBUG_PRINTLN(" TIMEOUT");
      }
    }
#endif
  }
}

void UsermodVL53L0XGestures::addToConfig(JsonObject &root)
{
  JsonObject top = root.createNestedObject(FPSTR(_name));
  top[FPSTR(_enabled)] = enabled;

  JsonArray pins = top.createNestedArray("pin");
  pins.add(i2c_scl);
  pins.add(i2c_sda);
}

bool UsermodVL53L0XGestures::readFromConfig(JsonObject &root)
{
  JsonObject top = root[FPSTR(_name)];
  if (top.isNull())
  {
    LOG("No config found. (Using defaults)");
    return false;
  }
  enabled = top[FPSTR(_enabled)] | enabled;
  return true;
}

// https://github.com/Aircoookie/WLED/blob/main/usermods/PIR_sensor_switch/usermod_PIR_sensor_switch.h#L200
bool UsermodVL53L0XGestures::isDayTime()
{
  updateLocalTime();
  uint8_t hr = hour(localTime);
  uint8_t mi = minute(localTime);

  if (sunrise && sunset)
  {
    if (hour(sunrise) < hr && hour(sunset) > hr)
    {
      return true;
    }
    else
    {
      if (hour(sunrise) == hr && minute(sunrise) < mi)
      {
        return true;
      }
      if (hour(sunset) == hr && minute(sunset) > mi)
      {
        return true;
      }
    }
  }
  return false;
}

// https://github.com/Aircoookie/WLED/blob/main/usermods/Temperature/usermod_temperature.h
const char UsermodVL53L0XGestures::_name[] PROGMEM = "Breakbeam dual VL53L0x";
const char UsermodVL53L0XGestures::_enabled[] PROGMEM = "enabled";
