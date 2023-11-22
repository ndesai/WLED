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

#include "wled.h"

#include <Wire.h>
#include <VL53L0X.h>

#ifndef VL53L0X_MAX_RANGE_MM
#define VL53L0X_MAX_RANGE_MM 230 // max height in millimeters to react for motions
#endif

#ifndef VL53L0X_MIN_RANGE_OFFSET
#define VL53L0X_MIN_RANGE_OFFSET 60 // minimal range in millimeters that sensor can detect. Used in long motions to correct brightness calculation.
#endif

#ifndef VL53L0X_DELAY_MS
#define VL53L0X_DELAY_MS 100 // how often to get data from sensor
#endif

#ifndef VL53L0X_LONG_MOTION_DELAY_MS
#define VL53L0X_LONG_MOTION_DELAY_MS 1000 // switch onto "long motion" action after this delay
#endif

class UsermodVL53L0XGestures : public Usermod {
  private:
    //Private class members. You can declare variables and functions only accessible to your usermod here
    unsigned long lastTime = 0;
    VL53L0X sensor;

    //
    VL53L0X sensorBottomOfStairs;
    VL53L0X sensorTopOfStairs;

    static const uint8_t sensorCount = 2;
    VL53L0X sensors[sensorCount] = { sensorBottomOfStairs, sensorTopOfStairs };

    // The Arduino pin connected to the XSHUT pin of each sensor.
    const uint8_t xshutPins[sensorCount] = { D5, D6 };
    //

    bool enabled = true;

    bool wasMotionBefore = false;
    bool isLongMotion = false;
    unsigned long motionStartTime = 0;

  public:

    void setup() {
      if (i2c_scl < 0 || i2c_sda < 0) {
        enabled = false;
        return;
      }

      sensor.setTimeout(150);
      if (!sensor.init())
      {
        DEBUG_PRINTLN(F("Failed to detect and initialize VL53L0X sensor!"));
      } else {
        sensor.setMeasurementTimingBudget(20000); // set high speed mode
      }

      //
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
          DEBUG_PRINTLN(F("Failed to detect and initialize sensor "));
          DEBUG_PRINTLN(i);
          while (1);
        }

        // Each sensor must have its address changed to a unique value other than
        // the default of 0x29 (except for the last one, which could be left at
        // the default). To make it simple, we'll just count up from 0x2A.
        sensors[i].setAddress(0x2A + i);

        sensors[i].startContinuous();
      }
    }


    void loop() {
      if (!enabled || strip.isUpdating()) return;

      //
      for (uint8_t i = 0; i < sensorCount; i++)
      {
        DEBUG_PRINTF("range: %d", sensors[i].readRangeSingleMillimeters());
        if (sensors[i].timeoutOccurred()) {
          DEBUG_PRINTLN(" TIMEOUT");
        }
      }
      //


      if (millis() - lastTime > VL53L0X_DELAY_MS)
      {
        lastTime = millis();

        if (sensor.last_status == 4) {
          DEBUG_PRINTF("out of range");
          return;
        }

        int range = sensor.readRangeSingleMillimeters();
        DEBUG_PRINTF("range: %d, brightness: %d\r\n", range, bri);

        if (range < VL53L0X_MAX_RANGE_MM)
        {
          if (!wasMotionBefore)
          {
            motionStartTime = millis();
            DEBUG_PRINTF("motionStartTime: %d\r\n", motionStartTime);
          }
          wasMotionBefore = true;

          if (millis() - motionStartTime > VL53L0X_LONG_MOTION_DELAY_MS) //long motion
          {
            DEBUG_PRINTF("long motion: %d\r\n", motionStartTime);
            if (!isLongMotion)
            {
              isLongMotion = true;
            }

            // set brightness according to range
            bri = (VL53L0X_MAX_RANGE_MM - max(range, VL53L0X_MIN_RANGE_OFFSET)) * 255 / (VL53L0X_MAX_RANGE_MM - VL53L0X_MIN_RANGE_OFFSET);
            DEBUG_PRINTF("new brightness: %d", bri);
            stateUpdated(1);
          }
        } else if (wasMotionBefore) { //released
          if (!isLongMotion)
          { //short press
            DEBUG_PRINTLN(F("shortPressAction..."));
            shortPressAction();
          }
          wasMotionBefore = false;
          isLongMotion = false;
        }
      }
    }

    /*
     * addToConfig() can be used to add custom persistent settings to the cfg.json file in the "um" (usermod) object.
     * It will be called by WLED when settings are actually saved (for example, LED settings are saved)
     * I highly recommend checking out the basics of ArduinoJson serialization and deserialization in order to use custom settings!
     */
    void addToConfig(JsonObject& root)
    {
      JsonObject top = root.createNestedObject("VL53L0x");
      JsonArray pins = top.createNestedArray("pin");
      pins.add(i2c_scl);
      pins.add(i2c_sda);
    }

    /*
     * getId() allows you to optionally give your V2 usermod an unique ID (please define it in const.h!).
     * This could be used in the future for the system to determine whether your usermod is installed.
     */
    uint16_t getId()
    {
      return USERMOD_ID_VL53L0X;
    }
};