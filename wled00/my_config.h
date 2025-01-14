#pragma once

/*
 * Welcome!
 * You can use the file "my_config.h" to make changes to the way WLED is compiled!
 * It is possible to enable and disable certain features as well as set defaults for some runtime changeable settings.
 *
 * How to use:
 * PlatformIO: Just compile the unmodified code once! The file "my_config.h" will be generated automatically and now you can make your changes.
 *
 * ArduinoIDE: Make a copy of this file and name it "my_config.h". Go to wled.h and uncomment "#define WLED_USE_MY_CONFIG" in the top of the file.
 *
 * DO NOT make changes to the "my_config_sample.h" file directly! Your changes will not be applied.
 */

// uncomment to force the compiler to show a warning to confirm that this file is included
//#warning **** my_config.h: Settings from this file are honored ****

/* Uncomment to use your WIFI settings as defaults
  //WARNING: this will hardcode these as the default even after a factory reset
#define CLIENT_SSID "Your_SSID"
#define CLIENT_PASS "Your_Password"
*/

//#define MAX_LEDS 1500       // Maximum total LEDs. More than 1500 might create a low memory situation on ESP8266.
//#define MDNS_NAME "wled"    // mDNS hostname, ie: *.local

#define SERVERNAME "stairs"
#define CLIENT_SSID "wifi"
#define CLIENT_PASS ""
#define WLED_AP_SSID "wled_stairs"
#define WLED_AP_PASS ""
#define MDNS_NAME SERVERNAME

#define I2CSDAPIN 4
#define I2CSCLPIN 5

#define DEFAULT_LED_COUNT 100
#define DEFAULT_LED_COLOR_ORDER COL_ORDER_RGB
#define ABL_MILLIAMPS_DEFAULT 0
