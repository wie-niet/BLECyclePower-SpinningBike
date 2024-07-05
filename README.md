# BLE Cycle Power for any Stupid Spinning Bike
ESP32 code to create an BLE Cycle Power sensor from a stupid spin bike with a wheel sensor, calculating fake/approximate power values.

This sketch creates a (virtual) BLE Power Sensor from a simple wheel sensor.
By using an fixed amount of energy per wheel revolution we calculate the Power in watts and broadcast it over BLE.
Feel free to improve the calculations, this was working good enough for me. ;-)

![Breadboard schema ESP32:BLE Cycle Power with wheel sensor](doc/schema_bb.png)

### Hardware
I've tested it on an ESP32 board (2AC7Z-ESPWROOM32)

### Power Calculations
This version working, but the power calculation may need some improvement.

### Deep sleep / power save mode:
This program will put the ESP32 into deep sleep mode when the wheel sensor is idle over 180 seconds (SLEEP_AFTER_MILLIS).
Sensing any change on the wheel sensor will wake-up the ESP32 from deep sleep.

### Config:
Change these values to what you need and you're good to go:
```
/* Device name which can be seen in BLE scanning software. */
#define BLE_DEVICE_NAME               "Diederik's bike"
/* Local name which should pop up when scanning for BLE devices. */
#define BLE_LOCAL_NAME                "Cycle Power BLE"

#define PIN_LED GPIO_NUM_2            // BLE pairing LED indicator; GPIO_NUM_2 is an onboard LED on my ESP32WROOM.
#define PIN_WHEEL_SENSOR GPIO_NUM_13  // RTC + INPUT; I am using GPIO_NUM_13, 3rd pin on BOOT btn side.
#define SLEEP_AFTER_MILLIS 180 * 1000 // deep sleep timer in milliseconds. Using 3 minutes.
```
