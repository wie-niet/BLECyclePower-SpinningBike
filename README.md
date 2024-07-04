# BLECyclePowerStupidSpinBike
ESP32 code to create an BLE Cycle Power sensor from a stupid spin bike with a wheel sensor. using fake/approximate power values.


my ESP board is an 2AC7Z-ESPWROOM32
This version working, but the power calculation may need some improvement.

This code creates a (virtual) BLE Power Sensor from a simple wheel sensor.
By using an fixed amount of energy per wheel revolution we calculate the Power in watts and broadcast them over BLE.
feel free to improve the calculations, this was good enough for me. ;-)

This program will put the ESP32 into deep sleep mode when the wheel sensor is idle over 180 seconds (SLEEP_AFTER_MILLIS).
sensing any change on the wheel sensor will wake-up the ESP32 from deep sleep.


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
