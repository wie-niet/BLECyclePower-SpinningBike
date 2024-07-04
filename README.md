# BLECyclePowerStupidSpinBike
ESP32 code to create an BLE Cycle Power sensor from a stupid spin bike with a wheel sensor. using fake/approximate power values.


my ESP board is an 2AC7Z-ESPWROOM32
This version working, but the power calculation may need some improvement.

This code creates a (virtual) BLE Power Sensor from a simple wheel sensor.
By using an fixed amount of energy per wheel revolution we calculate the Power in watts and broadcast them over BLE.
feel free to improve the calculations, this was good enough for me. ;-)

This program will put the ESP32 into deep sleep mode when the wheel sensor is idle over 180 seconds (SLEEP_AFTER_MILLIS).
sensing any change on the wheel sensor will wake-up the ESP32 from deep sleep.

