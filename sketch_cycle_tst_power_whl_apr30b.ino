// my ESP board is an 2AC7Z-ESPWROOM32
// This version working, but the power calculation may need some improvement.
// is the idea ported from 'sketch_cycle_tst_cad_whl_apr30a.ino' using the Cycle Power BLE Service
// 

#include "Arduino.h"
/* For the bluetooth funcionality */
#include <ArduinoBLE.h>
// #include <CurieBLE.h>

/* Device name which can be scene in BLE scanning software. */
#define BLE_DEVICE_NAME               "Diederiksbike"
/* Local name which should pop up when scanning for BLE devices. */
#define BLE_LOCAL_NAME                "Cycle Power BLE"

// old , to be removed
// BLEService cscService("1816");                          // cscService = CycleSpeedCadenceService
// BLECharacteristic cscChar("2A5B", BLENotify, 11); // cscChar = CycleSpeedCadenceService
// BLECharacteristic cscFtr("2A5C", BLERead, 1);         // cscFtr = CycleSpeedCadenceFeature
// BLECharacteristic cscSL("2A5D", BLERead, 1);  // cscSL = CycleSpeedCadenceSensorLocation

BLEService CyclePowerService("1818");
BLECharacteristic CyclePowerFeature("2A65", BLERead, 4);
BLECharacteristic CyclePowerMeasurement("2A63", BLERead | BLENotify, 10);
BLECharacteristic CyclePowerSensorLocation("2A5D", BLERead, 1);


unsigned char bleBuffer[10]; // 16bit flag, sint16 power , uint32 wheel_revolutions, uint16 Last Wheel Event Time (1/2048)
unsigned char slBuffer[1];
unsigned char fBuffer[4];



#define PIN_LED 2            // onboard red LED ESP32WROOM
#define PIN_WHEEL_SENSOR 13  // 3rd pin on BOOT btn side

volatile unsigned long previousMillis = 0;
volatile unsigned long lastMillis = 0;
volatile unsigned long currentMillis = 0;

volatile unsigned long time_prev_wheel = 0, time_now_wheel;
volatile unsigned long time_prev_crank = 0, time_now_crank;
volatile unsigned long time_chat = 100;

volatile unsigned int wheelRev = 0;
volatile unsigned int oldWheelRev = 0;
volatile unsigned long oldWheelMillis = 0;
volatile unsigned long lastWheeltime = 0, prevWheeltime = 0;

volatile short power = 0;

// unsigned short flags = 0x20; // bit 5 = Crank Revolution Data Present
unsigned short flags = 0x10; // bit 4 = Wheel Revolution Data Present
byte sensorlocation = 0x0D;

boolean centralConnected = false;

void setup() {
  Serial.begin(9600);       // initialize serial communication
  while(!Serial);
  Serial.println("\nSetup begins.");

  Serial.print( F("Compiled: "));
  Serial.print( F(__DATE__));
  Serial.print( F(", "));
  Serial.print( F(__TIME__));
  Serial.print( F(", "));
  Serial.println( F(__VERSION__));



  /* BLE Setup. For information, search for the many ArduinoBLE examples.*/
  if (!BLE.begin()) 
  {
    Serial.println("!BLE...");
    while (1);    
  } 
  else 
  {
    Serial.println("BLE.begin");
    BLE.setDeviceName(BLE_DEVICE_NAME);
    BLE.setLocalName(BLE_LOCAL_NAME);
    BLE.setAdvertisedService(CyclePowerService);
    CyclePowerService.addCharacteristic(CyclePowerFeature);
    CyclePowerService.addCharacteristic(CyclePowerMeasurement);
    CyclePowerService.addCharacteristic(CyclePowerSensorLocation);

    BLE.addService(CyclePowerService);
    BLE.advertise();
  }

  Serial.println("App Start");
  pinMode(PIN_LED, OUTPUT);      // initialize the LED on pin PIN_LED to indicate when a central is connected
  
  lastWheeltime = millis();

  const unsigned char bleBuffer[10] = {
    (unsigned char)flags, (unsigned char)(flags >> 8),
    (unsigned char)power, (unsigned char)(power >> 8),
    (unsigned char)wheelRev, (unsigned char)(wheelRev >> 8), (unsigned char)(wheelRev >> 16), (unsigned char)(wheelRev >> 24), 
    (unsigned char)lastWheeltime, (unsigned char)(lastWheeltime >> 8)
  };

  slBuffer[0] = sensorlocation & 0xff;

  fBuffer[0] = 0x00;
  fBuffer[1] = 0x00;
  fBuffer[2] = 0x00;
  fBuffer[3] = 0x08;

  CyclePowerFeature.writeValue(fBuffer, 4);
  CyclePowerMeasurement.writeValue(bleBuffer, 10);
  CyclePowerSensorLocation.writeValue(slBuffer, 1);

  Serial.println("Bluetooth device active, waiting for connections...");

  attachInterrupt(digitalPinToInterrupt(PIN_WHEEL_SENSOR), wheelAdd, RISING); // RISING FALLING CHANGE LOW 
}

void wheelAdd() {
  time_now_wheel = millis();

  if( time_now_wheel > time_prev_wheel + time_chat){
    wheelRev = wheelRev + 1;
    time_prev_wheel = time_now_wheel;
    prevWheeltime = lastWheeltime;
    lastWheeltime = millis();
  }

}


void loop() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(PIN_LED, HIGH);

    // check the csc mesurement every 200ms
    while (central.connected()) {
      centralConnected = true;
      currentMillis = millis();
      // if 200ms have passed, check the blood pressure mesurement:
      if (oldWheelRev < wheelRev && currentMillis - oldWheelMillis >= 500) {
        updateCSC("wheel");
      }
      else if (currentMillis - previousMillis >= 1000) {
        updateCSC("timer");
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(PIN_LED, LOW);
    centralConnected = false;
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

float current_speed() {
  // wheel_rev_time = (lastWheeltime - prevWheeltime) > (millis() - lastWheeltime) : (lastWheeltime - prevWheeltime) ? (millis() - lastWheeltime);
  unsigned long wheel_rev_time;
  float current_speed = 0;
  wheel_rev_time = (lastWheeltime - prevWheeltime) > (millis() - lastWheeltime) ? (lastWheeltime - prevWheeltime) : (millis() - lastWheeltime);

  if (wheel_rev_time == 0) {
    return current_speed;
  }

  // wheel size 2150mm
  current_speed = (2150 / wheel_rev_time) * 3.6;

  return current_speed;
}

float current_power() {
  unsigned long wheel_rev_time;
  short current_power = 0;
  wheel_rev_time = (lastWheeltime - prevWheeltime) > (millis() - lastWheeltime) ? (lastWheeltime - prevWheeltime) : (millis() - lastWheeltime);
 
  if (wheel_rev_time == 0) {
    return current_power;
  }

  // we get 0.033 cal per wheel revelution
  // 1 cal per seconds = 4.18400 Watt
  // 1000 / wheel_rev_time * cal_per_whlrev(=30~) * to_watt(4.184)
  // return 1000 / wheel_rev_time * 120;
  // return 1000 / wheel_rev_time * 61;
  // return 1000 / wheel_rev_time * 57;
  return 1000 / wheel_rev_time * 40; // ((MyWhoosh.57: 357kJ) : (Garmin: 535kJ)) * 57 = 38 # ===> (~40)
  // ((MyWhoosh.40: 208kcal) : (Garmin: 104kcal)) * 40 = 38 # ===> (~40)
}

void updateCSC(String sType) {
  oldWheelRev = wheelRev;
  oldWheelMillis = currentMillis;
  previousMillis = currentMillis;
  
  power = current_power();

  const unsigned char bleBuffer[10] = {
    (unsigned char)flags, (unsigned char)(flags >> 8),
    (unsigned char)power, (unsigned char)(power >> 8),
    (unsigned char)wheelRev, (unsigned char)(wheelRev >> 8), (unsigned char)(wheelRev >> 16), (unsigned char)(wheelRev >> 24), 
    (unsigned char)lastWheeltime, (unsigned char)(lastWheeltime >> 8)
  };
  CyclePowerMeasurement.writeValue(bleBuffer, 10);
  

  
  Serial.print(" Wheel Rev.: ");
  Serial.print(wheelRev);
  Serial.print(" WheelTime : ");
  Serial.print(lastWheeltime);
  
  Serial.print(" Speed: ");
  Serial.print(current_speed());
  
  Serial.print(" Watt: ");
  Serial.print(power);
  
  Serial.print("  ");
  Serial.println(sType);

}
