/**
 ******************************************************************************
 * @file    VL53L4CX_Sat_HelloWorld.ino
 * ...
 */

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <Servo.h> // Added for ESC control.

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
  #define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

// Components.
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, A1);

Servo esc1;  // Create a Servo object to control the ESC.
Servo esc2;  // Create a Servo object to control the ESC.
Servo esc3;  // Create a Servo object to control the ESC.
Servo esc4;  // Create a Servo object to control the ESC.

void setup()
{
  // Led.
  pinMode(LedPin, OUTPUT);
  SerialPort.begin(115200);
  SerialPort.println("Starting...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L4CX satellite component.
  sensor_vl53l4cx_sat.begin();

  // Switch off VL53L4CX satellite component.
  sensor_vl53l4cx_sat.VL53L4CX_Off();

  // Initialize VL53L4CX satellite component.
  sensor_vl53l4cx_sat.InitSensor(0x12);

  // Start Measurements.
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();

  // Initialize ESC on digital pin 5.
  esc1.attach(5);
  esc2.attach(1);
  esc3.attach(3);
  esc4.attach(2);
  esc1.writeMicroseconds(2000); // Minimum signal to arm the ESC (adjust if needed).
  esc2.writeMicroseconds(2000); // Minimum signal to arm the ESC (adjust if needed).
  esc3.writeMicroseconds(2000); // Minimum signal to arm the ESC (adjust if needed).
  esc4.writeMicroseconds(2000); // Minimum signal to arm the ESC (adjust if needed).

  SerialPort.println("ESC initialized. Enter a value (0-180) to control the ESC.");
}

void loop()
{
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0, j;
  char report[64];
  int status;

  do {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);

  // Led on.
  digitalWrite(LedPin, HIGH);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
    snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
    SerialPort.print(report);
    for (j = 0; j < no_of_object_found; j++) {
      if (j != 0) {
        SerialPort.print("\r\n                               ");
      }
      SerialPort.print("status=");
      SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
      SerialPort.print(", D=");
      SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
      SerialPort.print("mm");
      SerialPort.print(", Signal=");
      SerialPort.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0);
      SerialPort.print(" Mcps, Ambient=");
      SerialPort.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
      SerialPort.print(" Mcps");
    }
    SerialPort.println("");
    if (status == 0) {
      status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }

  digitalWrite(LedPin, LOW);

  // ESC Control.
  if (SerialPort.available()) {
    String input = SerialPort.readStringUntil('\n');  // Read serial input.
    int value = input.toInt();  // Convert to integer.

    // Constrain the value to be between 0 and 180.
    value = constrain(value, 0, 180);

    // Map the input to a PWM range (1000-2000 microseconds).
    int pwmSignal = map(value, 0, 180, 1000, 2000);

    // Send the PWM signal to the ESC.
    esc1.writeMicroseconds(pwmSignal);
    esc2.writeMicroseconds(pwmSignal);
    esc3.writeMicroseconds(pwmSignal);
    esc4.writeMicroseconds(pwmSignal);

    // Provide feedback.
    SerialPort.print("ESC set to: ");
    SerialPort.println(value);
  }
}

