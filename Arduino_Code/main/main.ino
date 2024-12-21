#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
  #define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

// Components.
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, A1);
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Create Servo objects for the ESCs
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

// Define pin numbers for the ESCs
const int esc1Pin = 3;
const int esc2Pin = 5;
const int esc3Pin = 6;
const int esc4Pin = 9;
/* Setup ---------------------------------------------------------------------*/

void setup()
{
  // Led.
  pinMode(LedPin, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);
  SerialPort.println("Starting...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L4CX satellite component.
  sensor_vl53l4cx_sat.begin();

  // Switch off VL53L4CX satellite component.
  sensor_vl53l4cx_sat.VL53L4CX_Off();

  //Initialize VL53L4CX satellite component.
  sensor_vl53l4cx_sat.InitSensor(0x12);

  // Start Measurements
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();

    /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

}

void loop()
{
  Serial.println("");
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0, j;
  char report[64];
  int status;

  do {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);

  // Led on
  digitalWrite(LedPin, HIGH);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    Serial.println(status);
    no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
    snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
    SerialPort.print(report);
    for (j = 0; j < no_of_object_found; j++) {
      if (j != 0) {
        SerialPort.print("\r\n                               ");
      }
      SerialPort.print(", D=");
      SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
      SerialPort.print("mm");
    }
    SerialPort.println("");
    if (status == 0) {
      status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }

  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // read serial data and put into motor commands
  if (SerialPort.available()) {
    String input = SerialPort.readStringUntil('\n');  // Read serial input.

    //split up serial message into 4 commands
    int commands[4] = {}; //make an empty array of size 4 all set to zeros

    stringstream ss(input);
    string pwm_cmd;
    char del = '/';
    count = 0;

    while (!ss.eof()) {
        getline(ss, pwm_cmd, del);
        int value = input.toInt();  // convert to integer 
        value = constrain(value, 0, 180); //constrain integer
        value = map(value, 0, 180, 1000, 1600); //map integer
        commands[count] = value;
        count++;
    }

    esc1.writeMicroseconds(commands[0]);
    esc2.writeMicroseconds(commands[1]);
    esc3.writeMicroseconds(commands[2]);
    esc4.writeMicroseconds(commands[3]);

    // Provide feedback.
    SerialPort.print("ESC set to: ");
    SerialPort.println(value);
  }

  /* Display the quat data */
  Serial.print(" qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);

  Serial.print(" Roll Rate: ");
  Serial.print(gyroscope.x());
  Serial.print(" Pitch Rate: ");
  Serial.print(gyroscope.y());
  Serial.print(" Yaw Rate: ");
  Serial.print(gyroscope.z());
  digitalWrite(LedPin, LOW);
}