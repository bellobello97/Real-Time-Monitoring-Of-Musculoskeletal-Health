/*
Readme:
Code version: 0.3
Code editor: Killian RENOU
Organisation: OSLOMET
Group 4 EPS

Arduino Nicla Sense ME WEB Bluetooth? Low Energy Sense dashboard demo
Hardware required: https://store.arduino.cc/nicla-sense-me
1) Upload this sketch to the Arduino Nano Bluetooth? Low Energy sense board
2) Open the following web page in the Chrome browser:
   https://arduino.github.io/ArduinoAI/NiclaSenseME-dashboard/

This section describes the code layout and functions

Comments about sections will follow the following format:
//###CODENAME_what the section does

This is the code layout:
//###LIB_Libraries inclusion
//###SD_Definitions
//###BLED_BLE definitions
//###SENID_DefineSensorID
//###
*/

//###LIB_Libraries inclusion
#include "Nicla_System.h"    // Requirement for NICLA Sense ME
#include "Arduino_BHY2.h"    // Requirement for the Bosch sensor
#include <ArduinoBLE.h>      // Bluetooth via web
#include <math.h>            // To use math functions
#include <SPI.h>             // To use SPI connectivity
#include "SdFat.h"           // To use SD card reader
#include <Madgwick.h>    // Madgwick filter for sensor fusion
SdFat SD;

//###SD_Definitions
#define SD_CS_PIN SS
File myFile;

//###BLED_BLE definitions
#define BLE_SENSE_UUID(val) ("19b10000-" val "-537e-4f6c-d104768a1214")
const int VERSION = 0x00000000;
BLEService service(BLE_SENSE_UUID("0000"));
BLEUnsignedIntCharacteristic versionCharacteristic(BLE_SENSE_UUID("1001"), BLERead);
BLEFloatCharacteristic temperatureCharacteristic(BLE_SENSE_UUID("2001"), BLERead);
BLEUnsignedIntCharacteristic humidityCharacteristic(BLE_SENSE_UUID("3001"), BLERead);
BLEFloatCharacteristic pressureCharacteristic(BLE_SENSE_UUID("4001"), BLERead);
BLECharacteristic accelerometerCharacteristic(BLE_SENSE_UUID("5001"), BLERead | BLENotify, sizeof(float) * 3);  // Array of 3x float, XYZ
BLECharacteristic gyroscopeCharacteristic(BLE_SENSE_UUID("6001"), BLERead | BLENotify, sizeof(float) * 3);     // Array of 3x float, XYZ
BLECharacteristic quaternionCharacteristic(BLE_SENSE_UUID("7001"), BLERead | BLENotify, sizeof(float) * 4);    // Array of 4x float, XYZW
BLECharacteristic rgbLedCharacteristic(BLE_SENSE_UUID("8001"), BLERead | BLEWrite, sizeof(byte) * 3);          // Array of 3 bytes, RGB
BLEFloatCharacteristic bsecCharacteristic(BLE_SENSE_UUID("9001"), BLERead);
BLEIntCharacteristic co2Characteristic(BLE_SENSE_UUID("9002"), BLERead);
BLEUnsignedIntCharacteristic gasCharacteristic(BLE_SENSE_UUID("9003"), BLERead);
String name;  // String to calculate the local and device name

//###SENID_DefineSensorID
Sensor temperature(SENSOR_ID_TEMP);
Sensor humidity(SENSOR_ID_HUM);
Sensor pressure(SENSOR_ID_BARO);
Sensor gas(SENSOR_ID_GAS);
SensorXYZ gyroscope(SENSOR_ID_GYRO);
SensorXYZ accelerometer(SENSOR_ID_LACC);
SensorQuaternion quaternion(SENSOR_ID_RV);
SensorOrientation orientation(SENSOR_ID_DEVICE_ORI);
SensorBSEC bsec(SENSOR_ID_BSEC);

//###VariableDeclarations
Madgwick filter;                 // Madgwick filter instance for sensor fusion
static unsigned long prevTime = 0;  // To track time between measurements
bool strt = false;                   // Initialization flag
unsigned long startTime, endTime;
unsigned long currentTime;
float deltaTime;
bool dir = false;                    // Direction flag
float vx = 0, vy = 0, vz = 0;       // Velocity components
float gx, gy, gz;
float Time, DeltaTime, LaccX, LaccY, LaccZ;
float Xorientation, Yorientation, Zorientation, Worientation;
float GyroX, GyroY, GyroZ;
float vX, vY, vZ, dX, dY, dZ;
float Bdx, Bdy, Bdz, printBdx, printBdy, printBdz;
float GxO = 0.0, GyO = 0.0, GzO = 0.0, Gx = 0.0, Gy = 0.0, Gz = 0.0;
float VxO = 0.0, VyO = 0.0, VzO = 0.0;

// Simple low-pass filter coefficients
const float alphaFilter = 0.5; // Adjust between 0 (no change) and 1 (full change)
float x_filtered = 0.0, y_filtered = 0.0, z_filtered = 0.0;

void setup() {
  Serial.begin(115200);
  Serial.println("Start");

  nicla::begin();
  nicla::leds.begin();
  nicla::leds.setColor(yellow);

  // Sensors initialization
  BHY2.begin(NICLA_STANDALONE);
  temperature.begin();
  humidity.begin();
  pressure.begin();
  gyroscope.begin();
  accelerometer.begin();
  quaternion.begin();
  bsec.begin();
  gas.begin();
  orientation.begin();

  filter.begin(64); // Initialize Madgwick filter with sample rate of 64 Hz

  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    // Handle SD initialization failure (e.g., retry or notify user)
    nicla::leds.setColor(red);
  } else {
    Serial.println("initialization done.");
  }

  // Add column titles to the CSV file
  const char* filename = "sensor_data.txt";
  myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {
    myFile.println("Time, DeltaTime, LaccX, LaccY, LaccZ, Xorientation, Yorientation, Zorientation, Worientation, GyroX, GyroY, GyroZ, vX, vY, vZ, dX, dY, dZ, Bdx, Bdy, Bdz");
    myFile.close();
  } else {
    Serial.println("Error opening sensor_data.txt for writing.");
    nicla::leds.setColor(red);
  }
  nicla::leds.setColor(green);
}

void loop() {
  BHY2.update();  // Update sensor readings

  currentTime = millis();
  Time = currentTime;

  // Handle millis() overflow and calculate deltaTime
  deltaTime = ((unsigned long)(currentTime - prevTime)) / 1000.0;  // Convert to seconds
  prevTime = currentTime;

  //--- Sensor Data Acquisition ---
  // Read gyroscope data
  GyroX = gyroscope.y();
  GyroY = gyroscope.x();
  GyroZ = gyroscope.z();

  // Read accelerometer data
  float ax = accelerometer.x();
  float ay = accelerometer.y();
  float az = accelerometer.z();

  // Convert gyroscope data from degrees per second to radians per second
  float gx = GyroX * DEG_TO_RAD;
  float gy = GyroY * DEG_TO_RAD;
  float gz = GyroZ * DEG_TO_RAD;

  // Update Madgwick filter with accelerometer and gyroscope data
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // Retrieve orientation as quaternion
  Xorientation = filter.q1;
  Yorientation = filter.q2;
  Zorientation = filter.q3;
  Worientation = filter.q0;

  // Store linear acceleration values
  LaccX = ax;
  LaccY = ay;
  LaccZ = az;

  //--- Reset Logic ---
  // Reset variables when GyroZ is less than or equal to 0 and direction flag is false
  if (GyroZ <= 0.0 && !dir) {
    const char* filename = "sensor_data.txt";
    myFile = SD.open(filename, FILE_WRITE);
    if (myFile) {
      char buffer[50];

      // Time
      sprintf(buffer, "%10.6f", Time);
      myFile.print(buffer);
      myFile.print(",");

      // DeltaTime
      sprintf(buffer, "%10.6f", deltaTime);
      myFile.print(buffer);
      myFile.print(",");

      // LaccX, LaccY, LaccZ
      sprintf(buffer, "%10.6f", LaccX);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", LaccY);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", LaccZ);
      myFile.print(buffer);
      myFile.print(",");

      // Xorientation, Yorientation, Zorientation, Worientation
      sprintf(buffer, "%10.6f", Xorientation);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", Yorientation);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", Zorientation);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", Worientation);
      myFile.print(buffer);
      myFile.print(",");

      // GyroX, GyroY, GyroZ
      sprintf(buffer, "%10.6f", GyroX);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", GyroY);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", GyroZ);
      myFile.print(buffer);
      myFile.print(",");

      // vX, vY, vZ
      sprintf(buffer, "%10.6f", vX);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", vY);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", vZ);
      myFile.print(buffer);
      myFile.print(",");

      // dX, dY, dZ
      sprintf(buffer, "%10.6f", dX);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", dY);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", dZ);
      myFile.print(buffer);
      myFile.print(",");

      // Bdx, Bdy, Bdz
      sprintf(buffer, "%10.6f", printBdx);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", printBdy);
      myFile.print(buffer);
      myFile.print(",");
      sprintf(buffer, "%10.6f", printBdz);
      myFile.print(buffer);
      myFile.println();

      myFile.close(); // Ensure the file is closed

      // Reset variables
      vX = vY = vZ = 0.0;
      dX = dY = dZ = 0.0;
      Bdx = Bdy = Bdz = 0.0;
      printBdx = printBdy = printBdz = 0.0;
      dir = true;  // Update direction flag

      Serial.println("reset");
    } else {
      Serial.println("Error opening sensor_data.txt for writing.");
    }
  }

  // Update direction flag when GyroZ > 0
  if (GyroZ > 0.0) {
    dir = false;
  }

  //--- BLE Notifications ---
  // Update BLE characteristics with latest sensor data
  temperatureCharacteristic.writeValue(temperature.value());
  humidityCharacteristic.writeValue((uint8_t)(humidity.value() + 0.5f)); // Round the float to uint8_t
  pressureCharacteristic.writeValue(pressure.value());
  bsecCharacteristic.writeValue(float(bsec.iaq()));
  co2Characteristic.writeValue(bsec.co2_eq());
  gasCharacteristic.writeValue(gas.value());

  // Prepare accelerometer data
  float accelerometerData[3] = {LaccX, LaccY, LaccZ};
  accelerometerCharacteristic.writeValue(accelerometerData, sizeof(accelerometerData));

  // Prepare gyroscope data
  float gyroscopeData[3] = {GyroX, GyroY, GyroZ};
  gyroscopeCharacteristic.writeValue(gyroscopeData, sizeof(gyroscopeData));

  // Prepare quaternion data
  float quaternionData[4] = {Xorientation, Yorientation, Zorientation, Worientation};
  quaternionCharacteristic.writeValue(quaternionData, sizeof(quaternionData));

  // Prepare RGB LED data
  byte rgbData[3] = {255, 255, 255}; // White color
  rgbLedCharacteristic.writeValue(rgbData, sizeof(rgbData));
}

// Event handler for BLE disconnection
void blePeripheralDisconnectHandler(BLEDevice central) {
  nicla::leds.setColor(red);
}

// Event handlers for BLE characteristics
void onTemperatureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float temperatureValue = temperature.value();
  temperatureCharacteristic.writeValue(temperatureValue);
}

void onHumidityCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  uint8_t humidityValue = (uint8_t)(humidity.value() + 0.5f);  // Round the float to uint8_t
  humidityCharacteristic.writeValue(humidityValue);
}

void onPressureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float pressureValue = pressure.value();
  pressureCharacteristic.writeValue(pressureValue);
}

void onBsecCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  float airQuality = float(bsec.iaq());
  bsecCharacteristic.writeValue(airQuality);
}

void onCo2CharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  uint32_t co2 = bsec.co2_eq();
  co2Characteristic.writeValue(co2);
}

void onGasCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  unsigned int g = gas.value();
  gasCharacteristic.writeValue(g);
}

void onRgbLedCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic) {
  byte r = rgbLedCharacteristic[0];
  byte g = rgbLedCharacteristic[1];
  byte b = rgbLedCharacteristic[2];

  nicla::leds.setColor(r, g, b);
}
