//###LIB_Libraries inclusion
#include "Nicla_System.h"    // Requirement for NICLA Sense ME
#include "Arduino_BHY2.h"    // Requirement for the Bosch sensor
#include <ArduinoBLE.h>      // Bluetooth via web
#include <math.h>            // To use math functions
#include <SPI.h>             // To use SPI connectivity
#include "SdFat.h"           // To use SD card reader
#include"CircularBuffer.hpp"  // Circular buffer for FIFO storage
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
static unsigned long prevTime = 0;  // To track time between measurements
bool strt = false;                   // Initialization flag
unsigned long currentTime;
float deltaTime;
bool dir = false;                    // Direction flag
float LaccX, LaccY, LaccZ;
float Xorientation, Yorientation, Zorientation, Worientation;
float GyroX, GyroY, GyroZ;
float vX, vY, vZ, dX, dY, dZ;

// Define FIFO buffer size
#define BUFFER_SIZE 50
CircularBuffer<String, BUFFER_SIZE> dataBuffer;  // Create FIFO buffer

// Function to save data to the SD card
void saveDataToSD() {
  const char* filename = "sensor_data.txt";
  myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {
    while (!dataBuffer.isEmpty()) {
      myFile.println(dataBuffer.shift());
    }
    myFile.close();
    Serial.println("Data saved to SD card.");
  } else {
    Serial.println("Error opening sensor_data.txt for writing.");
  }
}

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

  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    nicla::leds.setColor(red);
  } else {
    Serial.println("initialization done.");
  }

  // Add column titles to the CSV file
  const char* filename = "sensor_data.txt";
  myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {
    myFile.println("Time, DeltaTime, LaccX, LaccY, LaccZ, Xorientation, Yorientation, Zorientation, Worientation, GyroX, GyroY, GyroZ, vX, vY, vZ, dX, dY, dZ");
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
  float Time = currentTime;

  // Handle millis() overflow and calculate deltaTime
  deltaTime = ((unsigned long)(currentTime - prevTime)) / 1000.0;  // Convert to seconds
  prevTime = currentTime;

  //--- Sensor Data Acquisition ---
  GyroX = gyroscope.y();
  GyroY = gyroscope.x();
  GyroZ = gyroscope.z();

  float ax = accelerometer.x();
  float ay = accelerometer.y();
  float az = accelerometer.z();

  float gx = GyroX * DEG_TO_RAD;
  float gy = GyroY * DEG_TO_RAD;
  float gz = GyroZ * DEG_TO_RAD;


  Xorientation = 0;
  Yorientation = 0;
  Zorientation = 0;
  Worientation = 0;

  LaccX = ax;
  LaccY = ay;
  LaccZ = az;

  char buffer[150];
  sprintf(buffer,
          "%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f",
          Time, deltaTime, LaccX, LaccY, LaccZ, Xorientation, Yorientation, Zorientation, Worientation,
          GyroX, GyroY, GyroZ, vX, vY, vZ, dX, dY, dZ);

  if (!dataBuffer.isFull()) {
    dataBuffer.push(buffer);
  } else {
    Serial.println("Buffer overflow! Data loss.");
  }

  if (GyroZ <= 0.0 && !dir) {
    saveDataToSD();
    dataBuffer.clear();
    dir = true;
    Serial.println("Step complete. Data saved.");
  }

  if (GyroZ > 0.0) {
    dir = false;
  }
}

