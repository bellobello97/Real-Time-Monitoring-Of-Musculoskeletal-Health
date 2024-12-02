/*
Readme:
Code version: 1.3
Code editor: Killian RENOU
Organisation: OSLOMET
Group 4 EPS

This section describes the code layout and functions

Comments about sections will follow the following format:
//###CODENAME_what the section does

This is the code layout:
//###LIB_Libraries inclusion
//###SD_Definitions
//###SENID_DefineSensorID
//###VARINIT_Variables initialisation
//###FILTPARA_ Filtering parameters
//###SETUP_setup loop
//###WBUF_Write to buffer function
//###FBUF_Flush buffer function
//###MAINL_Main loop
*/

//###LIB_Libraries inclusion
#include "Nicla_System.h"    // Requirement for NICLA Sense ME
#include "Arduino_BHY2.h"    // Requirement for the Bosch sensor
#include <ArduinoBLE.h>      // Bluetooth via web
#include <math.h>            // To use math functions
#include <SPI.h>             // To use SPI connectivity
#include "SdFat.h"           // High-performance SD library
SdFat SD;

//###SD_Definitions
#define SD_CS_PIN SS
#define BUFFER_SIZE 16384    // Buffer size for SD writes
#define WRITE_INTERVAL 6000   // File flush interval in milliseconds

//###SENID_DefineSensorID
SensorXYZ gyroscope(SENSOR_ID_GYRO);
SensorXYZ accelerometer(SENSOR_ID_LACC);
SensorQuaternion quaternion(SENSOR_ID_RV);

//###VARINIT_Variables initialisation
static unsigned long prevTime = 0;  // To track time between measurements
unsigned long lastFlushTime = 0;   // Last time data was flushed
char dataBuffer[BUFFER_SIZE];
int bufferIndex = 0;
File myFile;

float Time, DeltaTime;
float LaccX, LaccY, LaccZ;
float Xorientation, Yorientation, Zorientation, Worientation;
float GyroX, GyroY, GyroZ;
float vX = 0, vY = 0, vZ = 0;      // Velocity components
float dX = 0, dY = 0, dZ = 0;      // Displacement components
float Gx = 0.0, Gy = 0.0, Gz = 0.0;
float GxO = 0.0, GyO = 0.0, GzO = 0.0;
float x_filtered = 0.0, y_filtered = 0.0, z_filtered = 0.0;

//###FILTPARA_ Filtering parameters
const float accelerationThreshold = 0.5; //threshold for setting velocity and displacment to 0 when the device is still
const float alphaFilter = 0.95;     // Low-pass filter coefficient

//###SETUP_setup loop
void setup() {
  Serial.begin(115200);
  Serial.println("Start");

  nicla::begin();
  nicla::leds.begin();
  nicla::leds.setColor(yellow);

  // Sensor Initialization
  BHY2.begin(NICLA_STANDALONE);
  gyroscope.begin();
  accelerometer.begin();
  quaternion.begin();

  // SD Card Initialization
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN, SD_SCK_MHZ(50))) { // Use high-speed SPI
    Serial.println("SD initialization failed!");
    nicla::leds.setColor(red);
    while (true); // Stop execution
  }
  Serial.println("SD initialization done.");

  // Open file and write header
  myFile = SD.open("sensor_data.txt", FILE_WRITE);
  if (myFile) {
    //myFile.println("Time, DeltaTime, LaccX, LaccY, LaccZ, Xorientation, Yorientation, Zorientation, Worientation, GyroX, GyroY, GyroZ, vX, vY, vZ, dX, dY, dZ");
    myFile.println("Time, DeltaTime, dX, dY, dZ");
    myFile.close();
  } else {
    Serial.println("Error opening sensor_data.txt for writing.");
    nicla::leds.setColor(red);
  }

  nicla::leds.setColor(green);
}

//###WBUF_Write to buffer function
void writeToBuffer(const char* data) {
  int len = strlen(data);
  if (bufferIndex + len < BUFFER_SIZE) {
    strcpy(&dataBuffer[bufferIndex], data);
    bufferIndex += len;
  } else {
    flushBuffer();          // Flush when buffer is full
    writeToBuffer(data);    // Add current data after flushing
  }
}

//###FBUF_Flush buffer function
void flushBuffer() {
  if (bufferIndex > 0) {
    myFile = SD.open("sensor_data.txt", FILE_WRITE);
    if (myFile) {
      nicla::leds.setColor(blue);
      myFile.write(dataBuffer, bufferIndex);
      myFile.close();
      bufferIndex = 0;      // Reset buffer
      
    } else {
      Serial.println("Error writing to SD card.");
      nicla::leds.setColor(red);
    }
  }
}

//###MAINL_Main loop
void loop() {
  BHY2.update();            // Update sensor readings
  nicla::leds.setColor(green);
  // Calculate timing
  unsigned long currentTime = millis();
  Time = currentTime / 1000.0; // Time in seconds
  DeltaTime = (currentTime - prevTime) / 1000.0; // Delta time in seconds
  prevTime = currentTime;

  //--- Sensor Data Acquisition ---
  GyroX = gyroscope.y();
  GyroY = gyroscope.x();
  GyroZ = gyroscope.z();

  // Quaternion normalization
  float qw = quaternion.w(), qx = quaternion.x(), qy = quaternion.y(), qz = quaternion.z();
  float norm = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (norm > 0) {
    qw /= norm; qx /= norm; qy /= norm; qz /= norm;
  }
  Xorientation = qx; Yorientation = qy; Zorientation = qz; Worientation = qw;

  // Linear acceleration
  float x = accelerometer.x() * 9.81 / 4096.0;
  float y = accelerometer.y() * 9.81 / 4096.0;
  float z = accelerometer.z() * 9.81 / 4096.0;

  // Apply low-pass filter
  x_filtered = alphaFilter * x + (1 - alphaFilter) * x_filtered;
  y_filtered = alphaFilter * y + (1 - alphaFilter) * y_filtered;
  z_filtered = alphaFilter * z + (1 - alphaFilter) * z_filtered;
  
  float accelerationMagnitude = sqrt(x_filtered * x_filtered + y_filtered * y_filtered + z_filtered * z_filtered);

  //--- Quaternion-Based Rotation ---
  float q_conjugate_x = -qx, q_conjugate_y = -qy, q_conjugate_z = -qz, q_conjugate_w = qw;
  float ax = x_filtered, ay = y_filtered, az = z_filtered;

  // First quaternion multiplication
  float qv0 = qw * 0 - qx * ax - qy * ay - qz * az;
  float qv1 = qw * ax + qy * az - qz * ay;
  float qv2 = qw * ay + qz * ax - qx * az;
  float qv3 = qw * az + qx * ay - qy * ax;

  // Second quaternion multiplication
  Gx = qv0 * q_conjugate_x + qv1 * q_conjugate_w + qv2 * q_conjugate_z - qv3 * q_conjugate_y;
  Gy = qv0 * q_conjugate_y - qv1 * q_conjugate_z + qv2 * q_conjugate_w + qv3 * q_conjugate_x;
  Gz = qv0 * q_conjugate_z + qv1 * q_conjugate_y - qv2 * q_conjugate_x + qv3 * q_conjugate_w;

  // Update velocities using trapezoidal integration
  vX += ((GxO + Gx) / 2.0) * DeltaTime;
  vY += ((GyO + Gy) / 2.0) * DeltaTime;
  vZ += ((GzO + Gz) / 2.0) * DeltaTime;

  // Update displacements
  dX += vX * DeltaTime;
  dY += vY * DeltaTime;
  dZ += vZ * DeltaTime;

  // Prepare data for logging
  char dataLine[50];
  /*sprintf(dataLine, "%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.1f,%10.1f,%10.1f,%10.1f,%10.1f,%10.1f,%10.1f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f\n",
          Time, DeltaTime, x_filtered, y_filtered, z_filtered, Xorientation, Yorientation, Zorientation, Worientation,
          GyroX, GyroY, GyroZ, vX, vY, vZ, dX, dY, dZ);
  writeToBuffer(dataLine);*/
  sprintf(dataLine, "%10.6f,%10.6f,%10.6f,%10.6f,%10.6f\n",
          Time, DeltaTime, dX, dY, dZ);
  writeToBuffer(dataLine);

  // Flush buffer periodically
  if (currentTime - lastFlushTime > WRITE_INTERVAL) {
    flushBuffer();
    lastFlushTime = currentTime;
  }

  //Set integrated variable to 0 when the device is still (see the parameter in >FILTPARA)
  if (accelerationMagnitude < accelerationThreshold) {
    // Reset velocities and displacements
    vX = vY = vZ = 0.0;
    dX = dY = dZ = 0.0;
  }
}
