#include <Arduino_BHY2.h>  // Library for Nicla Sense ME sensors
#include "Nicla_System.h" 
#include <math.h>   
//#include <SD.h>            // SD card library
#include <SPI.h>           // SPI library for SD card communication
#include "SdFat.h"           // To use SD card reader
SdFat SD;
// -------------------------------------------------------------------------
// Configuration
#define SD_CS_PIN SS
SensorXYZ gyroscope(SENSOR_ID_GYRO);
SensorXYZ accelerometer(SENSOR_ID_ACC);
SensorQuaternion quaternion(SENSOR_ID_RV);
SensorOrientation orientation(SENSOR_ID_DEVICE_ORI);
SensorBSEC bsec(SENSOR_ID_BSEC);
const int chipSelect = 4; // Pin for SD card chip select
const float samplePeriod = 1.0 / 256.0; // Sampling rate in seconds
const float gravity = 9.81; // Gravity constant in m/s^2

// AHRS variables
float Kp = 1.0, Ki = 0.0; // Gain parameters
float q[4] = {1.0, 0.0, 0.0, 0.0}; // Quaternion
float integralError[3] = {0.0, 0.0, 0.0}; // Integral error

// Motion variables
float velocity[3] = {0.0, 0.0, 0.0}; // Velocity in m/s
float position[3] = {0.0, 0.0, 0.0}; // Position in meters

// -------------------------------------------------------------------------
// Function prototypes
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void computeDisplacement(float ax, float ay, float az, bool stationary);
void writeToSD(float time, float posX, float posY, float posZ);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  nicla::begin();
  nicla::leds.begin();
  nicla::leds.setColor(yellow);

  // Sensors initialization
  BHY2.begin(NICLA_STANDALONE);
  gyroscope.begin();
  accelerometer.begin();
  quaternion.begin();
  bsec.begin();
  orientation.begin();
  // Initialize Nicla Sense ME
  if (!BHY2.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU initialized!");
  if (!SD.begin(SD_CS_PIN)) {
      Serial.println("initialization failed!");
      // Handle SD initialization failure (e.g., retry or notify user)
      nicla::leds.setColor(red);
    } else {
      Serial.println("initialization done.");
    }
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    // Handle SD initialization failure (e.g., retry or notify user)
    nicla::leds.setColor(red);
  } else {
    Serial.println("initialization done.");
  }
}
void loop() {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;

  // Sample IMU data
  BHY2.update();
  float gx = gyroscope.x(); // Gyroscope readings in rad/s
  float gy = gyroscope.y();
  float gz = gyroscope.z();
  float ax = accelerometer.x(); // Accelerometer readings in g
  float ay = accelerometer.y();
  float az = accelerometer.z();

  // Normalize accelerometer data
  float accNorm = sqrt(ax * ax + ay * ay + az * az);
  if (accNorm > 0.0) {
    ax /= accNorm;
    ay /= accNorm;
    az /= accNorm;
  }

  // Update AHRS
  updateIMU(gx, gy, gz, ax, ay, az);

  // Compute displacement
  bool stationary = (accNorm < 1.05 && accNorm > 0.95); // Approx stationary threshold
  computeDisplacement(ax, ay, az, stationary);

  // Write displacement data to SD card
  writeToSD(deltaTime, position[0], position[1], position[2]);

  lastTime = currentTime;
}

// -------------------------------------------------------------------------
// AHRS algorithm
void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  // Compute feedback error
  float vx = 2.0 * (q[1] * q[3] - q[0] * q[2]);
  float vy = 2.0 * (q[0] * q[1] + q[2] * q[3]);
  float vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  float error[3] = {
    vy * az - vz * ay,
    vz * ax - vx * az,
    vx * ay - vy * ax
  };

  // Integral feedback
  integralError[0] += error[0] * Ki;
  integralError[1] += error[1] * Ki;
  integralError[2] += error[2] * Ki;

  // Apply feedback
  gx += Kp * error[0] + integralError[0];
  gy += Kp * error[1] + integralError[1];
  gz += Kp * error[2] + integralError[2];

  // Compute quaternion rate of change
  float qDot[4] = {
    -0.5f * (q[1] * gx + q[2] * gy + q[3] * gz),
     0.5f * (q[0] * gx + q[2] * gz - q[3] * gy),
     0.5f * (q[0] * gy - q[1] * gz + q[3] * gx),
     0.5f * (q[0] * gz + q[1] * gy - q[2] * gx)
  };

  // Integrate to update quaternion
  for (int i = 0; i < 4; i++) {
    q[i] += qDot[i] * samplePeriod;
  }

  // Normalize quaternion
  float norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (int i = 0; i < 4; i++) {
    q[i] /= norm;
  }
}

// -------------------------------------------------------------------------
// Compute displacement from acceleration
void computeDisplacement(float ax, float ay, float az, bool stationary) {
  static float accEarth[3];
  // Rotate accelerations to Earth frame
  accEarth[0] = (1 - 2 * (q[2] * q[2] + q[3] * q[3])) * ax;
  accEarth[1] = (1 - 2 * (q[1] * q[1] + q[3] * q[3])) * ay;
  accEarth[2] = (1 - 2 * (q[1] * q[1] + q[2] * q[2])) * az - 1.0;

  // Convert to m/s²
  accEarth[0] *= gravity;
  accEarth[1] *= gravity;
  accEarth[2] *= gravity;

  // Integrate to get velocity
  if (stationary) {
    velocity[0] = velocity[1] = velocity[2] = 0.0; // Zero velocity during stationary
  } else {
    for (int i = 0; i < 3; i++) {
      velocity[i] += accEarth[i] * samplePeriod;
    }
  }

  // Integrate to get position
  for (int i = 0; i < 3; i++) {
    position[i] += velocity[i] * samplePeriod;
  }
}

// -------------------------------------------------------------------------
// Save data to SD card
void writeToSD(float time, float posX, float posY, float posZ) {
  File dataFile = SD.open("displacement.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(time);
    dataFile.print(",");
    dataFile.print(posX);
    dataFile.print(",");
    dataFile.print(posY);
    dataFile.print(",");
    dataFile.println(posZ);
    dataFile.close();
  } else {
    Serial.println("Error opening file for writing!");
  }
}

