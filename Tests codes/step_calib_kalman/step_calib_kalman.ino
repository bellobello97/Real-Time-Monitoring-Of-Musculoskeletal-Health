#include <Arduino.h>
#include <Arduino_BHY2.h>
#include <LibPrintf.h>
#include <Kalman.h>  // Assuming you are using a Kalman filter library

#define FREQUENCY_HZ    (100)
#define INTERVAL_MS     (10000 / FREQUENCY_HZ)
#define G_TO_MS2        (9.81)  // Conversion factor from g to m/s²

// IMU sensors (accelerometer and gyroscope)
SensorXYZ accel(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);

// Kalman filter objects for each axis (pitch and roll)
Kalman kalmanX;  
Kalman kalmanY;  

// Calibration offsets and scaling factors (adjust as needed)
float gyroX_raw = 0.0, gyroY_raw = 0.0, gyroZ_raw = 0.0;  // Raw data
float accX_offset = 0.0, accY_offset = 0.0, accZ_offset = 0.0;  // Offsets
float gyroX_offset = 0.0, gyroY_offset = 0.0, gyroZ_offset = 0.0;
float accX_scale = 1.0, accY_scale = 1.0, accZ_scale = 1.0;  // Scaling factors
float gyroX_scale = 1.0, gyroY_scale = 1.0, gyroZ_scale = 1.0;  // Scaling factors

// Time tracking variables
unsigned long prevTime = 0;  // To track time between measurements
float deltaTime = 0.0;

// Velocity variables for each axis (in m/s)
float velocityX = 0.0, velocityY = 0.0, velocityZ = 0.0;

// Variables to store angles (in radians)
float pitch = 0.0, roll = 0.0;

// Setup function
void setup() {
  Serial.begin(115200);
  while (!Serial);

  BHY2.begin(NICLA_I2C);
  accel.begin();
  gyro.begin();

  // Calibrate the sensor (ensure it is flat and still)
  calibrateSensor();

  // Calibrate the gyroscope offsets
  Serial.println("Hold the sensor still for gyro offset calibration");
  delay(5000);
  calibrateGyroscope();

  // Calibrate scaling factors for each axis of the gyroscope
  Serial.read();
  calibrateGyroScaling('X');
  delay(2000);
  Serial.read();
  calibrateGyroScaling('Y');
  delay(2000);
  Serial.read();
  calibrateGyroScaling('Z');
  delay(2000);
}

// Calibration function to determine offsets and scaling factors
void calibrateSensor() {
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  int samples = 1000;  // Number of samples to average

  // Collect data for offset calibration (sensor in position accX==accY==0)
  Serial.println("Calibrating sensor. Keep the sensor flat and still in position accX==accY==0...");

  for (int i = 0; i < samples; i++) {
    BHY2.update();
    sumX += accel.x() / 1000.0;
    sumY += accel.y() / 1000.0;
    //sumZ += accel.z() / 1000.0;
    delay(10);  // Small delay between readings
  }

  // Calculate average offsets
  accX_offset = sumX / samples;   // X-axis should read 1g (9.81 m/s²) when perpendicular to flat
  accY_offset = sumY / samples;
  //accZ_offset = (sumZ / samples); //- 1.0;  // Z-axis should read 1g (9.81 m/s²) when flat

  Serial.println("Offsets calculated:");
  Serial.print("accX_offset: ");
  Serial.println(accX_offset);
  Serial.print("accY_offset: ");
  Serial.println(accY_offset);
  //Serial.print("accZ_offset: ");
  //Serial.println(accZ_offset);

  // Collect data for offset calibration (sensor in position accZ==0)
  Serial.println("Flip the sensor in position AccZ==0 and press any key to continue...");
  while (!Serial.available());
  for (int i = 0; i < samples; i++) {
    BHY2.update();
    //sumX += accel.x() / 1000.0;
    //sumY += accel.y() / 1000.0;
    sumZ += accel.z() / 1000.0;
    delay(10);  // Small delay between readings
  }

  // Calculate average offsets
  //accX_offset = sumX / samples -1 ;   // X-axis should read 1g (9.81 m/s²) when perpendicular to flat
  //accY_offset = sumY / samples;
  accZ_offset = (sumZ / samples); //- 1.0;  // Z-axis should read 1g (9.81 m/s²) when flat

  Serial.println("Offsets calculated:");
  Serial.print("accX_offset: ");
  Serial.println(accX_offset);
  Serial.print("accY_offset: ");
  Serial.println(accY_offset);
  Serial.print("accZ_offset: ");
  Serial.println(accZ_offset);

  // Now flip the sensor manually for scaling calibration of Z axis
  Serial.println("Flip the sensor in position AccZ==1");
  delay(5000);

  sumX = sumY = sumZ = 0.0;  // Reset for scaling calibration

  // Collect data for scaling calibration 
  for (int i = 0; i < samples; i++) {
    BHY2.update();
    //sumX += (accel.x()) / 1000.0;
    //sumY += accel.y() / 1000.0;
    sumZ += (accel.z()) / 1000.0;
    delay(10);  // Small delay between readings
  }

  // Calculate the scaling factor for Z-axis 
  float accZ_flipped = (sumZ - accZ_offset) / samples;

  // Apply scaling based on expected value 
  accZ_scale = 1.0 / accZ_flipped;

  Serial.println("Scaling factors calculated:");
  Serial.print("accZ_flipped value: ");
  Serial.println(accZ_flipped);
  Serial.print("accZ_scale: ");
  Serial.println(accZ_scale);

  // Now flip the sensor manually for scaling calibration of X axis
  Serial.println("Flip the sensor in position AccX==1");
  delay(5000);

  sumX = sumY = sumZ = 0.0;  // Reset for scaling calibration

  // Collect data for scaling calibration 
  for (int i = 0; i < samples; i++) {
    BHY2.update();
    sumX += (accel.x()) / 1000.0;
    //sumY += accel.y() / 1000.0;
    //sumZ += (accel.z()) / 1000.0;
    delay(10);  // Small delay between readings
  }

  // Calculate the scaling factor for X-axis 
  float accX_flipped = (sumX - accX_offset) / samples;

  // Apply scaling based on expected value 
  accX_scale = 1.0 / accX_flipped;

  Serial.println("Scaling factors calculated:");
  Serial.print("accX_flipped value: ");
  Serial.println(accX_flipped);
  Serial.print("accX_scale: ");
  Serial.println(accX_scale);
 
  // Now flip the sensor manually for scaling calibration of Y axis
  Serial.println("Flip the sensor in position AccY==-1g");
  delay(5000);

  sumX = sumY = sumZ = 0.0;  // Reset for scaling calibration

  // Collect data for scaling calibration 
  for (int i = 0; i < samples; i++) {
    BHY2.update();
    //sumX += (accel.x()) / 1000.0;
    sumY += accel.y() / 1000.0;
    //sumZ += (accel.z()) / 1000.0;
    delay(10);  // Small delay between readings
  }

  // Calculate the scaling factor for X-axis 
  float accY_flipped = (sumY - accY_offset) / samples;

  // Apply scaling based on expected value 
  accY_scale = 1.0 / accY_flipped;

  Serial.println("Scaling factors calculated:");
  Serial.print("accY_flipped value: ");
  Serial.println(accY_flipped);
  Serial.print("accY_scale: ");
  Serial.println(accY_scale);

  // print the final results
  Serial.println("Calibration done see ofset and factor below :");
  Serial.print("accX_offset: ");
  Serial.println(accX_offset);
  Serial.print("accY_offset: ");
  Serial.println(accY_offset);
  Serial.print("accZ_offset: ");
  Serial.println(accZ_offset);
  Serial.print("accX_scale: ");
  Serial.println(accX_scale);
  Serial.print("accY_scale: ");
  Serial.println(accY_scale);
  Serial.print("accZ_scale: ");
  Serial.println(accZ_scale);
}
void calibrateGyroscope() {
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  int samples = 1000;  // Number of samples for averaging

  Serial.println("Calibrating gyroscope... Keep the sensor still.");
  delay(5000);
  // Collect gyroscope data while the sensor is still
  for (int i = 0; i < samples; i++) {
    BHY2.update();  // Update the sensor readings

    // Accumulate the raw gyroscope readings
    sumX += gyro.x() / 1000.0;  // Convert to rad/s
    sumY += gyro.y() / 1000.0;
    sumZ += gyro.z() / 1000.0;

    delay(10);  // Short delay between samples
  }

  // Compute the average offsets (bias)
  gyroX_offset = sumX / samples;
  gyroY_offset = sumY / samples;
  gyroZ_offset = sumZ / samples;

  // Print the calculated offsets
  Serial.println("Gyroscope offsets calculated:");
  Serial.print("gyroX_offset: ");
  Serial.println(gyroX_offset, 6);  // Print with 6 decimal places
  Serial.print("gyroY_offset: ");
  Serial.println(gyroY_offset, 6);
  Serial.print("gyroZ_offset: ");
  Serial.println(gyroZ_offset, 6);
}
// Gyroscope scaling calibration for each axis
void calibrateGyroScaling(char axis) {
  Serial.read();
  Serial.print("Rotate the sensor by 90° around the ");
  Serial.print(axis);
  Serial.println(" axis and press any key to start...");

  while (!Serial.available());

  // Reset integrated angles
  float angleX = 0.0, angleY = 0.0, angleZ = 0.0;
  unsigned long startTime = millis();
  prevTime = startTime;

  Serial.println("Starting 2-second integration...");

  while (millis() - startTime < 2000) {
    BHY2.update();
    unsigned long currentTime = millis();
    deltaTime = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    gyroX_raw = (gyro.x() / 1000.0 - gyroX_offset);
    gyroY_raw = (gyro.y() / 1000.0 - gyroY_offset);
    gyroZ_raw = (gyro.z() / 1000.0 - gyroZ_offset);

    angleX += gyroX_raw * deltaTime;
    angleY += gyroY_raw * deltaTime;
    angleZ += gyroZ_raw * deltaTime;
  }

  switch (axis) {
    case 'X':
      gyroX_scale = radians(90.0) / angleX;
      break;
    case 'Y':
      gyroY_scale = radians(90.0) / angleY;
      break;
    case 'Z':
      gyroZ_scale = radians(90.0) / angleZ;
      break;
  }

  Serial.print("Scaling factor for ");
  Serial.print(axis);
  Serial.println(" calculated.");
}

// Complementary Kalman filter-based orientation update
void updateOrientation(float accX, float accY, float accZ, float gyroX, float gyroY, float dt) {
  // Accelerometer-based pitch and roll
  float accelPitch = atan2(accY, sqrt(accX * accX + accZ * accZ));
  float accelRoll = atan2(-accX, accZ);

  // Update Kalman filters with accelerometer and gyroscope data
  pitch = kalmanX.getAngle(accelPitch, gyroX, dt);
  roll = kalmanY.getAngle(accelRoll, gyroY, dt);
}

// Rotate the gravity vector to match the sensor's orientation and subtract it
void removeGravity(float accX, float accY, float accZ, float &linAccX, float &linAccY, float &linAccZ) {
  float gravityX = -sin(roll) * G_TO_MS2;
  float gravityY = sin(pitch) * G_TO_MS2;
  float gravityZ = cos(roll) * cos(pitch) * G_TO_MS2;

  linAccX = accX - gravityX;
  linAccY = accY - gravityY;
  linAccZ = accZ - gravityZ;
}

void loop() {
  BHY2.update();
  unsigned long currentTime = millis();
  deltaTime = (currentTime - prevTime) / 1000.0; // Time difference in seconds
  prevTime = currentTime;

  // Read accelerometer and gyroscope data
  float accX = (accel.x() / 1000.0 - accX_offset) * accX_scale;
  float accY = (accel.y() / 1000.0 - accY_offset) * accY_scale;
  float accZ = (accel.z() / 1000.0 - accZ_offset) * accZ_scale;

  float gyroX = (gyro.x() / 1000.0 - gyroX_offset) * gyroX_scale;
  float gyroY = (gyro.y() / 1000.0 - gyroY_offset) * gyroY_scale;
  float gyroZ = (gyro.z() / 1000.0 - gyroZ_offset) * gyroZ_scale;

  // Remove gravity from accelerometer readings
  float linAccX, linAccY, linAccZ;
  removeGravity(accX, accY, accZ, linAccX, linAccY, linAccZ);

  // Update orientation based on sensor data
  updateOrientation(linAccX, linAccY, linAccZ, gyroX, gyroY, deltaTime);

  // Print the results
  Serial.print("Pitch: ");
  Serial.print(pitch * RAD_TO_DEG, 2);
  Serial.print(" Roll: ");
  Serial.print(roll * RAD_TO_DEG, 2);
  Serial.println();
}
