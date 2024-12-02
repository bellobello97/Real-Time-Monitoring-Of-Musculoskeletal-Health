#include <Arduino.h>
#include <Arduino_BHY2.h>
#include <LibPrintf.h>

#define FREQUENCY_HZ    (100)
#define INTERVAL_MS     (10000 / FREQUENCY_HZ)
#define G_TO_MS2        (9.81)  // Conversion factor from g to m/s²
#define ALPHA           0.98    // Complementary filter constant

// IMU sensors (accelerometer and gyroscope)
SensorXYZ accel(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);

// Calibration offsets and scaling factors (to be measured and adjusted)
float gyroX_raw = 0.0, gyroY_raw = 0.0, gyroZ_raw = 0.0;//raw data
float accX_offset = 0.0, accY_offset = 0.0, accZ_offset = 0.0;//offsets
float gyroX_offset = 0.0, gyroY_offset = 0.0, gyroZ_offset = 0.0;
float accX_scale = 1.0, accY_scale = 1.0, accZ_scale = 1.0;  // Scaling factors
float gyroX_scale = 0.0, gyroY_scale = 0.0, gyroZ_scale = 0.0;

// Time tracking variables
static unsigned long prevTime = 0;  // To track time between measurements
static float stepLength = 0.0;  // Step length in meters
static bool inStep = false;  // Flag to track if a step is happening
unsigned long startTime, endTime;
unsigned long currentTime;
float deltaTime;

// Velocity variables for each axis in m/s
static float velocityX = 0.0, velocityY = 0.0, velocityZ = 0.0;

// Variables to store integrated angles (in radians)
float angleX = 0.0, angleY = 0.0, angleZ = 0.0;

// Orientation angles (pitch and roll) in radians
float pitch = 0.0, roll = 0.0;

// Known rotation in degrees and radians
float knownRotationDeg = 90.0;          // 90 degrees
float knownRotationRad = radians(90.0); // Convert to radians

// Setup function
void setup() {
  Serial.begin(115200);
  while (!Serial);

  BHY2.begin(NICLA_I2C);
  accel.begin();
  gyro.begin();

  // Calibrate the sensor (ensure it is flat and still)
  calibrateSensor();

   // Calibrate the gyroscope
  Serial.println("hold the sensor still for gyro offset calibration");
  delay(5000);
  calibrateGyroscope();
  // Calibrate scaling factor for X axis
  Serial.read();
  calibrateGyroScaling('X');
  delay(2000);  // Wait for 2 seconds before next calibration
  Serial.read();
  // Calibrate scaling factor for Y axis
  calibrateGyroScaling('Y');
  delay(2000);  // Wait for 2 seconds before next calibration
  Serial.read();
  // Calibrate scaling factor for Z axis
  calibrateGyroScaling('Z');
  delay(2000);  // End of the calibration
  Serial.read();
}

// Calibration function to determine offsets and scaling factors
// To calibrate we must first determine the offset required to calibrate the 0 on each axis and than determine the scalling factor in order to have a correct reading of g
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


// Gyroscope calibration function (bias correction)
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

// Function to calculate the scaling factor for the gyroscope on each axis
void calibrateGyroScaling(char axis) {
  Serial.print("Rotate the sensor by 90° around the ");
  Serial.print(axis);
  Serial.println(" axis and press any key to start...");

  // Wait for key press
  Serial.read();
  while (!Serial.available()) {
    // Wait until a key is pressed
  }

  // Clear the serial buffer to avoid multiple triggers from the same key press
  while (Serial.available()) {
    Serial.read();
  }

  // Reset integrated angles
  angleX = angleY = angleZ = 0.0;
  startTime = millis();
  prevTime = startTime;

  Serial.println("Starting 2-second integration...");

  // Capture raw data and integrate for a fixed 2-second window
  while (millis() - startTime < 2000) {  // 2-second window for integration
    // Update sensor values
    BHY2.update();

    // Get current time and calculate delta time in seconds
    currentTime = millis();
    deltaTime = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    // Read raw gyro data (rotation rate in units/s)
    gyroX_raw = gyro.x() / 1000.0;
    gyroY_raw = gyro.y() / 1000.0;
    gyroZ_raw = gyro.z() / 1000.0;

    // Integrate angular velocity to estimate total rotation (in radians)
    angleX += gyroX_raw * deltaTime;
    angleY += gyroY_raw * deltaTime;
    angleZ += gyroZ_raw * deltaTime;

    // Print raw values and integrated angles for debugging
    Serial.print("gyroX_raw: ");
    Serial.print(gyroX_raw);
    Serial.print(", gyroY_raw: ");
    Serial.print(gyroY_raw);
    Serial.print(", gyroZ_raw: ");
    Serial.print(gyroZ_raw);
    Serial.print(" | angleX: ");
    Serial.print(angleX);
    Serial.print(", angleY: ");
    Serial.print(angleY);
    Serial.print(", angleZ: ");
    Serial.println(angleZ);

    delay(50);  // Small delay to control sampling rate
  }
  // Calculate the scaling factor for the specific axis
  switch (axis) {
    case 'X':
      gyroX_scale = knownRotationRad / angleX;  // Scaling factor based on integrated angle
      Serial.print("Estimated scaling factor for X-axis: ");
      Serial.println(gyroX_scale, 6);  // Print scaling factor with 6 decimal places
      break;
    case 'Y':
      gyroY_scale = knownRotationRad / angleY;
      Serial.print("Estimated scaling factor for Y-axis: ");
      Serial.println(gyroY_scale, 6);  // Print scaling factor with 6 decimal places
      break;
    case 'Z':
      gyroZ_scale = knownRotationRad / angleZ;
      Serial.print("Estimated scaling factor for Z-axis: ");
      Serial.println(gyroZ_scale, 6);  // Print scaling factor with 6 decimal places
      break;
  }

  Serial.println("Calibration complete for this axis.");
}


// Complementary filter to estimate pitch and roll
void updateOrientation(float accX, float accY, float accZ, float gyroX, float gyroY, float dt) {
  // Accelerometer-based angles
  float accelPitch = atan2(accY, sqrt(accX * accX + accZ * accZ));
  float accelRoll = atan2(-accX, accZ);  // atan2 uses accZ here for roll

  // Gyroscope-based angle rate integration
  float gyroPitchRate = gyroX;  // Rotation around X-axis
  float gyroRollRate = gyroY;   // Rotation around Y-axis

  // Integrate gyroscope data to get pitch and roll (in radians)
  pitch += gyroPitchRate * dt;
  roll += gyroRollRate * dt;

  // Complementary filter: blend accelerometer and gyroscope data
  pitch = ALPHA * pitch + (1.0 - ALPHA) * accelPitch;
  roll = ALPHA * roll + (1.0 - ALPHA) * accelRoll;
}

// Rotate the gravity vector to match the sensor's orientation and subtract it
void removeGravity(float accX, float accY, float accZ, float &linAccX, float &linAccY, float &linAccZ) {
  // Gravity vector in the sensor frame
  // float gravityX = sin(pitch) * G_TO_MS2;
  // float gravityY = -sin(roll) * G_TO_MS2;
  // float gravityZ = cos(pitch) * cos(roll) * G_TO_MS2;
  float gravityX = -sin(roll) * G_TO_MS2;
  float gravityY = sin(pitch) * G_TO_MS2;
  float gravityZ = cos(roll) *cos (pitch)* G_TO_MS2;
  // Subtract gravity from accelerometer readings to get linear acceleration
  linAccX = accX - gravityX;
  linAccY = accY - gravityY;
  linAccZ = accZ - gravityZ;
}


// Main loop
void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;  // Convert to seconds
  prevTime = currentTime;
  // Update function should be continuously polled
  BHY2.update();

  // Get current acceleration data, apply the calibration offsets and scaling
  float accX = ((accel.x() / 1000.0 - accX_offset) * accX_scale) * G_TO_MS2;  // Convert to m/s²
  float accY = ((accel.y() / 1000.0 - accY_offset) * accY_scale) * G_TO_MS2;  // Convert to m/s²
  float accZ = (((accel.z() / 1000.0)- accZ_offset) * accZ_scale) * G_TO_MS2;  // Convert to m/s²

  float gyroX = (gyro.x() / 1000.0 - gyroX_offset) * gyroX_scale;  // In rad/s
  float gyroY = (gyro.y() / 1000.0 - gyroY_offset) * gyroY_scale;  // In rad/s
  float gyroZ = (gyro.z() / 1000.0 - gyroZ_offset) * gyroZ_scale;  // In rad/s

  // Print calibrated accelerometer values in m/s²
  Serial.print("accX: ");
  Serial.print(accX);
  Serial.print(" m/s², accY: ");
  Serial.print(accY);
  Serial.print(" m/s², accZ: ");
  Serial.print(accZ);
  Serial.print(" m/s² ");

  Serial.print("gyro.x: ");
  Serial.print(gyroX);
  Serial.print(" rad/s, gyro.y: ");
  Serial.print(gyroY);
  Serial.print(" rad/s, gyro.z: ");
  Serial.print(gyroZ);
  Serial.println(" rad/s");

  Serial.print("pitch: ");
  Serial.print(pitch);
  Serial.print(" rad, roll: ");
  Serial.print(roll);
  Serial.print("gravity: ");


 // Update the orientation (pitch and roll)
  updateOrientation(accX, accY, accZ, gyroX, gyroY, deltaTime);

 // Remove gravity to get the linear acceleration
  float linAccX, linAccY, linAccZ;
  removeGravity(accX, accY, accZ, linAccX, linAccY, linAccZ);

  // Print linear accelerations
  Serial.print("Linear accX: ");
  Serial.print(linAccX);
  Serial.print(" m/s², accY: ");
  Serial.print(linAccY);
  Serial.print(" m/s², accZ: ");
  Serial.print(linAccZ);
  Serial.println(" m/s²");

// Simple velocity integration from acceleration (in m/s)
//  velocityX += accX * deltaTime;
//  velocityY += accY * deltaTime;
//  velocityZ += accZ * deltaTime;

 // Simple velocity integration from linear acceleration (in m/s)
  velocityX += linAccX * deltaTime;
  velocityY += linAccY * deltaTime;
  velocityZ += linAccZ * deltaTime;

   // Detect step start using Z-axis linear acceleration threshold
  if (!inStep && linAccZ > 1.5 * G_TO_MS2) {
    inStep = true;
    stepLength = 0.0;  // Reset step length for the new step
  }

  // Accumulate step length during the step
  if (inStep) {
    float stepIncrement = sqrt(velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ) * deltaTime;
    stepLength += stepIncrement;
  }

  // Detect step end using Z-axis linear acceleration threshold
  if (inStep && linAccZ < 0.5 * G_TO_MS2) {
    inStep = false;

    // Print the final calculated step length
    Serial.print("Step Length: ");
    Serial.print(stepLength);
    Serial.println(" meters");

    // Reset velocities for the next step
    velocityX = velocityY = velocityZ = 0;
    delay(1000);
  }

  // Optional delay to control printing frequency
  delay(INTERVAL_MS);
}

