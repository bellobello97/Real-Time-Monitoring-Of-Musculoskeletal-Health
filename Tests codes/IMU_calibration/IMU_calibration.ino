#include <Arduino.h>
#include <Arduino_BHY2.h>
#include <LibPrintf.h>

#define FREQUENCY_HZ    (100)
#define INTERVAL_MS     (10000 / FREQUENCY_HZ)
#define G_TO_MS2        (9.81)  // Conversion factor from g to m/s²

// IMU sensors (accelerometer and gyroscope)
SensorXYZ accel(SENSOR_ID_ACC);

// Calibration offsets and scaling factors (to be measured and adjusted)
float accX_offset = 0.0, accY_offset = 0.0, accZ_offset = 0.0;
float accX_scale = 1.0, accY_scale = 1.0, accZ_scale = 1.0;  // Scaling factors

// Setup function
void setup() {
  Serial.begin(115200);
  while (!Serial);

  BHY2.begin(NICLA_I2C);
  accel.begin();

  // Calibrate the sensor (ensure it is flat and still)
  calibrateSensor();
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

// Main loop
void loop() {
  // Update function should be continuously polled
  BHY2.update();

  // Get current acceleration data, apply the calibration offsets and scaling
  float accX = ((accel.x() / 1000.0 - accX_offset) * accX_scale) * G_TO_MS2;  // Convert to m/s²
  float accY = ((accel.y() / 1000.0 - accY_offset) * accY_scale) * G_TO_MS2;  // Convert to m/s²
  float accZ = (((accel.z() / 1000.0)- accZ_offset) * accZ_scale) * G_TO_MS2;  // Convert to m/s²

  // Print calibrated accelerometer values in m/s²
  Serial.print("accX: ");
  Serial.print(accX);
  Serial.print(" m/s², accY: ");
  Serial.print(accY);
  Serial.print(" m/s², accZ: ");
  Serial.print(accZ);
  Serial.println(" m/s²");

  // Optional delay to control printing frequency
  delay(INTERVAL_MS);
}
