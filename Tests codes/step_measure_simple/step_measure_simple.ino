#include <Arduino.h>
#include <Arduino_BHY2.h>
#include <LibPrintf.h>

#define FREQUENCY_HZ    (100)
#define INTERVAL_MS     (1000 / FREQUENCY_HZ)
#define G_TO_MS2        (9.81)  // Conversion factor from g to m/s²

// IMU sensors (accelerometer and gyroscope)
SensorXYZ accel(SENSOR_ID_ACC);

// Time tracking variables
static unsigned long prevTime = 0;  // To track time between measurements
static float stepLength = 0.0;  // Step length in meters
static bool inStep = false;  // Flag to track if a step is happening

// Velocity variables for each axis in m/s
static float velocityX = 0.0, velocityY = 0.0, velocityZ = 0.0;

// Setup function
void setup() {
  Serial.begin(115200);
  while (!Serial);

  BHY2.begin(NICLA_I2C);
  accel.begin();

  prevTime = millis();
}

// Main loop
void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;  // Convert to seconds
  prevTime = currentTime;

  // Update function should be continuously polled
  BHY2.update();

  // Get current acceleration data in g, then convert to m/s²
  float accX = accel.x() / 1000.0 * G_TO_MS2;  // Convert to m/s²
  float accY = accel.y() / 1000.0 * G_TO_MS2;  // Convert to m/s²
  float accZ = accel.z() / 1000.0 * G_TO_MS2;  // Convert to m/s²

  // Print accelerometer values in m/s² continuously
  Serial.print("accX: ");
  Serial.print(accX);
  Serial.print(" m/s², accY: ");
  Serial.print(accY);
  Serial.print(" m/s², accZ: ");
  Serial.print(accZ);
  Serial.println(" m/s²");

  // Simple velocity integration from acceleration (in m/s)
  velocityX += accX * deltaTime;
  velocityY += accY * deltaTime;
  velocityZ += accZ * deltaTime;

  // Detect step start (e.g., toe-off) using Z-axis acceleration threshold
  if (!inStep && accZ > 1.5 * G_TO_MS2) {  // Adjusted threshold for meters
    inStep = true;  // Step is starting
    stepLength = 0.0;  // Reset step length for the new step
  }

  // Accumulate step length during the step (distance = integrated velocity)
  if (inStep) {
    float stepIncrement = sqrt(velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ) * deltaTime;
    stepLength += stepIncrement;  // Add to step length
  }

  // Detect step end (e.g., heel-strike) using Z-axis acceleration threshold
  if (inStep && accZ < 0.5 * G_TO_MS2) {  // Adjusted threshold for meters
    inStep = false;  // Step is finished

    // Print the final calculated step length in meters
    Serial.print("Step Length: ");
    Serial.print(stepLength);
    Serial.println(" meters");

    // Reset velocities for the next step
    velocityX = velocityY = velocityZ = 0;
  }
}
