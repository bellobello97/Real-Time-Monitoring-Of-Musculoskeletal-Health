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
SensorXYZ accelerometer(SENSOR_ID_ACC);
SensorXYZ magnetometer(SENSOR_ID_MAG);
SensorQuaternion quaternion(SENSOR_ID_RV);
SensorOrientation orientation(SENSOR_ID_DEVICE_ORI);
SensorBSEC bsec(SENSOR_ID_BSEC);

//###VariableDeclarations
static unsigned long prevTime = 0;  // To track time between measurements
bool strt = false;                   // Initialization flag
unsigned long startTime, endTime;
unsigned long currentTime;
float deltaTime;
float dir = 0;                    // Direction flag
float vx = 0, vy = 0, vz = 0;       // Velocity components
float gx, gy, gz;
float Time, DeltaTime, LaccX, LaccY, LaccZ;
float Xorientation, Yorientation, Zorientation, Worientation;
float GyroX, GyroY, GyroZ;
float Magx, Magy, Magz;
float vX, vY, vZ, dX, dY, dZ;
float Bdx, Bdy, Bdz, printBdx, printBdy, printBdz;
float pitch, roll, yaw, pmean, rmean, ymean;
float GxO = 0.0, GyO = 0.0, GzO = 0.0, Gx = 0.0, Gy = 0.0, Gz = 0.0;
float VxO = 0.0, VyO = 0.0, VzO = 0.0;
  int Pnum = 0;
float ScaleFactor = 4096.0;
// Simple low-pass filter coefficients
const float alphaFilter = 1; // Adjust between 0 (no change) and 1 (full change)
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
  magnetometer.begin();


  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    // Handle SD initialization failure (e.g., retry or notify user)
    nicla::leds.setColor(red);
  } else {
    Serial.println("initialization done.");
  }

  /*if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    // Instead of halting, attempt to reinitialize or set a flag
    nicla::leds.setColor(red);
    // Optionally, implement a retry mechanism here
  }

  String address = BLE.address();

  Serial.print("address = ");
  Serial.println(address);

  address.toUpperCase();

  name = "NiclaSenseME-";
  name += address[address.length() - 5];
  name += address[address.length() - 4];
  name += address[address.length() - 2];
  name += address[address.length() - 1];

  Serial.print("name = ");
  Serial.println(name);

  BLE.setLocalName(name.c_str());
  BLE.setDeviceName(name.c_str());
  BLE.setAdvertisedService(service);

  // Add all the previously defined Characteristics
  if (!service.addCharacteristic(temperatureCharacteristic)) {
    Serial.println("Failed to add temperatureCharacteristic!");
  }
  if (!service.addCharacteristic(humidityCharacteristic)) {
    Serial.println("Failed to add humidityCharacteristic!");
  }
  if (!service.addCharacteristic(pressureCharacteristic)) {
    Serial.println("Failed to add pressureCharacteristic!");
  }
  if (!service.addCharacteristic(versionCharacteristic)) {
    Serial.println("Failed to add versionCharacteristic!");
  }
  if (!service.addCharacteristic(accelerometerCharacteristic)) {
    Serial.println("Failed to add accelerometerCharacteristic!");
  }
  if (!service.addCharacteristic(gyroscopeCharacteristic)) {
    Serial.println("Failed to add gyroscopeCharacteristic!");
  }
  if (!service.addCharacteristic(quaternionCharacteristic)) {
    Serial.println("Failed to add quaternionCharacteristic!");
  }
  if (!service.addCharacteristic(bsecCharacteristic)) {
    Serial.println("Failed to add bsecCharacteristic!");
  }
  if (!service.addCharacteristic(co2Characteristic)) {
    Serial.println("Failed to add co2Characteristic!");
  }
  if (!service.addCharacteristic(gasCharacteristic)) {
    Serial.println("Failed to add gasCharacteristic!");
  }
  if (!service.addCharacteristic(rgbLedCharacteristic)) {
    Serial.println("Failed to add rgbLedCharacteristic!");
  }*/

  // Disconnect event handler
  /*BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Sensors event handlers
  temperatureCharacteristic.setEventHandler(BLERead, onTemperatureCharacteristicRead);
  humidityCharacteristic.setEventHandler(BLERead, onHumidityCharacteristicRead);
  pressureCharacteristic.setEventHandler(BLERead, onPressureCharacteristicRead);
  bsecCharacteristic.setEventHandler(BLERead, onBsecCharacteristicRead);
  co2Characteristic.setEventHandler(BLERead, onCo2CharacteristicRead);
  gasCharacteristic.setEventHandler(BLERead, onGasCharacteristicRead);

  rgbLedCharacteristic.setEventHandler(BLEWritten, onRgbLedCharacteristicWrite);

  versionCharacteristic.setValue(VERSION);*/

 /* if (!BLE.addService(service)) {
    Serial.println("Failed to add BLE service!");
    // Handle BLE service addition failure
  }

  BLE.advertise();*/

  // Add column titles to the CSV file
  const char* filename = "sensor_data.txt";

 /* myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {
    myFile.println("Packet number,Gyroscope X (deg/s),Gyroscope Y (deg/s),Gyroscope Z (deg/s),Accelerometer X (g),Accelerometer Y (g),Accelerometer Z (g),Magnetometer X (G),Magnetometer Y (G),Magnetometer Z (G)");
    //myFile.close();
  } else {
    Serial.println("Error opening sensor_data.txt for writing.");
    nicla::leds.setColor(red);
  }
  nicla::leds.setColor(green);
}*/
  Serial.print("Pnum");
  Serial.print(",");  // Separate fields with commas
  Serial.print("GyroX");  // Print with 6 decimal places
  Serial.print(",");
  Serial.print("GyroY");
  Serial.print(",");
  Serial.print("GyroZ");
  Serial.print(",");
  Serial.print("x");
  Serial.print(",");
  Serial.print("y");
  Serial.print(",");
  Serial.print("z");
  Serial.print(",");
  Serial.print("Magx");
  Serial.print(",");
  Serial.print("Magy");
  Serial.print(",");
  Serial.println("Magz");}

void loop() {
  // Delay for initialization if not started yet
  /*if (!strt) {
    delay(2000);
    return; // Prevent further execution until initialization is complete
  }*/

  BHY2.update();  // Update sensor readings

  currentTime = millis();
  Time = currentTime;

  // Handle millis() overflow and calculate deltaTime
  deltaTime = ((unsigned long)(currentTime - prevTime)) / 1000.0;  // Convert to seconds
  prevTime = currentTime;

  //--- Sensor Data Acquisition ---
  // Read gyroscope data
  GyroX = gyroscope.x()/ ScaleFactor;
  GyroY = gyroscope.y()/ScaleFactor;
  GyroZ = gyroscope.z()/ScaleFactor;

//--- Read Magnetometer Data ---
  // Read magnetometer data
  Magx = magnetometer.x()/ScaleFactor;
  Magy = magnetometer.y()/ScaleFactor;
  Magz = magnetometer.z()/ScaleFactor;

  nicla::leds.setColor(blue);


  // Read linear acceleration data and apply low-pass filter
  float x = accelerometer.x() * 1 / ScaleFactor;
  float y = accelerometer.y() * 1 / ScaleFactor;
  float z = accelerometer.z() * 1 / ScaleFactor;
  


  //--- Reset Logic ---
  // Reset variables when GyroZ is less than or equal to 0 and direction flag is false
  //if (GyroZ <= -40.0 && dir == 0) {
  if ( true) {
    const char* filename = "sensor_data.txt";
    //myFile = SD.open(filename, FILE_WRITE);
    /*if (myFile) {
      // Write data to SD card
            // Use sprintf to format data into buffer
        char buffer[30];
        // Format and write each data field to the file
        sprintf(buffer, "%d,", Pnum); // Assuming packetNumber is an integer
        myFile.print(buffer);
        Pnum += 1;
        // Gyroscope data
        sprintf(buffer, "%10.6f,%10.6f,%10.6f,", GyroX, GyroY, GyroZ);
        myFile.print(buffer);

        // Accelerometer data
        sprintf(buffer, "%10.6f,%10.6f,%10.6f,", x, y, z);
        myFile.print(buffer);

        // Magnetometer data
        sprintf(buffer, "%10.6f,%10.6f,%10.6f", Magx, Magy, Magz); // Assuming Magnetometer variables are MagX, MagY, MagZ
        myFile.println(buffer);

        //myFile.close(); // Ensure the file is closed

    */
  
  Serial.print(Pnum);
  Serial.print(",");  // Separate fields with commas
  Serial.print(GyroX, 6);  // Print with 6 decimal places
  Serial.print(",");
  Serial.print(GyroY, 6);
  Serial.print(",");
  Serial.print(GyroZ, 6);
  Serial.print(",");
  Serial.print(x, 6);
  Serial.print(",");
  Serial.print(y, 6);
  Serial.print(",");
  Serial.print(z, 6);
  Serial.print(",");
  Serial.print(Magx, 6);
  Serial.print(",");
  Serial.print(Magy, 6);
  Serial.print(",");
  Serial.println(Magz, 6);  // Use println to end the line
  Pnum += 1;
    } else {
      //Serial.println("Error opening sensor_data.txt for writing.");
    }
  
  /*Serial.print (GyroZ);
  Serial.print ("dir: ");
  Serial.println (dir);*/
  // Update direction flag when GyroZ > 0


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
  // Assuming you have RGB values stored somewhere, otherwise this needs to be defined
  // For demonstration, using placeholder values
  byte rgbData[3] = {255, 255, 255}; // White color
  rgbLedCharacteristic.writeValue(rgbData, sizeof(rgbData));

  // Add Bdx, Bdy, Bdz, pitch, roll, yaw to BLE characteristics if needed
  // This requires defining additional BLE characteristics or encoding them into existing ones

  // Example: Could define new characteristics or pack into existing ones as needed
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

