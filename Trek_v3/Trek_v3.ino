/*
Readme:
Code version: 0.3
Code editor: Killian RENOU
Organisation: OSLOMET
Group 4 EPS

Arduino Nicla Sense ME WEB Bluetooth® Low Energy Sense dashboard demo
Hardware required: https://store.arduino.cc/nicla-sense-me
1) Upload this sketch to the Arduino Nano Bluetooth® Low Energy sense board
2) Open the following web page in the Chrome browser:
https://arduino.github.io/ArduinoAI/NiclaSenseME-dashboard/
This section describes the code layout and functions

Comments about section will follow the following format:
//###CODENAME_what the section does

This is the code layout:
//###LIB_Libraries inclusion
//###SD_Definitions
//###BLED_BLE definitions
//###SENID_DefineSensorID
//###
//###
//###
*/

//###LIB_Libraries inclusion
#include "Nicla_System.h" //Requirement for NICLA sense ME
#include "Arduino_BHY2.h" //Requirement for the bosch sensor
#include <ArduinoBLE.h> //bleutooth via web
#include "math.h" // to use maths functions
#include <SPI.h> // to use SPI connectivity
#include "SdFat.h" //to use SD card reader
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
BLECharacteristic accelerometerCharacteristic(BLE_SENSE_UUID("5001"), BLERead | BLENotify, 3 * sizeof(float));  // Array of 3x 2 Bytes, XY
BLECharacteristic gyroscopeCharacteristic(BLE_SENSE_UUID("6001"), BLERead | BLENotify, 3 * sizeof(float));    // Array of 3x 2 Bytes, XYZ
BLECharacteristic quaternionCharacteristic(BLE_SENSE_UUID("7001"), BLERead | BLENotify, 4 * sizeof(float));     // Array of 4x 2 Bytes, XYZW
BLECharacteristic rgbLedCharacteristic(BLE_SENSE_UUID("8001"), BLERead | BLEWrite, 3 * sizeof(byte)); // Array of 3 bytes, RGB
BLEFloatCharacteristic bsecCharacteristic(BLE_SENSE_UUID("9001"), BLERead);
BLEIntCharacteristic  co2Characteristic(BLE_SENSE_UUID("9002"), BLERead);
BLEUnsignedIntCharacteristic gasCharacteristic(BLE_SENSE_UUID("9003"), BLERead);
String name;// String to calculate the local and device name

//###SENID_DefineSensorID
Sensor temperature(SENSOR_ID_TEMP);
Sensor humidity(SENSOR_ID_HUM);
Sensor pressure(SENSOR_ID_BARO);
Sensor gas(SENSOR_ID_GAS);
SensorXYZ gyroscope(SENSOR_ID_GYRO);
SensorXYZ accelerometer(SENSOR_ID_LACC);
SensorQuaternion quaternion(SENSOR_ID_RV);
SensorBSEC bsec(SENSOR_ID_BSEC);

static unsigned long prevTime = 0;  // To track time between measurements
static float stepLength = 0.0;  // Step length in meters
static bool inStep = false;  // Flag to track if a step is happening
unsigned long startTime, endTime;
unsigned long currentTime;
float deltaTime;
float dir;
float vx = 0, vy=0, vz=0, dx=0, dy=0, dz=0;
float gx, gy, gz;
float Time, DeltaTime, LaccX, LaccY, LaccZ, Xorientation, Yorientation, Zorientation, Worientation, GyroX, GyroY, GyroZ, vX, vY, vZ, dX, dY, dZ, Bdx, Bdy, Bdz;

void setup() {
  Serial.begin(115200);
  Serial.println("Start");

  nicla::begin();
  nicla::leds.begin();
  nicla::leds.setColor(yellow);

  //Sensors initialization
  BHY2.begin(NICLA_STANDALONE);
  temperature.begin();
  humidity.begin();
  pressure.begin();
  gyroscope.begin();
  accelerometer.begin();
  quaternion.begin();
  bsec.begin();
  gas.begin();

  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN))
  {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");


  if (!BLE.begin()){
    Serial.println("Failed to initialized BLE!");

    while (1)
      ;
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
  service.addCharacteristic(temperatureCharacteristic);
  service.addCharacteristic(humidityCharacteristic);
  service.addCharacteristic(pressureCharacteristic);
  service.addCharacteristic(versionCharacteristic);
  service.addCharacteristic(accelerometerCharacteristic);
  service.addCharacteristic(gyroscopeCharacteristic);
  service.addCharacteristic(quaternionCharacteristic);
  service.addCharacteristic(bsecCharacteristic);
  service.addCharacteristic(co2Characteristic);
  service.addCharacteristic(gasCharacteristic);
  service.addCharacteristic(rgbLedCharacteristic);

  // Disconnect event handler
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Sensors event handlers
  temperatureCharacteristic.setEventHandler(BLERead, onTemperatureCharacteristicRead);
  humidityCharacteristic.setEventHandler(BLERead, onHumidityCharacteristicRead);
  pressureCharacteristic.setEventHandler(BLERead, onPressureCharacteristicRead);
  bsecCharacteristic.setEventHandler(BLERead, onBsecCharacteristicRead);
  co2Characteristic.setEventHandler(BLERead, onCo2CharacteristicRead);
  gasCharacteristic.setEventHandler(BLERead, onGasCharacteristicRead);

  rgbLedCharacteristic.setEventHandler(BLEWritten, onRgbLedCharacteristicWrite);

  versionCharacteristic.setValue(VERSION);

  BLE.addService(service);
  BLE.advertise();

  // add column titles to the csv file 
  myFile = SD.open("test.txt", FILE_WRITE);
  
  myFile.println("Time, DeltaTime, LaccX, LaccY, LaccY, Xorientation, Yorientation, Zorientation, Worientation, GyroX, GyroY, GyroZ, vX, vY, vZ,dX, dY, dZ, Bdx, Bdy, Bdz");
  myFile.close();
  nicla::leds.setColor(green);
}

void loop() {
  BHY2.update();
  unsigned long currentTime = millis();
  Time = currentTime;
  float deltaTime = (currentTime - prevTime) / 1000.0;  // Convert to seconds
  prevTime = currentTime;

  //###SensorPooling
  //if (gyroscopeCharacteristic.subscribed()){
    float x, y, z;
    x = gyroscope.y();
    y = gyroscope.x();
    z = gyroscope.z();
    GyroX = x;
    GyroY = y;
    GyroZ = z;
    Serial.print("gyro");
   // }

  //if (accelerometerCharacteristic.subscribed()){
    //float x, y, z;
    //Serial.println("test");
    x = - accelerometer.x() * 9.81/4096;
    y = - accelerometer.y() * 9.81/4096;
    z = accelerometer.z() * 9.81/4096;
    vx += x * deltaTime;
    vy += y * deltaTime;
    vz += z * deltaTime;
    dx += vx * deltaTime;
    dy += vy * deltaTime;
    dz += vz * deltaTime;
    LaccX = x;
    LaccY = y;
    LaccZ = z;
    vX = vx;
    vY = vy;
    vZ = vz;
    dX = dx;
    dY = dy;
    dZ = dz;
    Bdx = 0;
    Bdy = 0;
    Bdz = 0;
    Serial.print("acc");
    
    //mean reset
    if (GyroZ <= 0) {
        dir = 1;
        vx = 0;
        vy = 0;
        vz = 0;
        dx = 0;
        dy = 0;
        dz = 0;
        Serial.println("reset");
    }
  //}

  //if(quaternionCharacteristic.subscribed()){
    float w;
    x = quaternion.y();
    y = quaternion.x();
    z = - quaternion.z();
    w = quaternion.w();
    Xorientation = x;
    Yorientation = y;
    Zorientation = z;
    Worientation = w;
    Serial.print("qua");

  //}

  myFile = SD.open("test.txt", FILE_WRITE);
  myFile.print(String(Time,6));
  myFile.print(",");
  myFile.print(String(deltaTime,6));
  myFile.print(",");
  myFile.print(String(LaccX,6));
  myFile.print(",");
  myFile.print(String(LaccY,6));
  myFile.print(",");
  myFile.print(String(LaccZ,6));
  myFile.print(",");
  myFile.print(String(Xorientation,6));
  myFile.print(",");
  myFile.print(String(Yorientation,6));
  myFile.print(",");
  myFile.print(String(Zorientation,6));
  myFile.print(",");
  myFile.print(String(Worientation,6));
  myFile.print(",");
  myFile.print(String(GyroX,6));
  myFile.print(",");
  myFile.print(String(GyroY,6));
  myFile.print(",");
  myFile.print(String(GyroZ,6));
  myFile.print(",");
  myFile.print(String(vX,6));
  myFile.print(",");
  myFile.print(String(vY,6));
  myFile.print(",");
  myFile.print(String(vZ,6));
  myFile.print(",");
  myFile.print(String(dX,6));
  myFile.print(",");
  myFile.print(String(dY,6));
  myFile.print(",");
  myFile.print(String(dZ,6));
  myFile.print(",");
  myFile.print(String(Bdx,6));
  myFile.print(",");
  myFile.print(String(Bdy,6));
  myFile.print(",");
  myFile.println(String(Bdz,6));

  
  /*myFile.println(String(deltaTime, 6),",",String(x,6),",",String(y,6),",",String(z,6));
  myFile.print(String(deltaTime, 6));
  myFile.print(",");
  myFile.print(String(x,6));
  myFile.print(",");
  myFile.print(String(y,6));
  myFile.print(",");
  myFile.println(String(z,6));*/
  myFile.close();
}

void blePeripheralDisconnectHandler(BLEDevice central){
  nicla::leds.setColor(red);
}

void onTemperatureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic){
  float temperatureValue = temperature.value();
  temperatureCharacteristic.writeValue(temperatureValue);
}

void onHumidityCharacteristicRead(BLEDevice central, BLECharacteristic characteristic){
  uint8_t humidityValue = humidity.value() + 0.5f;  //since we are truncating the float type to a uint8_t, we want to round it
  humidityCharacteristic.writeValue(humidityValue);
}

void onPressureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic){
  float pressureValue = pressure.value();
  pressureCharacteristic.writeValue(pressureValue);
}

void onBsecCharacteristicRead(BLEDevice central, BLECharacteristic characteristic){
  float airQuality = float(bsec.iaq());
  bsecCharacteristic.writeValue(airQuality);
}

void onCo2CharacteristicRead(BLEDevice central, BLECharacteristic characteristic){
  uint32_t co2 = bsec.co2_eq();
  co2Characteristic.writeValue(co2);
}

void onGasCharacteristicRead(BLEDevice central, BLECharacteristic characteristic){
  unsigned int g = gas.value();
  gasCharacteristic.writeValue(g);
}

void onRgbLedCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic){
  byte r = rgbLedCharacteristic[0];
  byte g = rgbLedCharacteristic[1];
  byte b = rgbLedCharacteristic[2];

  nicla::leds.setColor(r, g, b);
}


