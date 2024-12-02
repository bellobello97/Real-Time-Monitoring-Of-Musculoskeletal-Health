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
SensorOrientation orientation(SENSOR_ID_DEVICE_ORI);
SensorBSEC bsec(SENSOR_ID_BSEC);

static unsigned long prevTime = 0;  // To track time between measurements
static float stepLength = 0.0;  // Step length in meters
static bool inStep = false;  // Flag to track if a step is happening
bool strt = false;
unsigned long startTime, endTime;
unsigned long currentTime;
float deltaTime;
float dir;
float vx = 0, vy=0, vz=0, dx=0, dy=0, dz=0;
float gx, gy, gz;
float Time, DeltaTime, LaccX, LaccY, LaccZ, Xorientation, Yorientation, Zorientation, Worientation, 
GyroX, GyroY, GyroZ, vX, vY, vZ, dX, dY, dZ, Bdx, Bdy, Bdz,printBdx, printBdy,printBdz, pitch, roll, 
yaw, pmean,rmean, ymean, GxO, GyO, GzO, Gx, Gy, Gz, VxO, VyO, VzO;

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
  orientation.begin();

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
  
  myFile.println("Time, DeltaTime, LaccX, LaccY, LaccY, Xorientation, Yorientation, Zorientation, Worientation, GyroX, GyroY, GyroZ, vX, vY, vZ,dX, dY, dZ, Bdx, Bdy, Bdz, pitch, roll, yaw");
  myFile.close();
  nicla::leds.setColor(green);
}

void loop() {
  if (strt = false){
    delay (2000);
  }
  BHY2.update();
  unsigned long currentTime = millis();
  Time = currentTime;
  float deltaTime = (currentTime - prevTime) / 1000.0;  // Convert to seconds
  prevTime = currentTime;
  //###SensorPooling

    float x, y, z;
    x = gyroscope.y();
    y = gyroscope.x();
    z = gyroscope.z();
    GyroX = x;
    GyroY = y;
    GyroZ = z;
    //Serial.print("gyro");

    

    float qx, qy, qz, qw, alpha, beta, gamma;
    qx = quaternion.y();
    qy = quaternion.x();
    qz =  quaternion.z();
    qw = quaternion.w();
    Xorientation = qx;
    Yorientation = qy;
    Zorientation = qz;
    Worientation = qw;
    //Serial.println("qua");


    pmean = 0;//pitch;
    rmean = 0;//roll;
    ymean = 0;//yaw;
    // Calculate heading
    yaw = atan2(2.0 * (qx * qy - qw * qz), qw * qw - qx * qx + qy * qy - qz * qz) ;

    // Calculate pitch
    pitch = asin(-2.0 * (qy * qz + qw * qx)) ;

    // Calculate roll
    roll = atan2(2.0 * (qx * qz - qw * qy), qw * qw + qx * qx - qy * qy - qz * qz) ;
    /*Serial.print("pitch: ");
    Serial.print(pitch);
    Serial.print("roll: ");
    Serial.print(roll);
    Serial.print("yaw: ");
    Serial.println(yaw);*/
    
    if (strt == false){
      pmean = pitch;
      rmean = roll;
      ymean = yaw;
      strt = true;
      nicla::leds.setColor(blue);
      }

    alpha = - pitch;
    beta =  - roll;
    gamma = - yaw;

    GxO = Gx;
    GyO = Gy;
    GzO = Gz;
    VxO = vx;
    VyO = vy;
    VzO = vz;
    x = - accelerometer.x() * 9.81/4096;
    y = - accelerometer.y() * 9.81/4096;
    z = accelerometer.z() * 9.81/4096;
    // Calculate trigonometric values once for efficiency
    float cosAlpha = cos(alpha);
    float sinAlpha = sin(alpha);
    float cosBeta = cos(beta);
    float sinBeta = sin(beta);
    float cosGamma = cos(gamma);
    float sinGamma = sin(gamma);
    /*Gx = x * cosBeta * cosGamma + 
       y * (-cosAlpha * sinGamma + sinAlpha * sinBeta * cosGamma) + 
       z * (cosAlpha * sinBeta * sinGamma + sinAlpha * cosGamma);
    
    Gy = x * cosBeta * sinGamma + 
       y * (cosAlpha * cosGamma + sinAlpha * sinBeta * sinGamma) + 
       z * (cosAlpha * sinGamma * sinBeta - sinAlpha * cosGamma);
    Gz = -x * sinBeta + 
       y * sinAlpha * cosBeta + 
       z * cosAlpha * cosBeta;*/
    // Step 1: Compute the conjugate of the quaternion (q^-1)
    float q_conjugate_x = -qx;
    float q_conjugate_y = -qy;
    float q_conjugate_z = -qz;
    float q_conjugate_w = qw;
    // Step 2: Perform the quaternion multiplication: q * v
    // q = (qw, qx, qy, qz) and v = (0, vx, vy, vz)
    float qv0 = qw * 0 - qx * x - qy * y - qz * z;  // Scalar part of q * v
    float qv1 = qw * x + qx * 0 + qy * z - qz * y;  // x component of q * v
    float qv2 = qw * y - qx * z + qy * 0 + qz * x;  // y component of q * v
    float qv3 = qw * z + qx * y - qy * x + qz * 0;  // z component of q * v

    // Step 3: Multiply the result by the conjugate of the quaternion (qv * q^-1)
    float finalX = qv0 * q_conjugate_w - qv1 * q_conjugate_x - qv2 * q_conjugate_y - qv3 * q_conjugate_z;
    float finalY = qv0 * q_conjugate_x + qv1 * q_conjugate_w + qv2 * q_conjugate_z - qv3 * q_conjugate_y;
    float finalZ = qv0 * q_conjugate_y - qv1 * q_conjugate_z + qv2 * q_conjugate_w + qv3 * q_conjugate_x;

    // The rotated vector (in the device's coordinate system)
    float Gx = finalX;
    float Gy = finalY;
    float Gz = finalZ;

    /*Serial.print(" Gx :");
    Serial.print(Gx);
    Serial.print(" Gy :");
    Serial.print(Gy);
    Serial.print(" Gz :");
    Serial.println(Gz);*/
    vx += ((GxO + Gx)/2)  * deltaTime;
    vy += ((GyO + Gy)/2) * deltaTime;
    vz += ((GzO + Gz)/2) * deltaTime;
    dx += ((VxO + vx)/2) * deltaTime;
    dy += ((VyO + vy)/2) * deltaTime;
    dz += ((VzO + vz)/2) * deltaTime;
    LaccX = x;
    LaccY = y;
    LaccZ = z;
    vX = vx;
    vY = vy;
    vZ = vz;
    dX = dx;
    dY = dy;
    dZ = dz;

    
    /*if (Bdx > printBdx){
      printBdx = Bdx;
    }
    if (Bdy > printBdy){
      printBdy = Bdy;
    }
    if (Bdz > printBdz){
      printBdz = Bdz;
    }
    Serial.print("acc");*/
    
    //mean reset

    if (GyroZ <= 0 && dir == 0) {
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
        myFile.print(String(printBdx,6));
        myFile.print(",");
        myFile.print(String(printBdy,6));
        myFile.print(",");
        myFile.print(String(printBdz,6));
        myFile.print(",");
        myFile.print(String(pitch,6));
        myFile.print(",");
        myFile.print(String(roll,6));
        myFile.print(",");
        myFile.println(String(yaw,6));
        Serial.print ("dX; ");
        Serial.print (dX);
        Serial.print (" dY; ");
        Serial.print (dY);
        Serial.print (" dZ; ");
        Serial.println (dZ);
        vx = 0;
        vy = 0;
        vz = 0;
        dx = 0;
        dy = 0;
        dz = 0;
        Bdx = 0;
        Bdy = 0;
        Bdz = 0;
        printBdx = 0;
        printBdy = 0;
        printBdz = 0;
        dir = 1;
      
        Serial.println("reset");
        /*while (GyroZ <= 0){
            BHY2.update();
            GyroZ = gyroscope.z();
            currentTime = millis();
            Time = currentTime;
            deltaTime = (currentTime - prevTime) / 1000.0;  // Convert to seconds
            prevTime = currentTime;
        }*/
    }
  
    if (GyroZ > 0) {
      dir = 0;
      }
  //}


  



  /*myFile = SD.open("test.txt", FILE_WRITE);
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
  myFile.print(String(Bdz,6));
  myFile.print(",");
  myFile.print(String(pitch,6));
  myFile.print(",");
  myFile.print(String(roll,6));
  myFile.print(",");
  myFile.println(String(yaw,6));*/
  
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
