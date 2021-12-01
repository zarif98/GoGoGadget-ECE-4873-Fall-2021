#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

unsigned long time;
long deltaTime = 0;
float rollFilteredOld = 0.0f;
float pitchFilteredOld = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;


// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);


void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}


void setup() 
{
  Serial.begin(115200);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  
  Serial.println("LSM9DS1 data read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
  pinMode(7, OUTPUT);
}

void loop() 
{
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  int heading_in_degrees = 90 - 180*atan2(m.magnetic.y,m.magnetic.x)/PI;


  calculateEulerAngles(a.acceleration.x, a.acceleration.y, 
                   a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z);

  Serial.println();
}

void printOrientation(float x, float y, float z)
{
  float pitch, roll;

  pitch = atan2(x, sqrt(y * y) + (z * z));
  roll = atan2(y, sqrt(x * x) + (z * z));
  pitch *= 180.0 / M_PI;
  roll *= 180.0 / M_PI;

  if(pitch > 45 || roll > 45) {
    digitalWrite(7, LOW); 
  }
  else {
    digitalWrite(7, HIGH);
  }

  Serial.print("Pitch: ");
  Serial.print(pitch, 2);
  Serial.println("°");
  Serial.print("Roll ");
  Serial.print(roll, 2);
  Serial.println("°");
}

void calculateEulerAngles(float ax, float ay, float az, float gx, float gy, float gz) {
  // Measured angle by the accelerometer
  float pitchMeasured, rollMeasured;
  float EARTH_GRAVITY = 9.80665f;

  pitchMeasured = (atan2(ax/ EARTH_GRAVITY, az / EARTH_GRAVITY)) / 2 /M_PI * 360;
  rollMeasured = (atan2(ay/ EARTH_GRAVITY, az/ EARTH_GRAVITY)) / 2 / M_PI * 360;

  float rollFiltered = 0.9f * rollFilteredOld + 0.1f * rollMeasured;
  float pitchFiltered = 0.9f * pitchFilteredOld + 0.1f * pitchMeasured;

  // Calculating deltaTime
  time = micros();
  int difference = (int) ((time - deltaTime) / 1000000);
  deltaTime = time;

  roll = 0.95f * (roll + gy * difference) + 0.05f * rollMeasured;
  pitch = 0.95f * (pitch - gy * difference) + 0.05f * pitchMeasured;


  if(abs(pitch) > 45 || abs(roll) > 45) {
    digitalWrite(7, HIGH); 
  }
  else {
    digitalWrite(7, LOW);
  }

  
  Serial.print("Pitch: ");
  Serial.print(pitch, 2);
  Serial.println("°");
  Serial.print("Roll ");
  Serial.print(roll, 2);
  Serial.println("°"); 

  
  }