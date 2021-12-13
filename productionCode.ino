// ===============================
// AUTHOR     :  Team GoGoGadget
// CREATE DATE     :  12/01/2021
// PURPOSE     : Supposed to simulate real world scenarios
// SPECIAL NOTES:
// ===============================
// Change History: Production level code for real world testing. This triggers the lawnmower at 15 seconds
//
//==================================
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Vector.h>
#include <Wire.h>

/* Define which pins are used for the LSM9DS1. */
#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
/* Define which pins are used for the UltraSonic sensor. */
#define ECHO_PIN 2
#define TRIG_PIN 3
/* Define which pins are used for the vibration sensor. */
#define VIBRATION_PIN A1
#define RELAY_PIN 7
#define LED_PIN 13
/* Other constants. */
#define SENSOR_BUFFER_LENGTH 10
#define VIBRATION_THRESHOLD 250
#define ANGLE_THRESHOLD 45.0f
#define TIME_THRESHOLD 15.0f

/**
 * This enum declaration contains all possible global states for the main() routine.
 */
enum State
{
    NORMAL,
    VIBRATING,
    TILTED
};

/* The following variables are required for the gyroscope sensor. */
unsigned long time;
long deltaTime = 0;
float rollFilteredOld = 0.0f;
float pitchFilteredOld = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;

/* The following variables are required for the UltraSonic sensor. */
long duration;
int distance;
int heights[10];

/* Initialize the LSM9DS1 in software using I2C. */
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

/* The following integers store the number of samples above or below the threshold level. */
int vibrationSampleCount = 0;
int angleSampleCount = 0;

/* The following floats are required to set a timeout for populating the sample buffer. */
float elapsedTime = 0.0f;
float pastTime = 0.0f;

/* This State enum represents the current global game state. */
State state = NORMAL;

/**
 * Runs at startup.
 * Initializes values.
 * Runs setup subroutines.
 */
void setup()
{
    /* Start up the serial monitor/plotter at 115200 baud. */
    Serial.begin(115200);
    /* Set the pins to the propper I/O mode. */
    pinMode(VIBRATION_PIN, INPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    /* Close the relay and turn off the status LED. */
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    /* Set up the LSM9DS1 gyro sensor. */
    setupGyroSensor();
    /* Set up the UltraSonic sensor. */
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    Serial.println("Ultrasonic Sensor HC-SR04 Test");
    Serial.println("with Arduino UNO R3");
    for (int i = 0; i < 10; i++)
    {
        heights[i] = 0;
    }
}


/**
 * Continuously runs after startup.
 * Responsible for checking sensors and adjusting state.
 */
void loop()
{
    switch (state)
    {
    // Under normal operation:
    case NORMAL:
        // Close the relay and turn off the LED
        digitalWrite(LED_PIN, LOW);
        digitalWrite(RELAY_PIN, LOW);
        // If the angle exceeds the threshold value, record a valid sample
        if (readGyro() >= ANGLE_THRESHOLD) 
            angleSampleCount++;
        // If the vibration magnituded exceeds the threshold value, record a valid sample
        if (readVibration() >= VIBRATION_THRESHOLD)
            vibrationSampleCount++;
        // Read the current time
        float currentTime = millis();
        // Calculate the time elapsed since the last reading
        float timeChange = currentTime - pastTime;
        // Set the past time to the current time
        pastTime = currentTime;
        // Update the total elapsed time with the time since the last reading
        elapsedTime += timeChange;
        // If the total elapsed time has exceeded the time threshold, reset all counts
        if (elapsedTime >= TIME_THRESHOLD)
        {
            angleSampleCount = 0;
            vibrationSampleCount = 0;
            elapsedTime = 0.0f;
            break;
        }
        // If there are more valid gyro angle samples recorded than the threshold, reset all counts and move to the TILTED state
        if (angleSampleCount >= SENSOR_BUFFER_LENGTH)
        {
            angleSampleCount = 0;
            vibrationSampleCount = 0;
            elapsedTime = 0.0f;
            state = TILTED;
            break;
        }
        // If there are more valid vibration samples recorded than the threshold, reset all counts and move to the VIBRATING state
        if (vibrationSampleCount >= SENSOR_BUFFER_LENGTH)
        {
            angleSampleCount = 0;
            vibrationSampleCount = 0;
            elapsedTime = 0.0f;
            state = VIBRATING;
            break;
        }
        break;
    // The mower is tilted:
    case TILTED:
        // Cut power to the motor with the relay
        digitalWrite(RELAY_PIN, HIGH);
        // Check for samples below the threshold
        if (readGyro() < ANGLE_THRESHOLD)
            angleSampleCount++;
        // Calculate time like before
        float currentTime = millis();
        float timeChange = currentTime - pastTime;
        pastTime = currentTime;
        elapsedTime += timeChange;
        // Check for a timeout like before
        if (elapsedTime >= TIME_THRESHOLD)
        {
            angleSampleCount = 0;
            vibrationSampleCount = 0;
            elapsedTime = 0.0f;
            break;
        }
        // If the mower has returned to an acceptable angle, reset counts and return to the NORMAL state
        if (angleSampleCount >= SENSOR_BUFFER_LENGTH)
        {
            angleSampleCount = 0;
            vibrationSampleCount = 0;
            elapsedTime = 0.0f;
            state = NORMAL;
            break;
        }
        break;
    // The mower is vibrating
    case VIBRATING:
        // Turn on the LED to display an error to the user
        digitalWrite(LED_PIN, HIGH);
        // Check for samples below the threshold
        if (readVibration() < VIBRATION_THRESHOLD)
            vibrationSampleCount++;
        // Calculate time like before
        float currentTime = millis();
        float timeChange = currentTime - pastTime;
        pastTime = currentTime;
        elapsedTime += timeChange;
        // Check for a timeout like before
        if (elapsedTime >= TIME_THRESHOLD)
        {
            angleSampleCount = 0;
            vibrationSampleCount = 0;
            elapsedTime = 0.0f;
            break;
        }
        // If the mower has ceased to vibrate, reset counts and return to the NORMAL state
        if (vibrationSampleCount >= SENSOR_BUFFER_LENGTH)
        {
            angleSampleCount = 0;
            vibrationSampleCount = 0;
            elapsedTime = 0.0f;
            state = NORMAL;
            break;
        }
        break;
    }
}

/**
 * Set up the LSM9DS1 sensor.
 */
void setupGyroSensor()
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

/**
 * Get the current gyroscope reading.
 */
float readGyro()
{
     // Read in the data from the gyroscope
    lsm.read();
    // Get a new sensor event
    sensors_event_t a, m, g, temp; 
    lsm.getEvent(&a, &m, &g, &temp);
    return readGyroEulerAngles(a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z);
}

/** 
 * Calculates the gyroscope euler angles based on the accelerometer and gyroscope readings from the LSM9DS1.
 */
float readGyroEulerAngles(float ax, float ay, float az, float gx, float gy, float gz)
{
    // Measured angle by the accelerometer
    float pitchMeasured, rollMeasured;
    float EARTH_GRAVITY = 9.80665f;

    pitchMeasured = (atan2(ax / EARTH_GRAVITY, az / EARTH_GRAVITY)) / 2 / M_PI * 360;
    rollMeasured = (atan2(ay / EARTH_GRAVITY, az / EARTH_GRAVITY)) / 2 / M_PI * 360;

    float rollFiltered = 0.9f * rollFilteredOld + 0.1f * rollMeasured;
    float pitchFiltered = 0.9f * pitchFilteredOld + 0.1f * pitchMeasured;

    // Calculating deltaTime
    time = micros();
    int difference = (int)((time - deltaTime) / 1000000);
    deltaTime = time;

    roll = 0.95f * (roll + gy * difference) + 0.05f * rollMeasured;
    pitch = 0.95f * (pitch - gy * difference) + 0.05f * pitchMeasured;

    //For testing our values
    /*
    Serial.print("Pitch: ");
    Serial.print(pitch, 2);
    Serial.println("°");
    Serial.print("Roll ");
    Serial.print(roll, 2);
    Serial.println("°");
    */
    // Return pitch or roll
    if (abs(pitch) > abs(roll))
    {
        return pitch;
    }
    return roll;
}

/**
 * Code to read the current UltraSonic sensor value in cm.
 */
float ultraSonicSensor()
{
    // Clears the TRIG_PIN condition
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    // Sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
    duration = pulseIn(ECHO_PIN, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor
    /*
    Serial.print("Distance: ");
    Serial.print(distance);
    */
    if (distance > 100)
    {
        ///
    }
    else if (distance < 20)
    {
        Serial.println("Lawn has been cut");
    }
    if (heights[9] == 0)
    {
        for (int i = 0; i < 10; i++)
        {
            heights[i] = 0;
        }
    }
    else
    {
        for (int i = 0; i < 10; i++)
        {
            heights[i] = distance;
        }
    }

    Serial.println(" cm");

    return distance;
}

/** 
 * Reads the piezoelectric vibration sensor and returns the magnitude of vibration.
 */
unsigned int readVibration()
{
    unsigned int wiredPiezo = analogRead(VIBRATION_PIN);

    delay(100);

    return wiredPiezo;
}
