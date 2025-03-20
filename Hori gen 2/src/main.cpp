#include <Wire.h>
#include "MS5837.h"
#include <Arduino.h>
#include <Servo.h>

// Define servo objects for thrusters
Servo thruster1;
Servo thruster2;
Servo thruster3;
Servo thruster4;

// Define pin numbers for thrusters
const int thruster1Pin = 0;
const int thruster2Pin = 1;
const int thruster3Pin = 2;
const int thruster4Pin = 3;

// Depth sensor setup
#define SENSOR_ADDRESS 0xED
MS5837 sensor;
float baselineDepth = 0.0;

// Store the last Kill_Switch value
String lastKillSwitch = "000";

// Time before running thrusters after kill switch off
bool waitPeriodActive = false;
unsigned long waitStartTime = 0;
const unsigned long waitDuration = 5000; // 5 seconds in ms

// Timer to track serial data reception
unsigned long lastDataReceivedTime = 0;

// Timer for sending data to Serial8
unsigned long lastSerial8SendTime = 0;
const unsigned long Serial8Interval = 500; // Interval in milliseconds

// Define pin for Kill Switch (pin 41)
const int killSwitchPin = 41;

// Initialize the PWM values for the thrusters to neutral position (1500 us)
void setNeutralPWM() {
    thruster1.writeMicroseconds(1500);
    thruster2.writeMicroseconds(1500);
    thruster3.writeMicroseconds(1500);
    thruster4.writeMicroseconds(1500);
}

void calibrateSensor(int numReadings) {
    float sumDepth = 0.0;
    for (int i = 0; i < numReadings; i++) {
        sensor.read();
        sumDepth += sensor.depth();
        delay(100);
    }
    baselineDepth = sumDepth / numReadings;
}

void setup() {
    // Initialize Serial communication
    Serial.begin(9600);
    Serial8.begin(9600);

    // Initialize the depth sensor
    Wire.begin();
    sensor.setModel(MS5837::MS5837_02BA);
    sensor.init();
    sensor.setFluidDensity(997);

    // Calibrate the sensor
    Serial.println("Calibrating sensor...");
    calibrateSensor(100);
    Serial.print("Baseline depth: ");
    Serial.println(baselineDepth);

    // Initialize the thrusters by attaching them to their respective pins
    thruster1.attach(thruster1Pin);
    thruster2.attach(thruster2Pin);
    thruster3.attach(thruster3Pin);
    thruster4.attach(thruster4Pin);

    // Set neutral PWM values initially
    setNeutralPWM();

    // Set pin 41 as input for Kill Switch control
    pinMode(killSwitchPin, INPUT_PULLUP);
}

void loop() {
    // Read pressure and temperature from MS5837 sensor
    sensor.read();
    
    // Calculate depth relative to the baseline depth
    float currentDepth = sensor.depth() - baselineDepth;

    bool dataReceived = false;

    // Check if there are available bytes in the serial buffer
    if (Serial.available() > 0) {
        dataReceived = true;

        // Read four integer values from serial input for PWM and a fifth value for Kill_Switch
        int pwmThruster1 = Serial.parseInt();
        int pwmThruster2 = Serial.parseInt();
        int pwmThruster3 = Serial.parseInt();
        int pwmThruster4 = Serial.parseInt();
        String killSwitchValue = Serial.readStringUntil('\n'); 

        // Validate and update PWM values only if they are within the range of 1100 to 1900 us
        if (pwmThruster1 >= 1100 && pwmThruster1 <= 1900 && digitalRead(killSwitchPin) != LOW && 
        (!waitPeriodActive || (millis() - waitStartTime >= waitDuration))) {
            thruster1.writeMicroseconds(pwmThruster1);
            Serial.println("Thruster 1: " + String(pwmThruster1));
        }
        if (pwmThruster2 >= 1100 && pwmThruster2 <= 1900 && digitalRead(killSwitchPin) != LOW && 
        (!waitPeriodActive || (millis() - waitStartTime >= waitDuration))) {
            thruster2.writeMicroseconds(pwmThruster2);
            Serial.println("Thruster 2: " + String(pwmThruster2));
        }
        if (pwmThruster3 >= 1100 && pwmThruster3 <= 1900 && digitalRead(killSwitchPin) != LOW && 
        (!waitPeriodActive || (millis() - waitStartTime >= waitDuration))) {
            thruster3.writeMicroseconds(pwmThruster3);
            Serial.println("Thruster 3: " + String(pwmThruster3));
        }
        if (pwmThruster4 >= 1100 && pwmThruster4 <= 1900 && digitalRead(killSwitchPin) != LOW && 
        (!waitPeriodActive || (millis() - waitStartTime >= waitDuration))) {
            thruster4.writeMicroseconds(pwmThruster4);
            Serial.println("Thruster 4: " + String(pwmThruster4));
        } 
        
        else {
            setNeutralPWM();
        }

        // Update Kill_Switch value if received and trim any whitespace or newline characters
        lastKillSwitch = killSwitchValue.trim();

        // Update the last data received time
        lastDataReceivedTime = millis();
    }

    // Check if no data has been received for more than a second and set neutral PWM if needed
    if (!dataReceived && millis() - lastDataReceivedTime > 1000) {
        setNeutralPWM();
    }

    // Check the state of pin 41 (Kill Switch)
    if (digitalRead(killSwitchPin) == LOW) {
        // If pin 41 is LOW, force lastKillSwitch to "000"
        lastKillSwitch = "000";
        setNeutralPWM(); // Optionally stop all motors if kill switch is engaged
        waitPeriodActive = false; // Cancel any ongoing wait period
        // Serial.println("Kill switch engaged, sending '000'.");
        
    } else {
        // Kill switch released - check transition
        if (!waitPeriodActive) {
            waitPeriodActive = true;
            waitStartTime = millis();
        }
    }

    // Send depth and Kill_Switch over Serial8 (UART) every interval
    if (millis() - lastSerial8SendTime >= Serial8Interval) {
        Serial8.print(currentDepth);
        Serial8.print(",");
        Serial8.println(lastKillSwitch); 

        // Update the last send time
        lastSerial8SendTime = millis();
        
        // Print depth and Kill_Switch to USB serial as well
        Serial.print("Depth: ");
        Serial.print(currentDepth);
        Serial.print(" m, Kill_Switch: ");
        Serial.println(lastKillSwitch);
    }
}