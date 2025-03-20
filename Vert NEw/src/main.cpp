#include <IntervalTimer.h>
#include <Servo.h>
#include "MS5837.h"
#include <Wire.h>

MS5837 sensor2;

const int bufferSize = 80;
char incomingData[bufferSize];
float fulldepth = 1.0; // max depth in meters
float baselineDepth = 0.0;

// PID variables
double Setpoint = 0.4; // Desired depth in meters
double Input = 0, Output = 0;
double initThrusterValue = 1700; // How much thrusters will run to the desired depth
double Kp = 0.8, Ki = 0.001, Kd =0.1; // PID tuning parameters original
double previousError = 0, integral = 0;
double error=0;
double errornow=0;
double thrusterPower = 1500; // Neutral depth

const int thrusterPins[4] = {0, 1, 2, 3}; // Pins on Teensy 4.1  (Front Left, Front Right, Back Left, Back Right)
Servo thrusters[4]; // Create an array of Servo objects

IntervalTimer serialTimer; // Timer for handling Serial7 input
volatile float Serial7Depth = -1.0;
volatile float kill_switch = -1.0;

void calibrateSensor(MS5837 &sensor, int numReadings);
float medianOfThree(float a, float b, float c);
void setThrusterPWM(int thrusterPin, int value);
void updatePID();
void initializeThrusters();
void readSerial7Data(); // Function to handle Serial7 input

void setup() {
    Wire.begin();
    Serial.begin(9600);  // Serial Monitor
    Serial7.begin(9600); // Communication with transmitting Teensy

    Serial.println("Starting");

    // Initialize sensor
    sensor2.setModel(MS5837::MS5837_02BA);
    if (!sensor2.init()) {
        Serial.println("Sensor initialization failed!");
        while (1); // Halt if sensor initialization fails
    }

    sensor2.setFluidDensity(997); // Set fluid density for water

    // Calibrate the sensor
    Serial.println("Calibrating sensor...");
    calibrateSensor(sensor2, 100);
    Serial.print("Baseline depth: ");
    Serial.println(baselineDepth);

    delay(2000);

    // Attach thrusters to Servo objects and set initial position
    for (int i = 0; i < 4; i++) {
        thrusters[i].attach(thrusterPins[i]);
    }
    
    initializeThrusters();

    // Start the timer for reading from Serial7 every 100ms (10Hz)
    serialTimer.begin(readSerial7Data, 100000); // Call readSerial7Data every 100ms
}

void loop() {
    if (kill_switch == -1.0) {
        Serial.println("Waiting for start signal...");
         // Turn off all thrusters 
         for (int i = 0; i < 4; i++) { 
         setThrusterPWM(thrusterPins[i], 1500); // Set thrusters to neutral 
         } 
        delay(500);
        return;
    }

    if (kill_switch == 0.0) { 
        for (int i = 0; i < 4; i++) {
            setThrusterPWM(thrusterPins[i], 1500);
        }
        Serial.println("All thrusters set to neutral.");
        delay(500);
        return;
    }

    if (kill_switch == 1.0) {
        // Setpoint = 0.5;
    }

    sensor2.read();
    float depth = sensor2.depth();
    float relativeDepth = depth - baselineDepth;

    Serial.print("D1: ");
    Serial.print(relativeDepth);
    Serial.println("cm");

    if (Serial7Depth != -1.0) {
        Serial.print("D2: ");
        Serial.print(Serial7Depth);
        Serial.println("cm");
    }    
        float trueDepth = relativeDepth;
        
    if (relativeDepth < -1.0 && relativeDepth > 10.0) { 
      trueDepth = Serial7Depth; 
    } else if (relativeDepth < -1.0 || relativeDepth > 10.0 || Serial7Depth < -1.0 || Serial7Depth > 10.0) { 
      float depth1 = sensor2.depth() - baselineDepth; 
      delay(100); 
      float depth2 = sensor2.depth() - baselineDepth; 
      delay(100); 
      float depth3 = sensor2.depth() - baselineDepth; 
  
      trueDepth = medianOfThree(depth1, depth2, depth3); 
  
      if (trueDepth < -1.0 || trueDepth > 10.0) { 
        Serial.println("Error: Depth"); 
  
        // Turn off all thrusters 
        for (int i = 0; i < 4; i++) { 
          setThrusterPWM(thrusterPins[i], 1500); // Set thrusters to neutral 
        } 
      } 
    } 

    Serial.print("True Depth: "); 
    Serial.println(trueDepth); 
  
    // Blind control 
    if (trueDepth < 0.9 * Setpoint) {
      int pwmValue = constrain(initThrusterValue, 1100, 1900); 
      for (int i = 0; i < 4; i++) { 
        setThrusterPWM(thrusterPins[i], pwmValue); 
      } 
    } else { 
      // PID control 
        for (int i = 0; i < 4; i++) {
            setThrusterPWM(thrusterPins[i], 1500);
        }
        updatePID(); 
  
       if ( trueDepth==Setpoint && trueDepth <=(Setpoint*0.95)) { 
         thrusterPower = initThrusterValue * (1 + Output); 

         for (int i = 0; i < 4; i++) {
            setThrusterPWM(thrusterPins[i], thrusterPower);
      }      
       if (errornow >= 0 && errornow < 0.1){
        integral=0;
       }
       }else {
          for (int i = 0; i < 4; i++) {
            setThrusterPWM(thrusterPins[i], 1500);
        }

        }    
    } 
    delay(5);
}

// Function to read data from Serial7 in a separate thread-like manner
void readSerial7Data() {
   static int consecutiveCount = 0; // Tracks consecutive identical values
   static int lastValue = -1;       // Tracks the last killSwitch value

   if (Serial7.available() > 0) {
      int bytesRead = Serial7.readBytesUntil('\n', incomingData, bufferSize - 1);
      incomingData[bytesRead] = '\0';

      char *token = strtok(incomingData, ",");
      float tempSerial7depth = -1.0;
      float tempKillSwitch = -1.0;

      if (token != NULL) {
         tempSerial7depth = atof(token);
         token = strtok(NULL, ",");
         if (token != NULL) {
            int killSwitchValue = atoi(token);

            // Logic for handling "000" state and checking consecutive values
            if (kill_switch == 0.0) { // Only apply logic when in "000" state
               if (killSwitchValue == lastValue) {
                  consecutiveCount++;
               } else {
                  consecutiveCount = 1; // Reset count if value changes
                  lastValue = killSwitchValue;
               }

               // Update kill_switch only if we have 3 consecutive identical values
               if (consecutiveCount >= 5) {
                  switch (killSwitchValue) {
                     case 0: tempKillSwitch = 0.0; break;
                     case 10: tempKillSwitch = 1.0; break;
                     default: tempKillSwitch = -1.0; break;
                  }
               } else {
                  tempKillSwitch = 0.0; // Keep "000" state if condition not met
               }
            } else { // Normal behavior for states other than "000"
               switch (killSwitchValue) {
                  case 0: tempKillSwitch = 0.0; break;
                  case 10: tempKillSwitch = 1.0; break;
                  default: tempKillSwitch = -1.0; break;
               }
            }
         }
      }

      noInterrupts(); // Disable interrupts while updating shared variables
      kill_switch = tempKillSwitch;
      Serial7Depth = tempSerial7depth;
      interrupts(); // Re-enable interrupts
   }
}

void setThrusterPWM(int thrusterPin, int value) {
   value = constrain(value, 1100, 1900); // Constrain between 1100 to 1900 pwm)
   for (int i=0;i<4;i++) {
       if (thrusters[i].attached() && thrusterPins[i] == thrusterPin) {
           thrusters[i].writeMicroseconds(value);
       }
   }
}

void initializeThrusters() {
   Serial.println("Initializing thrusters at neutral position...");
   for (int i=0;i<4;i++) {
       setThrusterPWM(thrusterPins[i],1500);
   }
   delay(3000); 
   Serial.println("Thrusters initialization complete.");
}

void updatePID() {
  error = Setpoint - Input;
  errornow = error / fulldepth;
  integral += errornow;
  double derivative = previousError - errornow;

  Output = Kp * errornow + Ki * integral + Kd * derivative;

  previousError=errornow;

  if (Output > 100) Output = 100; 
  // Debugging PID values 
  Serial.print("Error: "); 
  Serial.print(error); 
  Serial.print(", Integral: "); 
  Serial.print(integral); 
  Serial.print(", Derivative: "); 
  Serial.print(derivative); 
  Serial.print(", Output: "); 
  Serial.println(Output); 
}

void calibrateSensor(MS5837 &sensor, int numReadings) { 
  float sumDepth = 0.0; 
  for (int i = 0; i < numReadings; i++) { 
    sensor.read(); 
    sumDepth += sensor.depth(); 
    delay(100);  // Small delay between readings 
  } 
  baselineDepth = sumDepth / numReadings; 
} 

float medianOfThree(float a, float b, float c) { 
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b; 
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a; 
  return c;
}