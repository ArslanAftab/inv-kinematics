///////////////////////////////////////////////////
// Control 3 servo motors using 3 potentiometers //
///////////////////////////////////////////////////

// Calibrations to do:
// - Define joint pins
// - Angle offsets 
// - Tune gripper

#include <Servo.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define Joint4Pin 10
#define GripperPin 11

// Control pins
int Joint1ControlPin = A1;
int Joint2ControlPin = A2;
int Joint3ControlPin = A3;

// Control values
int Joint1Control = 512; // middle value between 0 and 1024
int Joint2Control = 512; // middle value between 0 and 1024
int Joint3Control = 512; // middle value between 0 and 1024

// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Joint4;
Servo Gripper;

// Starting Joint Angles
int Joint1Angle = 90;
int Joint2Angle = 90;
int Joint3Angle = 90;
int Joint4Angle = 180;
int GripperOpen = 60; // Open gripper; Need to tune value
int GripperClose = 120; // Close gripper; Need to tune value


// Joint Angle Offsets
int Joint1Offset = 0; // Your value may be different
int Joint2Offset = 0; // Your value may be different
int Joint3Offset = 0; // Your value may be different
int Joint4Offset = 0; // Your value may be different

void setup(){
    Serial.begin(9600);
    Joint1.attach(Joint1Pin);
    Joint2.attach(Joint2Pin);
    Joint3.attach(Joint3Pin);
    Joint4.attach(Joint4Pin);
    Gripper.attach(GripperPin);
    Joint1.write(Joint1Angle+Joint1Offset);
    Joint2.write(Joint2Angle+Joint2Offset);
    Joint3.write(Joint3Angle+Joint3Offset);
    Joint4.write(Joint4Angle+Joint4Offset);
    Gripper.write(GripperOpen); // Open gripper
    delay(5000); // 5 seconds before robot reads in potentiometer values
}

void loop(){
    // Read Potentiometer Values
    Joint1Control = analogRead(Joint1ControlPin);
    Joint2Control = analogRead(Joint2ControlPin);
    Joint3Control = analogRead(Joint3ControlPin);
    // Map Analog-Digital-Converted Values into Angles
    Joint1Angle = map(Joint1Control,0,1023,0,180);
    Joint2Angle = map(Joint2Control,0,1023,0,180);
    Joint3Angle = map(Joint3Control,0,1023,0,180);
    Serial.print("Joint 1: ");
    Serial.print(Joint1Angle);
    Serial.print(", Joint 2: ");
    Serial.print(Joint2Angle);
    Serial.print(", Joint 3: ");
    Serial.println(Joint3Angle);
    Joint1.write(Joint1Angle+Joint1Offset);
    Joint2.write(Joint2Angle+Joint2Offset);
    Joint3.write(Joint3Angle+Joint3Offset);
    Joint4.write(Joint4Angle+Joint4Offset);
    delay(10);
}
