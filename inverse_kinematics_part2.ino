///////////////////////////////////////////////////
// Control 3 servo motors using 3 potentiometers //
///////////////////////////////////////////////////

// Calibrations to do:
// - Define joint pins
// - Angle offsets 
// - Tune gripper
// - Define working space mapping

#include <Servo.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define Joint4Pin 10
#define GripperPin 11
#define Link2 95
#define Link3 190

// Control pins
int ControlPin1 = A0; //x
int ControlPin2 = A1; //y
int ControlPin3 = A2; //z

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

// Target Coordinates
int xTarget = 0;
int yTarget = 0;
int zTarget = 0;
float s3 =0;
float c3 = 0;
float theta3 = 0;
float r = 0;

// Inverse kinematics 
float CalculateTheta3(int x, int y, int z){
    c3 = sq(x) + sq(y) +sq(z);
    c3 -= (sq(Link3) + sq(Link2));
    c3 /= 2*(Link2*Link3);
    s3 = sqrt(1-sq(c3));
    return atan2(s3,c3);
}

float CalculaterR()
void setup(){
    Serial.begin(9600);
    Joint1.attach(Joint1Pin);
    Joint2.attach(Joint2Pin);
    Joint3.attach(Joint3Pin);
    Joint4.attach(Joint4Pin);
    Gripper.attach(GripperPin);
    //Joint1.write(Joint1Angle+Joint1Offset);
    //Joint2.write(Joint2Angle+Joint2Offset);
    //Joint3.write(Joint3Angle+Joint3Offset);
    //Joint4.write(Joint4Angle+Joint4Offset);
    //Gripper.write(GripperOpen); // Open gripper
    delay(5000); // 5 seconds before robot reads in potentiometer values
}

void loop(){
    // Read Potentiometer Values
    Joint1Control = analogRead(ControlPin1);
    Joint2Control = analogRead(ControlPin2);
    Joint3Control = analogRead(ControlPin3);
    // Map Analog-Digital-Converted Values into Angles
    xTarget = map(Joint1Control,0,1023,0,180);
    yTarget = map(Joint2Control,0,1023,0,180);
    zTarget = map(Joint3Control,0,1023,0,180);
    //Calculate Angles
    theta3 = CalculateTheta3(xTarget, yTarget, zTarget);
    Serial.print("x: ");
    Serial.print(xTarget);
    Serial.print(", y: ");
    Serial.print(yTarget);
    Serial.print(", z: ");
    Serial.println(zTarget);    
  	Serial.print("Theta: ");
    Serial.println(theta3);    

    //Joint1.write(Joint1Angle+Joint1Offset);
    //Joint2.write(Joint2Angle+Joint2Offset);
    //Joint3.write(Joint3Angle+Joint3Offset);
    //Joint4.write(Joint4Angle+Joint4Offset);
    delay(3000);
}
