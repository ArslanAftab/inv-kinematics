///////////////////////////////////////////////////
//         Control Robot Arm given X,Y,Z         //
///////////////////////////////////////////////////

// Calibrations to do:

// Known tasks
// - Define working space mapping of analogue read
// - Elbow up solution
// - Write theta values to joints

#include <Servo.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 5
#define Joint4Pin 6
#define GripperPin 11

// Inverse kinematics constants
float Link2  = 95.0;
float Link3  = 190.0;
float divisor = 36100.0;
float rad2degree = 180.0 / M_PI;

// Ammend the workspace of the robot arm
#define minX 50
#define maxX 200
#define minY 50
#define maxY 200
#define minZ 0
#define maxZ 200

// Control pins
int ControlPin1 = A1; //x
int ControlPin2 = A2; //y
int ControlPin3 = A3; //z

// Control values
float Joint1Control = 512; // middle value between 0 and 1024
float Joint2Control = 512; // middle value between 0 and 1024
float Joint3Control = 512; // middle value between 0 and 1024

// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Joint4;
Servo Gripper;

// Starting Joint Angles
float Joint1Angle = 90;
float Joint2Angle = 90;
float Joint3Angle = 90;
float Joint4Angle = 180;
float GripperOpen = 50;
float GripperClose = 142; 


// Joint Angle Offsets
float Joint1Offset = 0; // Your value may be different
float Joint2Offset = 5; // Your value may be different
float Joint3Offset = 8; // Your value may be different
float Joint4Offset = -90; // Your value may be different

// Target Coordinates
float xTarget = 0;
float yTarget = 0;
float zTarget = 0;

// Calculation variables
float s3 =0;
float c3 = 0;
float theta1 = 0;
float theta2 = 0;
float theta3 = 0;
float r = 0;
float k1 = 0;
float k2 = 0;
float k3 = 0;
float beta = 0;
float gamma = 0;

// Inverse kinematics 
float CalculateTheta3(float x, float y, float z){
    c3 = sq(x) + sq(y) +sq(z) - sq(Link2) -sq(Link3);
    c3 = c3/divisor;
    s3 = sqrtf(1-sq(c3));
    return atan2(s3,c3)* rad2degree;
}

float CalculateR(){
    k1 = (Link3 * c3) + Link2;
    k2 = Link3 * s3;
    return sqrtf(sq(k1)+sq(k2));
}

float CalculateGamma()
{
    return atan2(k2, k1)* rad2degree;
}

float CalculateTheta2(float x, float y, float z){
    float temp1 = -z/r;
    float temp2 = sqrtf(sq(x)+sq(y))/r;
    return (atan2(temp1, temp2) + atan2(k2,k1))* rad2degree;
}

float CalculateTheta1(float x, float y)
{
    beta = theta2-theta3;
    k3 = (Link3 * cos(beta)) + (Link2 * cos(theta2));
    float val = (atan2((y/k3), (x/k3)))* rad2degree;
    if (val <0)
    {
        val +=180;
    }
    return val;
}
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
    Joint1Control = analogRead(ControlPin1);
    Joint2Control = analogRead(ControlPin2);
    Joint3Control = analogRead(ControlPin3);
    // Map Analog-Digital-Converted Values into Angles
    // xTarget = 34;
    // yTarget = 80;
    // zTarget = 61;
    xTarget = map(Joint1Control,0,1023,minX,maxX);
    yTarget = map(Joint2Control,0,1023,minY, maxY);
    zTarget = map(Joint3Control,0,1023,minZ, maxZ);

    xTarget = -35;
    yTarget = 76;
    zTarget =-133;
    
    //Calculate Angles
    theta3 = CalculateTheta3(xTarget, yTarget, zTarget);
    r = CalculateR();
    gamma = CalculateGamma();
    theta2 = CalculateTheta2(xTarget, yTarget, zTarget);
    theta1 = CalculateTheta1(xTarget,yTarget);

    // Output results
    Serial.print("x: ");
    Serial.print(xTarget);
    Serial.print(", y: ");
    Serial.print(yTarget);
    Serial.print(", z: ");
    Serial.println(zTarget);    

    // Helpers
//    Serial.print("C3: ");
//    Serial.print(c3);
//    Serial.print(", S3: ");
//    Serial.print(s3);
//    Serial.print(", Gamma: ");
//    Serial.print(gamma);
//    Serial.print(", R: ");
//    Serial.println(r);

    // Angles
    Serial.print("Theta1: ");
    Serial.print(theta1);
    Serial.print(", Theta2: ");
    Serial.print(theta2);
    Serial.print(", Theta3: ");
    Serial.println(theta3);
    Serial.println();
    Joint1.write(theta1+Joint1Offset);
    Joint2.write(theta2+Joint2Offset);
    Joint3.write(theta3+Joint3Offset);
    Joint4.write(Joint4Angle+Joint4Offset);
    delay(10);
}
