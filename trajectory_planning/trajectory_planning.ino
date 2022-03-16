///////////////////////////////////////////////////
//   Control Robot Arm given X,Y,Z w/ Traj Plan  //
///////////////////////////////////////////////////

// Known tasks
// - Implement the functionality for trajectory planning


#include <Servo.h>
// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 5
#define Joint4Pin 6
#define GripperPin 11
// Starting xyz
float startxTarget = 0;
float startyTarget = 190;
float startzTarget = -95;

// Inverse kinematics constants
float Link2  = 95.0;
float Link3  = 190.0;
float divisor = 36100.0;
float rad2degree = 180.0 / M_PI;

// Ammend the workspace of the robot arm
#define minX -285
#define maxX 285
#define minY 60
#define maxY 285
#define minZ 60
#define maxZ -285

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
float Joint1Offset = 8; 
float Joint2Offset = 0; 
float Joint3Offset = 0; 
float Joint4Offset = -90; 

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

// Valid angle tracking
float lastValid1 = Joint1Angle;
float lastValid2 = Joint2Angle;
float lastValid3 = Joint3Angle;

bool checkValid(float angle){
    if (isnan(angle)) {
        return false;
    }
    if (isinf(angle)) {
        return false;
    }
    return true;
}

void checkAllAngles()
{
    if (! checkValid(theta1)) {
        Serial.println("Theta1 is not valid");
        theta1 = lastValid1;
    }
    else{
        lastValid1 = theta1;
    }
    if (! checkValid(theta2)) {
        Serial.println("Theta2 is not valid");
        theta2 = lastValid2;
    }
    else{
        lastValid2 = theta2;
    }
    if (! checkValid(theta3)) {
        Serial.println("Theta3 is not valid");
        theta3 = lastValid3;
    }
    else{
        lastValid3 = theta3;
    }
}

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

float CalculateGamma() {
    return atan2(k2, k1)* rad2degree;
}

float CalculateTheta2(float x, float y, float z){
    float temp1 = -z/r;
    float temp2 = sqrtf(sq(x)+sq(y))/r;
    return (atan2(temp1, temp2) + atan2(k2,k1))* rad2degree;
}

float CalculateTheta1(float x, float y) {
    beta = theta2-theta3;
    k3 = (Link3 * cos(beta)) + (Link2 * cos(theta2));
    float val = (atan2((y/k3), (x/k3)))* rad2degree;
    // Catch the inverted angle
    if (val <0)
    {
        val +=180;
    }
    return val;
}

void moveTo(float x, float y, float z){
    //Calculate Angles
    theta3 = CalculateTheta3(x, y, z);
    r = CalculateR();
    gamma = CalculateGamma();
    theta2 = CalculateTheta2(x, y, z);
    theta1 = CalculateTheta1(x,y);

    // Output results
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(", y: ");
    Serial.print(y);
    Serial.print(", z: ");
    Serial.println(z);  
  
    // Ignore invalid angles
    checkAllAngles();  

    // Helpers
    Serial.print("C3: ");
    Serial.print(c3);
    Serial.print(", S3: ");
    Serial.print(s3);
    Serial.print(", Gamma: ");
    Serial.print(gamma);
    Serial.print(", R: ");
    Serial.println(r);

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

}

void setup(){
    Serial.begin(9600);
    Joint1.attach(Joint1Pin);
    Joint2.attach(Joint2Pin);
    Joint3.attach(Joint3Pin);
    Joint4.attach(Joint4Pin);
    Gripper.attach(GripperPin);

    moveTo(startxTarget, startyTarget, startzTarget);
    Gripper.write(GripperOpen); // Open gripper
    delay(500); // 5 seconds before starting traj planning
}

void loop(){
    // // Read Potentiometer Values
    // Joint1Control = analogRead(ControlPin1);
    // Joint2Control = analogRead(ControlPin2);
    // Joint3Control = analogRead(ControlPin3);
    
    // // Map Analog-Digital-Converted Values into Angles
    // xTarget = map(Joint1Control,0,1023,minX,maxX);
    // yTarget = map(Joint2Control,0,1023,minY, maxY);
    // zTarget = map(Joint3Control,0,1023,minZ, maxZ);

    // Constant xyz 
    xTarget = 120;
    yTarget = 120;
    zTarget = -80;
        
    delay(1000);
    moveTo(xTarget,yTarget,zTarget);

    delay(1000);
    moveTo(startxTarget, startyTarget, startzTarget);

}
