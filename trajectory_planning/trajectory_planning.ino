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
float Joint3Offset = -10; 
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

void trajectoryPlan(float x0, float y0, float z0, float xf, float yf, float zf){

    // Duration of movement
    float duration = 5;

    //Calculate Start position angles
    float startTheta3 = CalculateTheta3(x0, y0, z0);
    float startR = CalculateR();
    float StartGamma = CalculateGamma();
    float startTheta2 = CalculateTheta2(x0, y0, z0);
    float startTheta1 = CalculateTheta1(x0,y0);

    //Calculate end position angles
    float endTheta3 = CalculateTheta3(xf, yf, zf);
    float endR = CalculateR();
    float endGamma = CalculateGamma();
    float endTheta2 = CalculateTheta2(xf, yf, zf);
    float endTheta1 = CalculateTheta1(xf,yf);

    // Checking
    // Serial.print("theta1i: ");
    // Serial.print(startTheta1);
    // Serial.print(" theta2i: ");
    // Serial.print(startTheta2);    
    // Serial.print(" theta3i: ");
    // Serial.println(startTheta3);

    // Serial.print("theta1f: ");
    // Serial.print(endTheta1);
    // Serial.print(" theta2f: ");
    // Serial.print(endTheta2);    
    // Serial.print(" theta3f: ");
    // Serial.println(endTheta3);

    // Helper variables
    // Theta1 
    float a0_1 = startTheta1;
    float a2_1 = (3/sq(duration)) * (endTheta1-startTheta1);
    float a3_1 = -(2/(duration*duration*duration)) * (endTheta1-startTheta1);
    // Theta2
    float a0_2 = startTheta2;
    float a2_2 = (3/sq(duration)) * (endTheta2-startTheta2);
    float a3_2 = -(2/(duration*duration*duration)) * (endTheta2-startTheta2);

    // Theta2
    float a0_3 = startTheta3;
    float a2_3 = (3/sq(duration)) * (endTheta3-startTheta3);
    float a3_3 = -(2/(duration*duration*duration)) * (endTheta3-startTheta3);

    // Trajectory planning for each angle one by one
    for (float time  =0; time < 5; time+=0.1)
    {
        // Theta 1 calculations
        theta1 = a0_1 + (a2_1 * sq(time)) + (a3_1 * time * time * time);
        // Theta 2 calculations
        theta2 = a0_2 + (a2_2 * sq(time)) + (a3_2 * time * time * time);
        // Theta 3 calculations
        theta3 = a0_3 + (a2_3 * sq(time)) + (a3_3 * time * time * time);
        // Ignore invalid angles
        checkAllAngles();  

        // Write angles        
        Serial.print("Time: ");
        Serial.print(time);
        Serial.print(", Theta1: ");
        Serial.print(theta1);
        Serial.print(", Theta2: ");
        Serial.print(theta2);
        Serial.print(", Theta3: ");
        Serial.println(theta3);
        Serial.println();
        Joint1.write(theta1+Joint1Offset);
        Joint2.write(theta2+Joint2Offset);
        Joint3.write(theta3+Joint3Offset);
        delay(10);
    }
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
}

void loop(){
    // Constant xyz 
    xTarget = 120;
    yTarget = 120;
    zTarget = -80;

        
    delay(10000);
    trajectoryPlan(startxTarget, startyTarget, startzTarget, xTarget, yTarget, zTarget);
    
    delay(10000);
    trajectoryPlan(xTarget, yTarget, zTarget, startxTarget, startyTarget, startzTarget);
}