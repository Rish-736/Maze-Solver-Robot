#include <Arduino.h>
#include <Encoder.h>
#include "Adafruit_VL53L0X.h"
#include <MPU6050_light.h>
#include <Wire.h>

/* ------------------- MOTOR DRIVER ------------------- */
#define AIN1 6
#define AIN2 7
#define PWMA 3
#define BIN1 9
#define BIN2 5
#define PWMB 4
#define STBY 8  // standby pin

void setupMotors() {
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Left motor
    if (leftSpeed >= 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        leftSpeed = -leftSpeed;
    }
    analogWrite(PWMA, constrain(leftSpeed, 0, 255));

    // Right motor
    if (rightSpeed >= 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        rightSpeed = -rightSpeed;
    }
    analogWrite(PWMB, constrain(rightSpeed, 0, 255));
}

void stopMotors() {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}

/* ------------------- ENCODERS ------------------- */
Encoder motorAEnc(14, 15);   // Left motor encoder pins
Encoder motorBEnc(40, 41);   // Right motor encoder pins
long oldPosA = 0, oldPosB = 0;

void setupEncoders() {
    oldPosA = motorAEnc.read();
    oldPosB = motorBEnc.read();
}

long getLeftEncoderTicks() { return motorAEnc.read(); }
long getRightEncoderTicks() { return motorBEnc.read(); }

long getLeftEncoderDelta() {
    long posA = motorAEnc.read();
    long delta = posA - oldPosA;
    oldPosA = posA;
    return delta;
}

long getRightEncoderDelta() {
    long posB = motorBEnc.read();
    long delta = posB - oldPosB;
    oldPosB = posB;
    return delta;
}

void resetEncoders() {
    motorAEnc.write(0);
    motorBEnc.write(0);
    oldPosA = 0;
    oldPosB = 0;
}

/* ------------------- TOF SENSORS ------------------- */
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33
#define LOX5_ADDRESS 0x34

#define FRONT   33
#define RIGHT   34
#define F_RIGHT 35
#define F_LEFT  36
#define LEFT    37

Adafruit_VL53L0X lox1, lox2, lox3, lox4, lox5;
VL53L0X_RangingMeasurementData_t measure1, measure2, measure3, measure4, measure5;

int X_front = -1, X_right = -1, X_FRight = -1, X_FLeft = -1, X_left = -1;

void setupTOF() {
    Wire.begin();
    pinMode(FRONT, OUTPUT); pinMode(RIGHT, OUTPUT);
    pinMode(F_RIGHT, OUTPUT); pinMode(F_LEFT, OUTPUT); pinMode(LEFT, OUTPUT);

    digitalWrite(FRONT, LOW); digitalWrite(RIGHT, LOW);
    digitalWrite(F_RIGHT, LOW); digitalWrite(F_LEFT, LOW); digitalWrite(LEFT, LOW);
    delay(10);

    digitalWrite(FRONT, HIGH);  delay(10); lox1.begin(LOX1_ADDRESS);
    digitalWrite(RIGHT, HIGH);  delay(10); lox2.begin(LOX2_ADDRESS);
    digitalWrite(F_RIGHT, HIGH); delay(10); lox3.begin(LOX3_ADDRESS);
    digitalWrite(F_LEFT, HIGH);  delay(10); lox4.begin(LOX4_ADDRESS);
    digitalWrite(LEFT, HIGH);    delay(10); lox5.begin(LOX5_ADDRESS);
}

void readTOF() {
    lox1.rangingTest(&measure1, false); 
    lox2.rangingTest(&measure2, false); 
    lox3.rangingTest(&measure3, false); 
    lox4.rangingTest(&measure4, false); 
    lox5.rangingTest(&measure5, false); 

    X_front  = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : -1;
    X_right  = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : -1;
    X_FRight = (measure3.RangeStatus != 4) ? measure3.RangeMilliMeter : -1;
    X_FLeft  = (measure4.RangeStatus != 4) ? measure4.RangeMilliMeter : -1;
    X_left   = (measure5.RangeStatus != 4) ? measure5.RangeMilliMeter : -1;
}

int getFrontDistance() { return X_front; }
int getRightDistance() { return X_right; }
int getFRightDistance() { return X_FRight; }
int getFLeftDistance() { return X_FLeft; }
int getLeftDistance() { return X_left; }

/* ------------------- MPU6050 ------------------- */
MPU6050 mpu(Wire1);

void setupIMU() {
    Wire.begin();
    Wire1.begin();
    byte status = mpu.begin();
    if (status != 0) {
        Serial.print(F("MPU6050 init failed code: ")); Serial.println(status); while(1);
    }
    Serial.println(F("MPU6050 calibration...")); delay(1000);
    mpu.calcOffsets();
    Serial.println(F("Calibration done"));
}

void updateIMU() { mpu.update(); }
float getYawDeg() { return mpu.getAngleZ(); }
float getGyroRateZ() { return mpu.getGyroZ(); }
float getPitchDeg() { return mpu.getAngleX(); }
float getRollDeg() { return mpu.getAngleY(); }

/* ------------------- MAZE MAPPING ------------------- */
#define MAZE_SIZE 16
struct Cell { uint8_t walls; bool visited; };
Cell maze[MAZE_SIZE][MAZE_SIZE];

struct Pose { int x,y,heading; };
Pose robot;

void initMaze() {
    for(int i=0;i<MAZE_SIZE;i++)
        for(int j=0;j<MAZE_SIZE;j++) { maze[i][j].walls=0; maze[i][j].visited=false; }
    robot.x=0; robot.y=0; robot.heading=0; // bottom-left facing North
}

void updateWalls() {
    const int THRESHOLD = 150;
    int front=getFrontDistance(), right=getRightDistance(), left=getLeftDistance();
    int x=robot.x, y=robot.y;
    maze[x][y].walls=0;
    if(front>0 && front<THRESHOLD) maze[x][y].walls|=(1<<robot.heading);
    int leftDir=(robot.heading+3)%4; if(left>0 && left<THRESHOLD) maze[x][y].walls|=(1<<leftDir);
    int rightDir=(robot.heading+1)%4; if(right>0 && right<THRESHOLD) maze[x][y].walls|=(1<<rightDir);
    maze[x][y].visited=true;
}

void moveForwardCell() {
    switch(robot.heading){
        case 0: robot.y++; break;
        case 1: robot.x++; break;
        case 2: robot.y--; break;
        case 3: robot.x--; break;
    }
}

void turnRobot(int direction) {
    robot.heading=(robot.heading+direction+4)%4;
}

void printMaze() {
    for(int y=MAZE_SIZE-1;y>=0;y--){
        for(int x=0;x<MAZE_SIZE;x++){
            Serial.print(maze[x][y].visited ? "." : "#");
        }
        Serial.println();
    }
}

/* ------------------- WALL CHECK HELPERS ------------------- */
bool isWallAhead() { return maze[robot.x][robot.y].walls & (1<<robot.heading); }
bool isWallLeft()  { return maze[robot.x][robot.y].walls & (1<<((robot.heading+3)%4)); }
bool isWallRight() { return maze[robot.x][robot.y].walls & (1<<((robot.heading+1)%4)); }

/* ------------------- MOVEMENT FUNCTIONS ------------------- */
void moveForward1Cell(int speed) {
    resetEncoders();
    long targetTicks=4000;
    while((getLeftEncoderTicks()+getRightEncoderTicks())/2 < targetTicks){
        setMotorSpeeds(speed,speed);
        updateIMU();
    }
    stopMotors();
    moveForwardCell();
}

void turn90PID(int direction,int speed){
    float startYaw=getYawDeg();
    float targetYaw=startYaw+direction*90.0;
    if(targetYaw>180) targetYaw-=360;
    if(targetYaw<-180) targetYaw+=360;
    while(abs(getYawDeg()-targetYaw)>2.0){
        float error=targetYaw-getYawDeg();
        int turnSpeed=constrain((int)(error*2.0),-speed,speed);
        setMotorSpeeds(-turnSpeed,turnSpeed);
        updateIMU();
    }
    stopMotors();
    turnRobot(direction);
}

/* ------------------- EXPLORATION LOGIC ------------------- */
void exploreMaze(int speed){
    while(true){
        readTOF();
        updateWalls();

        if(!isWallLeft()){
            turn90PID(-1,speed); moveForward1Cell(speed);
        }
        else if(!isWallAhead()){
            moveForward1Cell(speed);
        }
        else if(!isWallRight()){
            turn90PID(1,speed); moveForward1Cell(speed);
        }
        else{
            turn90PID(2,speed); moveForward1Cell(speed);
        }

        printMaze();
    }
}

/* ------------------- SETUP & LOOP ------------------- */
void setup(){
    Serial.begin(115200);
    setupMotors(); setupEncoders(); setupTOF(); setupIMU();
    initMaze();
}

void loop(){
    exploreMaze(150);
}