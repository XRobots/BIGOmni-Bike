//ODrive
#include <ODriveArduino.h>

//PID
#include <PID_v1.h>

// IMU
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

// IMU variables

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define Gyr_Gain 0.00763358 

float AccelX;
float AccelY;
float AccelZ;

float GyroX;
float GyroY;
float GyroZ;

float mixX;
float mixY;

float pitchAccel, rollAccel;

float IMUpitch;
float IMUroll;

int requested_state = 0;

float pot1;
float pot2;
float pot3;
int sw1;
int sw2;

float output1;

// PID

double Pk1 = 2; 
double Ik1 = 15;
double Dk1 = 0.01;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
unsigned long count;

//ODrive Objects
ODriveArduino odrive1(Serial1);

// ****************** SETUP ******************************

void setup() {

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  // initialize serial communication
  Serial.begin(115200);
  Serial1.begin(115200);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-50, 50);
  PID1.SetSampleTime(10);

  Wire.begin();   
  accelgyro.initialize();

}   // end of setup

// ********************* MAIN LOOP *******************************

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event

    previousMillis = currentMillis;

    // get switches and pots
    
    pot3 = analogRead(A2);
    pot3 = (pot3-512) / 200;      // balancing trim pot
    sw1 = digitalRead(2);
    sw2 = digitalRead(3);

    if (sw1 == 0) {
      OdriveInit1();
    }

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
    AccelX = ax;
    AccelY = ay;
    AccelZ = az;
    GyroX = Gyr_Gain * (gx);
    GyroY = Gyr_Gain * (gy)*-1;
    GyroZ = Gyr_Gain * (gz);
  
    AccelY = (atan2(AccelY, AccelZ) * 180 / PI);
    AccelX = (atan2(AccelX, AccelZ) * 180 / PI);

    float dt = 0.01;
    float K = 0.9;
    float A = K / (K + dt);
  
    mixX = A *(mixX+GyroX*dt) + (1-A)*AccelY;    
    mixY = A *(mixY+GyroY*dt) + (1-A)*AccelX; 

    IMUpitch = mixY + pot3 - 1.3;       //use trim pot  + fudge factor to get 0

    Setpoint1 = 0;
    Input1 = IMUpitch;
    PID1.Compute();

    if (sw2 == 0) {
        odrive1.SetVelocity(0, 0);
        IMUpitch = pot3 - 1.3;
        Output1 = 0;

    }   

    else {
        odrive1.SetVelocity(0, Output1); 
    }




   }     // end of timed loop

}       // end  of main loop
