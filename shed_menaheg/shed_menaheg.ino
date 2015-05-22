/*
shed menaheg is based on inimu-9-ahrs-arduino by  kevin-pololu:
https://github.com/pololu/minimu-9-ahrs-arduino

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

shed menaheg is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

 */

// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the left 
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>
#include <Servo.h>
#include <L3G.h>
#include <LSM303.h>


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// AHRS declarations ////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -2638
#define M_Y_MIN -2475
#define M_Z_MIN -2692
#define M_X_MAX 2516
#define M_Y_MAX 2022
#define M_Z_MAX 1625

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

L3G gyro;
LSM303 compass;

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
    ,{
      0,1,0  }
      ,{
        0,0,1  }
      }; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
    ,{
      0,0,0  }
      ,{
        0,0,0  }
      };
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// shed menaheg declarations ////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PARACHUTE_SERVO_PIN 11
#define PARACHUTE_CLOSED_POS 170
#define PARACHUTE_RELEASE_POS 30
#define NAVIGATION_SERVO_1_PIN 5
#define NAVIGATION_SERVO_2_PIN 6
#define NAVIGATION_SERVO_3_PIN 9
#define NAVIGATION_SERVO_4_PIN 10
#define NAVIGATION_SERVO_1_START_POS 90
#define NAVIGATION_SERVO_2_START_POS 90
#define NAVIGATION_SERVO_3_START_POS 90
#define NAVIGATION_SERVO_4_START_POS 90
#define NAVIGATION_SERVO_1_STRAIGHT_POS 90
#define NAVIGATION_SERVO_2_STRAIGHT_POS 90
#define NAVIGATION_SERVO_3_STRAIGHT_POS 90
#define NAVIGATION_SERVO_4_STRAIGHT_POS 90
#define NAVIGATION_SERVO_1_MIN_POS 1
#define NAVIGATION_SERVO_2_MIN_POS 1
#define NAVIGATION_SERVO_3_MIN_POS 1
#define NAVIGATION_SERVO_4_MIN_POS 1
#define NAVIGATION_SERVO_1_MAX_POS 180
#define NAVIGATION_SERVO_2_MAX_POS 180
#define NAVIGATION_SERVO_3_MAX_POS 180
#define NAVIGATION_SERVO_4_MAX_POS 180
#define PARACHUTE_RELEASE_TIME 200

#define LAUNCH_DETECTION_THRESHHOLD 7 // in g, 8 g full scale

Servo parachute_servo;
Servo navigation_servo_1;
Servo navigation_servo_2;
Servo navigation_servo_3;
Servo navigation_servo_4;



enum Missile_modes_t {
  IN_PREFLIGHT_MODE, 
  IN_FLIGHT_MODE, 
  IN_PARACHUTE_MODE
};

Missile_modes_t missile_status = IN_PREFLIGHT_MODE;
//Missile_modes_t missile_status = IN_FLIGHT_MODE;

int Parachute_release_couner = 0;
bool flag = false;
float roll_init, pitch_init, yaw_init;
byte Srevo_1_new_pos, Srevo_2_new_pos, Srevo_3_new_pos, Srevo_4_new_pos;


void releas_parachute(){
  if(missile_status == IN_FLIGHT_MODE){
    parachute_servo.write(PARACHUTE_RELEASE_POS);
    timer=millis();
    missile_status = IN_PARACHUTE_MODE; 
 }
}

void launch_detection_init(){

}

bool launch_detected(){

  compass.readAcc();
  if(abs(((compass.a.x >> 4)/256.0)) >= LAUNCH_DETECTION_THRESHHOLD){ // shift left 4 bits to use 12-bit representation (1 g = 256)
    return (true);
  } 

  return (false);
}

void Calculate_heading(){
  // *** DCM algorithm
  // Data adquisition
  Read_Gyro();   // This read gyro data
  Read_Accel();     // Read I2C accelerometer

  if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
  {
    counter=0;
    Read_Compass();    // Read I2C magnetometer
    Compass_Heading(); // Calculate magnetic heading  
  }

  // Calculations...
  Matrix_update(); 
  Normalize();
  Drift_correction();
  Euler_angles();
}

void Calc_new_navigation_servos_pos(){

}

void Update_navigation_servos(){
  navigation_servo_1.write(Srevo_1_new_pos);
  navigation_servo_2.write(Srevo_2_new_pos);
  navigation_servo_3.write(Srevo_3_new_pos);
  navigation_servo_4.write(Srevo_4_new_pos);
}

void setup()
{ 
  Serial.begin(9600);

  parachute_servo.attach(PARACHUTE_SERVO_PIN);
  navigation_servo_1.attach(NAVIGATION_SERVO_1_PIN);
  navigation_servo_2.attach(NAVIGATION_SERVO_2_PIN);
  navigation_servo_3.attach(NAVIGATION_SERVO_3_PIN);
  navigation_servo_4.attach(NAVIGATION_SERVO_4_PIN);

  Srevo_1_new_pos = NAVIGATION_SERVO_1_START_POS;
  Srevo_2_new_pos = NAVIGATION_SERVO_2_START_POS;
  Srevo_3_new_pos = NAVIGATION_SERVO_3_START_POS;
  Srevo_4_new_pos = NAVIGATION_SERVO_4_START_POS;

  parachute_servo.write(PARACHUTE_CLOSED_POS);
  Update_navigation_servos();




  I2C_Init();

  Serial.println("shed menaheg C&C");

  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for(int i=0;i<32;i++)    // We take some readings...
  {

  //Read_Gyro();
  Read_Accel();
  for(int y=0; y<6; y++)   // Cumulate values
  AN_OFFSET[y] += AN[y];
  delay(20);
  }

  for(int y=0; y<6; y++)
  AN_OFFSET[y] = AN_OFFSET[y]/32;

  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];


  delay(2000);


}

void loop() //Main Loop
{

  if(missile_status == IN_PREFLIGHT_MODE){

    if(launch_detected()){
      missile_status = IN_FLIGHT_MODE;
      timer= millis() - 20; // switch to in_Flight_mode immediately
    }

  }

  if(missile_status == IN_FLIGHT_MODE){
     if((millis()-timer)>=20)  // Main loop runs at 50Hz
     {
      if(Parachute_release_couner <= PARACHUTE_RELEASE_TIME ){ // wait 2 sec in IN_FLIGHT_MODE before releasing the parachute
          releas_parachute();
      }
      else{
        Parachute_release_couner++;
      }
      counter++;
      timer_old = timer;
      timer=millis();
      if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      else
      G_Dt = 0;

      Calculate_heading();
      Calc_new_navigation_servos_pos();
      Update_navigation_servos();

      if(!flag){
        roll_init = ToDeg(roll);
        pitch_init = ToDeg(pitch);
        yaw_init = ToDeg(yaw);
        flag = true;
      }
      

      printdata();
    } 
  }
  if(missile_status == IN_PARACHUTE_MODE){
    if((millis()-timer)>=2000){ // wait one sec
      parachute_servo.write(PARACHUTE_CLOSED_POS);
    }
    
  }


}
