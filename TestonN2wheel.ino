#include "PID_v1.h"
#include <ESP32Servo.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#define SD_CS 5
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "encodeSpeed.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;


int16_t ax,ay,az;
int16_t gx,gy,gz;
int16_t accData[3],gyrData[3];



Servo ESC;

int neutral = 1488; 
int fullForward = 1832; 
int fullReverse = 1312;


//long timeCur, timePrev, timeStart; 
double Input;
static double Output;
double Setpoint = 0;
float Kp = 2.0 ;
float Ki = 0.0;
float Kd = 0.0;
float change=0;
float prevInput=0;

//parameters used in determination of the motor speed

//int InterruptPin=25;

//in finding the rocket roll velocity values
#define sampleTime 10
double elapsedTime = 0;
double  timeCur = 0;
double timePrev = 0;
double anglePrev = 0;
double angleCur = 0;
double _speed = 0;

//variables that hold the SD logged values
double rollVel=0;
double pwm = 0;


    //response time parameters  
double distimecur=0;
double respondedtime=0;
double responsetime=0;
double input=0;

//-------variablesmotor-----

//String dataMessage;
//File file;


PID balancePID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);

#define OUTPUT_READABLE_YAWPITCHROLL //outputs yaw,pitch and roll

#define INTERRUPT_PIN 2
#define LED_PIN 22 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define SDA 21
#define SCL 22
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in  
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//void dmpDataReady() {
//    mpuInterrupt = true;
//}

void write_pwm(float _speed){
  ESC.writeMicroseconds(_speed);  //returns a pulse of width (_speed)microseconds every 20 milliseconds
 
}


double constrainpwm (double pwm, double Min, double Max){
  if   (pwm< Min) return Min;
  else if    (pwm> Max) return Max;

  else return pwm;

}

void calibrateESC () {
   //Serial.println("Calibration procedure for Mamba ESC.");
  //Serial.println("Turn on ESC.");
  ESC.writeMicroseconds(0);
  //Serial.println("Starting Calibration.");
  delay(1000);
  ESC.writeMicroseconds(1832);
  //Serial.println("Writing Full Throttle.");
  delay(1000);
  ESC.writeMicroseconds(1312);
  //Serial.println("Writing Full Reverse.");
  delay(1000);
  ESC.writeMicroseconds(1488);
 // Serial.println("Writing Neutral.");
  delay(1000);
  //Serial.println("Calibration Complete.");
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
 // setCpuFrequencyMhz(160);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(SDA, SCL, 400000);
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
Serial.begin(115200);
   //pinMode(InterruptPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(InterruptPin),stepsCounter, RISING); //count pulses using interrupts from the motor
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);  

    // verify connection
   
    devStatus = mpu.dmpInitialize();//INITIALISING

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //.println(F(")..."));
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 

    ESC.attach(14);  //49
    balancePID.SetMode(AUTOMATIC); //
    balancePID.SetOutputLimits(-176,344);//to range from 1312 to 1832
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    //calibrateESC();
//     revCounts = 0;

//

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything

    unsigned long start = millis();
     //read_speed();

    if (!dmpReady) return;
    // read a packet from FIFO
    if ((mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) && (elapsedTime > sampleTime));  { // Get the Latest packet 
              // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

         mpu.getMotion6(&accData[0], &accData[1], &accData[2], &gyrData[0], &gyrData[1], &gyrData[2]);
         
        Input = ypr[0] * 180/M_PI; 
        timePrev = timeCur;
        timeCur = millis();
        angleCur=ypr[0];
        change=prevInput-Input;
        prevInput=Input;
        Serial.println("CHANGE");
            Serial.println(change);
        elapsedTime = timeCur - timePrev;
        rollVel = (((angleCur-anglePrev)/(elapsedTime/1000.00))*(30/M_PI));//to convert the velocity to rpm multiply the value by (60/2*M_PI)
//             Serial.println("rollVel  ");
//            Serial.println(rollVel);
                  
            
            while ( Input<= -180) Input += 360; 
            while (Input > 180)   Input  -= 360;
       
//          Serial.print("Input:\t");
//          Serial.println(Input);
     

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        Setpoint = 0;  
        if(int(Input)!=0){     
        if(change < 0){
        balancePID.SetControllerDirection(DIRECT);
        }
        else{
        balancePID.SetControllerDirection(REVERSE);
        }
        }   
        balancePID.Compute(); 

         
           timePrev = timeCur;
           anglePrev = angleCur;
          
        pwm=1488+Output;
        constrainpwm(pwm,1312,1832);
      
        write_pwm(pwm);
        
        delay(10);
            
//      
//    if((int) input!=0){  
//    distimecur=millis();
//    Serial.print("distimecur");
//    Serial.print(distimecur);
//    if((int) pwm==1488){
//      respondedtime=distimecur;
//       responsetime=respondedtime-distimecur;
//            Serial.print("responsetime");
//            Serial.print(responsetime);
//            }
//            
//        }

    }
    unsigned long endtime = millis();
    float freq = 1000 / (endtime - start);
//    Serial.print("rate: ");
//    Serial.println(freq);
    
}
