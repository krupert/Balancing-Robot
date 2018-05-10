
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
int mode = 1;
#define PRINT_TO_SERIAL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
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

// Motors
const int Motor1_DIR = 7;
const int Motor2_DIR = 8;
const int Motor1_PWM = 9;
const int Motor2_PWM = 10;

float speed;

//PID
double originalSetpoint = 0;//173;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//adjust these values to fit your own design
double Kp = 15;   
double Kd = 0;//1.4;
double Ki = 0;//25;//60;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
float threshold = 55;
void set_speed(float vel) {
  speed = abs(((255-threshold)/100)*abs(vel)+threshold);
  if (vel > 0) {
    digitalWrite(Motor1_DIR, HIGH);
    digitalWrite(Motor2_DIR, HIGH);
    analogWrite(Motor1_PWM, speed);
    analogWrite(Motor2_PWM, speed);
  }
  else if (vel < 0) {
    digitalWrite(Motor1_DIR, LOW);
    digitalWrite(Motor2_DIR, LOW);
    analogWrite(Motor1_PWM, speed);
    analogWrite(Motor2_PWM, speed);
  }
  else {
    digitalWrite(Motor1_DIR, HIGH);
    digitalWrite(Motor2_DIR, HIGH);
    analogWrite(Motor1_PWM, 0);
    analogWrite(Motor2_PWM, 0);
  }
}


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();

    #ifdef PRINT_TO_SERIAL
        Serial.begin(115200);
        while (!Serial); // wait for Leonardo enumeration, others continue immediately

        // verify connection
        Serial.println(F("Testing device connections..."));
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
        // wait for ready
        Serial.println(F("\nSend any character to begin: "));
        while (Serial.available() && Serial.read()); // empty buffer
        while (!Serial.available());                 // wait for data
        while (Serial.available() && Serial.read()); // empty buffer again
    
        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
    #endif
    
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-35);
    mpu.setYGyroOffset(-124);
    mpu.setZGyroOffset(28);
    mpu.setZAccelOffset(1798); // 1688 factory default for my test chip
    mpu.setYAccelOffset(3040);
    mpu.setXAccelOffset(-1451);
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed, 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        #ifdef PRINT_TO_SERIAL
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
        #endif
    }

    // configure motor PWM
    pinMode(Motor1_PWM, OUTPUT);
    pinMode(Motor2_PWM, OUTPUT);
    speed = 0;
    
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-100, 100); 
      
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();
        set_speed(output);
        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        #ifdef PRINT_TO_SERIAL
            Serial.print("Output:\t");
            Serial.print(output);
            Serial.print("Gyro:\t");
            //Serial.print(euler[0] * 180/M_PI);
            //Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            //Serial.println(euler[2] * 180/M_PI);
            Serial.print("Kp:\t");
            Serial.print(Kp);
            Serial.print("Ki:\t");
            Serial.print(Ki);
            Serial.print("Kd:\t");
            Serial.println(Kd);
            
            
        #endif

        input = euler[1] * 180/M_PI;
  
    
    double newKp = 0;
    if (Serial.available() > 0) {
      newKp = Serial.parseFloat();
      Serial.print("k set to: ");
      Serial.println(newKp);
      if (newKp > 0){
        if (mode == 1){
          Kp = newKp;
        }
        else if (mode == 2){
          Ki=newKp;
        }
        else if (mode == 3){
          Kd = newKp;
        }
        pid.SetTunings(Kp, Ki, Kd);
      }
      else if (newKp == -1 or newKp == -2 or newKp == -3){
        mode = newKp*-1;
      }
    }
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        
    }
}
