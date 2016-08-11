// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#define senTemp_break  32
#include "Wire.h"
//#include "AHRS.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050     mpu;

float   dt,senTemp;// meanGx = 0, meanGy = 0, meanGz = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
unsigned long t_now,t_syn=0;

size_t printFloat(float number,uint8_t digits){ 
    size_t n = 0;
    
    if (isnan(number)) return Serial.print("nan");
    if (isinf(number)) return Serial.print("inf");
  
    if (number < 0.0){
      n += Serial.print('-');
      number = -number;
    }
    unsigned long int_part = (unsigned long)number;
    float remainder = number - (float)int_part;
    n += Serial.print(int_part);
    if (digits > 0) {
      n += Serial.print("."); 
    }
    while (digits-- > 0){
      remainder *= 10.0;
      int toPrint = int(remainder);
      n += Serial.print(toPrint);
      remainder -= toPrint; 
    } 
  
    return n;
}
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



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
/*void dmpDataReady() {
    mpuInterrupt = true;
}*/


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    TWBR = 24;
    
    
    Serial.begin(115200);   
    
    
    mpu.initialize();    
    //-2229	-76	1282	-8	-64	15
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    devStatus = mpu.dmpInitialize();
    
    mpu.setXGyroOffset(-7);
    mpu.setYGyroOffset(-61);
    mpu.setZGyroOffset(6);
    mpu.setXAccelOffset(-2221);
    mpu.setYAccelOffset(-33);
    mpu.setZAccelOffset(1254); // 1688 factory default for my test chip
    //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    //mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    mpu.setTempSensorEnabled(true);
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        //mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {
    if (!dmpReady) return;
    
    while (fifoCount < packetSize){
         fifoCount = mpu.getFIFOCount();
    }
    t_now = micros();  
    dt = (t_now - t_syn)*1e-6;
    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        if(dt >= 0.050){
          //printFloat(senTemp,8); Serial.print("\t");  
          printFloat(q.w,8); Serial.print("\t");
          printFloat(q.x,8); Serial.print("\t");
          printFloat(q.y,8); Serial.print("\t");
          Serial.println(q.z,8);
          t_syn = t_now;
        }
    }
}
