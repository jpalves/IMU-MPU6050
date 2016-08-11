// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#define senTemp_break  32
#include "Wire.h"
#include "AHRS.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
MahonyAHRS  a;
float   dt,senTemp,meanGx = 0, meanGy = 0, meanGz = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
unsigned long t_now,t_syn=0,n=2000;

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


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    TWBR = 24;
    Serial.begin(115200);   
    accelgyro.initialize();    
    //-2221	-33	1254	-7	-61	6
    accelgyro.setXGyroOffset(2);
    accelgyro.setYGyroOffset(-59);
    accelgyro.setZGyroOffset(7);
    accelgyro.setXAccelOffset(-2150);
    accelgyro.setYAccelOffset(-6);
    accelgyro.setZAccelOffset(1267); // 1688 factory default for my test chip
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    accelgyro.setTempSensorEnabled(true);
    a.getQ0() = 1;
    a.getQ1() = 0;
    a.getQ2() = 0;
    a.getQ3() = 0;
    a.Kp() = 2*3; 
    a.Ki() = 0.00025;//2*0.4;
    for (int i = 0; i < n ; i ++){                  //Run this code 2000 times
      //if(i % 125 == 0) Serial.print(".");
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      meanGx += (float)gx;
      meanGy += (float)gy;
      meanGz += (float)gz;
    }
    meanGx /= n;
    meanGy /= n;
    meanGz /= n;
    //Serial.println();
}

void loop() {
    t_now = micros();  
    dt = (t_now - t_syn)*1e-6;
    if(dt >= 0.070){
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        a.TAmostragem() = dt;
        senTemp = accelgyro.getTemperature()/340+36.53;
        
        a.mahonyAHRSupdate(((float)gx-meanGx)/32.8*GRAU2RAD,((float)gy-meanGy)/32.8*GRAU2RAD,
                           ((float)gz-meanGz)/32.8*GRAU2RAD,(float)ax/8192,(float)ay/8192,(float)az/8192);  
        //printFloat(senTemp,8); Serial.print("\t");  
        printFloat(a.getQ0(),8); Serial.print("\t");
        printFloat(a.getQ1(),8); Serial.print("\t");
        printFloat(a.getQ2(),8); Serial.print("\t");
        Serial.println(a.getQ3(),8);
        t_syn = t_now;
      }
}
