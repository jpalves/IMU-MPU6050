#include <math.h>
#define GRAU2RAD 0.01745329251 

float invSqrt(float x);

class MahonyAHRS{
  		float q0, q1, q2, q3;
                float qw, qx, qy, qz;
		float T, twoKp, twoKi;
                float integralFBx,  integralFBy, integralFBz;
	public:
    		MahonyAHRS(){
                        integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
                        qw = 0, qx = 0, qy=0, qz = 0;
                }
                void mahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);
		inline float &TAmostragem(){
			return T;	
		}
		inline float &getQ0(){
			return q0;		
		}
		inline float &getQ1(){
			return q1;		
		}
		inline float &getQ2(){
			return q2;		
		}
		inline float &getQ3(){
			return q3;		
		}
		inline float &Kp(){
			return twoKp;
		}
		inline float &Ki(){
			return twoKi;
		}
};

