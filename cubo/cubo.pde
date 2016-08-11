/**
Visualize a cube which will assumes the orientation described
in a quaternion coming from the serial port.

INSTRUCTIONS: 
This program has to be run when you have the FreeIMU_serial
program running on your Arduino and the Arduino connected to your PC.
Remember to set the serialPort variable below to point to the name the
Arduino serial port has in your system. You can get the port using the
Arduino IDE from Tools->Serial Port: the selected entry is what you have
to use as serialPort variable.


Copyright (C) 2011-2012 Fabio Varesano - http://www.varesano.net/

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

import processing.serial.*;
import processing.opengl.*;

Serial myPort;  // Create object from Serial class

final String serialPort = "/dev/ttyACM0"; // replace this with your serial port. On windows you will need something like "COM1".

float [] q = new float [4];
float [] hq = null;
float [] Euler = new float [3]; // psi, theta, phi

int lf = 10; // 10 is '\n' in ASCII
byte[] inBuffer = new byte[22]; // this is the number of chars on each line from the Arduino (including /r/n)

PFont font;
final int VIEW_SIZE_X = 800, VIEW_SIZE_Y = 600; //isto dá merda

final int burst = 32;
int count = 0;

void myDelay(int time) {
  try {
    Thread.sleep(time);
  } catch (InterruptedException e) { }
}

void setup() 
{
  size(800,600,P3D);
  myPort = new Serial(this, serialPort, 115200);
  
  // The font must be located in the sketch's "data" directory to load successfully
  font = loadFont("CourierNew36.vlw"); 
}

void serialEvent(Serial p) {
  if(p.available() > 0) {
    String inputString = p.readStringUntil('\n');
    if (inputString != null && inputString.length() > 0) {
        String [] inputStringArr = split(inputString, "\t");
      
        if(inputStringArr.length >= 4) { // q1,q2,q3,q4,\r\n so we have 5 elements ???
          for(int n = 0; n < inputStringArr.length; n++)
            q[n] = Float.parseFloat(inputStringArr[n]);
      }
    }
  }
}



void buildBoxShape() {
  //box(60, 10, 40);
  noStroke();
  beginShape(QUADS);
  
  //Z+ (to the drawing area)
  fill(#00ff00);
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  //Z-
  fill(#0000ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);
  
  //X-
  fill(#ff0000);
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);
  
  //X+
  fill(#ffff00);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);
  
  //Y-
  fill(#ff00ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);
  
  //Y+
  fill(#00ffff);
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  endShape();
}


void drawCube() {  
  pushMatrix();
    translate(VIEW_SIZE_X/2, VIEW_SIZE_Y/2 + 50, 0);
    scale(5,5,5);
    
    // a demonstration of the following is at 
    // http://www.varesano.net/blog/fabio/ahrs-sensor-fusion-orientation-filter-3d-graphical-rotating-cube
    rotateZ(-Euler[2]);
    rotateX(-Euler[1]);
    rotateY(-Euler[0]);
    
    buildBoxShape();
    
  popMatrix();
}


void draw() {
  background(#000000);
  fill(#ffffff);
  
  if(hq != null) { // use home quaternion
    quaternionToEuler(quatProd(hq, q), Euler);
    text("Disable home position by pressing \"n\"", 20, VIEW_SIZE_Y - 30);
  }
  else {
    quaternionToEuler(q, Euler);
    text("Point FreeIMU's X axis to your monitor then press \"h\"", 20, VIEW_SIZE_Y - 30);
  }
  
  textFont(font, 20);
  textAlign(LEFT, TOP);
  text("Q:\n" + q[0] + "\n" + q[1] + "\n" + q[2] + "\n" + q[3], 20, 20);
  text("Euler Angles:\nYaw (psi)  : " + degrees(Euler[0]) + "\nPitch (theta): " + degrees(Euler[1]) + "\nRoll (phi)  : " + degrees(Euler[2]), 200, 20);
  
  drawCube();
  //myPort.write("q" + 1);
}


void keyPressed() {
  if(key == 'h') {
    println("pressed h");
    
    // set hq the home quaternion as the quatnion conjugate coming from the sensor fusion
    hq = quatConjugate(q);
    
  }
  else if(key == 'n') {
    println("pressed n");
    hq = null;
  }
}

// See Sebastian O.H. Madwick report 
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation

void quaternionToEuler(float [] q, float [] euler) {
  euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

float [] quatProd(float [] a, float [] b) {
  float [] q = new float[4];
  
  q[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  q[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  q[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  q[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
  
  return q;
}

// returns a quaternion from an axis angle representation
float [] quatAxisAngle(float [] axis, float angle) {
  float [] q = new float[4];
  
  float halfAngle = angle / 2.0;
  float sinHalfAngle = sin(halfAngle);
  q[0] = cos(halfAngle);
  q[1] = -axis[0] * sinHalfAngle;
  q[2] = -axis[1] * sinHalfAngle;
  q[3] = -axis[2] * sinHalfAngle;
  
  return q;
}

// return the quaternion conjugate of quat
float [] quatConjugate(float [] quat) {
  float [] conj = new float[4];
  
  conj[0] = quat[0];
  conj[1] = -quat[1];
  conj[2] = -quat[2];
  conj[3] = -quat[3];
  
  return conj;
}