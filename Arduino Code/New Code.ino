#include <Servo.h>
#include <math.h>

Servo rfh1, rfh2, rfk, lfh1, lfh2, lfk, rbh1, rbh2, rbk, lbh1, lbh2, lbk;
Servo servo_list[12] = {rfh1, rfh2, rfk, lfh1, lfh2, lfk, rbh1, rbh2, rbk, lbh1, lbh2, lbk};

#define SHOULDER_WIDTH 40 //NEEDS CALIBRATION!!!
#define L1_LENGTH 12.5 //NEEDS CALIBRATION!!!
#define L2_LENGTH 12.2 //NEEDS CALIBRATION!!!
#define WIDTH 26
#define LENGTH 17.4

#define RADIAN_TO_ANGLE 57.296

//int zero_positions[12] = {83, 110, 90, 85, 70, 90, 50, 110, 90, 130, 70, 90};
const int servo_pins[12] = {29, 28, 30, 32, 31, 33, 35, 34, 36, 38, 37, 39};
const double jointLimit[16] = {170,10,40,220,63,-117,-70,110,-72,108,85,-95,63,-117,-71,109}; //NEEDS CALIBRATION!!!
double jointLimitRad[16];

int t0;
int period = 3200;
double coordinates[8] = {0, 18, 0, 18, 0, 18, 0, 18};
int lseq[4] = {0,1,3,2};

//typedef struct frame{
//  float x;
//  float y;
//  float z;
//};
//
//typedef struct frames{
//  frame rf;
////  Frame lf;
////  Frame rb;
////  Frame lb;
//};
//
//frames local_frames; 
//
//float lfrfx = 50 ;
//float lfrfy = -90;
//float lfrfz = 0 ;

void DegToRad (double a1[16], double (&a2)[16], int len) {
  for (int i=0; i<len; i++) {
    a2[i] = a1[i]/RADIAN_TO_ANGLE;  
  }
  return;
}

double IKq2 (double x, double y) {
  double q2 = acos( 
    ((x*x + y*y) - (L1_LENGTH*L1_LENGTH + L2_LENGTH*L2_LENGTH)) / 
    (2*L1_LENGTH*L2_LENGTH) 
    );
  return q2;
}

double IKq1 (double x, double y, double q2) {
  double q1 = ( atan2(y, x) - atan2 (
    L2_LENGTH*sin(q2), (L1_LENGTH + (L2_LENGTH*cos(q2)))
    ) );
  return q1;
}

void SetPos(double xPos, double yPos, int leg) {
  double q2 = IKq2(xPos, yPos);
  double q1 = IKq1(xPos, yPos, q2);

  double mq1 = map(q1, 0, 180, jointLimitRad[0+leg*4], jointLimitRad[1+leg*4]);
  double mq2 = map(q2, 0, 180, jointLimit[2+leg*4], jointLimit[3+leg*4]);

  servo_list[0+leg*3].write(0);
  servo_list[1+leg*3].write(mq1);
  servo_list[2+leg*3].write(mq2);

  return;
}

void GaitPattern(double &xPos, double &yPos, int t, int p) {
  //updates x and y with respect to time
  if (t < (p/4)) {
    xPos = 0;
    yPos = 18; 
  }
  else if (t < (p/2)) {
    xPos = 5;
    yPos = 13;
  }
  else if (t < (3*p/4)) {
    xPos = 10;
    yPos = 18;  
  }
  
}

void setup() {
  Serial.begin(9600);
  
  //put your setup code here, to run once:
  rfk.attach(servo_pins[2]);
  rfh1.attach(servo_pins[0]);
  rfh2.attach(servo_pins[1]);
//  lfk.attach(servo_pins[3]);
//  lfh1.attach(servo_pins[4]);
//  lfh2.attach(servo_pins[5]);
//  rbk.attach(servo_pins[6]);
//  rbh1.attach(servo_pins[7]);
//  rbh2.attach(servo_pins[8]);
//  lbk.attach(servo_pins[9]);
//  lbh1.attach(servo_pins[10]);
//  lbh2.attach(servo_pins[11]);

  DegToRad(jointLimit, jointLimitRad, 16);

  delay(1000);

  t0 = millis();
}

void loop() {
  int tv, k, li;
  tv = (millis()-t0)  % (4*period);
  k = int(tv/period);
  li = lseq[k];
  
//for (int i=0; i<4; i++) {
//  setPos(0, 18, li);
//}
//
//if (tv%200 < 100) {
//  setPos(-5,13,li);
//}
//else if (100 <= tv%200 < 200) {
//  setPos(-10,18,li);
//}

//Implement movement pattern here by using SetPos() function!
  GaitPattern(coordinates[li*2], coordinates[li*2+1], tv, period);


//1: upper limb, 2: lower limb
  double q1, q2;
  q2 = IKq2(coordinates[li*2], coordinates[li*2+1]);
  q1 = IKq1(coordinates[li*2], coordinates[li*2+1], q2);

  servo_list[li*3].write(q1);
  servo_list[li*3+1].write(q2);

  Serial.print("Moving leg ");
  Serial.println(li);
  
  Serial.print("q1 = ");
  Serial.print(RADIAN_TO_ANGLE*q1);
  Serial.print(", q2 = ");
  Serial.println(RADIAN_TO_ANGLE*q2);

  Serial.print("xPos = ");
  Serial.print(coordinates[li*2]);
  Serial.print(", yPos = ");
  Serial.println(coordinates[li*2+1]);

  delay(1000);
}
