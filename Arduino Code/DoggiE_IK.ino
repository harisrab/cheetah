#include <Servo.h>
#include <math.h>

Servo rfk, h1, rfh2, lfk, lfh1, lfh2, rbk, rbh1, rbh2, lbk, lbh1, lbh2;
Servo servo_list[12] = {rfk, h1, rfh2, lfk, lfh1, lfh2, rbk, rbh1, rbh2, lbk, lbh1, lbh2};

#define SHOULDER_WIDTH = 40 //NEEDS CALIBRATION!!!
#define L1_LENGTH = 12.5 //NEEDS CALIBRATION!!!
#define L2_LENGTH = 12.2 //NEEDS CALIBRATION!!!
#define WIDTH = 26
#define LENGTH = 17.4

#define RADIAN_TO_ANGLE 57.296

#define FOOT_ARC_HEIGHT 8.0
#define INIT_FOOT_HEIGHT 18

//int zero_positions[12] = {83, 110, 90, 85, 70, 90, 50, 110, 90, 130, 70, 90};
const int servo_pins[12] = {28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39};
const double jointLimit[16] = {-60,120,74,-106,63,-117,-70,110,-72,108,85,-95,63,-117,-71,109}; //NEEDS CALIBRATION!!!
const double jointLimitRad[16];

double t0;
double walkLoopSpd = 1;
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

void DegToRad (double a1[], double (&a2)[], int len) {
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

double parabola(double x)
{
  // FOOT_HEIGHT is the height of the Parabola (How high the foot is off the ground)
  // f(x) = -16/(9D)[x+3D/8]^2 + (D/4)
  return (FOOT_ARC_HEIGHT / 4) - (16 / (9 * FOOT_ARC_HEIGHT)) * (x * x);
}

void GaitPattern(double &xPos, double &yPos, int t)
{
  // updates x and y with respect to time

  // x(t) = 3D/8 * (1 - x/100) # Where  is t / (NUM_STEPS/2) gives the linear relation between x and t
  double dx = (3.0 * FOOT_ARC_HEIGHT / 8.0) * (1 - (t / (NUM_STEPS / 2.0)));

  // cout << "(" << t << ", " << dx << ")" << endl;

  xPos = dx;
  yPos = INIT_FOOT_HEIGHT + parabola(dx);

  // cout << dx << ", " << yPos << endl;
}

void setup() {
  Serial.begin(9600);
  
  //put your setup code here, to run once:
  rfk.attach(servo_pins[0]);
  rfh1.attach(servo_pins[1]);
  rfh2.attach(servo_pins[2]);
  lfk.attach(servo_pins[3]);
  lfh1.attach(servo_pins[4]);
  lfh2.attach(servo_pins[5]);
  rbk.attach(servo_pins[6]);
  rbh1.attach(servo_pins[7]);
  rbh2.attach(servo_pins[8]);
  lbk.attach(servo_pins[9]);
  lbh1.attach(servo_pins[10]);
  lbh2.attach(servo_pins[11]);

  DegToRad(jointLimit, jointLimitRad, 16);

  delay(1000);

  t0 = millis();
}

void loop() {
  int tv, k, li;
  // Gives us how far we far in the 800ms cycle
  tv = int((time.time() - t0)  % 12800);
  // The k-th occurrence of 3200
  k = int(tv/3200);
  // li = lseq[k];
  
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

  GaitPattern(coordinates[li*2], coordinates[li*2+1], tv);

  // delay(1000);
}
