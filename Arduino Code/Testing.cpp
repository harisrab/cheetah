// Testing Parabolic Foot Movement with a Leg
// Created by Shaheer Ziya & Bryan Lam

#include <cmath>
#include <fstream>
#include <iostream>
using namespace std;

#define FOOT_ARC_HEIGHT 8
#define NUM_STEPS 4
#define INIT_FOOT_HEIGHT 18

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


int main()
{
  // Open the file to store the data in
  ofstream testPar;
  testPar.open("testPar.txt");
  if (testPar.fail())
  {
    cout << "Error opening file" << endl;
    return 1;
  }
  
  // Random Variables
  int tv, k, li;
  // Initial Position of Doggie's Foot With Shoulder Bases
  double x = 0, y = INIT_FOOT_HEIGHT;
  // Simulate Void Loop()
  for (int i = 0; i < 12800; i++)
  {
    tv = i;
    k = int(tv / 3200);
    GaitPattern(x, y, k);

    testPar << x << ", " << y << endl;
  }

  
  // Close the file
  testPar.close();
  
  return 0;
}