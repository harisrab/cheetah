// Parabolic Movement Implementer
// Created by Shaheer Ziya

#include <iostream>
#include <cmath>
#include <iomanip>
using namespace std;

// Function for calculating the height of the parabola at a given x (away from origin)
double parabola(double x, double D)
{
  // f(x) = -16/(9D)[x+3D/8]^2 + (D/4)
  return (D/4) - (16/(9*D))*(x+3*D/8)*(x+3*D/8);
}


int main()
{
  // Height of the parabola
  double D = 2.0;
  // Number of Steps
  double stepSize = 100.0;
  
  // Starting point 
  int a0 = 0; 
  
  cout << fixed << setprecision(3);

  for (int i=0; i <= stepSize; i++)
  {
    
    double dx =  (3*D/4) * i/stepSize;
    cout << "(" << dx << ", " << parabola(-(a0 + dx), D) << ")" << endl;
  } 
  
  
  
  
  
  return 0; 
}