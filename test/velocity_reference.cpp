#include "VelocityReference.h"
#include <cstdlib>
#include <iostream>
void require(bool ok,const char* message) {
  if (!ok) {std::cerr<<"FAIL: "<<message<<'\n';std::exit(1);}
}
int main() {
  for (float goal : {-3.f,-2.f,-1.f,0.f,1.f,2.f,3.f}) {
    require(velocityReference(goal,0,800000)==0,"ramp begins at zero");
    require(velocityReference(goal,400000,800000)==goal*.5f,"half ramp and sign");
    require(velocityReference(goal,800000,800000)==goal,"exact endpoint");
    require(velocityReference(goal,3000000,800000)==goal,"hold through run end");
    require(velocityReference(goal,0,0)==goal,"old step-reference profiles unchanged");
    for (uint32_t us=0;us<=3000000;us+=997)
      require(std::abs(velocityReference(goal,us,800000))<=std::abs(goal),"no reference overshoot");
  }
  require(!std::isfinite(velocityReference(NAN,0,800000)),"nonfinite goal propagates");
  require(!std::isfinite(velocityReference(INFINITY,800000,800000)),"infinite goal propagates");
  for (float goal : {-3.f,-2.5f,2.5f,3.f}) {
    require(velocityReference(goal,750000,1500000)==goal*.5f,"1.5s ramp midpoint and direction");
    require(velocityReference(goal,1500000,1500000)==goal,"1.5s ramp exact endpoint");
    for (uint32_t us=0;us<=3000000;us+=997)
      require(std::abs(velocityReference(goal,us,1500000))<=std::abs(goal),"1.5s reference has no overshoot");
  }
  std::cout<<"PASS: signed ramp, zero, endpoint, constant goal, unchanged step, invalid goals\n";
}
