#ifndef EXP_THRUSTER_MODEL_H
#define EXP_THRUSTER_MODEL_H
// here we have thruster model
#include "functionlib/utilts/vector.h"

double girona_poly_nominal_model(double input){
    double output = 0.0;
    if (input >= 0.0) {
        // Forward thrust polynomial (f >= 0)
        constexpr double a0 =  0.0473235;
        constexpr double a1 =  0.063145;
        constexpr double a2 = -0.00256629;
        constexpr double a3 =  0.000052432528;
        constexpr double a4 = -0.00000050454768;
        constexpr double a5 =  0.000000001842152;
        const double f = input;
        output = a0 + a1*f + a2*f*f + a3*f*f*f + a4*f*f*f*f + a5*f*f*f*f*f;
    } else {
        // Reverse thrust polynomial (f < 0): evaluate on |f|, then negate
        constexpr double b0 =  0.04997604;
        constexpr double b1 =  0.05975017;
        constexpr double b2 = -0.0022952;
        constexpr double b3 =  0.0000439588388;
        constexpr double b4 = -0.00000039543457;
        constexpr double b5 =  0.00000000134487;
        const double f = -input;
        output = -(b0 + b1*f + b2*f*f + b3*f*f*f + b4*f*f*f*f + b5*f*f*f*f*f);
    }
    return output;
}

// as the horizontal thrusters are stronger, 
// so we now do a calibration of the horizontal thrusters
double horizontal_thruster_calibration(double force_expected){
  return 0.652*force_expected + 0.814;
}

inline sfc::Vector<6> convertThrustsToSetpoints(const sfc::Vector<6> force){
  sfc::Vector<6> setpoints{};
  for(int8_t i=0; i<6;i++){
    // horizontal thrusters
    if(i<4){
      setpoints(i) = girona_poly_nominal_model(horizontal_thruster_calibration(force(i)));
    }else{
      setpoints(i) = girona_poly_nominal_model(force(i));
    }
    
  }
  return setpoints;
}


#endif

