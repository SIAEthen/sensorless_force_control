#ifndef EXP_UTILTS_H
#define EXP_UTILTS_H
// here we have thruster model
#include "functionlib/utilts/vector.h"
#include "functionlib/utilts/rotation.h"
#include "functionlib/utilts/linear_algebra.h"

inline void velcmd2configurations(const sfc::Vector6& vel_cmd,
                                  sfc::Vector3& x_ee_d,
                                  sfc::Quaternion& q_ee_d,
                                  sfc::Real dt) {
  if (!vel_cmd.isFinite() || !x_ee_d.isFinite() || !sfc::isFinite(dt) || dt <= sfc::zero()) {
    throw std::runtime_error("velcmd2configurations: invalid input");
  }
  if (!std::isfinite(q_ee_d.w) || !std::isfinite(q_ee_d.x) ||
      !std::isfinite(q_ee_d.y) || !std::isfinite(q_ee_d.z)) {
    throw std::runtime_error("velcmd2configurations: invalid quaternion");
  }

  // Integrate position.
  x_ee_d(0) += vel_cmd(0) * dt;
  x_ee_d(1) += vel_cmd(1) * dt;
  x_ee_d(2) += vel_cmd(2) * dt;

  // Integrate orientation from angular velocity [rx, ry, rz].
  const sfc::Vector3 omega{vel_cmd(3), vel_cmd(4), vel_cmd(5)};
  const sfc::Real omega_norm = sfc::vectorNorm(omega);

  sfc::Quaternion dq{};
  if (omega_norm > static_cast<sfc::Real>(1e-6)) {
    const sfc::Real half_theta = static_cast<sfc::Real>(0.5) * omega_norm * dt;
    const sfc::Real sin_half = std::sin(half_theta);
    const sfc::Real cos_half = std::cos(half_theta);
    const sfc::Real scale = sin_half / omega_norm;
    dq.w = cos_half;
    dq.x = omega(0) * scale;
    dq.y = omega(1) * scale;
    dq.z = omega(2) * scale;
  } else {
    const sfc::Real scale = static_cast<sfc::Real>(0.5) * dt;
    dq.w = static_cast<sfc::Real>(1.0);
    dq.x = omega(0) * scale;
    dq.y = omega(1) * scale;
    dq.z = omega(2) * scale;
  }

  // post multiply means rotate within local frame, not inertia frame.
  q_ee_d = (q_ee_d * dq).normalized();
}


// we need to detect if they are any collisions
inline sfc::Vector<6> getRandomJointPosition(){
  sfc::Vector<6> joint_position{};
  sfc::Vector<6> min{-2.5*sfc::kPi4,      0.0,      0.0,  -2*sfc::kPi4, -sfc::kPi2,            0.0};
  sfc::Vector<6> max{2.5*sfc::kPi4, sfc::kPi2,sfc::kPi2,   2*sfc::kPi4,        0.0,   3.5*sfc::kPi4};
  bool collision = true;
  while(collision){
    // 1 sample
    for(int8_t i=0; i<6;i++){
      double r = static_cast<double>(rand()%100)/100.0;
      joint_position(i) = r*(max(i)-min(i)) + min(i);
    }
    // 2 judge if collision
    collision = false; //never collision
  }
  
  return joint_position;
}
inline sfc::Real getRandomAngle(sfc::Real min,sfc::Real max){
  sfc::Real angle;
  sfc::Real r = static_cast<sfc::Real>(rand()%100)/100.0;
  angle = r*(max-min) + min;
  return angle;
}

#endif

