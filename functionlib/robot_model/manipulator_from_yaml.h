#ifndef SFC_MANIPULATOR_FROM_YAML_H_
#define SFC_MANIPULATOR_FROM_YAML_H_

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "manipulator_base.h"
#include "utilts/rotation.h"

namespace sfc {

namespace detail {

inline sfc::Vector3 normalizedAxis(const sfc::Vector3& axis) {
  const sfc::Real n = std::sqrt(axis(0) * axis(0) + axis(1) * axis(1) + axis(2) * axis(2));
  if (n <= static_cast<sfc::Real>(1e-12)) {
    return sfc::Vector3{0.0, 0.0, 1.0};
  }
  return sfc::Vector3{axis(0) / n, axis(1) / n, axis(2) / n};
}

inline sfc::RotationMatrix rotationFromAxisAngle(const sfc::Vector3& axis,
                                                 sfc::Real angle) {
  const sfc::Vector3 a = normalizedAxis(axis);
  const sfc::Real x = a(0);
  const sfc::Real y = a(1);
  const sfc::Real z = a(2);
  const sfc::Real c = std::cos(angle);
  const sfc::Real s = std::sin(angle);
  const sfc::Real v = static_cast<sfc::Real>(1.0) - c;

  sfc::RotationMatrix r{};
  r(0, 0) = x * x * v + c;
  r(0, 1) = x * y * v - z * s;
  r(0, 2) = x * z * v + y * s;
  r(1, 0) = y * x * v + z * s;
  r(1, 1) = y * y * v + c;
  r(1, 2) = y * z * v - x * s;
  r(2, 0) = z * x * v - y * s;
  r(2, 1) = z * y * v + x * s;
  r(2, 2) = z * z * v + c;
  return r;
}

inline sfc::HomogeneousMatrix transformFromOrigin(const sfc::Vector3& xyz,
                                                  const sfc::Vector3& rpy) {
  const sfc::RotationMatrix r = sfc::RotationMatrix::fromRPY(rpy);
  return sfc::HomogeneousMatrix::fromRotationTranslation(r, xyz);
}

inline sfc::HomogeneousMatrix transformFromAxisTranslation(const sfc::Vector3& axis,
                                                           sfc::Real distance) {
  const sfc::Vector3 a = normalizedAxis(axis);
  const sfc::Vector3 p{a(0) * distance, a(1) * distance, a(2) * distance};
  return sfc::HomogeneousMatrix::fromRotationTranslation(sfc::RotationMatrix::identity(), p);
}

inline sfc::Vector3 parseVector3(const YAML::Node& node, const std::string& field) {
  if (!node[field] || !node[field].IsSequence() || node[field].size() != 3) {
    throw std::runtime_error("Invalid or missing field: " + field);
  }
  return sfc::Vector3{static_cast<sfc::Real>(node[field][0].as<double>()),
                      static_cast<sfc::Real>(node[field][1].as<double>()),
                      static_cast<sfc::Real>(node[field][2].as<double>())};
}

}  // namespace detail

enum class JointMotionType {
  kFixed,
  kRevolute,
  kPrismatic
};

struct JointOrigin {
  sfc::Vector3 xyz{0.0, 0.0, 0.0};
  sfc::Vector3 rpy{0.0, 0.0, 0.0};
};

struct JointParam {
  JointMotionType type{JointMotionType::kFixed};
  sfc::Vector3 axis{0.0, 0.0, 1.0};
  JointOrigin origin{};
};

template <std::size_t Dof>
class ManipulatorFromYAML : public ManipulatorBase<Dof> {
 public:
  using Base = ManipulatorBase<Dof>;
  using Jacobian = typename Base::Jacobian;

  explicit ManipulatorFromYAML(std::string name)
      : Base(std::move(name)) {}

  void setParametersFromFile(const std::string& yaml_path,
                             const std::array<std::string, Dof>& joint_names) {

    YAML::Node root = YAML::LoadFile(yaml_path);
    if (!root["joints"]) {
      throw std::runtime_error("Missing 'joints' section in YAML");
    }

    joint_params_.fill(JointParam{});
    joint_names_.assign(joint_names.begin(), joint_names.end());

    for (std::size_t i = 0; i < Dof; ++i) {
      const std::string& name = joint_names[i];
      const YAML::Node node = root["joints"][name];
      if (!node) {
        throw std::runtime_error("Joint not found in YAML: " + name);
      }

      JointParam param{};
      const std::string type_str = node["type"].as<std::string>();
      if (type_str == "revolute") {
        param.type = JointMotionType::kRevolute;
      } else if (type_str == "prismatic") {
        param.type = JointMotionType::kPrismatic;
      } else if (type_str == "fixed") {
        param.type = JointMotionType::kFixed;
      } else {
        throw std::runtime_error("Unsupported joint type: " + type_str);
      }

      param.axis = detail::parseVector3(node, "axis");

      if (!node["origin"] || !node["origin"].IsMap()) {
        throw std::runtime_error("Missing origin for joint: " + name);
      }
      param.origin.xyz = detail::parseVector3(node["origin"], "xyz");
      param.origin.rpy = detail::parseVector3(node["origin"], "rpy");

      joint_params_[i] = param;
      joint_transforms_[i] = detail::transformFromOrigin(param.origin.xyz, param.origin.rpy);
      joint_axes_[i] = detail::normalizedAxis(param.axis);
    }

    initialized_ = true;
  }


  HomogeneousMatrix forwardKinematics() const override {
    ensureInitialized();
    HomogeneousMatrix t = HomogeneousMatrix::identity();

    for (std::size_t i = 0; i < Dof; ++i) {
      t = t * joint_transforms_[i];

      const JointParam& param = joint_params_[i];
      if (param.type == JointMotionType::kRevolute) {
        const RotationMatrix r = detail::rotationFromAxisAngle(joint_axes_[i], this->state_.q[i]);
        t = t * HomogeneousMatrix::fromRotationTranslation(r, Vector3{0.0, 0.0, 0.0});
      } else if (param.type == JointMotionType::kPrismatic) {
        t = t * detail::transformFromAxisTranslation(joint_axes_[i], this->state_.q[i]);
      }
    }
    return t* this->toolTransformation();
  }

  std::array<HomogeneousMatrix, Dof + 1> jointTransforms() const {
    ensureInitialized();
    std::array<HomogeneousMatrix, Dof + 1> Ts{};
    HomogeneousMatrix t = HomogeneousMatrix::identity();

    for (std::size_t i = 0; i < Dof; ++i) {
      t = t * joint_transforms_[i];

      const JointParam& param = joint_params_[i];
      if (param.type == JointMotionType::kRevolute) {
        const RotationMatrix r = detail::rotationFromAxisAngle(joint_axes_[i], this->state_.q[i]);
        t = t * HomogeneousMatrix::fromRotationTranslation(r, Vector3{0.0, 0.0, 0.0});
      } else if (param.type == JointMotionType::kPrismatic) {
        t = t * detail::transformFromAxisTranslation(joint_axes_[i], this->state_.q[i]);
      }
      Ts[i] = t;
    }
    Ts[Dof] = t * this->toolTransformation();
    return Ts;
  }



  Jacobian jacobian() const override {
    ensureInitialized();

    std::array<Vector3, Dof + 1> origins{};
    std::array<Vector3, Dof> axes{};

    HomogeneousMatrix t = HomogeneousMatrix::identity();
    origins[0] = t.translation();

    for (std::size_t i = 0; i < Dof; ++i) {
      t = t * joint_transforms_[i];

      const RotationMatrix r = t.rotation();
      const Vector3 axis = joint_axes_[i];
      const Vector3 axis_base{r(0, 0) * axis(0) + r(0, 1) * axis(1) + r(0, 2) * axis(2),
                              r(1, 0) * axis(0) + r(1, 1) * axis(1) + r(1, 2) * axis(2),
                              r(2, 0) * axis(0) + r(2, 1) * axis(1) + r(2, 2) * axis(2)};
      axes[i] = axis_base;

      const JointParam& param = joint_params_[i];
      if (param.type == JointMotionType::kRevolute) {
        const RotationMatrix r_joint = detail::rotationFromAxisAngle(axis, this->state_.q[i]);
        t = t * HomogeneousMatrix::fromRotationTranslation(r_joint, Vector3{0.0, 0.0, 0.0});
      } else if (param.type == JointMotionType::kPrismatic) {
        t = t * detail::transformFromAxisTranslation(axis, this->state_.q[i]);
      }
      origins[i + 1] = t.translation();
    }

    HomogeneousMatrix t_tip = t * this->toolTransformation();
    const Vector3 o_tip = t_tip.translation();

    Jacobian j{};
    for (std::size_t i = 0; i < Dof; ++i) {
      const JointParam& param = joint_params_[i];
      if (param.type == JointMotionType::kPrismatic) {
        j(0, i) = axes[i](0);
        j(1, i) = axes[i](1);
        j(2, i) = axes[i](2);
        j(3, i) = 0.0;
        j(4, i) = 0.0;
        j(5, i) = 0.0;
      } else if (param.type == JointMotionType::kRevolute) {
        const Vector3 p = o_tip - origins[i];
        const Vector3 jv = cross(axes[i], p);
        j(0, i) = jv(0);
        j(1, i) = jv(1);
        j(2, i) = jv(2);
        j(3, i) = axes[i](0);
        j(4, i) = axes[i](1);
        j(5, i) = axes[i](2);
      } else {
        j(0, i) = 0.0;
        j(1, i) = 0.0;
        j(2, i) = 0.0;
        j(3, i) = 0.0;
        j(4, i) = 0.0;
        j(5, i) = 0.0;
      }
    }
    return j;
  }

 private:
  void ensureInitialized() const {
    if (!initialized_) {
      throw std::runtime_error("ManipulatorFromYAML not initialized. Call setParametersFromFile().");
    }
  }

  std::array<JointParam, Dof> joint_params_{};
  std::array<HomogeneousMatrix, Dof> joint_transforms_{};
  std::array<Vector3, Dof> joint_axes_{};
  std::vector<std::string> joint_names_{};
  bool initialized_{false};
};

}  // namespace sfc

#endif  // SFC_MANIPULATOR_FROM_YAML_H_
