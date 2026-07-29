// Copyright 2026 RyuYamamoto.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#ifndef ELTANIN__CORE__TYPES_HPP_
#define ELTANIN__CORE__TYPES_HPP_

#include <eltanin/core/angle.hpp>

#include <Eigen/Core>

#include <cmath>

namespace eltanin
{

/// Planar state. Orientation is a yaw angle; this library never carries quaternions in 2D.
struct Pose2D
{
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  double yaw{0.0};
};

/// Planar velocity. `linear` is 2D so that omnidirectional motion stays representable.
struct Twist2D
{
  Eigen::Vector2d linear{Eigen::Vector2d::Zero()};
  double angular{0.0};
};

/// SE(2) rigid transform. Distinct from `Pose2D` (a state) with no implicit conversion.
class Transform2D
{
public:
  Transform2D() = default;

  Transform2D(const Eigen::Vector2d & translation, double rotation)
  : translation_(translation), rotation_(normalize_angle(rotation))
  {
  }

  const Eigen::Vector2d & translation() const noexcept { return translation_; }

  /// Always in (-pi, pi]: repeated composition cannot let the angle drift.
  double rotation() const noexcept { return rotation_; }

  Transform2D inverse() const
  {
    const double c = std::cos(rotation_);
    const double s = std::sin(rotation_);
    const Eigen::Vector2d t{
      -(c * translation_.x() + s * translation_.y()),
      -(-s * translation_.x() + c * translation_.y())};
    return Transform2D(t, -rotation_);
  }

  Eigen::Vector2d operator*(const Eigen::Vector2d & point) const
  {
    const double c = std::cos(rotation_);
    const double s = std::sin(rotation_);
    return Eigen::Vector2d{
      c * point.x() - s * point.y() + translation_.x(),
      s * point.x() + c * point.y() + translation_.y()};
  }

  Pose2D operator*(const Pose2D & pose) const
  {
    return Pose2D{*this * pose.position, normalize_angle(rotation_ + pose.yaw)};
  }

  /// Composition; `rhs` is applied first.
  Transform2D operator*(const Transform2D & rhs) const
  {
    return Transform2D(*this * rhs.translation_, rotation_ + rhs.rotation_);
  }

  static Transform2D from_pose(const Pose2D & pose)
  {
    return Transform2D(pose.position, pose.yaw);
  }

  Pose2D to_pose() const { return Pose2D{translation_, rotation_}; }

private:
  Eigen::Vector2d translation_{Eigen::Vector2d::Zero()};
  double rotation_{0.0};
};

}  // namespace eltanin

#endif  // ELTANIN__CORE__TYPES_HPP_
