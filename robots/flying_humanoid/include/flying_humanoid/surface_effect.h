// -*- mode: c++ -*-
#pragma once

#include <Eigen/Core>

namespace flying_humanoid
{
// Tilt-aware ceiling-effect model in Eq. (9) of
// "Near-Ceiling Horizontal Maneuvering of Multirotors via
// Ceiling-Effect Compensation Considering Rotor-Axis Tilt".
//
// We use the same model for an arbitrary planar surface.  The surface normal
// points from the surface toward the robot.  The tilt is the unsigned angle
// between the rotor axis and the surface-normal line, so the same convention
// can be used for a floor, ceiling, or vertical wall.
struct SurfaceEffectParams
{
  double rotor_radius = 0.0635;  // [m], radius of a 5-inch-diameter propeller
  double a0 = -6.8407;
  double a1 = 1.6194;
  double b1 = 7.2711;
};

struct SurfaceEffect
{
  double signed_distance = 0.0;      // [m], positive on the surface-normal side
  double normalized_distance = 0.0;  // signed_distance / rotor_radius
  double tilt = 0.0;                 // [rad]
  double radicand = 0.0;             // expression inside the square root
  double thrust_ratio = 1.0;         // actual thrust / free-air thrust
};

class TiltAwareSurfaceEffect
{
public:
  explicit TiltAwareSurfaceEffect(const SurfaceEffectParams& params = SurfaceEffectParams());

  // plane_point is any point on the surface. plane_normal must point toward
  // the side occupied by the robot. rotor_axis is expressed in the same frame.
  SurfaceEffect evaluate(const Eigen::Vector3d& rotor_position, const Eigen::Vector3d& rotor_axis,
                         const Eigen::Vector3d& plane_point, const Eigen::Vector3d& plane_normal) const;

  double thrustRatio(double normalized_distance, double tilt) const;
  double radicand(double normalized_distance, double tilt) const;

  const SurfaceEffectParams& params() const { return params_; }

private:
  SurfaceEffectParams params_;
};
}  // namespace flying_humanoid
