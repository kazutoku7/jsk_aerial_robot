#include <flying_humanoid/surface_effect.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flying_humanoid
{
TiltAwareSurfaceEffect::TiltAwareSurfaceEffect(const SurfaceEffectParams& params) : params_(params)
{
  if (!(params_.rotor_radius > 0.0))
    throw std::invalid_argument("rotor_radius must be positive");
}

double TiltAwareSurfaceEffect::radicand(double normalized_distance, double tilt) const
{
  const double d = std::max(std::abs(normalized_distance), 1e-9);
  const double coefficient = params_.a0 + params_.a1 * std::sin(tilt) + params_.b1 * std::cos(tilt);
  return 1.0 + coefficient / (8.0 * d * d);
}

double TiltAwareSurfaceEffect::thrustRatio(double normalized_distance, double tilt) const
{
  // During an IPOPT line search an infeasible trial point can temporarily make
  // the radicand negative.  The NLP separately constrains it to be positive;
  // this floor merely keeps the function finite at those trial points.
  return 0.5 + 0.5 * std::sqrt(std::max(radicand(normalized_distance, tilt), 1e-12));
}

SurfaceEffect TiltAwareSurfaceEffect::evaluate(const Eigen::Vector3d& rotor_position,
                                               const Eigen::Vector3d& rotor_axis,
                                               const Eigen::Vector3d& plane_point,
                                               const Eigen::Vector3d& plane_normal) const
{
  if (rotor_axis.norm() < 1e-12 || plane_normal.norm() < 1e-12)
    throw std::invalid_argument("rotor_axis and plane_normal must be nonzero");

  const Eigen::Vector3d axis = rotor_axis.normalized();
  const Eigen::Vector3d normal = plane_normal.normalized();
  SurfaceEffect result;
  result.signed_distance = normal.dot(rotor_position - plane_point);
  result.normalized_distance = result.signed_distance / params_.rotor_radius;
  const double cosine = std::clamp(std::abs(axis.dot(normal)), 0.0, 1.0);
  const double sine = std::sqrt(std::max(0.0, 1.0 - cosine * cosine));
  result.tilt = std::atan2(sine, cosine);

  // sin(alpha)=sqrt(1-cos(alpha)^2) is nondifferentiable at alpha=0
  // when alpha is obtained from a 3-D axis. A very small smoothing term keeps
  // the optimization Jacobian finite while changing k by less than numerical
  // measurement precision.
  constexpr double sine_smoothing = 1e-6;
  const double smooth_sine = std::sqrt(sine * sine + sine_smoothing * sine_smoothing);
  const double d = std::max(std::abs(result.normalized_distance), 1e-9);
  const double coefficient = params_.a0 + params_.a1 * smooth_sine + params_.b1 * cosine;
  result.radicand = 1.0 + coefficient / (8.0 * d * d);
  result.thrust_ratio = 0.5 + 0.5 * std::sqrt(std::max(result.radicand, 1e-12));
  return result;
}
}  // namespace flying_humanoid
