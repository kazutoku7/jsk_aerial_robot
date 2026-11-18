#include <flying_humanoid/static_wrench_optimizer.h>

#include <ros/ros.h>

#include <Eigen/Core>

#include <exception>
#include <string>
#include <vector>

namespace
{
template <class T>
T param(ros::NodeHandle& nh, const std::string& name, const T& default_value)
{
  T value;
  nh.param<T>(name, value, default_value);
  return value;
}

template <int Size>
Eigen::Matrix<double, Size, 1> vectorParam(ros::NodeHandle& nh, const std::string& name,
                                          const Eigen::Matrix<double, Size, 1>& default_value)
{
  std::vector<double> value;
  if (!nh.getParam(name, value))
    return default_value;
  if (static_cast<int>(value.size()) != Size)
    throw std::runtime_error(name + " must contain " + std::to_string(Size) + " values");
  return Eigen::Map<Eigen::Matrix<double, Size, 1>>(value.data());
}

flying_humanoid::StaticWrenchOptimizerParams loadParams(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  flying_humanoid::StaticWrenchOptimizerParams p;
  p.lfoot = param(pnh, "lfoot", p.lfoot);
  p.rfoot = param(pnh, "rfoot", p.rfoot);
  p.end_effector = param(pnh, "end_effector", p.end_effector);
  p.wall_point = vectorParam<3>(pnh, "wall_point", p.wall_point);
  p.wall_normal = vectorParam<3>(pnh, "wall_normal", p.wall_normal);
  p.foot_parallel_tolerance = param(pnh, "foot_parallel_tolerance", p.foot_parallel_tolerance);
  p.ee_target_point = vectorParam<3>(pnh, "ee_target_point", p.ee_target_point);
  p.ee_offset = param(pnh, "ee_offset", p.ee_offset);
  p.ee_target_axis = vectorParam<3>(pnh, "ee_target_axis", p.ee_target_axis);
  p.floor_point = vectorParam<3>(pnh, "floor_point", p.floor_point);
  p.floor_normal = vectorParam<3>(pnh, "floor_normal", p.floor_normal);
  p.minimum_floor_clearance = param(pnh, "minimum_floor_clearance", p.minimum_floor_clearance);
  p.friction_coefficient = param(pnh, "friction_coefficient", p.friction_coefficient);
  p.foot_size = vectorParam<2>(pnh, "foot_size", p.foot_size);
  p.minimum_contact_force = param(pnh, "minimum_contact_force", p.minimum_contact_force);

  p.surface_effect.rotor_radius = param(pnh, "surface_effect/rotor_radius", p.surface_effect.rotor_radius);
  p.surface_effect.a0 = param(pnh, "surface_effect/a0", p.surface_effect.a0);
  p.surface_effect.a1 = param(pnh, "surface_effect/a1", p.surface_effect.a1);
  p.surface_effect.b1 = param(pnh, "surface_effect/b1", p.surface_effect.b1);
  p.moment_to_force_rate = param(pnh, "moment_to_force_rate", p.moment_to_force_rate);
  nh.getParam("motor_info/m_f_rate", p.moment_to_force_rate);

  std::vector<double> initial;
  if (pnh.getParam("initial_configuration", initial))
    p.initial_configuration = Eigen::Map<Eigen::VectorXd>(initial.data(), initial.size());
  p.initialize_for_wall = param(pnh, "initialize_for_wall", p.initialize_for_wall);
  p.root_translation_limit = param(pnh, "root_translation_limit", p.root_translation_limit);
  p.contact_wrench_limit = param(pnh, "contact_wrench_limit", p.contact_wrench_limit);
  p.maximum_ee_torque = param(pnh, "maximum_ee_torque", p.maximum_ee_torque);
  p.max_iterations = param(pnh, "max_iterations", p.max_iterations);
  p.tolerance = param(pnh, "tolerance", p.tolerance);
  p.finite_difference_step = param(pnh, "finite_difference_step", p.finite_difference_step);
  p.print_level = param(pnh, "print_level", p.print_level);
  return p;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_wrench_optimizer");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    std::string robot_description;
    if (!nh.getParam("robot_description", robot_description))
      throw std::runtime_error("robot_description is not available");

    const auto params = loadParams(nh, pnh);
    flying_humanoid::StaticWrenchOptimizer optimizer(robot_description, params);
    ROS_INFO("solving the static maximum arm_ee torque problem at wall-normal offset %.6g m along axis "
             "[%.6g, %.6g, %.6g]...",
             params.ee_offset, params.ee_target_axis.x(), params.ee_target_axis.y(), params.ee_target_axis.z());
    const flying_humanoid::StaticWrenchOptimizerResult result = optimizer.solve();

    const std::string output = param<std::string>(pnh, "output", "static_wrench_optimizer.yaml");
    if (!optimizer.saveResult(output, result))
      ROS_ERROR("failed to save result to %s", output.c_str());

    if (!result.success)
    {
      ROS_ERROR("static wrench optimization failed: %s (max violation %.6g). Result: %s",
                result.solver_status_text.c_str(), result.maximum_constraint_violation, output.c_str());
      return 2;
    }
    ROS_INFO("maximum arm_ee torque: %.6f Nm; max constraint violation: %.3e; result: %s", result.ee_torque,
             result.maximum_constraint_violation, output.c_str());
    for (std::size_t i = 0; i < result.surface_effects.size(); ++i)
    {
      const auto& effect = result.surface_effects[i];
      ROS_INFO("rotor%zu: d=%.4f m, d/R=%.4f, alpha=%.4f rad, k=%.5f, free=%.3f N, actual=%.3f N", i + 1,
               effect.signed_distance, effect.normalized_distance, effect.tilt, effect.thrust_ratio,
               result.rotor_thrusts[i], result.actual_rotor_thrusts[i]);
    }
    if (result.floor_clearances.size() >= 2)
    {
      ROS_INFO("floor clearance: lfoot=%.4f m, rfoot=%.4f m", result.floor_clearances[0],
               result.floor_clearances[1]);
      for (std::size_t i = 2; i < result.floor_clearances.size(); ++i)
        ROS_INFO("floor clearance: rotor%zu=%.4f m", i - 1, result.floor_clearances[i]);
    }
    return 0;
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("static wrench optimizer: %s", e.what());
    return 1;
  }
}
