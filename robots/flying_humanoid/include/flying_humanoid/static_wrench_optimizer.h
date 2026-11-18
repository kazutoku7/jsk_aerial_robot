// -*- mode: c++ -*-
#pragma once

#include <pinocchio/fwd.hpp>  // include before other Pinocchio headers

#include <flying_humanoid/surface_effect.h>

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/force.hpp>

#include <memory>
#include <string>
#include <vector>

namespace flying_humanoid
{
struct StaticWrenchOptimizerParams
{
  std::string lfoot = "lfoot";
  std::string rfoot = "rfoot";
  std::string end_effector = "arm_ee";

  // A vertical wall is represented by default as x=0.  The normal points from
  // the wall toward the robot.  Absolute translation within the wall is a
  // gauge freedom; root y and z are therefore fixed during optimization.
  Eigen::Vector3d wall_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d wall_normal = Eigen::Vector3d::UnitX();
  double foot_parallel_tolerance = 1e-3;  // [rad]

  // ee_target_point is the reference point on the wall.  The end-effector
  // origin is fixed ee_offset metres from it along the outward wall normal,
  // and its local +z axis is aligned with ee_target_axis.
  Eigen::Vector3d ee_target_point = Eigen::Vector3d(0.0, 0.0, 1.0);
  double ee_offset = 0.0;  // [m], positive toward the robot
  Eigen::Vector3d ee_target_axis = Eigen::Vector3d::UnitX();

  // Keep the wall-contact configuration sufficiently far from the takeoff
  // floor that the floor surface effect can be neglected.  Both foot contact
  // origins and all rotor centers must remain above this clearance.
  Eigen::Vector3d floor_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d floor_normal = Eigen::Vector3d::UnitZ();
  double minimum_floor_clearance = 0.3175;  // [m] = 5R for a 5-inch propeller

  double friction_coefficient = 0.7;
  Eigen::Vector2d foot_size = Eigen::Vector2d(0.08, 0.05);  // full length and width [m]
  double minimum_contact_force = 5.0;                       // [N], per foot

  SurfaceEffectParams surface_effect;
  double moment_to_force_rate = -0.0067;  // rotor drag moment / thrust [m]

  // Optional q used as the tangent-space reference.  If its size differs from
  // model.nq, a wall-contact seed is generated automatically.
  Eigen::VectorXd initial_configuration;
  bool initialize_for_wall = true;

  double root_translation_limit = 1.0;  // [m], each world direction about the reference
  double contact_wrench_limit = 1000.0;
  double maximum_ee_torque = 100.0;  // [Nm], numerical safety bound

  int max_iterations = 500;
  double tolerance = 1e-6;
  double finite_difference_step = 1e-6;
  int print_level = 5;
};

struct StaticWrenchOptimizerResult
{
  bool success = false;
  int solver_status = 0;
  std::string solver_status_text;
  double ee_torque = 0.0;  // torque exerted about arm_ee +z [Nm]
  double maximum_constraint_violation = 0.0;
  Eigen::Vector3d ee_position_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d ee_axis_world = Eigen::Vector3d::Zero();

  Eigen::VectorXd configuration;
  std::vector<std::string> joint_names;
  Eigen::VectorXd joint_positions;
  Eigen::VectorXd joint_torques;
  Eigen::VectorXd rotor_thrusts;         // free-air-equivalent actuator thrust [N]
  Eigen::VectorXd actual_rotor_thrusts;  // surface-effect-enhanced thrust [N]
  std::vector<Eigen::Vector3d> contact_positions_world;  // [lfoot, rfoot]
  std::vector<pinocchio::Force> contact_wrenches;  // world-aligned at lfoot, rfoot
  std::vector<SurfaceEffect> surface_effects;
  // Ordered as [lfoot, rfoot, rotor1, ...], measured along floor_normal [m].
  std::vector<double> floor_clearances;
};

// Solves one static nonlinear program.  It does not alter the walking planner
// and does not command the robot.
class StaticWrenchOptimizer
{
public:
  StaticWrenchOptimizer(const std::string& robot_description, const StaticWrenchOptimizerParams& params);
  ~StaticWrenchOptimizer();

  StaticWrenchOptimizerResult solve();
  bool saveResult(const std::string& filename, const StaticWrenchOptimizerResult& result) const;

  const pinocchio::Model& model() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace flying_humanoid
