#include <flying_humanoid/static_wrench_optimizer.h>

#include <flying_humanoid/robot_model.h>

#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>
#include <IpTNLP.hpp>
#include <crocoddyl/multibody/wrench-cone.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace flying_humanoid
{
namespace
{
constexpr double kIpoptInfinity = 2e19;

double finiteBound(double value, double replacement)
{
  return std::isfinite(value) ? value : replacement;
}

std::string statusText(Ipopt::ApplicationReturnStatus status)
{
  switch (status)
  {
    case Ipopt::Solve_Succeeded:
      return "Solve_Succeeded";
    case Ipopt::Solved_To_Acceptable_Level:
      return "Solved_To_Acceptable_Level";
    case Ipopt::Infeasible_Problem_Detected:
      return "Infeasible_Problem_Detected";
    case Ipopt::Search_Direction_Becomes_Too_Small:
      return "Search_Direction_Becomes_Too_Small";
    case Ipopt::Diverging_Iterates:
      return "Diverging_Iterates";
    case Ipopt::User_Requested_Stop:
      return "User_Requested_Stop";
    case Ipopt::Maximum_Iterations_Exceeded:
      return "Maximum_Iterations_Exceeded";
    case Ipopt::Restoration_Failed:
      return "Restoration_Failed";
    case Ipopt::Error_In_Step_Computation:
      return "Error_In_Step_Computation";
    case Ipopt::Not_Enough_Degrees_Of_Freedom:
      return "Not_Enough_Degrees_Of_Freedom";
    case Ipopt::Invalid_Problem_Definition:
      return "Invalid_Problem_Definition";
    case Ipopt::Invalid_Option:
      return "Invalid_Option";
    case Ipopt::Invalid_Number_Detected:
      return "Invalid_Number_Detected";
    case Ipopt::Unrecoverable_Exception:
      return "Unrecoverable_Exception";
    case Ipopt::NonIpopt_Exception_Thrown:
      return "NonIpopt_Exception_Thrown";
    case Ipopt::Insufficient_Memory:
      return "Insufficient_Memory";
    case Ipopt::Internal_Error:
      return "Internal_Error";
    default:
      return "Ipopt_status_" + std::to_string(static_cast<int>(status));
  }
}

bool successfulStatus(Ipopt::ApplicationReturnStatus status)
{
  return status == Ipopt::Solve_Succeeded || status == Ipopt::Solved_To_Acceptable_Level;
}

struct VariableLayout
{
  int dq = 0;
  int tau = 0;
  int thrust = 0;
  int contacts = 0;
  int eta = 0;
  int size = 0;
};

struct ConstraintLayout
{
  int statics = 0;
  int feet = 0;
  int end_effector = 0;
  int surface = 0;
  int floor_clearance = 0;
  int size = 0;
};
}  // namespace

class StaticWrenchNlp final : public Ipopt::TNLP
{
public:
  StaticWrenchNlp(const std::string& robot_description, const StaticWrenchOptimizerParams& params)
    : params_(params), model_(buildPinocchioModel(robot_description, &rotors_)), data_(model_),
      surface_model_(params.surface_effect)
  {
    if (rotors_.empty())
      throw std::runtime_error("robot_description has no rotor<number> joints");
    if (model_.nv < 6)
      throw std::runtime_error("the optimizer requires a free-flyer model");
    if (params_.wall_normal.norm() < 1e-12)
      throw std::runtime_error("wall_normal must be nonzero");
    if (params_.floor_normal.norm() < 1e-12)
      throw std::runtime_error("floor_normal must be nonzero");
    if (params_.ee_target_axis.norm() < 1e-12)
      throw std::runtime_error("ee_target_axis must be nonzero");
    if (!std::isfinite(params_.ee_offset))
      throw std::runtime_error("ee_offset must be finite");
    if (params_.minimum_floor_clearance < 0.0)
      throw std::runtime_error("minimum_floor_clearance must be nonnegative");
    wall_normal_ = params_.wall_normal.normalized();
    floor_normal_ = params_.floor_normal.normalized();
    ee_target_axis_ = params_.ee_target_axis.normalized();
    if (std::abs(wall_normal_.dot(params_.ee_target_point - params_.wall_point)) > 1e-9)
      throw std::runtime_error("ee_target_point must lie on the wall plane");
    ee_target_position_ = params_.ee_target_point + params_.ee_offset * wall_normal_;

    // Raising the configuration must not move the feet away from the wall.
    floor_lift_direction_ = floor_normal_ - wall_normal_ * wall_normal_.dot(floor_normal_);
    if (floor_lift_direction_.norm() < 1e-12)
      throw std::runtime_error("floor_normal must not be parallel to wall_normal");
    floor_lift_direction_.normalize();

    // Construct an orthonormal basis of the wall plane.
    const Eigen::Vector3d helper = std::abs(wall_normal_.z()) < 0.9 ? Eigen::Vector3d::UnitZ() :
                                                                    Eigen::Vector3d::UnitY();
    wall_tangent1_ = (helper - wall_normal_ * wall_normal_.dot(helper)).normalized();
    wall_tangent2_ = wall_normal_.cross(wall_tangent1_).normalized();

    const Eigen::Vector3d ee_helper = std::abs(ee_target_axis_.z()) < 0.9 ? Eigen::Vector3d::UnitZ() :
                                                                          Eigen::Vector3d::UnitY();
    ee_axis_tangent1_ = (ee_helper - ee_target_axis_ * ee_target_axis_.dot(ee_helper)).normalized();
    ee_axis_tangent2_ = ee_target_axis_.cross(ee_axis_tangent1_).normalized();

    lfoot_id_ = checkedFrame(params_.lfoot);
    rfoot_id_ = checkedFrame(params_.rfoot);
    ee_id_ = checkedFrame(params_.end_effector);
    foot_ids_ = { lfoot_id_, rfoot_id_ };

    nv_ = static_cast<int>(model_.nv);
    nq_ = static_cast<int>(model_.nq);
    nj_ = nv_ - 6;
    nr_ = static_cast<int>(rotors_.size());
    const crocoddyl::WrenchCone reference_cone(Eigen::Matrix3d::Identity(), params_.friction_coefficient,
                                               params_.foot_size);
    cone_size_ = static_cast<int>(reference_cone.get_A().rows());

    variables_.dq = 0;
    variables_.tau = variables_.dq + nv_;
    variables_.thrust = variables_.tau + nj_;
    variables_.contacts = variables_.thrust + nr_;
    variables_.eta = variables_.contacts + 12;
    variables_.size = variables_.eta + 1;

    constraints_.statics = 0;
    constraints_.feet = constraints_.statics + nv_;
    constraints_.end_effector = constraints_.feet + 2 * (4 + cone_size_);
    constraints_.surface = constraints_.end_effector + 6;
    constraints_.floor_clearance = constraints_.surface + nr_;
    constraints_.size = constraints_.floor_clearance + nr_ + 2;

    initializeReference();
    initializeBounds(reference_cone);
    initializeStartingPoint();
  }

  const pinocchio::Model& model() const { return model_; }

  bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag,
                    IndexStyleEnum& index_style) override
  {
    n = variables_.size;
    m = constraints_.size;
    nnz_jac_g = n * m;  // dense numerical Jacobian
    nnz_h_lag = 0;      // IPOPT limited-memory Hessian is used
    index_style = TNLP::C_STYLE;
    return true;
  }

  bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index m,
                       Ipopt::Number* g_l, Ipopt::Number* g_u) override
  {
    if (n != variables_.size || m != constraints_.size)
      return false;
    Eigen::Map<Eigen::VectorXd>(x_l, n) = x_lb_;
    Eigen::Map<Eigen::VectorXd>(x_u, n) = x_ub_;
    Eigen::Map<Eigen::VectorXd>(g_l, m) = g_lb_;
    Eigen::Map<Eigen::VectorXd>(g_u, m) = g_ub_;
    return true;
  }

  bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x, bool init_z, Ipopt::Number*,
                          Ipopt::Number*, Ipopt::Index, bool init_lambda, Ipopt::Number*) override
  {
    if (n != variables_.size || !init_x || init_z || init_lambda)
      return false;
    Eigen::Map<Eigen::VectorXd>(x, n) = x0_;
    return true;
  }

  bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool, Ipopt::Number& obj_value) override
  {
    if (n != variables_.size)
      return false;
    obj_value = -x[variables_.eta];
    return std::isfinite(obj_value);
  }

  bool eval_grad_f(Ipopt::Index n, const Ipopt::Number*, bool, Ipopt::Number* grad_f) override
  {
    if (n != variables_.size)
      return false;
    Eigen::Map<Eigen::VectorXd>(grad_f, n).setZero();
    grad_f[variables_.eta] = -1.0;
    return true;
  }

  bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool, Ipopt::Index m, Ipopt::Number* g) override
  {
    if (n != variables_.size || m != constraints_.size)
      return false;
    const Eigen::Map<const Eigen::VectorXd> xv(x, n);
    Eigen::Map<Eigen::VectorXd> gv(g, m);
    return evaluateConstraints(xv, gv);
  }

  bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool, Ipopt::Index m, Ipopt::Index nele_jac,
                  Ipopt::Index* i_row, Ipopt::Index* j_col, Ipopt::Number* values) override
  {
    if (n != variables_.size || m != constraints_.size || nele_jac != n * m)
      return false;
    if (values == nullptr)
    {
      Ipopt::Index k = 0;
      for (Ipopt::Index row = 0; row < m; ++row)
        for (Ipopt::Index col = 0; col < n; ++col, ++k)
        {
          i_row[k] = row;
          j_col[k] = col;
        }
      return true;
    }

    const Eigen::Map<const Eigen::VectorXd> xv(x, n);
    Eigen::VectorXd g0(m), gp(m), gm(m), xp = xv, xm = xv;
    if (!evaluateConstraints(xv, g0))
      return false;

    Eigen::MatrixXd jac(m, n);
    for (Ipopt::Index col = 0; col < n; ++col)
    {
      const double h = params_.finite_difference_step * std::max(1.0, std::abs(xv[col]));
      const bool forward = xv[col] - h < x_lb_[col];
      const bool backward = xv[col] + h > x_ub_[col];
      xp = xv;
      xm = xv;
      if (forward && !backward)
      {
        xp[col] += h;
        if (!evaluateConstraints(xp, gp))
          return false;
        jac.col(col) = (gp - g0) / h;
      }
      else if (backward && !forward)
      {
        xm[col] -= h;
        if (!evaluateConstraints(xm, gm))
          return false;
        jac.col(col) = (g0 - gm) / h;
      }
      else if (!forward && !backward)
      {
        xp[col] += h;
        xm[col] -= h;
        if (!evaluateConstraints(xp, gp) || !evaluateConstraints(xm, gm))
          return false;
        jac.col(col) = (gp - gm) / (2.0 * h);
      }
      else
      {
        jac.col(col).setZero();  // fixed variable
      }
    }

    Ipopt::Index k = 0;
    for (Ipopt::Index row = 0; row < m; ++row)
      for (Ipopt::Index col = 0; col < n; ++col, ++k)
        values[k] = jac(row, col);
    return jac.allFinite();
  }

  bool eval_h(Ipopt::Index, const Ipopt::Number*, bool, Ipopt::Number, Ipopt::Index, const Ipopt::Number*, bool,
              Ipopt::Index, Ipopt::Index*, Ipopt::Index*, Ipopt::Number*) override
  {
    return false;
  }

  void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number*,
                         const Ipopt::Number*, Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number*,
                         Ipopt::Number, const Ipopt::IpoptData*, Ipopt::IpoptCalculatedQuantities*) override
  {
    solution_status_ = status;
    solution_x_ = Eigen::Map<const Eigen::VectorXd>(x, n);
    solution_g_ = Eigen::Map<const Eigen::VectorXd>(g, m);
  }

  StaticWrenchOptimizerResult result(Ipopt::ApplicationReturnStatus app_status)
  {
    StaticWrenchOptimizerResult out;
    out.solver_status = static_cast<int>(app_status);
    out.solver_status_text = statusText(app_status);
    out.success = successfulStatus(app_status);
    if (solution_x_.size() != variables_.size)
      return out;

    out.ee_torque = solution_x_[variables_.eta];
    out.configuration = configuration(solution_x_);
    out.joint_positions.resize(nj_);
    int joint_offset = 0;
    for (pinocchio::JointIndex jid = 2; jid < model_.njoints; ++jid)
    {
      out.joint_names.push_back(model_.names[jid]);
      out.joint_positions[joint_offset++] = out.configuration[model_.joints[jid].idx_q()];
    }
    out.joint_torques = solution_x_.segment(variables_.tau, nj_);
    out.rotor_thrusts = solution_x_.segment(variables_.thrust, nr_);
    out.actual_rotor_thrusts.resize(nr_);
    out.contact_positions_world.resize(2, Eigen::Vector3d::Zero());
    out.contact_wrenches.resize(2, pinocchio::Force::Zero());
    for (int i = 0; i < 2; ++i)
      out.contact_wrenches[i].toVector() = solution_x_.segment<6>(variables_.contacts + 6 * i);

    pinocchio::forwardKinematics(model_, data_, out.configuration);
    pinocchio::updateFramePlacements(model_, data_);
    out.ee_position_world = data_.oMf[ee_id_].translation();
    out.ee_axis_world = data_.oMf[ee_id_].rotation() * Eigen::Vector3d::UnitZ();
    for (std::size_t i = 0; i < foot_ids_.size(); ++i)
    {
      const pinocchio::FrameIndex foot_id = foot_ids_[i];
      out.contact_positions_world[i] = data_.oMf[foot_id].translation();
      out.floor_clearances.push_back(
          floor_normal_.dot(data_.oMf[foot_id].translation() - params_.floor_point));
    }
    for (int i = 0; i < nr_; ++i)
    {
      const pinocchio::SE3& pose = data_.oMf[rotors_[i].frame_id];
      const SurfaceEffect effect =
          surface_model_.evaluate(pose.translation(), pose.rotation() * Eigen::Vector3d::UnitZ(), params_.wall_point,
                                  wall_normal_);
      out.surface_effects.push_back(effect);
      out.actual_rotor_thrusts[i] = effect.thrust_ratio * out.rotor_thrusts[i];
      out.floor_clearances.push_back(floor_normal_.dot(pose.translation() - params_.floor_point));
    }

    if (solution_g_.size() == constraints_.size)
    {
      double violation = 0.0;
      for (int i = 0; i < constraints_.size; ++i)
      {
        violation = std::max(violation, std::max(g_lb_[i] - solution_g_[i], solution_g_[i] - g_ub_[i]));
      }
      out.maximum_constraint_violation = std::max(0.0, violation);
    }
    return out;
  }

private:
  pinocchio::FrameIndex checkedFrame(const std::string& name) const
  {
    const pinocchio::FrameIndex id = model_.getFrameId(name);
    if (id >= model_.frames.size())
      throw std::runtime_error("frame not found in robot_description: " + name);
    return id;
  }

  Eigen::VectorXd configuration(const Eigen::Ref<const Eigen::VectorXd>& x) const
  {
    // Pinocchio expresses the free-flyer translational tangent in the moving
    // root frame. Parameterize translation explicitly in the world frame; the
    // fixed end-effector point removes the former wall-tangential gauge freedom.
    Eigen::VectorXd delta = x.segment(variables_.dq, nv_);
    delta.head<3>().setZero();
    Eigen::VectorXd q = pinocchio::integrate(model_, q_ref_, delta);
    q.head<3>() = q_ref_.head<3>() + x.segment<3>(variables_.dq);
    return q;
  }

  void initializeReference()
  {
    if (params_.initial_configuration.size() == nq_)
      q_ref_ = params_.initial_configuration;
    else
      q_ref_ = pinocchio::neutral(model_);

    if (q_ref_.segment<4>(3).norm() < 1e-12)
      q_ref_.segment<4>(3) << 0.0, 0.0, 0.0, 1.0;
    q_ref_.segment<4>(3).normalize();

    if (!params_.initialize_for_wall)
      return;

    // Turn each gimbal by pi so that, after aligning the soles with the wall,
    // the initial thrust points into the wall and generates positive normal
    // contact force.
    for (int i = 1; i <= nr_; ++i)
    {
      const std::string name = "gimbal" + std::to_string(i);
      if (model_.existJointName(name))
      {
        const pinocchio::JointIndex jid = model_.getJointId(name);
        if (model_.joints[jid].nq() == 1)
          q_ref_[model_.joints[jid].idx_q()] = M_PI;
      }
    }

    pinocchio::forwardKinematics(model_, data_, q_ref_);
    pinocchio::updateFramePlacements(model_, data_);
    Eigen::Vector3d sole_normal = data_.oMf[lfoot_id_].rotation().col(2) +
                                  data_.oMf[rfoot_id_].rotation().col(2);
    if (sole_normal.norm() < 1e-9)
      sole_normal = data_.oMf[lfoot_id_].rotation().col(2);
    sole_normal.normalize();
    const Eigen::Quaterniond align = Eigen::Quaterniond::FromTwoVectors(sole_normal, wall_normal_);
    const Eigen::Quaterniond root(q_ref_[6], q_ref_[3], q_ref_[4], q_ref_[5]);
    const Eigen::Quaterniond aligned_root = (align * root).normalized();
    q_ref_.segment<4>(3) << aligned_root.x(), aligned_root.y(), aligned_root.z(), aligned_root.w();

    // The two tangent equalities used for axis alignment have two disconnected
    // branches (+target and -target).  If the neutral arm starts in the wrong
    // hemisphere, IPOPT cannot move locally from the rejected -target branch.
    // Fold the two pitch joints toward the opposite arm direction so the
    // starting axis is on the branch selected by target_axis.dot(axis) >= 0.
    pinocchio::forwardKinematics(model_, data_, q_ref_);
    pinocchio::updateFramePlacements(model_, data_);
    if (ee_target_axis_.dot(data_.oMf[ee_id_].rotation().col(2)) < 0.0)
    {
      for (const std::string name : { "arm_joint1", "arm_joint2" })
      {
        if (!model_.existJointName(name))
          continue;
        const pinocchio::JointIndex jid = model_.getJointId(name);
        if (model_.joints[jid].nq() != 1)
          continue;
        const int iq = model_.joints[jid].idx_q();
        q_ref_[iq] = std::clamp(0.45 * M_PI, model_.lowerPositionLimit[iq], model_.upperPositionLimit[iq]);
      }
    }

    // Remove the two tangential gauge freedoms, then place the mean sole origin
    // on the wall.  Relative foot motion in the wall remains unconstrained.
    Eigen::Vector3d root_relative = q_ref_.head<3>() - params_.wall_point;
    q_ref_.head<3>() = params_.wall_point + wall_normal_ * wall_normal_.dot(root_relative);
    pinocchio::forwardKinematics(model_, data_, q_ref_);
    pinocchio::updateFramePlacements(model_, data_);
    const double mean_distance =
        0.5 * wall_normal_.dot(data_.oMf[lfoot_id_].translation() + data_.oMf[rfoot_id_].translation() -
                               2.0 * params_.wall_point);
    q_ref_.head<3>() -= mean_distance * wall_normal_;

    // Select the otherwise arbitrary height along the infinite wall so the
    // two contact points and every rotor center are well clear of the takeoff
    // floor.  A small margin keeps the numerical starting point off the bound.
    pinocchio::forwardKinematics(model_, data_, q_ref_);
    pinocchio::updateFramePlacements(model_, data_);
    double minimum_distance = std::numeric_limits<double>::infinity();
    for (const pinocchio::FrameIndex foot_id : foot_ids_)
      minimum_distance = std::min(
          minimum_distance, floor_normal_.dot(data_.oMf[foot_id].translation() - params_.floor_point));
    for (const RotorInfo& rotor : rotors_)
      minimum_distance = std::min(
          minimum_distance, floor_normal_.dot(data_.oMf[rotor.frame_id].translation() - params_.floor_point));
    const double required_lift = params_.minimum_floor_clearance + 0.1 * params_.surface_effect.rotor_radius -
                                 minimum_distance;
    if (required_lift > 0.0)
    {
      const double lift_gain = floor_normal_.dot(floor_lift_direction_);
      q_ref_.head<3>() += floor_lift_direction_ * (required_lift / lift_gain);
    }
  }

  void initializeBounds(const crocoddyl::WrenchCone& reference_cone)
  {
    x_lb_ = Eigen::VectorXd::Constant(variables_.size, -kIpoptInfinity);
    x_ub_ = Eigen::VectorXd::Constant(variables_.size, kIpoptInfinity);

    // Root translation is world-aligned and is bounded about the reference.
    x_lb_.segment<3>(variables_.dq).setConstant(-params_.root_translation_limit);
    x_ub_.segment<3>(variables_.dq).setConstant(params_.root_translation_limit);
    x_lb_.segment<3>(variables_.dq + 3).setConstant(-M_PI);
    x_ub_.segment<3>(variables_.dq + 3).setConstant(M_PI);

    for (pinocchio::JointIndex jid = 2; jid < model_.njoints; ++jid)
    {
      const auto& joint = model_.joints[jid];
      if (joint.nq() != 1 || joint.nv() != 1)
        throw std::runtime_error("only one-DoF actuated joints are supported: " + model_.names[jid]);
      const int iq = joint.idx_q();
      const int iv = joint.idx_v();
      x_lb_[variables_.dq + iv] = finiteBound(model_.lowerPositionLimit[iq] - q_ref_[iq], -2.0 * M_PI);
      x_ub_[variables_.dq + iv] = finiteBound(model_.upperPositionLimit[iq] - q_ref_[iq], 2.0 * M_PI);
    }

    for (int i = 0; i < nj_; ++i)
    {
      const double limit = std::min(std::abs(model_.upperEffortLimit[6 + i]), params_.contact_wrench_limit);
      x_lb_[variables_.tau + i] = -limit;
      x_ub_[variables_.tau + i] = limit;
    }
    for (int i = 0; i < nr_; ++i)
    {
      x_lb_[variables_.thrust + i] = rotors_[i].min_thrust;
      x_ub_[variables_.thrust + i] = rotors_[i].max_thrust;
    }
    x_lb_.segment(variables_.contacts, 12).setConstant(-params_.contact_wrench_limit);
    x_ub_.segment(variables_.contacts, 12).setConstant(params_.contact_wrench_limit);
    x_lb_[variables_.eta] = 0.0;
    x_ub_[variables_.eta] = params_.maximum_ee_torque;

    g_lb_ = Eigen::VectorXd::Constant(constraints_.size, -kIpoptInfinity);
    g_ub_ = Eigen::VectorXd::Constant(constraints_.size, kIpoptInfinity);
    g_lb_.segment(constraints_.statics, nv_).setZero();
    g_ub_.segment(constraints_.statics, nv_).setZero();

    int offset = constraints_.feet;
    for (int foot = 0; foot < 2; ++foot)
    {
      // Foot origin on the plane; two tangent components of the sole normal
      // vanish; the final inequality selects +normal instead of -normal.
      g_lb_[offset] = g_ub_[offset] = 0.0;
      const double tangent_tolerance = std::sin(params_.foot_parallel_tolerance);
      g_lb_[offset + 1] = g_lb_[offset + 2] = -tangent_tolerance;
      g_ub_[offset + 1] = g_ub_[offset + 2] = tangent_tolerance;
      // The two tangent bounds make the sole normal parallel to the wall
      // normal. This non-degenerate inequality chooses +normal rather than
      // -normal; using dot(n,z) ~= 1 here would have a zero Jacobian.
      g_lb_[offset + 3] = 0.0;
      g_ub_[offset + 3] = kIpoptInfinity;
      offset += 4;

      for (int j = 0; j < cone_size_; ++j)
      {
        g_lb_[offset + j] = finiteBound(reference_cone.get_lb()[j], -kIpoptInfinity);
        g_ub_[offset + j] = finiteBound(reference_cone.get_ub()[j], kIpoptInfinity);
      }
      // WrenchCone's normal-force row is nf=4 for the default four facets.
      g_lb_[offset + static_cast<int>(reference_cone.get_nf())] = params_.minimum_contact_force;
      offset += cone_size_;
    }

    g_lb_.segment<5>(constraints_.end_effector).setZero();
    g_ub_.segment<5>(constraints_.end_effector).setZero();
    g_lb_[constraints_.end_effector + 5] = 0.0;
    g_ub_[constraints_.end_effector + 5] = kIpoptInfinity;

    for (int i = 0; i < nr_; ++i)
    {
      g_lb_[constraints_.surface + i] = 1e-8;
      g_ub_[constraints_.surface + i] = kIpoptInfinity;
    }

    g_lb_.segment(constraints_.floor_clearance, nr_ + 2).setConstant(params_.minimum_floor_clearance);
    g_ub_.segment(constraints_.floor_clearance, nr_ + 2).setConstant(kIpoptInfinity);
  }

  void initializeStartingPoint()
  {
    x0_ = 0.5 * (x_lb_ + x_ub_);
    for (int i = 0; i < variables_.size; ++i)
    {
      if (!std::isfinite(x0_[i]) || std::abs(x0_[i]) > 1e18)
        x0_[i] = 0.0;
    }
    x0_.segment(variables_.dq, nv_).setZero();
    x0_.segment(variables_.tau, nj_).setZero();
    x0_.segment(variables_.contacts, 12).setZero();
    x0_[variables_.eta] = 0.0;

    const double mass = [&]() {
      double value = 0.0;
      for (const auto& inertia : model_.inertias)
        value += inertia.mass();
      return value;
    }();
    const double weight = mass * model_.gravity.linear().norm();
    const double normal_force =
        std::max(2.0 * params_.minimum_contact_force, 1.2 * weight / params_.friction_coefficient);
    for (int foot = 0; foot < 2; ++foot)
    {
      Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
      wrench.head<3>() = 0.5 * normal_force * wall_normal_ - 0.5 * mass * model_.gravity.linear();
      x0_.segment<6>(variables_.contacts + 6 * foot) = wrench;
    }

    pinocchio::forwardKinematics(model_, data_, q_ref_);
    pinocchio::updateFramePlacements(model_, data_);
    for (int i = 0; i < nr_; ++i)
    {
      const pinocchio::SE3& pose = data_.oMf[rotors_[i].frame_id];
      const SurfaceEffect effect =
          surface_model_.evaluate(pose.translation(), pose.rotation() * Eigen::Vector3d::UnitZ(), params_.wall_point,
                                  wall_normal_);
      const double free_air = normal_force / (nr_ * std::max(effect.thrust_ratio, 1e-6));
      x0_[variables_.thrust + i] =
          std::clamp(free_air, x_lb_[variables_.thrust + i], x_ub_[variables_.thrust + i]);
    }

    // Cancel the actuated-joint part of the initial static residual as far as
    // the torque limits permit. Base equilibrium is handled by IPOPT.
    Eigen::VectorXd g(constraints_.size);
    if (evaluateConstraints(x0_, g))
    {
      for (int i = 0; i < nj_; ++i)
      {
        x0_[variables_.tau + i] =
            std::clamp(-g[constraints_.statics + 6 + i], x_lb_[variables_.tau + i], x_ub_[variables_.tau + i]);
      }
    }
  }

  bool evaluateConstraints(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> g)
  {
    if (x.size() != variables_.size || g.size() != constraints_.size || !x.allFinite())
      return false;

    const Eigen::VectorXd q = configuration(x);
    if (!q.allFinite())
      return false;
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    Eigen::VectorXd generalized = -pinocchio::computeGeneralizedGravity(model_, data_, q);
    generalized.tail(nj_) += x.segment(variables_.tau, nj_);

    Eigen::MatrixXd jacobian(6, nv_);
    for (int i = 0; i < nr_; ++i)
    {
      jacobian.setZero();
      pinocchio::getFrameJacobian(model_, data_, rotors_[i].frame_id, pinocchio::LOCAL, jacobian);
      const pinocchio::SE3& pose = data_.oMf[rotors_[i].frame_id];
      const SurfaceEffect effect =
          surface_model_.evaluate(pose.translation(), pose.rotation() * Eigen::Vector3d::UnitZ(), params_.wall_point,
                                  wall_normal_);

      pinocchio::Force unit = pinocchio::Force::Zero();
      unit.linear() = Eigen::Vector3d::UnitZ();
      const double drag = (rotors_[i].direction == 1 ? -1.0 : 1.0) * std::abs(params_.moment_to_force_rate);
      unit.angular() = drag * Eigen::Vector3d::UnitZ();
      generalized.noalias() += jacobian.transpose() * unit.toVector() *
                               (effect.thrust_ratio * x[variables_.thrust + i]);
    }

    for (int foot = 0; foot < 2; ++foot)
    {
      jacobian.setZero();
      pinocchio::getFrameJacobian(model_, data_, foot_ids_[foot], pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
      generalized.noalias() += jacobian.transpose() * x.segment<6>(variables_.contacts + 6 * foot);
    }

    jacobian.setZero();
    pinocchio::getFrameJacobian(model_, data_, ee_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    pinocchio::Force ee_wrench = pinocchio::Force::Zero();
    ee_wrench.angular() = ee_target_axis_ * x[variables_.eta];
    // x[eta] is the torque exerted by the robot; the environment applies its
    // negative to the robot in the static balance.
    generalized.noalias() -= jacobian.transpose() * ee_wrench.toVector();
    g.segment(constraints_.statics, nv_) = generalized;

    int offset = constraints_.feet;
    for (int foot = 0; foot < 2; ++foot)
    {
      const pinocchio::SE3& pose = data_.oMf[foot_ids_[foot]];
      const Eigen::Vector3d sole_normal = pose.rotation().col(2);
      g[offset++] = wall_normal_.dot(pose.translation() - params_.wall_point);
      g[offset++] = wall_tangent1_.dot(sole_normal);
      g[offset++] = wall_tangent2_.dot(sole_normal);
      g[offset++] = wall_normal_.dot(sole_normal);

      crocoddyl::WrenchCone cone(pose.rotation(), params_.friction_coefficient, params_.foot_size);
      cone.set_min_nforce(params_.minimum_contact_force);
      cone.update();
      g.segment(offset, cone_size_).noalias() =
          cone.get_A() * x.segment<6>(variables_.contacts + 6 * foot);
      offset += cone_size_;
    }

    const pinocchio::SE3& ee_pose = data_.oMf[ee_id_];
    const Eigen::Vector3d ee_axis = ee_pose.rotation() * Eigen::Vector3d::UnitZ();
    g.segment<3>(constraints_.end_effector) = ee_pose.translation() - ee_target_position_;
    g[constraints_.end_effector + 3] = ee_axis_tangent1_.dot(ee_axis);
    g[constraints_.end_effector + 4] = ee_axis_tangent2_.dot(ee_axis);
    g[constraints_.end_effector + 5] = ee_target_axis_.dot(ee_axis);

    for (int i = 0; i < nr_; ++i)
    {
      const pinocchio::SE3& pose = data_.oMf[rotors_[i].frame_id];
      const SurfaceEffect effect =
          surface_model_.evaluate(pose.translation(), pose.rotation() * Eigen::Vector3d::UnitZ(), params_.wall_point,
                                  wall_normal_);
      g[constraints_.surface + i] = effect.radicand;
    }


    int floor_offset = constraints_.floor_clearance;
    for (const pinocchio::FrameIndex foot_id : foot_ids_)
      g[floor_offset++] = floor_normal_.dot(data_.oMf[foot_id].translation() - params_.floor_point);
    for (const RotorInfo& rotor : rotors_)
      g[floor_offset++] = floor_normal_.dot(data_.oMf[rotor.frame_id].translation() - params_.floor_point);
    return g.allFinite();
  }

  StaticWrenchOptimizerParams params_;
  std::vector<RotorInfo> rotors_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  TiltAwareSurfaceEffect surface_model_;

  pinocchio::FrameIndex lfoot_id_;
  pinocchio::FrameIndex rfoot_id_;
  pinocchio::FrameIndex ee_id_;
  std::vector<pinocchio::FrameIndex> foot_ids_;
  Eigen::Vector3d wall_normal_;
  Eigen::Vector3d wall_tangent1_;
  Eigen::Vector3d wall_tangent2_;
  Eigen::Vector3d ee_target_position_;
  Eigen::Vector3d ee_target_axis_;
  Eigen::Vector3d ee_axis_tangent1_;
  Eigen::Vector3d ee_axis_tangent2_;
  Eigen::Vector3d floor_normal_;
  Eigen::Vector3d floor_lift_direction_;

  int nq_ = 0;
  int nv_ = 0;
  int nj_ = 0;
  int nr_ = 0;
  int cone_size_ = 0;
  VariableLayout variables_;
  ConstraintLayout constraints_;

  Eigen::VectorXd q_ref_;
  Eigen::VectorXd x_lb_;
  Eigen::VectorXd x_ub_;
  Eigen::VectorXd g_lb_;
  Eigen::VectorXd g_ub_;
  Eigen::VectorXd x0_;
  Eigen::VectorXd solution_x_;
  Eigen::VectorXd solution_g_;
  Ipopt::SolverReturn solution_status_ = Ipopt::INTERNAL_ERROR;
};

class StaticWrenchOptimizer::Impl
{
public:
  Impl(const std::string& robot_description, const StaticWrenchOptimizerParams& params)
    : params(params), nlp(new StaticWrenchNlp(robot_description, params))
  {
  }

  StaticWrenchOptimizerParams params;
  Ipopt::SmartPtr<StaticWrenchNlp> nlp;
};

StaticWrenchOptimizer::StaticWrenchOptimizer(const std::string& robot_description,
                                             const StaticWrenchOptimizerParams& params)
  : impl_(std::make_unique<Impl>(robot_description, params))
{
}

StaticWrenchOptimizer::~StaticWrenchOptimizer() = default;

StaticWrenchOptimizerResult StaticWrenchOptimizer::solve()
{
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->Options()->SetIntegerValue("print_level", impl_->params.print_level);
  app->Options()->SetIntegerValue("max_iter", impl_->params.max_iterations);
  app->Options()->SetNumericValue("tol", impl_->params.tolerance);
  app->Options()->SetNumericValue("acceptable_tol", 10.0 * impl_->params.tolerance);
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  app->Options()->SetStringValue("mu_strategy", "adaptive");
  app->Options()->SetStringValue("honor_original_bounds", "yes");
  app->Options()->SetNumericValue("bound_relax_factor", 0.0);

  const Ipopt::ApplicationReturnStatus init_status = app->Initialize();
  if (init_status != Ipopt::Solve_Succeeded)
  {
    StaticWrenchOptimizerResult result;
    result.solver_status = static_cast<int>(init_status);
    result.solver_status_text = "IPOPT initialization failed: " + statusText(init_status);
    return result;
  }
  const Ipopt::ApplicationReturnStatus status = app->OptimizeTNLP(impl_->nlp);
  return impl_->nlp->result(status);
}

const pinocchio::Model& StaticWrenchOptimizer::model() const
{
  return impl_->nlp->model();
}

bool StaticWrenchOptimizer::saveResult(const std::string& filename, const StaticWrenchOptimizerResult& result) const
{
  const std::filesystem::path output_path(filename);
  if (!output_path.parent_path().empty())
    std::filesystem::create_directories(output_path.parent_path());
  std::ofstream out(filename);
  if (!out)
    return false;
  out << std::setprecision(12) << std::boolalpha;
  out << "success: " << result.success << "\n";
  out << "solver_status: \"" << result.solver_status_text << "\"\n";
  out << "maximum_constraint_violation: " << result.maximum_constraint_violation << "\n";
  out << "ee_wrench_arm_ee: [0, 0, 0, 0, 0, " << result.ee_torque << "]\n";

  auto vector = [&out](const std::string& name, const auto& value) {
    out << name << ": [";
    for (Eigen::Index i = 0; i < value.size(); ++i)
      out << (i == 0 ? "" : ", ") << value[i];
    out << "]\n";
  };
  vector("wall_point_world", impl_->params.wall_point);
  vector("wall_normal_world", impl_->params.wall_normal.normalized());
  vector("ee_wall_target_point_world", impl_->params.ee_target_point);
  out << "ee_offset_along_wall_normal: " << impl_->params.ee_offset << "\n";
  vector("ee_target_point_world", impl_->params.ee_target_point +
                                      impl_->params.ee_offset * impl_->params.wall_normal.normalized());
  vector("ee_target_axis_input", impl_->params.ee_target_axis);
  vector("ee_target_axis_world", impl_->params.ee_target_axis.normalized());
  vector("ee_position_world", result.ee_position_world);
  vector("ee_axis_world", result.ee_axis_world);
  vector("configuration", result.configuration);
  out << "joint_names: [";
  for (std::size_t i = 0; i < result.joint_names.size(); ++i)
    out << (i == 0 ? "" : ", ") << result.joint_names[i];
  out << "]\n";
  vector("joint_positions", result.joint_positions);
  vector("joint_torques", result.joint_torques);
  vector("rotor_thrusts_free_air", result.rotor_thrusts);
  vector("rotor_thrusts_actual", result.actual_rotor_thrusts);
  for (std::size_t i = 0; i < result.contact_wrenches.size(); ++i)
  {
    if (i < result.contact_positions_world.size())
      vector("contact_point_" + std::to_string(i + 1) + "_world", result.contact_positions_world[i]);
    vector("contact_wrench_" + std::to_string(i + 1) + "_world", result.contact_wrenches[i].toVector());
  }
  out << "floor_clearances: [";
  for (std::size_t i = 0; i < result.floor_clearances.size(); ++i)
  {
    if (i)
      out << ", ";
    out << result.floor_clearances[i];
  }
  out << "]\n";
  for (std::size_t i = 0; i < result.surface_effects.size(); ++i)
  {
    const SurfaceEffect& e = result.surface_effects[i];
    out << "surface_effect_" << i + 1 << ": {distance: " << e.signed_distance
        << ", normalized_distance: " << e.normalized_distance << ", tilt: " << e.tilt
        << ", radicand: " << e.radicand << ", thrust_ratio: " << e.thrust_ratio << "}\n";
  }

  // This section follows mujoco_ros' initial-joint-state schema, so the same
  // result file can be passed directly to launch_server.launch.
  if (result.configuration.size() >= 7 && result.joint_names.size() ==
                                               static_cast<std::size_t>(result.joint_positions.size()))
  {
    out << "initial_joint_positions:\n  joint_map:\n";
    out << "    root_joint: \"" << result.configuration[0] << " " << result.configuration[1] << " "
        << result.configuration[2] << " " << result.configuration[6] << " " << result.configuration[3] << " "
        << result.configuration[4] << " " << result.configuration[5] << "\"\n";
    for (std::size_t i = 0; i < result.joint_names.size(); ++i)
      out << "    " << result.joint_names[i] << ": \"" << result.joint_positions[i] << "\"\n";

    out << "initial_joint_velocities:\n  joint_map:\n";
    out << "    root_joint: \"0 0 0 0 0 0\"\n";
    for (const std::string& name : result.joint_names)
      out << "    " << name << ": \"0\"\n";
  }
  return true;
}
}  // namespace flying_humanoid
