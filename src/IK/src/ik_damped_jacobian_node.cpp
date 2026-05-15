#include <rclcpp/rclcpp.hpp>

#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Kinematics: base_link → SP_1 → FE_1 → RU_1
//
//   joint_1: axis -Y  (in base_link frame)
//   joint_2: axis -Z  (in SP_1 frame)
//   joint_3: axis +X  (in FE_1 frame)
//
// Orientation FK:
//   R = Ry(-q1) * Rz(-q2) * Rx(q3)
//
// Closed-form IK (from rows of R):
//   q2 = -asin(  R(1,0) )
//   q1 =  atan2(  R(2,0),  R(0,0) )
//   q3 =  atan2( -R(1,2),  R(1,1) )
//
// Angular Jacobian  ω = J(q) * q̇  (columns = joint axes in base frame):
//   col_0 = [  0,  -1,   0  ]
//   col_1 = [  s1,  0,  -c1 ]
//   col_2 = [  c1*c2, -s2, s1*c2 ]

class IKDampedJacobianNode : public rclcpp::Node
{
public:
  IKDampedJacobianNode()
  : Node("ik_damped_jacobian_node")
  {
    input_mode_ = declare_parameter<std::string>("input_mode", "direct");  // "tf" or "direct"

    base_frame_  = declare_parameter<std::string>("base_frame",  "base_link");
    tip_frame_   = declare_parameter<std::string>("tip_frame",   "RU_1");

    publish_topic_ = declare_parameter<std::string>(
      "publish_topic",
      "/joint_trajectory_controller/joint_trajectory");

    orientation_topic_ = declare_parameter<std::string>(
      "orientation_topic", "/desired_orientation");
    angular_velocity_topic_ = declare_parameter<std::string>(
      "angular_velocity_topic", "/desired_angular_velocity");
    angular_acceleration_topic_ = declare_parameter<std::string>(
      "angular_acceleration_topic", "/desired_angular_acceleration");

    damping_lambda_          = declare_parameter<double>("damping_lambda",          0.01);
    max_direct_input_age_s_  = declare_parameter<double>("max_direct_input_age_s",  0.1);

    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(publish_topic_, 10);

    if (input_mode_ == "tf") {
      tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 100,
        std::bind(&IKDampedJacobianNode::tfCallback, this, std::placeholders::_1));

      tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", 100,
        std::bind(&IKDampedJacobianNode::tfStaticCallback, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(),
        "Started in TF mode. base='%s', tip='%s'",
        base_frame_.c_str(), tip_frame_.c_str());

    } else if (input_mode_ == "direct") {
      orientation_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
        orientation_topic_, 10,
        std::bind(&IKDampedJacobianNode::orientationCallback, this, std::placeholders::_1));

      omega_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        angular_velocity_topic_, 10,
        std::bind(&IKDampedJacobianNode::omegaCallback, this, std::placeholders::_1));

      alpha_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        angular_acceleration_topic_, 10,
        std::bind(&IKDampedJacobianNode::alphaCallback, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(),
        "Started in direct mode. orientation='%s', omega='%s', alpha='%s'",
        orientation_topic_.c_str(),
        angular_velocity_topic_.c_str(),
        angular_acceleration_topic_.c_str());
    } else {
      RCLCPP_FATAL(get_logger(), "Unknown input_mode '%s'", input_mode_.c_str());
      throw std::runtime_error("Unknown input_mode");
    }
  }

private:
  // ── Data types ─────────────────────────────────────────────────────────────

  struct TfEdge
  {
    std::string parent;
    std::string child;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  struct MotionInput
  {
    Eigen::Matrix3d R     = Eigen::Matrix3d::Identity();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  // ── Maths helpers ──────────────────────────────────────────────────────────

  static double clamp(double x, double lo, double hi)
  {
    return std::max(lo, std::min(hi, x));
  }

  static double wrapAngle(double a)
  {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static Eigen::Matrix3d rotX(double a)
  {
    return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()).toRotationMatrix();
  }

  static Eigen::Matrix3d rotY(double a)
  {
    return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix();
  }

  static Eigen::Matrix3d rotZ(double a)
  {
    return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  }

  // ── FK / IK ────────────────────────────────────────────────────────────────

  // R = Ry(-q1) * Rz(-q2) * Rx(q3)
  static Eigen::Matrix3d forwardOrientation(const Eigen::Vector3d & q)
  {
    return rotY(-q[0]) * rotZ(-q[1]) * rotX(q[2]);
  }

  // Closed-form solution for R = Ry(-q1)*Rz(-q2)*Rx(q3):
  //   R(1,0) = -sin(q2)
  //   R(0,0) =  cos(q1)*cos(q2)
  //   R(2,0) =  sin(q1)*cos(q2)
  //   R(1,1) =  cos(q2)*cos(q3)
  //   R(1,2) = -cos(q2)*sin(q3)
  static Eigen::Vector3d solveOrientationIK(const Eigen::Matrix3d & R)
  {
    const double q2 = -std::asin(clamp(R(1, 0), -1.0, 1.0));
    const double q1 =  std::atan2( R(2, 0), R(0, 0));
    const double q3 =  std::atan2(-R(1, 2), R(1, 1));
    return Eigen::Vector3d(q1, q2, q3);
  }

  // ── Jacobian ───────────────────────────────────────────────────────────────

  // ω = J(q) * q̇
  // col_0 = [0, -1, 0]                 (joint_1 axis -Y in base)
  // col_1 = [s1, 0, -c1]               (Ry(-q1)*[0,0,-1])
  // col_2 = [c1*c2, -s2, s1*c2]        (Ry(-q1)*Rz(-q2)*[1,0,0])
  static Eigen::Matrix3d angularJacobian(const Eigen::Vector3d & q)
  {
    const double s1 = std::sin(q[0]), c1 = std::cos(q[0]);
    const double s2 = std::sin(q[1]), c2 = std::cos(q[1]);

    Eigen::Matrix3d J;
    J.col(0) <<  0.0, -1.0,  0.0;
    J.col(1) <<  s1,   0.0, -c1;
    J.col(2) <<  c1 * c2, -s2,  s1 * c2;
    return J;
  }

  // Ĵ = dJ/dt
  // d(col_1)/dt = [c1*q̇1, 0, s1*q̇1]
  // d(col_2)/dt = [-s1*c2*q̇1 - c1*s2*q̇2,  -c2*q̇2,  c1*c2*q̇1 - s1*s2*q̇2]
  static Eigen::Matrix3d angularJacobianDot(
    const Eigen::Vector3d & q,
    const Eigen::Vector3d & qdot)
  {
    const double s1 = std::sin(q[0]), c1 = std::cos(q[0]);
    const double s2 = std::sin(q[1]), c2 = std::cos(q[1]);
    const double q1d = qdot[0], q2d = qdot[1];

    Eigen::Matrix3d Jd = Eigen::Matrix3d::Zero();

    Jd.col(1) <<  c1 * q1d,  0.0,  s1 * q1d;

    Jd.col(2) <<
      -s1 * c2 * q1d - c1 * s2 * q2d,
      -c2 * q2d,
       c1 * c2 * q1d - s1 * s2 * q2d;

    return Jd;
  }

  // J† = Jᵀ (J Jᵀ + λ² I)⁻¹  — damped least-squares pseudo-inverse
  static Eigen::Matrix3d dampedPseudoInverse(const Eigen::Matrix3d & J, double lambda)
  {
    const Eigen::Matrix3d JJt = J * J.transpose();
    return J.transpose() * (JJt + lambda * lambda * Eigen::Matrix3d::Identity()).inverse();
  }

  // ── Orientation error (log map) ────────────────────────────────────────────

  static Eigen::Vector3d vee(const Eigen::Matrix3d & S)
  {
    return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
  }

  static Eigen::Vector3d rotationLog(const Eigen::Matrix3d & R)
  {
    const double cos_theta = clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
    const double theta = std::acos(cos_theta);
    if (theta < 1e-9) return Eigen::Vector3d::Zero();
    const Eigen::Matrix3d skew = (R - R.transpose()) / (2.0 * std::sin(theta));
    return theta * vee(skew);
  }

  // ── Nearest equivalent (branch disambiguation) ─────────────────────────────

  Eigen::Vector3d chooseNearestEquivalent(const Eigen::Vector3d & q_raw) const
  {
    if (!have_last_q_) {
      return Eigen::Vector3d(
        wrapAngle(q_raw[0]), wrapAngle(q_raw[1]), wrapAngle(q_raw[2]));
    }

    Eigen::Vector3d best = q_raw;
    double best_cost = (best - last_q_).squaredNorm();

    for (int k1 = -1; k1 <= 1; ++k1) {
      for (int k2 = -1; k2 <= 1; ++k2) {
        for (int k3 = -1; k3 <= 1; ++k3) {
          Eigen::Vector3d cand = q_raw;
          cand[0] += 2.0 * M_PI * k1;
          cand[1] += 2.0 * M_PI * k2;
          cand[2] += 2.0 * M_PI * k3;
          const double cost = (cand - last_q_).squaredNorm();
          if (cost < best_cost) { best = cand; best_cost = cost; }
        }
      }
    }
    return best;
  }

  // ── TF store / chain lookup ────────────────────────────────────────────────

  static Eigen::Isometry3d transformMsgToEigen(const geometry_msgs::msg::Transform & tf)
  {
    Eigen::Quaterniond q(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = q.normalized().toRotationMatrix();
    T.translation() << tf.translation.x, tf.translation.y, tf.translation.z;
    return T;
  }

  static Eigen::Matrix3d quatMsgToRotation(const geometry_msgs::msg::Quaternion & q_msg)
  {
    return Eigen::Quaterniond(q_msg.w, q_msg.x, q_msg.y, q_msg.z)
           .normalized().toRotationMatrix();
  }

  bool lookupChainTransform(
    const std::string & base,
    const std::string & tip,
    Eigen::Isometry3d & T_out,
    rclcpp::Time & stamp_out)
  {
    T_out = Eigen::Isometry3d::Identity();
    std::string current = tip;
    bool found_any = false;
    rclcpp::Time latest_stamp(0, 0, RCL_ROS_TIME);

    while (current != base) {
      TfEdge edge;
      auto it = tf_edges_.find(current);
      if (it != tf_edges_.end()) {
        edge = it->second;
      } else {
        auto it_s = tf_static_edges_.find(current);
        if (it_s == tf_static_edges_.end()) return false;
        edge = it_s->second;
      }
      T_out = edge.T * T_out;
      if (edge.stamp > latest_stamp) latest_stamp = edge.stamp;
      current = edge.parent;
      found_any = true;
    }
    if (!found_any) return false;
    stamp_out = latest_stamp;
    return true;
  }

  // ── ROS callbacks ──────────────────────────────────────────────────────────

  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & tf : msg->transforms) {
      TfEdge edge;
      edge.parent = tf.header.frame_id;
      edge.child  = tf.child_frame_id;
      edge.T      = transformMsgToEigen(tf.transform);
      edge.stamp  = rclcpp::Time(tf.header.stamp);
      tf_edges_[edge.child] = edge;
    }
    tryProcessTfInput();
  }

  void tfStaticCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & tf : msg->transforms) {
      TfEdge edge;
      edge.parent = tf.header.frame_id;
      edge.child  = tf.child_frame_id;
      edge.T      = transformMsgToEigen(tf.transform);
      edge.stamp  = rclcpp::Time(tf.header.stamp);
      tf_static_edges_[edge.child] = edge;
    }
    tryProcessTfInput();
  }

  void tryProcessTfInput()
  {
    Eigen::Isometry3d T_base_tip;
    rclcpp::Time stamp;
    if (!lookupChainTransform(base_frame_, tip_frame_, T_base_tip, stamp)) return;

    if (have_last_processed_stamp_ && stamp <= last_processed_stamp_) return;

    MotionInput in;
    in.R     = T_base_tip.linear();
    in.stamp = stamp;

    if (have_last_tf_R_) {
      const double dt = (stamp - last_tf_stamp_).seconds();
      if (dt > 1e-9) {
        const Eigen::Matrix3d dR = in.R * last_tf_R_.transpose();
        in.omega = rotationLog(dR) / dt;

        if (have_last_tf_omega_) {
          in.alpha = (in.omega - last_tf_omega_) / dt;
        }
      }
    }

    processMotionInput(in);

    last_tf_R_     = in.R;
    last_tf_stamp_ = stamp;
    last_tf_omega_ = in.omega;
    have_last_tf_R_     = true;
    have_last_tf_omega_ = true;

    last_processed_stamp_  = stamp;
    have_last_processed_stamp_ = true;
  }

  void orientationCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    latest_orientation_ = msg;
    tryProcessDirectInput();
  }

  void omegaCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    latest_omega_ = msg;
  }

  void alphaCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    latest_alpha_ = msg;
  }

  void tryProcessDirectInput()
  {
    if (!latest_orientation_ || !latest_omega_ || !latest_alpha_) return;

    const rclcpp::Time stamp = latest_orientation_->header.stamp;
    if (have_last_processed_stamp_ && stamp <= last_processed_stamp_) return;

    const double age_omega = std::abs((stamp - latest_omega_->header.stamp).seconds());
    const double age_alpha = std::abs((stamp - latest_alpha_->header.stamp).seconds());

    if (age_omega > max_direct_input_age_s_ || age_alpha > max_direct_input_age_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Direct inputs not time-aligned: |dt_omega|=%.6f s, |dt_alpha|=%.6f s",
        age_omega, age_alpha);
      return;
    }

    MotionInput in;
    in.R = quatMsgToRotation(latest_orientation_->quaternion);
    in.omega << latest_omega_->vector.x, latest_omega_->vector.y, latest_omega_->vector.z;
    in.alpha << latest_alpha_->vector.x, latest_alpha_->vector.y, latest_alpha_->vector.z;
    in.stamp = stamp;

    processMotionInput(in);

    last_processed_stamp_      = stamp;
    have_last_processed_stamp_ = true;
  }

  // ── Core IK solve ──────────────────────────────────────────────────────────

  static bool withinJointLimits(const Eigen::Vector3d & q)
  {
    return (q[0] >= -1.134464 && q[0] <=  1.134464) &&
           (q[1] >= -1.047198 && q[1] <=  1.047198) &&
           (q[2] >= -0.530000 && q[2] <=  0.530000);
  }

  void processMotionInput(const MotionInput & in)
  {
    // Position IK
    Eigen::Vector3d q = solveOrientationIK(in.R);
    q = chooseNearestEquivalent(q);

    if (!withinJointLimits(q)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "IK solution outside joint limits — suppressing command");
      return;
    }

    // Verify FK round-trip
    const Eigen::Matrix3d R_check = forwardOrientation(q);
    const double err = rotationLog(in.R * R_check.transpose()).norm();
    if (err > 1e-3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "FK round-trip error = %.6e rad", err);
    }

    // Velocity IK:  q̇ = J†(q) * ω
    const Eigen::Matrix3d J    = angularJacobian(q);
    const Eigen::Matrix3d Jinv = dampedPseudoInverse(J, damping_lambda_);
    const Eigen::Vector3d qdot = Jinv * in.omega;

    // Acceleration IK:  q̈ = J†(q) * (α - J̇(q,q̇) * q̇)
    const Eigen::Matrix3d Jdot  = angularJacobianDot(q, qdot);
    const Eigen::Vector3d qddot = Jinv * (in.alpha - Jdot * qdot);

    publishTrajectory(q, qdot, qddot, in.stamp);

    last_q_      = q;
    have_last_q_ = true;
  }

  // ── Publish ────────────────────────────────────────────────────────────────

  void publishTrajectory(
    const Eigen::Vector3d & q,
    const Eigen::Vector3d & qdot,
    const Eigen::Vector3d & qddot,
    const rclcpp::Time & stamp)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = stamp;
    traj.joint_names  = {"joint_1", "joint_2", "joint_3"};

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions      = {q[0],    q[1],    q[2]};
    pt.velocities     = {qdot[0], qdot[1], qdot[2]};
    pt.accelerations  = {qddot[0],qddot[1],qddot[2]};
    pt.time_from_start.sec     = 0;
    pt.time_from_start.nanosec = 10'000'000;  // 10 ms

    traj.points.push_back(pt);
    traj_pub_->publish(traj);
  }

  // ── Member variables ───────────────────────────────────────────────────────

  std::string input_mode_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string publish_topic_;
  std::string orientation_topic_;
  std::string angular_velocity_topic_;
  std::string angular_acceleration_topic_;

  double damping_lambda_{0.01};
  double max_direct_input_age_s_{0.1};

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr omega_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr alpha_sub_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

  std::unordered_map<std::string, TfEdge> tf_edges_;
  std::unordered_map<std::string, TfEdge> tf_static_edges_;

  geometry_msgs::msg::QuaternionStamped::SharedPtr latest_orientation_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr    latest_omega_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr    latest_alpha_;

  Eigen::Matrix3d last_tf_R_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d last_tf_omega_{Eigen::Vector3d::Zero()};
  rclcpp::Time    last_tf_stamp_{0, 0, RCL_ROS_TIME};
  bool have_last_tf_R_{false};
  bool have_last_tf_omega_{false};

  rclcpp::Time last_processed_stamp_{0, 0, RCL_ROS_TIME};
  bool have_last_processed_stamp_{false};

  Eigen::Vector3d last_q_{Eigen::Vector3d::Zero()};
  bool have_last_q_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKDampedJacobianNode>());
  rclcpp::shutdown();
  return 0;
}
