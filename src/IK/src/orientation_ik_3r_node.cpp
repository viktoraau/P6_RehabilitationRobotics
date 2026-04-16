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
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

class OrientationIK3RNode : public rclcpp::Node
{
public:
  OrientationIK3RNode()
  : Node("orientation_ik_3r_node")
  {
    input_mode_ = declare_parameter<std::string>("input_mode", "tf");  // "tf" or "direct"

    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    tip_frame_ = declare_parameter<std::string>("tip_frame", "Body2__3__1");

    publish_topic_ = declare_parameter<std::string>(
      "publish_topic", "/joint_trajectory_controller/joint_trajectory");

    orientation_topic_ = declare_parameter<std::string>(
      "orientation_topic", "/desired_orientation");
    angular_velocity_topic_ = declare_parameter<std::string>(
      "angular_velocity_topic", "/desired_angular_velocity");
    angular_acceleration_topic_ = declare_parameter<std::string>(
      "angular_acceleration_topic", "/desired_angular_acceleration");

    damping_lambda_ = declare_parameter<double>("damping_lambda", 1e-4);
    singularity_det_threshold_ = declare_parameter<double>("singularity_det_threshold", 1e-6);
    max_direct_input_age_s_ = declare_parameter<double>("max_direct_input_age_s", 0.1);

    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(publish_topic_, 10);

    if (input_mode_ == "tf") {
      tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 100,
        std::bind(&OrientationIK3RNode::tfCallback, this, std::placeholders::_1));

      tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", 100,
        std::bind(&OrientationIK3RNode::tfStaticCallback, this, std::placeholders::_1));

      RCLCPP_INFO(
        get_logger(),
        "Started in TF mode. base_frame='%s', tip_frame='%s'",
        base_frame_.c_str(), tip_frame_.c_str());

    } else if (input_mode_ == "direct") {
      orientation_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
        orientation_topic_, 10,
        std::bind(&OrientationIK3RNode::orientationCallback, this, std::placeholders::_1));

      omega_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        angular_velocity_topic_, 10,
        std::bind(&OrientationIK3RNode::omegaCallback, this, std::placeholders::_1));

      alpha_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        angular_acceleration_topic_, 10,
        std::bind(&OrientationIK3RNode::alphaCallback, this, std::placeholders::_1));

      RCLCPP_INFO(
        get_logger(),
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
  struct TfEdge
  {
    std::string parent;
    std::string child;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  struct MotionInput
  {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();  // expressed in base frame
    Eigen::Vector3d alpha = Eigen::Vector3d::Zero();  // expressed in base frame
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  static double clamp(double x, double lo, double hi)
  {
    return std::max(lo, std::min(hi, x));
  }

  static double wrapAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static Eigen::Vector3d wrapEach(const Eigen::Vector3d & q)
  {
    return Eigen::Vector3d(wrapAngle(q[0]), wrapAngle(q[1]), wrapAngle(q[2]));
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
    Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    return q.normalized().toRotationMatrix();
  }

  // Robot model:
  // joint_1 about +Y
  // joint_2 about -X
  // joint_3 about +Z
  //
  // R = Ry(q1) * Rx(-q2) * Rz(q3)
  static Eigen::Matrix3d forwardOrientation(const Eigen::Vector3d & q)
  {
    return rotY(q[0]) * rotX(-q[1]) * rotZ(q[2]);
  }

  // Closed-form IK for:
  // R(1,2) = sin(q2)
  // R(0,2) = sin(q1) cos(q2)
  // R(2,2) = cos(q1) cos(q2)
  // R(1,0) = sin(q3) cos(q2)
  // R(1,1) = cos(q3) cos(q2)
  static Eigen::Vector3d solveOrientationIK(const Eigen::Matrix3d & R)
  {
    const double q2 = std::asin(clamp(R(1, 2), -1.0, 1.0));
    const double q1 = std::atan2(R(0, 2), R(2, 2));
    const double q3 = std::atan2(R(1, 0), R(1, 1));
    return Eigen::Vector3d(q1, q2, q3);
  }

  Eigen::Vector3d chooseNearestEquivalent(const Eigen::Vector3d & q_raw) const
  {
    if (!have_last_q_) {
      return wrapEach(q_raw);
    }

    Eigen::Vector3d best = q_raw;
    double best_cost = (best - last_q_).squaredNorm();

    for (int k1 = -1; k1 <= 1; ++k1) {
      for (int k2 = -1; k2 <= 1; ++k2) {
        for (int k3 = -1; k3 <= 1; ++k3) {
          Eigen::Vector3d cand = q_raw;
          cand[0] += 2.0 * M_PI * static_cast<double>(k1);
          cand[1] += 2.0 * M_PI * static_cast<double>(k2);
          cand[2] += 2.0 * M_PI * static_cast<double>(k3);

          const double cost = (cand - last_q_).squaredNorm();
          if (cost < best_cost) {
            best = cand;
            best_cost = cost;
          }
        }
      }
    }

    return best;
  }

  bool withinJointLimits(const Eigen::Vector3d & q) const
  {
    // Replace if your URDF limits are different
    return (q[0] >= -1.221731 && q[0] <=  1.221731) &&
           (q[1] >= -0.523599 && q[1] <=  0.523599) &&
           (q[2] >= -1.047198 && q[2] <=  1.047198);
  }

  // Angular velocity Jacobian in base frame:
  // omega = Jw(q) * qdot
  //
  // Columns are the 3 joint axes expressed in base coordinates.
  static Eigen::Matrix3d angularJacobian(const Eigen::Vector3d & q)
  {
    const double q1 = q[0];
    const double q2 = q[1];

    const double s1 = std::sin(q1);
    const double c1 = std::cos(q1);
    const double s2 = std::sin(q2);
    const double c2 = std::cos(q2);

    Eigen::Matrix3d J;
    J.col(0) << 0.0, 1.0, 0.0;
    J.col(1) << -c1, 0.0, s1;
    J.col(2) << s1 * c2, s2, c1 * c2;
    return J;
  }

  static Eigen::Matrix3d angularJacobianDot(const Eigen::Vector3d & q,
                                            const Eigen::Vector3d & qdot)
  {
    const double q1 = q[0];
    const double q2 = q[1];
    const double q1d = qdot[0];
    const double q2d = qdot[1];

    const double s1 = std::sin(q1);
    const double c1 = std::cos(q1);
    const double s2 = std::sin(q2);
    const double c2 = std::cos(q2);

    Eigen::Matrix3d Jdot = Eigen::Matrix3d::Zero();

    Jdot.col(0).setZero();

    // d/dt [-c1, 0, s1]
    Jdot.col(1) << s1 * q1d, 0.0, c1 * q1d;

    // d/dt [s1*c2, s2, c1*c2]
    Jdot.col(2) <<
      (c1 * c2 * q1d - s1 * s2 * q2d),
      (c2 * q2d),
      (-s1 * c2 * q1d - c1 * s2 * q2d);

    return Jdot;
  }

  static Eigen::Matrix3d dampedPseudoInverse(const Eigen::Matrix3d & J, double lambda)
  {
    const Eigen::Matrix3d JJt = J * J.transpose();
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    return J.transpose() * (JJt + lambda * lambda * I).inverse();
  }

  Eigen::Vector3d solveLinear3x3(const Eigen::Matrix3d & A,
                                 const Eigen::Vector3d & b) const
  {
    const double det = std::abs(A.determinant());
    if (det > singularity_det_threshold_) {
      return A.colPivHouseholderQr().solve(b);
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Jacobian near singular; using damped solve");
    return dampedPseudoInverse(A, damping_lambda_) * b;
  }

  static Eigen::Vector3d vee(const Eigen::Matrix3d & S)
  {
    return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
  }

  // Rotation log map: returns rotation vector theta*u
  static Eigen::Vector3d rotationLog(const Eigen::Matrix3d & R)
  {
    const double cos_theta = clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
    const double theta = std::acos(cos_theta);

    if (theta < 1e-9) {
      return Eigen::Vector3d::Zero();
    }

    Eigen::Matrix3d skew = (R - R.transpose()) / (2.0 * std::sin(theta));
    Eigen::Vector3d axis = vee(skew);
    return theta * axis;
  }

  bool lookupChainTransform(const std::string & base,
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
      bool found = false;

      auto it_dyn = tf_edges_.find(current);
      if (it_dyn != tf_edges_.end()) {
        edge = it_dyn->second;
        found = true;
      } else {
        auto it_static = tf_static_edges_.find(current);
        if (it_static != tf_static_edges_.end()) {
          edge = it_static->second;
          found = true;
        }
      }

      if (!found) {
        return false;
      }

      T_out = edge.T * T_out;
      if (edge.stamp > latest_stamp) {
        latest_stamp = edge.stamp;
      }

      current = edge.parent;
      found_any = true;
    }

    if (!found_any) {
      return false;
    }

    stamp_out = latest_stamp;
    return true;
  }

  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    bool touched_chain = false;

    for (const auto & tf : msg->transforms) {
      TfEdge edge;
      edge.parent = tf.header.frame_id;
      edge.child = tf.child_frame_id;
      edge.T = transformMsgToEigen(tf.transform);
      edge.stamp = rclcpp::Time(tf.header.stamp);

      tf_edges_[edge.child] = edge;

      if (edge.child == tip_frame_ || edge.parent == base_frame_) {
        touched_chain = true;
      }
    }

    (void)touched_chain;
    tryProcessTfInput();
  }

  void tfStaticCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & tf : msg->transforms) {
      TfEdge edge;
      edge.parent = tf.header.frame_id;
      edge.child = tf.child_frame_id;
      edge.T = transformMsgToEigen(tf.transform);
      edge.stamp = rclcpp::Time(tf.header.stamp);
      tf_static_edges_[edge.child] = edge;
    }

    tryProcessTfInput();
  }

  void tryProcessTfInput()
  {
    Eigen::Isometry3d T_base_tip;
    rclcpp::Time stamp;

    if (!lookupChainTransform(base_frame_, tip_frame_, T_base_tip, stamp)) {
      return;
    }

    if (have_last_processed_input_stamp_ && stamp <= last_processed_input_stamp_) {
      return;
    }

    MotionInput in;
    in.R = T_base_tip.linear();
    in.stamp = stamp;

    if (have_last_tf_R_) {
      const double dt = (stamp - last_tf_stamp_).seconds();
      if (dt > 1e-9) {
        const Eigen::Matrix3d R_delta = in.R * last_tf_R_.transpose();
        in.omega = rotationLog(R_delta) / dt;

        if (have_last_tf_omega_) {
          in.alpha = (in.omega - last_tf_omega_) / dt;
        } else {
          in.alpha.setZero();
        }
      } else {
        in.omega.setZero();
        in.alpha.setZero();
      }
    } else {
      in.omega.setZero();
      in.alpha.setZero();
    }

    processMotionInput(in);

    last_tf_R_ = in.R;
    last_tf_stamp_ = stamp;
    last_tf_omega_ = in.omega;
    have_last_tf_R_ = true;
    have_last_tf_omega_ = true;

    last_processed_input_stamp_ = stamp;
    have_last_processed_input_stamp_ = true;
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
    if (!latest_orientation_ || !latest_omega_ || !latest_alpha_) {
      return;
    }

    const rclcpp::Time stamp = latest_orientation_->header.stamp;

    if (have_last_processed_input_stamp_ && stamp <= last_processed_input_stamp_) {
      return;
    }

    const double age_omega = std::abs((stamp - latest_omega_->header.stamp).seconds());
    const double age_alpha = std::abs((stamp - latest_alpha_->header.stamp).seconds());

    if (age_omega > max_direct_input_age_s_ || age_alpha > max_direct_input_age_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Direct inputs not time-aligned enough: |dt_omega|=%.6f s, |dt_alpha|=%.6f s",
        age_omega, age_alpha);
      return;
    }

    MotionInput in;
    in.R = quatMsgToRotation(latest_orientation_->quaternion);
    in.omega << latest_omega_->vector.x, latest_omega_->vector.y, latest_omega_->vector.z;
    in.alpha << latest_alpha_->vector.x, latest_alpha_->vector.y, latest_alpha_->vector.z;
    in.stamp = stamp;

    processMotionInput(in);

    last_processed_input_stamp_ = stamp;
    have_last_processed_input_stamp_ = true;
  }

  void processMotionInput(const MotionInput & in)
  {
    Eigen::Vector3d q = solveOrientationIK(in.R);
    q = chooseNearestEquivalent(q);

    //if (!withinJointLimits(q)) {
     // RCLCPP_WARN_THROTTLE(
     //   get_logger(), *get_clock(), 1000,
     //   "IK solution outside joint limits");
     // return;
    //}

    const Eigen::Matrix3d R_check = forwardOrientation(q);
    const double orientation_error = rotationLog(in.R * R_check.transpose()).norm();
    if (orientation_error > 1e-3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Orientation reconstruction error = %.6e rad", orientation_error);
    }

    const Eigen::Matrix3d Jw = angularJacobian(q);
    const Eigen::Vector3d qdot = solveLinear3x3(Jw, in.omega);

    const Eigen::Matrix3d Jwdot = angularJacobianDot(q, qdot);
    const Eigen::Vector3d qddot = solveLinear3x3(Jw, in.alpha - Jwdot * qdot);

    publishTrajectory(q, qdot, qddot, in.stamp);

    last_q_ = q;
    have_last_q_ = true;
  }

  void publishTrajectory(const Eigen::Vector3d & q,
                         const Eigen::Vector3d & qdot,
                         const Eigen::Vector3d & qddot,
                         const rclcpp::Time & stamp)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = stamp;
    traj.joint_names = {"joint_1", "joint_2", "joint_3"};

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {q[0], q[1], q[2]};
    pt.velocities = {qdot[0], qdot[1], qdot[2]};
    pt.accelerations = {qddot[0], qddot[1], qddot[2]};
    pt.time_from_start.sec = 0;
    pt.time_from_start.nanosec = 10000000;  // 10 ms

    traj.points.push_back(pt);
    traj_pub_->publish(traj);
  }

  std::string input_mode_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string publish_topic_;

  std::string orientation_topic_;
  std::string angular_velocity_topic_;
  std::string angular_acceleration_topic_;

  double damping_lambda_{1e-4};
  double singularity_det_threshold_{1e-6};
  double max_direct_input_age_s_{0.25};

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr omega_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr alpha_sub_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

  std::unordered_map<std::string, TfEdge> tf_edges_;
  std::unordered_map<std::string, TfEdge> tf_static_edges_;

  geometry_msgs::msg::QuaternionStamped::SharedPtr latest_orientation_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr latest_omega_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr latest_alpha_;

  Eigen::Matrix3d last_tf_R_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d last_tf_omega_{Eigen::Vector3d::Zero()};
  rclcpp::Time last_tf_stamp_{0, 0, RCL_ROS_TIME};
  bool have_last_tf_R_{false};
  bool have_last_tf_omega_{false};

  rclcpp::Time last_processed_input_stamp_{0, 0, RCL_ROS_TIME};
  bool have_last_processed_input_stamp_{false};

  Eigen::Vector3d last_q_{Eigen::Vector3d::Zero()};
  bool have_last_q_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrientationIK3RNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
