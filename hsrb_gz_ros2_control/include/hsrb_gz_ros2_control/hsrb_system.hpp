/// @copyright Copyright (C) 2025 Toyota Motor Corporation
#ifndef HSRB_GZ_ROS2_CONTROL__GZ_SYSTEM_HPP_
#define HSRB_GZ_ROS2_CONTROL__GZ_SYSTEM_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gz_ros2_control/gz_system_interface.hpp>

#include <pluginlib/class_loader.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <tmc_exxx_servo_motor_protocol/exxx_common.hpp>

namespace hsrb_gz_ros2_control {

struct JointData {
  /// \brief Joint's names.
  std::string name;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief Current cmd joint effort
  double joint_effort_cmd;

  /// \brief flag if joint is actuated (has command interfaces) or passive
  bool is_actuated;

  /// \brief handles to the joints from within Gazebo
  sim::Entity sim_joint;

  /// \brief Control method defined in the URDF for each joint.
  gz_ros2_control::GazeboSimSystemInterface::ControlMethod joint_control_method;
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// TODO(MasayukiMasuda): 他のグリッパーを追加したくなったら，インターフェースクラスを作って管理する
class GazeboSimGripperSystem {
 public:
  GazeboSimGripperSystem()
      : hand_l_spring_proximal_joint_pos_(0.0),
        hand_r_spring_proximal_joint_pos_(0.0),
        hand_l_spring_proximal_joint_vel_(0.0),
        hand_r_spring_proximal_joint_vel_(0.0),
        hand_spring_coeff_(1.0),
        drive_mode_(tmc_exxx_servo_motor_protocol::kDriveModeNoControl),
        drive_mode_cmd_(tmc_exxx_servo_motor_protocol::kDriveModeNoControl),
        grasping_flag_(0.0),
        grasping_flag_cmd_(0.0) {}

  void read(const sim::EntityComponentManager* ecm);
  void write(sim::EntityComponentManager* ecm);

  bool initGripper(
      const std::map<std::string, gz::sim::Entity>& enableJoints,
      const hardware_interface::HardwareInfo& hardware_info,
      const std::string& prefix,
      std::vector<hardware_interface::CommandInterface>& command_interfaces,
      std::vector<hardware_interface::CommandInterface>& parent_command_interfaces,
      std::vector<hardware_interface::StateInterface>& state_interfaces,
      std::vector<hardware_interface::StateInterface>& parent_state_interfaces);

  std::string motor_joint_name() const { return this->motor_joint_.name; }

 private:
  JointData motor_joint_;

  // GazeboSimSystemInterfaceの構造体にアクセスは不可能なので，Interfaceを横取りして，必要な情報を取る，書き換える
  std::unique_ptr<hardware_interface::CommandInterface> command_interface_;
  std::unique_ptr<hardware_interface::StateInterface> state_interface_;

  sim::Entity hand_l_spring_proximal_joint_;
  sim::Entity hand_r_spring_proximal_joint_;
  double hand_l_spring_proximal_joint_pos_;
  double hand_r_spring_proximal_joint_pos_;
  double hand_l_spring_proximal_joint_vel_;
  double hand_r_spring_proximal_joint_vel_;

  // spring coefficient [Nm/rad]
  double hand_spring_coeff_;
  double grasp_velocity_gain_;
  double grasp_effort_tolerance_;

  double drive_mode_;
  double drive_mode_cmd_;
  double grasping_flag_;
  double grasping_flag_cmd_;
  double current_;
};

class GazeboSimSystem : public gz_ros2_control::GazeboSimSystemInterface {
 public:
  GazeboSimSystem()
      : gz_system_loader_("gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface") {}

  CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

  bool initSim(
      rclcpp::Node::SharedPtr& model_nh,
      std::map<std::string, sim::Entity>& joints,
      const hardware_interface::HardwareInfo& hardware_info,
      sim::EntityComponentManager& _ecm,
      int& update_rate) override;

  std::string get_name() const override { return this->parent_->get_name(); }

 private:
  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  std::unique_ptr<gz_ros2_control::GazeboSimSystemInterface> parent_;
  pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> gz_system_loader_;

  sim::EntityComponentManager* ecm_{nullptr};

  std::vector<std::unique_ptr<GazeboSimGripperSystem>> gripper_systems_;

  struct JointSaturation {
    sim::Entity joint;
    double lower;
    double upper;
  };
  std::vector<JointSaturation> joint_saturations_;

  struct DriveMode {
    std::string name;
    double command_value;
    double previous_command_value;

    explicit DriveMode(const std::string& name) : name(name), command_value(0.0), previous_command_value(0.0) {}
  };
  std::vector<DriveMode> drive_modes_;
};

}  // namespace hsrb_gz_ros2_control
#endif  // HSRB_GZ_ROS2_CONTROL__GZ_SYSTEM_HPP_
