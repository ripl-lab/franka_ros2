#include <franka_example_controllers/joint_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

controller_interface::return_type
JointImpedanceController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
      JointImpedanceController::update_desired_positions(time);
  updateJointStates();
  Eigen::Map<const Vector7d> coriolis(
    franka_robot_model_->getCoriolisForceVector().data());
  Vector7d q_goal = q_d_;
  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_goal - q_) + d_gains_.cwiseProduct(
        -dq_filtered_) + coriolis;
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn
JointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    sub_desired_joint_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/joint_impedance/joints_desired", 1,
      std::bind(&JointImpedanceController::desiredJointCallback, this, std::placeholders::_1)
    );
    trajectory_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/joint_impedance_controller/follow_joint_trajectory", 10,
      std::bind(&JointImpedanceController::trajectoryCallback, this, std::placeholders::_1)
    );
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model",
                                                   arm_id_));
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  q_d_ = q_;
  return CallbackReturn::SUCCESS;
}

void JointImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void JointImpedanceController::desiredJointCallback(
  const std_msgs::msg::Float64MultiArray& msg) {
  if (msg.data[0]){
    for (auto i = 0; i < num_joints; ++i) {
      q_d_(i) = msg.data[i];
    }
  }
}

void JointImpedanceController::trajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory& msg) {
    if(msg.points.empty()) {
      RCLCPP_WARN(get_node()->get_logger(), "received empty trajectory");
      return;
    }
    
    current_trajectory_ = msg;
    trajectory_start_time_ = get_node()->now();
    trajectory_active_ = true;
    current_segment_ =  0;
  }

  void JointImpedanceController::update_desired_positions(
    const rclcpp::Time& current_time) {
      if(current_trajectory_.points.empty()) {
        trajectory_active_ = false;
        return;
      }

      double elapsed_time = (current_time - trajectory_start_time_).seconds();

      auto& last_point = current_trajectory_.points.back();
      double total_time = rclcpp::Duration(last_point.time_from_start).seconds();

      if (elapsed_time >= total_time) {
            // Trajectory complete - hold final position
            for (int i = 0; i < num_joints; ++i) {
                q_d_(i) = last_point.positions[i];
            }
            trajectory_active_ = false;
            RCLCPP_INFO(get_node()->get_logger(), "Trajectory completed");
            return;
        }
        
        // Find current segment
        size_t next_point_idx = 0;
        for (size_t i = 0; i < current_trajectory_.points.size(); ++i) {
            double point_time = rclcpp::Duration(current_trajectory_.points[i].time_from_start).seconds();
            if (elapsed_time <= point_time) {
                next_point_idx = i;
                break;
            }
        }
        
        if (next_point_idx == 0) {
            // Before first point - use first point
            for (int i = 0; i < num_joints; ++i) {
                q_d_(i) = current_trajectory_.points[0].positions[i];
            }
            return;
        }
        
        // Interpolate between points
        auto& prev_point = current_trajectory_.points[next_point_idx - 1];
        auto& next_point = current_trajectory_.points[next_point_idx];
        
        double prev_time = (next_point_idx == 0) ? 0.0 : 
            rclcpp::Duration(prev_point.time_from_start).seconds();
        double next_time = rclcpp::Duration(next_point.time_from_start).seconds();
        
        // Linear interpolation factor
        double alpha = (elapsed_time - prev_time) / (next_time - prev_time);
        alpha = std::clamp(alpha, 0.0, 1.0);
        
        // Interpolate positions
        for (int i = 0; i < num_joints; ++i) {
            double prev_pos = (next_point_idx == 0) ? next_point.positions[i] : prev_point.positions[i];
            double next_pos = next_point.positions[i];
            q_d_(i) = prev_pos + alpha * (next_pos - prev_pos);
        }

    }


}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceController,
                       controller_interface::ControllerInterface)