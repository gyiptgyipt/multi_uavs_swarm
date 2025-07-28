/**
 * @brief Offboard control for multiple vehicles (3)
 * @file offboard_control_multi.cpp
 * @author Adapted version
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControlMulti : public rclcpp::Node
{
public:
	OffboardControlMulti()
		: Node("offboard_control_multi")
	{
		vehicle_names_ = {"px4_1", "px4_2", "px4_3"};

		for (const auto &name : vehicle_names_) {
			VehicleState state;
			state.name = name;
			state.setpoint_counter = 0;

			std::string base_topic = "/" + name + "/fmu/in";

			state.offboard_control_mode_pub = this->create_publisher<OffboardControlMode>(base_topic + "/offboard_control_mode", 10);
			state.trajectory_setpoint_pub = this->create_publisher<TrajectorySetpoint>(base_topic + "/trajectory_setpoint", 10);
			state.vehicle_command_pub = this->create_publisher<VehicleCommand>(base_topic + "/vehicle_command", 10);

			vehicles_.push_back(state);
		}

		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControlMulti::timer_callback, this));
	}

private:
	struct VehicleState {
		std::string name;
		uint64_t setpoint_counter;
		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub;
		rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub;
	};

	std::vector<std::string> vehicle_names_;
	std::vector<VehicleState> vehicles_;
	rclcpp::TimerBase::SharedPtr timer_;

	void timer_callback()
	{
		uint64_t timestamp = this->get_clock()->now().nanoseconds() / 1000;

		for (auto &vehicle : vehicles_) {

			// Send set mode and arm after 10 setpoints
			if (vehicle.setpoint_counter == 10) {
				publish_vehicle_command(vehicle, VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				publish_vehicle_command(vehicle, VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
				RCLCPP_INFO(this->get_logger(), "[%s] Offboard mode + Arm command sent", vehicle.name.c_str());
			}

			// Publish Offboard Control Mode
			OffboardControlMode offboard_msg{};
			offboard_msg.position = true;
			offboard_msg.velocity = false;
			offboard_msg.acceleration = false;
			offboard_msg.attitude = false;
			offboard_msg.body_rate = false;
			offboard_msg.timestamp = timestamp;
			vehicle.offboard_control_mode_pub->publish(offboard_msg);

			// Publish trajectory setpoint
			TrajectorySetpoint traj_msg{};
			traj_msg.position = {0.0, 0.0, -5.0};  // Hover at 5m height
			traj_msg.yaw = -3.14;                  // Face 180 degrees
			traj_msg.timestamp = timestamp;
			vehicle.trajectory_setpoint_pub->publish(traj_msg);

			if (vehicle.setpoint_counter < 11) {
				vehicle.setpoint_counter++;
			}
		}
	}

	void publish_vehicle_command(VehicleState &vehicle, uint16_t command, float param1 = 0.0, float param2 = 0.0)
	{
		VehicleCommand cmd{};
		cmd.param1 = param1;
		cmd.param2 = param2;
		cmd.command = command;
		cmd.target_system = 1;
		cmd.target_component = 1;
		cmd.source_system = 1;
		cmd.source_component = 1;
		cmd.from_external = true;
		cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		vehicle.vehicle_command_pub->publish(cmd);
	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting multi-vehicle offboard control node..." << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControlMulti>());
	rclcpp::shutdown();
	return 0;
}
