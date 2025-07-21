#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class MultiOffboardControl : public rclcpp::Node {
public:
    MultiOffboardControl() : Node("multi_offboard_node") {
        for (int id = 1; id <= 3; ++id) {
            std::string ns = "/px4_" + std::to_string(id);

            // Publishers
            offboard_pub[id] = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                ns + "/fmu/in/offboard_control_mode", 10);

            traj_pub[id] = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                ns + "/fmu/in/trajectory_setpoint", 10);

            cmd_pub[id] = this->create_publisher<px4_msgs::msg::VehicleCommand>(
                ns + "/fmu/in/vehicle_command", 10);

            // Timer for sending commands
            timers[id] = this->create_wall_timer(100ms, [this, id]() {
                publish_offboard_control(id);
            });

            // Arm and set mode
            arm_vehicle(id);
            set_offboard_mode(id);
        }
    }

private:
    std::map<int, rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr> offboard_pub;
    std::map<int, rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr> traj_pub;
    std::map<int, rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr> cmd_pub;
    std::map<int, rclcpp::TimerBase::SharedPtr> timers;

    uint64_t get_timestamp() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    void arm_vehicle(int id) {
        auto msg = px4_msgs::msg::VehicleCommand();
        msg.timestamp = get_timestamp();
        msg.param1 = 1.0;  // Arm
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.target_system = id;
        msg.target_component = 1;
        msg.source_system = id;
        msg.source_component = 1;
        msg.from_external = true;
        cmd_pub[id]->publish(msg);
    }

    void set_offboard_mode(int id) {
        auto msg = px4_msgs::msg::VehicleCommand();
        msg.timestamp = get_timestamp();
        msg.param1 = 1.0;  // Offboard
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.target_system = id;
        msg.target_component = 1;
        msg.source_system = id;
        msg.source_component = 1;
        msg.from_external = true;
        msg.param2 = 6.0;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
        cmd_pub[id]->publish(msg);
    }

    void publish_offboard_control(int id) {
        uint64_t timestamp = get_timestamp();

        // Offboard mode message
        px4_msgs::msg::OffboardControlMode offboard_msg{};
        offboard_msg.timestamp = timestamp;
        offboard_msg.position = true;
        offboard_msg.velocity = false;
        offboard_msg.acceleration = false;
        offboard_msg.attitude = false;
        offboard_msg.body_rate = false;
        offboard_pub[id]->publish(offboard_msg);

        // Trajectory setpoint message
        px4_msgs::msg::TrajectorySetpoint traj_msg{};
        traj_msg.timestamp = timestamp;
        traj_msg.position = {float(id), float(id), -3.0f};  // each UAV moves to a different (x, y)
        traj_msg.yaw = 0.0;
        traj_pub[id]->publish(traj_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiOffboardControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
