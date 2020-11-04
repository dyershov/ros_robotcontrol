#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "robotcontrol_interfaces/msg/motor_duty.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class MecanumDrive : public rclcpp::Node
{
public:
    MecanumDrive() :
        Node("mecanum_drive")
    {
        this->declare_parameter<double>("separation_to_radius_ratio", 7.5);
        this->get_parameter("separation_to_radius_ratio", separation_to_radius_ratio_);

        this->declare_parameter<int>("watchdog_duration_ms", 50);
        unsigned int watchdog_duration_ms = 0;
        this->get_parameter("watchdog_duration_ms", watchdog_duration_ms);
        duration_ = std::chrono::milliseconds(watchdog_duration_ms);

        subscriber_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MecanumDrive::cmd_vel_callback, this, _1));

        publisher_motor1_ = this->create_publisher<robotcontrol_interfaces::msg::MotorDuty>("motor_1", 1);
        publisher_motor2_ = this->create_publisher<robotcontrol_interfaces::msg::MotorDuty>("motor_2", 1);
        publisher_motor3_ = this->create_publisher<robotcontrol_interfaces::msg::MotorDuty>("motor_3", 1);
        publisher_motor4_ = this->create_publisher<robotcontrol_interfaces::msg::MotorDuty>("motor_4", 1);
    }
private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {
        const auto vel_x = cmd_vel_msg->linear.x;
        const auto vel_y = cmd_vel_msg->linear.y;
        const auto ang_z = cmd_vel_msg->angular.z;

        RCLCPP_INFO(this->get_logger(), "Received twist: linear.x = %5.3f, linear.y = %5.3f, angular.z = %5.3f", vel_x, vel_y, ang_z);

        const auto motor1 = vel_x + vel_y - ang_z;
        const auto motor2 = vel_x - vel_y - ang_z;
        const auto motor3 = vel_x + vel_y + ang_z;
        const auto motor4 = vel_x - vel_y + ang_z;

        const auto motor_max = std::max(std::max(std::max(std::abs(motor1), std::abs(motor2)), std::max(std::abs(motor3), std::abs(motor4))), 1.0);

        robotcontrol_interfaces::msg::MotorDuty motor1_msg;
        motor1_msg.duty = motor1 / motor_max;
        motor1_msg.duration = duration_;
        robotcontrol_interfaces::msg::MotorDuty motor2_msg;
        motor2_msg.duty = motor2 / motor_max;
        motor2_msg.duration = duration_;
        robotcontrol_interfaces::msg::MotorDuty motor3_msg;
        motor3_msg.duty = motor3 / motor_max;
        motor3_msg.duration = duration_;
        robotcontrol_interfaces::msg::MotorDuty motor4_msg;
        motor4_msg.duty = motor4 / motor_max;
        motor4_msg.duration = duration_;

        RCLCPP_INFO(this->get_logger(), "Output motor duty: motor1 = %5.3f, motor2 = %5.3f, motor3 = %5.3f, motor4 = %5.3f",
                    motor1_msg.duty, motor2_msg.duty, motor3_msg.duty, motor4_msg.duty);
        publisher_motor1_->publish(motor1_msg);
        publisher_motor2_->publish(motor2_msg);
        publisher_motor3_->publish(motor3_msg);
        publisher_motor4_->publish(motor4_msg);
    }

    double separation_to_radius_ratio_;
    rclcpp::Duration duration_{0,0};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_cmd_vel_;

    rclcpp::Publisher<robotcontrol_interfaces::msg::MotorDuty>::SharedPtr publisher_motor1_;
    rclcpp::Publisher<robotcontrol_interfaces::msg::MotorDuty>::SharedPtr publisher_motor2_;
    rclcpp::Publisher<robotcontrol_interfaces::msg::MotorDuty>::SharedPtr publisher_motor3_;
    rclcpp::Publisher<robotcontrol_interfaces::msg::MotorDuty>::SharedPtr publisher_motor4_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumDrive>());
  rclcpp::shutdown();
  return 0;
}
