#include <chrono>
#include <memory>
extern "C"
{
#include <rc/motor.h>
}

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "robotcontrol_interfaces/msg/motor_duty.hpp"
#include "builtin_interfaces/msg/duration.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MotorDutyControl : public rclcpp::Node
{
public:
    MotorDutyControl() :
        Node("rc_motor")
    {
        rc_motor_init();
        std::function<void(const robotcontrol_interfaces::msg::MotorDuty::SharedPtr)> callback_motor1 =
            std::bind(&MotorDutyControl::motor_duty_control_callback, this, 1, _1);
        std::function<void(const robotcontrol_interfaces::msg::MotorDuty::SharedPtr)> callback_motor2 =
            std::bind(&MotorDutyControl::motor_duty_control_callback, this, 2, _1);
        std::function<void(const robotcontrol_interfaces::msg::MotorDuty::SharedPtr)> callback_motor3 =
            std::bind(&MotorDutyControl::motor_duty_control_callback, this, 3, _1);
        std::function<void(const robotcontrol_interfaces::msg::MotorDuty::SharedPtr)> callback_motor4 =
            std::bind(&MotorDutyControl::motor_duty_control_callback, this, 4, _1);

        subscriber_motor1_ = this->create_subscription<robotcontrol_interfaces::msg::MotorDuty>(
            "motor_1", 1, callback_motor1);
        subscriber_motor2_ = this->create_subscription<robotcontrol_interfaces::msg::MotorDuty>(
            "motor_2", 1, callback_motor2);
        subscriber_motor3_ = this->create_subscription<robotcontrol_interfaces::msg::MotorDuty>(
            "motor_3", 1, callback_motor3);
        subscriber_motor4_ = this->create_subscription<robotcontrol_interfaces::msg::MotorDuty>(
            "motor_4", 1, callback_motor4);

        duration_monitor_ = this->create_wall_timer(10ms, std::bind(&MotorDutyControl::duration_monitor_callback, this));
    }

    ~MotorDutyControl()
    {
        rc_motor_cleanup();
    }

private:
    void motor_duty_control_callback(int channel, const robotcontrol_interfaces::msg::MotorDuty::SharedPtr motor_duty_msg) {
        duration_[channel] = motor_duty_msg->duration;
        if (motor_duty_msg->duty < -1.0) {
            rc_motor_brake(channel);
        } else {
            rc_motor_set(channel, motor_duty_msg->duty);
        }
        motor_set_time_point_[channel] = now();
    }

    void duration_monitor_callback() {
        const auto current_time_point = now();
        for (int channel = 1; channel <= 4; ++channel) {
            if (current_time_point - motor_set_time_point_[channel] > duration_[channel]) {
                rc_motor_brake(channel);
            }
        }
    }

    rclcpp::Subscription<robotcontrol_interfaces::msg::MotorDuty>::SharedPtr subscriber_motor1_;
    rclcpp::Subscription<robotcontrol_interfaces::msg::MotorDuty>::SharedPtr subscriber_motor2_;
    rclcpp::Subscription<robotcontrol_interfaces::msg::MotorDuty>::SharedPtr subscriber_motor3_;
    rclcpp::Subscription<robotcontrol_interfaces::msg::MotorDuty>::SharedPtr subscriber_motor4_;

    rclcpp::TimerBase::SharedPtr duration_monitor_;

    rclcpp::Duration duration_[5] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
    rclcpp::Time motor_set_time_point_[5];
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDutyControl>());
  rclcpp::shutdown();
  return 0;
}
