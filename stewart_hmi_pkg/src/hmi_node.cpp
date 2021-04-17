#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"

#include "stewart_interfaces_pkg/msg/transforms_pos_vel.hpp"
#include "oscillator.h"

using std::placeholders::_1;
using namespace std::chrono_literals;


class HMI : public rclcpp::Node
{
    public:
        HMI()
        : Node("hmi"), oscillator(new Oscillator(&_timecount))
        {   
            RCLCPP_INFO(this->get_logger(), "HMI Node running!");

            transform_publisher = this->create_publisher<stewart_interfaces_pkg::msg::TransformsPosVel>(
                "dof_ref", 1);
            
            RCLCPP_INFO(this->get_logger(), "Publishing transformations (DOF) ref on dof_ref topic");

            timer_ = this->create_wall_timer(
                10ms, std::bind(&HMI::timer_callback, this));
        }

        ~HMI()
        {
            delete oscillator;
        }

    private:
        void timer_callback()
        {
            static std::vector<std::vector<float>> output(2, std::vector<float>(6));

            oscillator->sample_sine(output);

            auto msg_ = stewart_interfaces_pkg::msg::TransformsPosVel();
            
            msg_.pose.x = output[0][0];
            msg_.pose.y = output[0][1];
            msg_.pose.z = output[0][2];
            
            msg_.twist.x = output[1][0];
            msg_.twist.y = output[1][1];
            msg_.twist.z = output[1][2];

            msg_.pose.roll  = output[0][3];
            msg_.pose.pitch = output[0][4];
            msg_.pose.yaw   = output[0][5];

            msg_.twist.roll  = output[1][3];
            msg_.twist.pitch = output[1][4];
            msg_.twist.yaw   = output[1][5];

            transform_publisher->publish(msg_);
            
            this->_timecount += 0.01;
        }
        float _timecount = 0.;
        Oscillator* oscillator;
        rclcpp::Publisher<stewart_interfaces_pkg::msg::TransformsPosVel>::SharedPtr transform_publisher;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HMI>());
    rclcpp::shutdown();
    return 0;
}



