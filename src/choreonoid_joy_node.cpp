/**
    Joy Topic Publisher
    @author Kenta Suzuki
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <cnoid/Joystick>
#include <unistd.h>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::string device = "/dev/input/js0";
    cnoid::Joystick joystick(device.c_str());

    rclcpp::Node::SharedPtr node  = std::make_shared<rclcpp::Node>("joy_topic_publisher");
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher = node->create_publisher<sensor_msgs::msg::Joy>("joy", 30);
    rclcpp::WallRate loop_rate(60);
    bool stateChanged = false;

    joystick.sigButton().connect(
        [&](int, bool){ stateChanged = true; });
    joystick.sigAxis().connect(
        [&](int, double){ stateChanged = true; });

    if(!joystick.isReady()) {
        RCLCPP_INFO(node->get_logger(), "Joystick is not ready.");
    }
    bool isBeforeInitialReading = true;

    while(rclcpp::ok()) {
        if(!joystick.isReady()) {
            if(!joystick.makeReady()) {
                usleep(500000);
                continue;
            }
        }
        if(isBeforeInitialReading) {
            RCLCPP_INFO(node->get_logger(), "Joystick \"%s\" is ready.", joystick.device().c_str());
            isBeforeInitialReading = false;
        }
        joystick.readCurrentState();
        if(stateChanged) {
            auto joy = sensor_msgs::msg::Joy();
            joy.header.stamp = node->get_clock()->now();
            const int numAxes = joystick.numAxes();
            joy.axes.resize(numAxes);
            for(int i = 0; i < numAxes; ++i) {
                joy.axes[i] = joystick.getPosition(i);
            }
            const int numButtons = joystick.numButtons();
            joy.buttons.resize(numButtons);
            for(int i = 0; i < numButtons; ++i) {
                joy.buttons[i] = joystick.getButtonState(i) ? 1 : 0;
            }
            publisher->publish(joy);
            stateChanged = false;
        }
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}