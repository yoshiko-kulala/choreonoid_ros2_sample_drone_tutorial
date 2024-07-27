/**
   Twist Controller
   @author Kenta Suzuki
*/

#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <memory>
#include <mutex>

class TwistTopicSubscriberController : public cnoid::SimpleController, public cnoid::JoystickInterface
{
public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override;
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;

    virtual int numAxes() const override;
    virtual int numButtons() const override;
    virtual bool readCurrentState() override;
    virtual double getPosition(int axis) const override;
    virtual bool getButtonState(int button) const override;

private:
    cnoid::SimpleControllerIO* io;
    cnoid::SharedJoystick* sharedJoystick;

    std::ostream* os;
    std::mutex joyMutex;

    double timeStep;
    double time;
    double durationn;
    double percentage;

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
    geometry_msgs::msg::Twist command;
    sensor_msgs::msg::Joy joyState;
    rclcpp::executors::StaticSingleThreadedExecutor::UniquePtr executor;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TwistTopicSubscriberController)

bool TwistTopicSubscriberController::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());

    publisher = node->create_publisher<sensor_msgs::msg::BatteryState>("battery_status", 10);
    subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg){
            command = *msg;
        });

    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);

    return true;
}

bool TwistTopicSubscriberController::initialize(cnoid::SimpleControllerIO* io)
{
    this->io = io;
    sharedJoystick = io->getOrCreateSharedObject<cnoid::SharedJoystick>("joystick");
    sharedJoystick->setJoystick(this);

    timeStep = io->timeStep();
    time = durationn = 60.0 * 40.0;
    percentage = 100.0;

    os = &io->os();

    return true;
}

bool TwistTopicSubscriberController::control()
{
    executor->spin_some();

    time -= timeStep;
    // (*os) << time << std::endl;
    percentage = time / durationn * 100.0;

    auto message = sensor_msgs::msg::BatteryState();
    message.voltage = 15.0;
    message.percentage = percentage;
    publisher->publish(message);

    return true;
}

int TwistTopicSubscriberController::numAxes() const
{
    return joyState.axes.size();
}

int TwistTopicSubscriberController::numButtons() const
{
    return joyState.buttons.size();
}

bool TwistTopicSubscriberController::readCurrentState()
{
    std::lock_guard<std::mutex> lock(joyMutex);
    if(percentage > 0.0) {
        joyState.axes[0] = command.angular.z;
        joyState.axes[1] = command.linear.x;
        joyState.axes[2] = command.linear.y;
        joyState.axes[3] = command.linear.z;
        // joyState.buttons[1] = false;
    } else {
        joyState.axes[0] = 0.0;
        joyState.axes[1] = 0.0;
        joyState.axes[2] = 0.0;
        joyState.axes[3] = 0.0;
    }
    return true;
}

double TwistTopicSubscriberController::getPosition(int axis) const
{
    if(axis < (int)joyState.axes.size()) {
        return joyState.axes[axis];
    }
    return 0.0;
}

bool TwistTopicSubscriberController::getButtonState(int button) const
{
    if(button < (int)joyState.buttons.size()) {
        return joyState.buttons[button];
    }
    return false;
}