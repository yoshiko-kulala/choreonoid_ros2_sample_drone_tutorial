/**
    Joy Topic Subscriber Controller
    @author Kenta Suzuki
 */

#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <memory>
#include <thread>
#include <mutex>

class JoyTopicSubscriberController : public cnoid::SimpleController, public cnoid::JoystickInterface
{
public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override;
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;
    virtual void unconfigure() override;

    virtual int numAxes() const override;
    virtual int numButtons() const override;
    virtual bool readCurrentState() override;
    virtual double getPosition(int axis) const override;
    virtual bool getButtonState(int button) const override;

private:
    cnoid::SimpleControllerIO* io;
    cnoid::SharedJoystick* sharedJoystick;
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription;
    sensor_msgs::msg::Joy tmpJoyState;
    sensor_msgs::msg::Joy joyState;
    std::unique_ptr<rclcpp::executors::StaticSingleThreadedExecutor> executor;
    std::thread executorThread;
    std::mutex commandMutex;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(JoyTopicSubscriberController)

bool JoyTopicSubscriberController::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());

    subscription = node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1,
        [this](const sensor_msgs::msg::Joy::SharedPtr msg){
            std::lock_guard<std::mutex> lock(commandMutex);
            tmpJoyState = *msg;
        });

    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this](){ executor->spin(); });

    return true;
}

bool JoyTopicSubscriberController::initialize(cnoid::SimpleControllerIO* io)
{
    this->io = io;
    sharedJoystick = io->getOrCreateSharedObject<cnoid::SharedJoystick>("joystick");
    sharedJoystick->setJoystick(this);
    return true;
}

bool JoyTopicSubscriberController::control()
{
    return false;
}

void JoyTopicSubscriberController::unconfigure()
{
    if(executor) {
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}

int JoyTopicSubscriberController::numAxes() const
{
    return (int)joyState.axes.size();
}

int JoyTopicSubscriberController::numButtons() const
{
    return (int)joyState.buttons.size();
}

bool JoyTopicSubscriberController::readCurrentState()
{
    std::lock_guard<std::mutex> lock(commandMutex);
    joyState = tmpJoyState;
    return true;
}

double JoyTopicSubscriberController::getPosition(int axis) const
{
    if(axis < (int)joyState.axes.size()) {
        return joyState.axes[axis];
    }
    return 0.0;
}

bool JoyTopicSubscriberController::getButtonState(int button) const
{
    if(button < (int)joyState.buttons.size()) {
        return joyState.buttons[button];
    }
    return false;
}