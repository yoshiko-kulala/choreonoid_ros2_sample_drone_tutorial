choreonoid_add_simple_controller(
    TwistTopicSubscriberController TwistTopicSubscriberController.cpp)

ament_target_dependencies(
    TwistTopicSubscriberController rclcpp geometry_msgs sensor_msgs)