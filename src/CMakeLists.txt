choreonoid_add_drone_controller(
    SampleDroneFlightController SampleDroneFlightController.cpp)

ament_target_dependencies(
    SampleDroneFlightController rclcpp geometry_msgs sensor_msgs)

choreonoid_add_drone_controller(
    TwistPublisherController TwistPublisherController.cpp)

ament_target_dependencies(
    TwistPublisherController rclcpp geometry_msgs sensor_msgs)