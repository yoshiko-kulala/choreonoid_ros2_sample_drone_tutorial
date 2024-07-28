choreonoid_add_drone_controller(
    SampleDroneFlightController SampleDroneFlightController.cpp)

ament_target_dependencies(
    SampleDroneFlightController rclcpp geometry_msgs sensor_msgs)