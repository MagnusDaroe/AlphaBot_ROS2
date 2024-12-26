cmake_minimum_required(VERSION 3.10)
project(alpha_bot)

# Ensure that we have a ROS 2 workspace
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Python dependencies, if any, can be added here

# Install Python executables (your ROS 2 Python scripts)
install(PROGRAMS
  scripts/alphabot_node.py
  scripts/alphabot_remote_controller_node.py
  scripts/alphabot_remote_digital_node.py
  scripts/alphabot_cam_node.py
  scripts/alphabot_cam_view_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml for ROS 2 package details
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Ament package (finalize)
ament_package()
