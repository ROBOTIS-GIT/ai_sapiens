cmake_minimum_required(VERSION 3.8)
project(ai_sapiens_joint_group_impedance_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(ai_sapiens_interfaces REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ai_sapiens_joint_group_impedance_controller.cpp
)
generate_parameter_library(${PROJECT_NAME}_parameters
  src/ai_sapiens_joint_group_impedance_controller_parameters.yaml
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_parameters
)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  ai_sapiens_interfaces
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  hardware_interface
)

pluginlib_export_plugin_description_file(
  controller_interface
  ai_sapiens_joint_group_impedance_controller_plugin.xml
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_joint_group_impedance_controller
    test/test_joint_group_impedance_controller.cpp
  )
  target_link_libraries(test_joint_group_impedance_controller
    ${PROJECT_NAME}
  )

  ament_add_gmock(test_load_joint_group_impedance_controller
    test/test_load_joint_group_impedance_controller.cpp
  )
  ament_target_dependencies(test_load_joint_group_impedance_controller
    controller_manager
    ros2_control_test_assets
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  ai_sapiens_interfaces
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  hardware_interface
  generate_parameter_library
)

ament_package()
