cmake_minimum_required(VERSION 3.16)
project(ai_sapiens_rc_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ai_sapiens_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)

set(PLUGIN_TARGET ${PROJECT_NAME}_plugin)

generate_parameter_library(${PROJECT_NAME}_parameters
  src/ai_sapiens_rc_broadcaster_parameters.yaml
)

add_library(${PLUGIN_TARGET} SHARED
  src/ai_sapiens_rc_broadcaster.cpp
)
set_target_properties(${PLUGIN_TARGET}
  PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
)
target_compile_features(${PLUGIN_TARGET} PUBLIC cxx_std_17)
target_link_libraries(${PLUGIN_TARGET}
  ${PROJECT_NAME}_parameters
)
target_include_directories(${PLUGIN_TARGET} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PLUGIN_TARGET}
  ai_sapiens_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)

pluginlib_export_plugin_description_file(controller_interface ai_sapiens_rc_broadcaster.xml)

install(TARGETS
  ${PLUGIN_TARGET}
  ${PROJECT_NAME}_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(${PLUGIN_TARGET})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ai_sapiens_interfaces
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_ai_sapiens_rc_broadcaster
    test/test_ai_sapiens_rc_broadcaster.cpp
  )
  target_link_libraries(test_ai_sapiens_rc_broadcaster
    ${PLUGIN_TARGET}
  )

  ament_add_gmock(test_load_ai_sapiens_rc_broadcaster
    test/test_load_ai_sapiens_rc_broadcaster.cpp
  )
  ament_target_dependencies(test_load_ai_sapiens_rc_broadcaster
    controller_manager
    ros2_control_test_assets
  )
endif()

ament_package()
