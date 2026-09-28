cmake_minimum_required(VERSION 3.16)
project(radiomaster_usb_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/radiomaster_usb_hardware_interface.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface radiomaster_usb_hardware_interface.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Wrap only the driver's joystick syscalls; exercise the real hardware lifecycle/read path.
  ament_add_gtest(test_dual_receiver
    test/test_dual_receiver.cpp src/radiomaster_usb_hardware_interface.cpp)
  target_compile_features(test_dual_receiver PRIVATE cxx_std_17)
  target_include_directories(test_dual_receiver PRIVATE include)
  ament_target_dependencies(test_dual_receiver
    hardware_interface pluginlib rclcpp rclcpp_lifecycle)
  target_link_options(test_dual_receiver PRIVATE
    -Wl,--wrap=open -Wl,--wrap=open64 -Wl,--wrap=ioctl -Wl,--wrap=poll -Wl,--wrap=read -Wl,--wrap=close)
endif()

ament_package()
