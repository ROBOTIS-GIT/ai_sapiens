cmake_minimum_required(VERSION 3.16)
project(ai_sapiens_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ai_sapiens_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(glfw3 REQUIRED)

find_library(MUJOCO_LIBRARY mujoco REQUIRED)
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/contact_force_visualizer.cpp
  src/gantry_service_node.cpp
  src/mujoco_simulation.cpp
  src/mujoco_system.cpp
  src/mujoco_viewer.cpp
  src/mujoco_viewer_ui.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} ${MUJOCO_LIBRARY} glfw)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle std_srvs
  ai_sapiens_interfaces ament_index_cpp)

pluginlib_export_plugin_description_file(hardware_interface ai_sapiens_mujoco.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES ai_sapiens_mujoco.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mujoco_simulation test/test_mujoco_simulation.cpp)
  target_link_libraries(test_mujoco_simulation ${PROJECT_NAME})
  ament_target_dependencies(test_mujoco_simulation ament_index_cpp)

  ament_add_gtest(test_mujoco_system test/test_mujoco_system.cpp)
  target_link_libraries(test_mujoco_system ${PROJECT_NAME})
  ament_target_dependencies(test_mujoco_system
    ament_index_cpp hardware_interface rclcpp std_srvs ai_sapiens_interfaces)

  ament_add_gtest(
    test_contact_force_visualizer test/test_contact_force_visualizer.cpp)
  target_link_libraries(test_contact_force_visualizer ${PROJECT_NAME})
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()
