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
  src/mujoco_simulation.cpp
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

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mujoco_simulation test/test_mujoco_simulation.cpp)
  target_link_libraries(test_mujoco_simulation ${PROJECT_NAME})
  ament_target_dependencies(test_mujoco_simulation ament_index_cpp)
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()
