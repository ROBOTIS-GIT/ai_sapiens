cmake_minimum_required(VERSION 3.8)
project(ai_sapiens_sim2real)

# Default to C++17 (required for ONNX Runtime)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ai_sapiens_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)

# ONNX Runtime (https://github.com/microsoft/onnxruntime/releases).
# Install under /usr/local or pass -DONNXRUNTIME_ROOT=<extracted dir>.
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include
  PATHS /usr/local/include/onnxruntime /usr/include/onnxruntime)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  HINTS ${ONNXRUNTIME_ROOT}/lib
  PATHS /usr/local/lib)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime not found. Download a release from "
    "https://github.com/microsoft/onnxruntime/releases and install it under "
    "/usr/local, or pass -DONNXRUNTIME_ROOT=<extracted directory>.")
endif()
include_directories(${ONNXRUNTIME_INCLUDE_DIR})

# Include directories
include_directories(include)

# ============================================================================
# Executable
# ============================================================================
add_executable(ai_sapiens_sim2real_node
  src/policy/action_pipeline.cpp
  src/command_publishers/joint_command_publisher.cpp
  src/config/config_utils.cpp
  src/control_loop.cpp
  src/controllers/mode_controller.cpp
  src/controllers/mode_ros_interface.cpp
  src/controllers/policy_controller.cpp
  src/main.cpp
  src/sim2real_node.cpp
  src/config/root_sections/selectors_config.cpp
  src/config/root_sections/state_behaviors_config.cpp
  src/config/root_sections/state_machine_config.cpp
  src/config/root_sections/teleop_conditions_config.cpp
  src/config/root_config_validator.cpp
  src/mode_runtime/authority_runtime.cpp
  src/mode_runtime/mode_state_machine.cpp
  src/mode_runtime/operator_command_inputs.cpp
  src/mode_runtime/startup_gate.cpp
  src/policy/gait_clock.cpp
  src/policy/motion_reference.cpp
  src/policy/mimic_policy_runtime.cpp
  src/observation/observation_manager.cpp
  src/observation/observations.cpp
  src/policy/onnx_inference.cpp
  src/policy/policy_runtime.cpp
  src/config/root_config.cpp
  src/sensor_handles/api_heartbeat_handle.cpp
  src/sensor_handles/imu_sensor_handle.cpp
  src/sensor_handles/joint_states_sensor_handle.cpp
  src/sensor_handles/teleop_input_handle.cpp
  src/sensor_handles/twist_command_handle.cpp
  src/config/sim2real_config.cpp
)

ament_target_dependencies(ai_sapiens_sim2real_node
  rclcpp
  realtime_tools
  sensor_msgs
  geometry_msgs
  std_msgs
  control_msgs
  ai_sapiens_interfaces
  pluginlib
)

target_link_libraries(ai_sapiens_sim2real_node
  yaml-cpp
  Eigen3::Eigen
  ${ONNXRUNTIME_LIBRARY}
)

# ============================================================================
# teleop input plugins
# ============================================================================
add_library(radiomaster_pocket_teleop_input_plugin SHARED
  src/plugins/teleop_input/radiomaster_pocket_teleop_input_plugin.cpp
)

add_library(keyboard_teleop_core SHARED
  src/keyboard_teleop/keyboard_teleop.cpp
)

add_library(keyboard_teleop_input_plugin SHARED
  src/plugins/teleop_input/keyboard_teleop_input_plugin.cpp
)

add_executable(keyboard_teleop_node
  src/keyboard_teleop/keyboard_teleop_node.cpp
)

ament_target_dependencies(radiomaster_pocket_teleop_input_plugin
  rclcpp
  realtime_tools
  ai_sapiens_interfaces
  pluginlib
)

ament_target_dependencies(keyboard_teleop_core
  ai_sapiens_interfaces
)

ament_target_dependencies(keyboard_teleop_input_plugin
  rclcpp
  realtime_tools
  ai_sapiens_interfaces
  pluginlib
)

ament_target_dependencies(keyboard_teleop_node
  rclcpp
  ai_sapiens_interfaces
  ament_index_cpp
)

target_link_libraries(radiomaster_pocket_teleop_input_plugin
  yaml-cpp
  Eigen3::Eigen
)

target_link_libraries(keyboard_teleop_core
  yaml-cpp
)

target_link_libraries(keyboard_teleop_input_plugin
  keyboard_teleop_core
  yaml-cpp
  Eigen3::Eigen
)

target_link_libraries(keyboard_teleop_node
  keyboard_teleop_core
  yaml-cpp
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins/teleop_input_plugins.xml)
pluginlib_export_plugin_description_file(
  ${PROJECT_NAME} plugins/keyboard_teleop_input_plugin.xml)

# ============================================================================
# Install
# ============================================================================
install(TARGETS
  ai_sapiens_sim2real_node
  radiomaster_pocket_teleop_input_plugin
  keyboard_teleop_core
  keyboard_teleop_input_plugin
  keyboard_teleop_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install config directory
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install runtime assets when present. Policy assets are often managed outside
# git, but a local install should still carry whatever is available.
install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
  OPTIONAL
)

# Install launch directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # RealtimeRequestGate: accept/consume/discard/busy semantics (header-only).
  ament_add_gtest(test_request_gate test/test_request_gate.cpp)
  ament_target_dependencies(test_request_gate realtime_tools)

  # GaitClock: free-running phase advance/wrap/reset and config parsing.
  ament_add_gtest(test_gait_clock test/test_gait_clock.cpp src/policy/gait_clock.cpp)
  target_link_libraries(test_gait_clock yaml-cpp Eigen3::Eigen)

  # StateMachineConfig/RootConfig: config validation and mimic-state detection.
  ament_add_gtest(test_state_machine_config test/test_state_machine_config.cpp
    src/config/config_utils.cpp
    src/config/root_sections/selectors_config.cpp
    src/config/root_sections/state_behaviors_config.cpp
    src/config/root_sections/state_machine_config.cpp
    src/config/root_sections/teleop_conditions_config.cpp
    src/config/root_config_validator.cpp
    src/config/root_config.cpp
    src/mode_runtime/mode_state_machine.cpp)
  target_link_libraries(test_state_machine_config yaml-cpp Eigen3::Eigen)

  # Startup teleop gate: only Damping/ReadyPose positions release the gate, and
  # an unavailable input never does.
  ament_add_gtest(test_startup_gate test/test_startup_gate.cpp
    src/config/config_utils.cpp
    src/mode_runtime/mode_state_machine.cpp
    src/config/root_sections/selectors_config.cpp
    src/config/root_sections/state_behaviors_config.cpp
    src/config/root_sections/state_machine_config.cpp
    src/config/root_sections/teleop_conditions_config.cpp
    src/config/root_config_validator.cpp)
  target_link_libraries(test_startup_gate yaml-cpp Eigen3::Eigen)

  # ModeController: exercise the actual decide/apply path for startup, teleop
  # transitions, and failsafes.
  ament_add_gtest(test_mode_controller test/test_mode_controller.cpp
    src/config/config_utils.cpp
    src/config/root_config.cpp
    src/config/root_config_validator.cpp
    src/config/root_sections/selectors_config.cpp
    src/config/root_sections/state_behaviors_config.cpp
    src/config/root_sections/state_machine_config.cpp
    src/config/root_sections/teleop_conditions_config.cpp
    src/controllers/mode_controller.cpp
    src/mode_runtime/authority_runtime.cpp
    src/mode_runtime/mode_state_machine.cpp)
  ament_target_dependencies(test_mode_controller rclcpp realtime_tools)
  target_link_libraries(test_mode_controller yaml-cpp Eigen3::Eigen)

  # MotionPlayback: absolute-motion-time seek/completion window.
  ament_add_gtest(test_motion_playback
    test/test_motion_playback.cpp
    src/policy/motion_reference.cpp)
  target_link_libraries(test_motion_playback Eigen3::Eigen)

  # AxisRange: zero-preserving normalize/scale/clamp helpers (header-only).
  ament_add_gtest(test_axis_range test/test_axis_range.cpp)

  ament_add_gtest(
    test_keyboard_teleop
    test/test_keyboard_teleop.cpp)
  target_link_libraries(
    test_keyboard_teleop
    keyboard_teleop_core
    yaml-cpp)
  ament_target_dependencies(
    test_keyboard_teleop
    ai_sapiens_interfaces)

  ament_add_gtest(
    test_keyboard_teleop_input_plugin
    test/test_keyboard_teleop_input_plugin.cpp)
  target_link_libraries(
    test_keyboard_teleop_input_plugin
    keyboard_teleop_input_plugin
    yaml-cpp)
  ament_target_dependencies(
    test_keyboard_teleop_input_plugin
    rclcpp
    ai_sapiens_interfaces)
endif()

ament_package()
