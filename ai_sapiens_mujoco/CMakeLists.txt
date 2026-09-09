cmake_minimum_required(VERSION 3.16)
project(ai_sapiens_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(IMGUI REQUIRED IMPORTED_TARGET imgui)
find_package(mujoco_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ai_sapiens_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/center_of_mass_visualizer.cpp
  src/contact_force_visualizer.cpp
  src/mujoco_simulation.cpp
  src/simulation_physics_settings.cpp
  src/mujoco_viewer.cpp
  src/mujoco_viewer_ui.cpp
  src/viewer_teleop.cpp
  src/viewer_scene_styling.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} mujoco::mujoco glfw yaml-cpp ${IMGUI_LINK_LIBRARIES} ${OPENGL_LIBRARIES} ${CMAKE_DL_LIBS})
target_include_directories(${PROJECT_NAME} PRIVATE ${IMGUI_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} rclcpp ai_sapiens_interfaces ament_index_cpp)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  ament_add_gtest(test_viewer_teleop test/test_viewer_teleop.cpp)
  target_link_libraries(test_viewer_teleop ${PROJECT_NAME})
  ament_target_dependencies(test_viewer_teleop ament_index_cpp)

  ament_add_gtest(test_mujoco_simulation test/test_mujoco_simulation.cpp)
  target_link_libraries(test_mujoco_simulation ${PROJECT_NAME})
  ament_target_dependencies(test_mujoco_simulation ament_index_cpp)

  ament_add_gtest(
    test_contact_force_visualizer test/test_contact_force_visualizer.cpp)
  target_link_libraries(test_contact_force_visualizer ${PROJECT_NAME})

  ament_add_gtest(
    test_center_of_mass_visualizer test/test_center_of_mass_visualizer.cpp)
  target_link_libraries(test_center_of_mass_visualizer ${PROJECT_NAME})

  ament_add_gtest(
    test_viewer_scene_styling test/test_viewer_scene_styling.cpp)
  target_link_libraries(test_viewer_scene_styling ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(mujoco_vendor rclcpp ai_sapiens_interfaces ament_index_cpp)
ament_package()
