cmake_minimum_required(VERSION 3.8)
project(cia402_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cia402_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/cia402_controller.cpp
  src/cia402_state_machine.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  cia402_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  realtime_tools
)

pluginlib_export_plugin_description_file(controller_interface cia402_controller.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  cia402_interfaces
  controller_interface
  hardware_interface
  rclcpp_lifecycle
  rclcpp
  rclcpp_action
  realtime_tools
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_cia402_state_machine
    test/test_cia402_state_machine.cpp
  )
  target_link_libraries(test_cia402_state_machine ${PROJECT_NAME})
  ament_target_dependencies(test_cia402_state_machine cia402_interfaces)

  ament_add_gtest(test_cia402_controller
    test/test_cia402_controller.cpp
  )
  target_link_libraries(test_cia402_controller ${PROJECT_NAME})
  ament_target_dependencies(test_cia402_controller
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
  )
endif()

ament_package()
