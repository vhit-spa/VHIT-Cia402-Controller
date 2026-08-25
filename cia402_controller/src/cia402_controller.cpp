#include "cia402_controller/cia402_controller.hpp"
#include "rclcpp/logging.hpp"

namespace cia402_controller
{

  cia402_controller::Cia402Controller() 
  : controller_interface::ControllerInterface 
  {
  }
  

}// namespace cia402_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cia402_controller::Cia402Controller,
  controller_interface::ControllerInterface)