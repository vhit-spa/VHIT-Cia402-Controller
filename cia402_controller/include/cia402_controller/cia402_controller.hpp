#ifndef CIA402_CONTROLLER_
#define CIA402_CONTROLLER_

#include <controller_interface/controller_interface.hpp>


#include <rclcpp_action/server.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/clock.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
namespace cia402_controller
{

  class Cia402Controller : controller_interface::ControllerInterface
  {

    public:
      Cia402Controller();

      controller_interface::CallbackReturn on_init() override;
      controller_interface::InterfaceConfiguration command_interface_configuration() const override;
      controller_interface::InterfaceConfiguration state_interface_configuration() const override;
      controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
      override;
      controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
      override;
      controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
      override;
      controller_interface::return_type update(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;
    
    protected:

  };

}// namespace cia402_controller
#endif // CIA402_CONTROLLER_