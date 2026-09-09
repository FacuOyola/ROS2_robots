// aca van las clases 
#ifndef MI_ROBOT_HARDWARE_HPP
#define MI_ROBOT_HARDWARE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/hardware_info.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

 #include <cmath>
#include <memory>

#include "std_msgs/msg/int32_multi_array.hpp"

#include <thread> // libreria para tarea en paralelo

namespace mi_robot_hardware
{

class MiRobotHardware : public hardware_interface::SystemInterface
{
public:
    MiRobotHardware() = default;  // constructor
// lifecycle node override
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
// read, write
    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;
            std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

private:

void ticksCallback(
        const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;


    // Hilo para ejecutar rclcpp::spin()
    std::thread executor_thread_;
// definimos variables de la clase.
    int left_ticks;
    int right_ticks;

    // Estado de la rueda izquierda
    double left_position_;
    double left_velocity_;

    // Estado de la rueda derecha
    double right_position_;
    double right_velocity_;

     double prev_left_position_;
     double prev_right_position_;

     double wheel_radius_;
     double wheel_base_;
     double ticks_per_rev_;
     double meters_per_tick;
     double rad_per_tick;

     double left_command_;
double right_command_;


};

} // namespace mi_robot_hardware

#endif