#ifndef MI_ROBOT_HARDWARE_HPP
#define MI_ROBOT_HARDWARE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <vector>

extern "C" {
#include "my_robot_hardware/modbus_driver.h"
}

namespace my_robot_hardware
{

class MiRobotHardware: public hardware_interface::SystemInterface
{
public:

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State & previous_state) override;    

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:

    // Comandos de velocidad
    // 0 = front_right
    // 1 = front_left
    // 2 = rear_left
    // 3 = rear_right
    std::vector<double> hw_commands_{
        0.0, 0.0, 0.0, 0.0
    };

    // Posición de las ruedas
    std::vector<double> hw_positions_{
        0.0, 0.0, 0.0, 0.0
    };

    // Velocidad de las ruedas
    std::vector<double> hw_velocities_{
        0.0, 0.0, 0.0, 0.0
    };
/*
      // setpoint de las ruedas
    std::vector<double> hw_setpoints_{
        0.0, 0.0, 0.0, 0.0
    };
*/
       // Corriente de las ruedas
    std::vector<double> hw_currents_{
        0.0, 0.0, 0.0, 0.0
    };
};

} // namespace my_robot_hardware

#endif // MOBILE_BASE_HARDWARE_INTERFACE_HPP