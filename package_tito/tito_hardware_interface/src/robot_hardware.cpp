//aca va on_init(), read() del hardware_interface , que tomara datos del topic wheels_ticks.
 #include "mi_robot_hardware/robot_hardware.hpp"


 namespace mi_robot_hardware
{
 hardware_interface::CallbackReturn MiRobotHardware::on_init(
        const hardware_interface::HardwareInfo & info) {
            //aca va lo que queremos iniciar
            if(hardware_interface::SystemInterface::on_init(info) !=hardware_interface::CallbackReturn::SUCCESS)
            {
              return hardware_interface::CallbackReturn::ERROR;
            }
          //  info=info;

            // creo nodo suscriptor para el topico wheels ticks. 
   node_ = std::make_shared<rclcpp::Node>("hardware_interface");

    sub_ = node_->create_subscription<
        std_msgs::msg::Int32MultiArray>(
        "/wheels_ticks",
        rclcpp::QoS(10).best_effort(),
        std::bind(
            &MiRobotHardware::ticksCallback,
            this,
            std::placeholders::_1));

   left_position_ = 0.0;
right_position_ = 0.0;

left_velocity_ = 0.0;
right_velocity_ = 0.0;

prev_left_position_ = 0.0;
prev_right_position_ = 0.0;
    wheel_radius_ = 0.041;
        wheel_base_   = 0.22;
        ticks_per_rev_ =600;
         left_ticks=0;
                right_ticks=0;
                meters_per_tick=0.0;
                rad_per_tick=0.0;
// abro una tarea en paralelo
/*
        executor_thread_ = std::thread([this]()
{
    rclcpp::spin(node_);
});
*/

          return hardware_interface::CallbackReturn::SUCCESS;
        }

 // defino procedimiento que lee los ticks de los encoders        
void MiRobotHardware::ticksCallback(
const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) {
        RCLCPP_WARN(rclcpp::get_logger("MiRobotHardware"),
                    "Received encoder message with insufficient data.");
        return;
    }

    left_ticks = msg->data[0];
    right_ticks = msg->data[1];
     RCLCPP_INFO(node_->get_logger(),
                "Ticks recibidos: %d %d",
                left_ticks,
                right_ticks);


}

hardware_interface::return_type
MiRobotHardware::read(
const rclcpp::Time &,
const rclcpp::Duration &)
{
   rclcpp::spin_some(node_);
  
meters_per_tick =
            (2.0 * M_PI * wheel_radius_) /
            static_cast<double>(ticks_per_rev_);  //0,000429=4,29 mm por tick

rad_per_tick =
            (2.0 * M_PI) /
            static_cast<double>(ticks_per_rev_);  //2*PI/600 rad por tick

    //(void)time;
    //(void)period;
    double period =10;
    
    prev_left_position_=left_position_;
    prev_right_position_=right_position_;

    left_position_ = left_ticks*rad_per_tick;
    right_position_ = right_ticks*rad_per_tick;
    
    left_velocity_=get_command("left_wheel_joint/velocity");
    right_velocity_=get_command("right_wheel_joint/velocity");
    
    /*
    left_position_ += 0.001;
    right_position_ += 0.001;

    left_velocity_ = 0.1;
    right_velocity_ = 0.1;
*/
    RCLCPP_INFO(
        node_->get_logger(),
        "left: %.3f right: %.3f",
        left_position_,
        right_position_);
    // comparto data 
    // comparto data 
/*
    set_state("left_wheel_joint/position",left_position_); 
    set_state("right_wheel_joint/position",right_position_); // 
    /*
    set_state("left_wheel_joint/velocity",left_velocity_); 
    set_state("right_wheel_joint/velocity",right_velocity_);
*/
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MiRobotHardware::write(
const rclcpp::Time &,
const rclcpp::Duration &)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn
MiRobotHardware::on_activate(
const rclcpp_lifecycle::State &)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MiRobotHardware::on_deactivate(
const rclcpp_lifecycle::State &)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MiRobotHardware::export_state_interfaces()
{
    
    return {
        
        hardware_interface::StateInterface(
            "left_wheel_joint",
            hardware_interface::HW_IF_POSITION,
            &left_position_),

        hardware_interface::StateInterface(
            "left_wheel_joint",
            hardware_interface::HW_IF_VELOCITY,
            &left_velocity_),

        hardware_interface::StateInterface(
            "right_wheel_joint",
            hardware_interface::HW_IF_POSITION,
            &right_position_),

        hardware_interface::StateInterface(
            "right_wheel_joint",
            hardware_interface::HW_IF_VELOCITY,
            &right_velocity_)
    };
    
    /*
        return {
        hardware_interface::StateInterface(
            "left_wheel_joint",
            hardware_interface::HW_IF_POSITION,
            1),

        hardware_interface::StateInterface(
            "left_wheel_joint",
            hardware_interface::HW_IF_VELOCITY,
            1.2),

        hardware_interface::StateInterface(
            "right_wheel_joint",
            hardware_interface::HW_IF_POSITION,
            1),

        hardware_interface::StateInterface(
            "right_wheel_joint",
            hardware_interface::HW_IF_VELOCITY,
            2.4)
    };
*/
    }


}


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mi_robot_hardware::MiRobotHardware,hardware_interface::SystemInterface)