#include "my_robot_hardware/mobile_base_hardware_interface.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>

/*************************************
Dirección   Función

0           Setpoint (Motor 1)
1           Sentido (Motor 1)
2           Velocidad (Motor 1) H
3           Velocidad (Motor 1) L
4           Corriente (Motor 1) H
5           Corriente (Motor 1) L

6           Setpoint (Motor 2)
7           Sentido (Motor 2)
8           Velocidad (Motor 2) H
9           Velocidad (Motor 2) L
10          Corriente (Motor 2) H
11          Corriente (Motor 2) L

12          Setpoint (Motor 3)
13          Sentido (Motor 3)
14          Velocidad (Motor 3) H
15          Velocidad (Motor 3) L
16          Corriente (Motor 3) H
17          Corriente (Motor 3) L

18          Setpoint (Motor 4)
19          Sentido (Motor 4)
20          Velocidad (Motor 4) H
21          Velocidad (Motor 4) L
22          Corriente (Motor 4) H
23          Corriente (Motor 4) L

24          Reservado
25          Reservado
26          Tensión de Baterías H
27          Tensión de Baterías L
28          Armado de sistema
29          Reservado
30          Reservado
31          Reservado
*************************************/

#define STATUS_BASE 24
#define ARMADO_BASE 28

/*
typedef struct __attribute__((packed)) MotorStruct {
    uint16_t setpoint;
    uint16_t sentido;
    float velocidad;
    float corriente;
} MotorStruct_t;
*/
modbus_t *ctx = nullptr;

uint16_t armado = 0;

    double lastcommand1=0;
    double lastcommand2=0;
    double lastcommand3=0;
    double lastcommand4=0;

namespace my_robot_hardware
{

// ============================================================
// INIT
// ============================================================

hardware_interface::CallbackReturn
MiRobotHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    char serialport[16] = "/dev/ttyUSB0";
    char serialport1[16]="/dev/ttyUSB1";
    int baudrate = 115200;

    ctx = modbus_init(serialport, baudrate);
    if (!ctx){
      ctx = modbus_init(serialport1, baudrate);  
    }

    modbus_set_response_timeout(ctx, 0, 500000);


    if (!ctx)
    {
        std::cerr << "Error: no se pudo inicializar Modbus"
                  << std::endl;

        return hardware_interface::CallbackReturn::ERROR;
    }

    armado=1;
    if (modbus_wr(ctx, ARMADO_BASE, &armado, 1) < 0)
    {
        std::cerr << "Error al armar el sistema mediante Modbus"
                  << std::endl;

       return hardware_interface::CallbackReturn::ERROR;
    }


    std::cout << "Hardware Modbus inicializado correctamente"
              << std::endl;

    return hardware_interface::CallbackReturn::SUCCESS;
}


// ============================================================
// ACTIVATE
// ============================================================

hardware_interface::CallbackReturn
MiRobotHardware::on_activate(
    const rclcpp_lifecycle::State & previous_state/*previous_state*/)
{
    std::fill(
        hw_commands_.begin(),
        hw_commands_.end(),
        0.0);

    std::fill(
        hw_positions_.begin(),
        hw_positions_.end(),
        0.0);

    std::fill(
        hw_velocities_.begin(),
        hw_velocities_.end(),
        0.0);

  std::fill(
        hw_currents_.begin(),
        hw_currents_.end(),
        0.0);
        


    std::cout << "MobileBaseHardwareInterface activado"
              << std::endl;

    return hardware_interface::CallbackReturn::SUCCESS;
}


// ============================================================
// DEACTIVATE
// ============================================================

hardware_interface::CallbackReturn
MiRobotHardware::on_deactivate(
    const rclcpp_lifecycle::State & previous_state /*previous_state*/)
{
    if (ctx)
    {
        uint16_t stop = 0;

        modbus_wr(
            ctx,
            ARMADO_BASE,
            &stop,
            1);
    }

           modbus_close(ctx); // cierra la comunicacion
            modbus_free(ctx); //libera  el ctx
              //return hardware_interface::CallbackReturn::SUCCESS;
              ctx = nullptr;

    std::cout << "MobileBaseHardwareInterface desactivado"
              << std::endl;
    std::cout << "Comunicación Modbus cerrada correctamente" << std::endl;

    return hardware_interface::CallbackReturn::SUCCESS;
}
//=================================================================
// SHUTDOWN: apago hardware interface y corto comunicacion
//=================================================================
hardware_interface::CallbackReturn
MiRobotHardware::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// EXPORT STATE INTERFACES
// ============================================================

std::vector<hardware_interface::StateInterface>
MiRobotHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface>
        state_interfaces;

    // ========================================================
    // FRONT RIGHT
    // ========================================================

    state_interfaces.emplace_back(
        "front_right_wheel_joint",
        hardware_interface::HW_IF_POSITION,
        &hw_positions_[0]);

    state_interfaces.emplace_back(
        "front_right_wheel_joint",
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[0]);
        
          state_interfaces.emplace_back(
              "front_right_wheel_joint",
            "current",
            &hw_currents_[0]);




    // ========================================================
    // FRONT LEFT
    // ========================================================

    state_interfaces.emplace_back(
        "front_left_wheel_joint",
        hardware_interface::HW_IF_POSITION,
        &hw_positions_[1]);

    state_interfaces.emplace_back(
        "front_left_wheel_joint",
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[1]);

        state_interfaces.emplace_back(
            "front_left_wheel_joint",
            "current",
            &hw_currents_[1]);


    // ========================================================
    // REAR LEFT
    // ========================================================

    state_interfaces.emplace_back(
        "rear_left_wheel_joint",
        hardware_interface::HW_IF_POSITION,
        &hw_positions_[2]);

    state_interfaces.emplace_back(
        "rear_left_wheel_joint",
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[2]);

        state_interfaces.emplace_back(
    "rear_left_wheel_joint",
    "current",
    &hw_currents_[2]);




    // ========================================================
    // REAR RIGHT
    // ========================================================

    state_interfaces.emplace_back(
        "rear_right_wheel_joint",
        hardware_interface::HW_IF_POSITION,
        &hw_positions_[3]);

    state_interfaces.emplace_back(
        "rear_right_wheel_joint",
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[3]);

        state_interfaces.emplace_back(
    "rear_right_wheel_joint",
    "current",
    &hw_currents_[3]);


    return state_interfaces;
}


// ============================================================
// EXPORT COMMAND INTERFACES
// ============================================================

std::vector<hardware_interface::CommandInterface>
MiRobotHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface>
        command_interfaces;


    command_interfaces.emplace_back(
        "front_right_wheel_joint",
        hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[1]);

    command_interfaces.emplace_back(
        "front_left_wheel_joint",
        hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[0]);

    command_interfaces.emplace_back(
        "rear_left_wheel_joint",
        hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[2]);

    command_interfaces.emplace_back(
        "rear_right_wheel_joint",
        hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[3]);


    return command_interfaces;
}


// ============================================================
// READ
// ============================================================

hardware_interface::return_type
MiRobotHardware::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & period)
{
    /*
     * IMPORTANTE:
     *
     * En tu mapa Modbus tenemos registros de VELOCIDAD:
     *
     * Motor 1 -> 2,3
     * Motor 2 -> 8,9
     * Motor 3 -> 14,15
     * Motor 4 -> 20,21
     *
     * Todavía necesitamos conocer exactamente cómo
     * modbus_rd() devuelve esos registros y qué unidad utiliza
     * el controlador.
     *
     * Por ahora dejamos las velocidades en cero.
     */
/*
    (void)period;

    uint16_t motor[6];
    int8_t sig;
    
    modbus_rd(ctx,0, motor, 6);
    /*
            MotorStruct_t *pt = (MotorStruct_t *)motor;
            printf("%d %d %f %f\n", pt->setpoint,   //
                                    pt->sentido,    // 
                                    pt->velocidad,  //
                                    pt->corriente);
      */  
      /*
      sig=motor[1]?1:-1;                            
    hw_velocities_[0]=(double)sig*motor[2];  
    hw_currents_[0]=(double)motor[3];   
    modbus_rd(ctx,6, motor, 6);   
      sig=motor[1]?1:-1;        
    hw_velocities_[1]=(double)sig*motor[2];
    hw_currents_[1]=(double)motor[3];                             
       modbus_rd(ctx,12, motor, 6);
         sig=motor[1]?1:-1;           
    hw_velocities_[2]=(double)sig*motor[2];
    hw_currents_[2]=(double)motor[3];   
         modbus_rd(ctx,18, motor, 6);
           sig=motor[1]?1:-1;           
    hw_velocities_[3]=(double)sig*motor[2];
    hw_currents_[3]=(double)motor[3];   
  */   

    return hardware_interface::return_type::OK;
}


// ============================================================
// WRITE
// ============================================================

hardware_interface::return_type
MiRobotHardware::write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
// Armamos el sistema
    
    
    if (!ctx)
    {
        return hardware_interface::return_type::ERROR;
    }

    uint16_t data[2];


    // ========================================================
    // MOTOR 1 - FRONT RIGHT
    // ========================================================
bool comando = false;

for (size_t i = 0; i < 4; ++i)
{
    if (hw_commands_[i] != 0.0)
    {
        comando = true;
        break;
    }
}

armado = comando ? 1 : 0;
if(comando!=0){
    double command =
        hw_commands_[0];

    
 if(command!=lastcommand1){
    if (command >= 0.0)
    {
        data[1] = 0;
    }
    else
    {
        data[1] = 1;
    }

    data[0] =
       static_cast<uint16_t>(std::abs(command));
   //data[0] = static_cast<uint16_t>(
    //std::abs(get_command("front_left_wheel_joint/velocity"))
//);
   modbus_wr(ctx, 0, data, 2);
   lastcommand1=command;
//std::this_thread::sleep_for(std::chrono::milliseconds(10));
 }

    // ========================================================
    // MOTOR 2 - FRONT LEFT
    // ========================================================

    command =
        hw_commands_[1];
        if(command!=lastcommand2){

    if (command >= 0.0)
    {
        data[1] = 1;
    }
    else
    {
        data[1] = 0;
    }

    data[0] =
        static_cast<uint16_t>(std::abs(command));

   
      modbus_wr(  ctx,6,data,2);
      lastcommand2=command;
}
//std::this_thread::sleep_for(std::chrono::milliseconds(10));


    // ========================================================
    // MOTOR 3 - REAR LEFT
    // ========================================================

    command =
        hw_commands_[2];
 if(command!=lastcommand3){
    if (command >= 0.0) // truco para garantizar el sentido de la direccion
    {
        data[1] = 0;
    }
    else
    {
        data[1] = 1;
    }

    data[0] =
        static_cast<uint16_t>(std::abs(command));

    modbus_wr(ctx,12,data,2);
    lastcommand3=command;
}
//std::this_thread::sleep_for(std::chrono::milliseconds(10));


    // ========================================================
    // MOTOR 4 - REAR RIGHT
    // ========================================================

    command =
        hw_commands_[3];
 if(command!=lastcommand4){
    if (command >= 0.0)
    {
        data[1] = 1;
    }
    else
    {
        data[1] = 0;
    }

    data[0] =
        static_cast<uint16_t>(std::abs(command));

    modbus_wr(ctx,18,data,2);
    lastcommand4=command;
}
armado=1;
modbus_wr(ctx, ARMADO_BASE,&armado, 1); 
}
else{
     armado=0;
    modbus_wr(ctx, ARMADO_BASE,&armado, 1); 
}
//std::this_thread::sleep_for(std::chrono::milliseconds(10));
//armado=0;
  //modbus_wr(ctx, ARMADO_BASE, &armado, 1);
  
    return hardware_interface::return_type::OK;
}


} // namespace my_robot_hardware


// ============================================================
// PLUGINLIB
// ============================================================

PLUGINLIB_EXPORT_CLASS(
    my_robot_hardware::MiRobotHardware,
    hardware_interface::SystemInterface
)