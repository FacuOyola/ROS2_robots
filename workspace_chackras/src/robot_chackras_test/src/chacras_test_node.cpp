#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <modbus/modbus-rtu.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#define CHACRAS_ADDR  1
#define MOTOR_BASE    0
#define STATUS_BASE   24
#define ARMADO_BASE   28

class MiRobotNode : public rclcpp::Node
{
public:
    MiRobotNode()
    : Node("mi_robot_node")
    {
        // Parámetros
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<int>("slave_id", CHACRAS_ADDR);
        this->declare_parameter<double>("publish_rate", 20.0);

        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        slave_id_ = this->get_parameter("slave_id").as_int();
        publish_rate_ = this->get_parameter("publish_rate").as_double();

        // Publishers
        speed_pub_ =
            this->create_publisher<std_msgs::msg::Float32MultiArray>(
                "/motor_speeds", 10);

        status_pub_ =
            this->create_publisher<std_msgs::msg::Int32MultiArray>(
                "/motor_status", 10);

        armed_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/armed", 10);

        // Inicializar Modbus
        if (!init_modbus()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "No se pudo inicializar Modbus");
            throw std::runtime_error("Modbus initialization failed");
        }

        // Timer de lectura
        auto period =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / publish_rate_));

        timer_ = this->create_wall_timer(
            period,
            std::bind(&MiRobotNode::read_modbus, this));

        RCLCPP_INFO(
            this->get_logger(),
            "MiRobotNode iniciado");
        RCLCPP_INFO(
            this->get_logger(),
            "Puerto: %s | Baudrate: %d | Slave: %d",
            port_.c_str(),
            baudrate_,
            slave_id_);
    }

    ~MiRobotNode()
    {
        if (ctx_) {
            modbus_close(ctx_);
            modbus_free(ctx_);
            ctx_ = nullptr;
        }
    }

private:

    bool init_modbus()
    {
        ctx_ = modbus_new_rtu(
            port_.c_str(),
            baudrate_,
            'N',
            8,
            1);

        if (!ctx_) {
            RCLCPP_ERROR(
                this->get_logger(),
                "modbus_new_rtu(): %s",
                modbus_strerror(errno));

            return false;
        }

        if (modbus_set_slave(ctx_, slave_id_) == -1) {
            RCLCPP_ERROR(
                this->get_logger(),
                "modbus_set_slave(): %s",
                modbus_strerror(errno));

            modbus_free(ctx_);
            ctx_ = nullptr;

            return false;
        }

        if (modbus_connect(ctx_) == -1) {
            RCLCPP_ERROR(
                this->get_logger(),
                "modbus_connect(): %s",
                modbus_strerror(errno));

            modbus_free(ctx_);
            ctx_ = nullptr;

            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Conectado a Modbus RTU");

        return true;
    }

    bool read_registers(
        int address,
        int count,
        uint16_t *data)
    {
        int rc = modbus_read_registers(
            ctx_,
            address,
            count,
            data);

        if (rc != count) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Error leyendo registros [%d - %d]: %s",
                address,
                address + count - 1,
                modbus_strerror(errno));

            return false;
        }

        return true;
    }

    void read_modbus()
    {
        if (!ctx_) {
            return;
        }

        /*
         * ==========================================================
         * VELOCIDADES
         * ==========================================================
         *
         * Suponemos:
         *
         * Registro 0 -> velocidad motor izquierdo
         * Registro 1 -> velocidad motor derecho
         */

        uint16_t motor_data[2];

        if (read_registers(
                MOTOR_BASE,
                2,
                motor_data))
        {
            std_msgs::msg::Float32MultiArray msg;

            msg.data.resize(2);

            /*
             * Conversión:
             *
             * Si el PLC manda directamente RPM:
             */

            msg.data[0] =
                static_cast<float>(
                    static_cast<int16_t>(motor_data[0]));

            msg.data[1] =
                static_cast<float>(
                    static_cast<int16_t>(motor_data[1]));

            speed_pub_->publish(msg);

            RCLCPP_DEBUG(
                this->get_logger(),
                "Velocidad: motor1=%f motor2=%f",
                msg.data[0],
                msg.data[1]);
        }

        /*
         * ==========================================================
         * STATUS
         * ==========================================================
         *
         * Leemos 4 registros desde STATUS_BASE:
         *
         * 24
         * 25
         * 26
         * 27
         */

        uint16_t status_data[4];

        if (read_registers(
                STATUS_BASE,
                4,
                status_data))
        {
            std_msgs::msg::Int32MultiArray msg;

            msg.data.resize(4);

            for (int i = 0; i < 4; ++i) {
                msg.data[i] =
                    static_cast<int32_t>(status_data[i]);
            }

            status_pub_->publish(msg);
        }

        /*
         * ==========================================================
         * ARMADO
         * ==========================================================
         *
         * Registro 28
         */

        uint16_t armado_data[1];

        if (read_registers(
                ARMADO_BASE,
                1,
                armado_data))
        {
            std_msgs::msg::Bool msg;

            msg.data = (armado_data[0] != 0);

            armed_pub_->publish(msg);
        }
    }

private:

    modbus_t *ctx_{nullptr};

    std::string port_;
    int baudrate_;
    int slave_id_;
    double publish_rate_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<
        std_msgs::msg::Float32MultiArray
    >::SharedPtr speed_pub_;

    rclcpp::Publisher<
        std_msgs::msg::Int32MultiArray
    >::SharedPtr status_pub_;

    rclcpp::Publisher<
        std_msgs::msg::Bool
    >::SharedPtr armed_pub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<MiRobotNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}