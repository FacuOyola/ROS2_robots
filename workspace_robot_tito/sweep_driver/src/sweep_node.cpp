
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <chrono>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

using namespace std::chrono_literals;


// ============================================================
// CONFIGURACIÓN
// ============================================================

constexpr int MOTOR_HZ = 3;
constexpr int NUM_POINTS = 360;

constexpr const char* SERIAL_PORT = "/dev/ttyUSB0";


// ============================================================
// NODE ROS2
// ============================================================

class SweepNode : public rclcpp::Node
{
public:

    SweepNode()
        : Node("sweep_node")
    {
        publisher_ =
            this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/scan",
                rclcpp::QoS(10)
            );

        RCLCPP_INFO(
            this->get_logger(),
            "SweepNode iniciado. Motor configurado a %d Hz",
            MOTOR_HZ
        );
    }


    // ========================================================
    // PUBLICAR UNA VUELTA COMPLETA
    // ========================================================

    void publish_scan(
        const std::vector<float>& scan_data,
        double scan_time,
        int valid_points)
    {
        sensor_msgs::msg::LaserScan scan;


        // ----------------------------------------------------
        // Timestamp
        // ----------------------------------------------------

        scan.header.stamp =
            this->get_clock()->now();


        // ----------------------------------------------------
        // Frame
        // ----------------------------------------------------

        scan.header.frame_id =
            "laser_link";


        // ----------------------------------------------------
        // Ángulos
        // ----------------------------------------------------

        scan.angle_min =
            -static_cast<float>(M_PI);


        scan.angle_increment =
            (2.0f * static_cast<float>(M_PI))
            /
            static_cast<float>(NUM_POINTS);


        scan.angle_max =
            scan.angle_min +
            scan.angle_increment *
            static_cast<float>(NUM_POINTS - 1);


        // ----------------------------------------------------
        // Tiempo REAL de una revolución
        // ----------------------------------------------------

        scan.scan_time =
            static_cast<float>(scan_time);


        scan.time_increment =
            static_cast<float>(
                scan_time /
                static_cast<double>(NUM_POINTS)
            );


        // ----------------------------------------------------
        // Rango
        // ----------------------------------------------------

        scan.range_min = 0.1f;
        scan.range_max = 40.0f;


        // ----------------------------------------------------
        // Reservar datos
        // ----------------------------------------------------

        scan.ranges.resize(NUM_POINTS);


        // ----------------------------------------------------
        // Copiar datos
        // ----------------------------------------------------

        for (int i = 0; i < NUM_POINTS; i++)
        {
            float distance =
                scan_data[i];


            if (!std::isfinite(distance) ||
                distance < scan.range_min ||
                distance > scan.range_max)
            {
                scan.ranges[i] =
                    std::numeric_limits<float>::infinity();
            }
            else
            {
                scan.ranges[i] =
                    distance;
            }
        }


        // ----------------------------------------------------
        // Publicar
        // ----------------------------------------------------

        publisher_->publish(scan);


        // ----------------------------------------------------
        // Información de diagnóstico
        // ----------------------------------------------------

        RCLCPP_INFO(
            this->get_logger(),
            "Scan publicado | "
            "Tiempo: %.4f s | "
            "Frecuencia: %.2f Hz | "
            "Puntos validos: %d/%d",
            scan_time,
            1.0 / scan_time,
            valid_points,
            NUM_POINTS
        );
    }


private:

    rclcpp::Publisher<
        sensor_msgs::msg::LaserScan
    >::SharedPtr publisher_;
};


// ============================================================
// CONFIGURACIÓN SERIAL
// ============================================================

int configurar_serial(int fd)
{
    struct termios tty;

    memset(&tty, 0, sizeof(tty));


    if (tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr");
        return -1;
    }


    // --------------------------------------------------------
    // 115200 baud
    // --------------------------------------------------------

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);


    // --------------------------------------------------------
    // 8N1
    // --------------------------------------------------------

    tty.c_cflag |=
        (CLOCAL | CREAD);

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;


    // --------------------------------------------------------
    // Raw mode
    // --------------------------------------------------------

    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;


    // --------------------------------------------------------
    // read()
    // --------------------------------------------------------

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;


    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        return -1;
    }


    return 0;
}


// ============================================================
// CHECKSUM
// ============================================================

bool paquete_valido(uint8_t* buf)
{
    uint32_t sum = 0;


    for (int i = 0; i < 6; i++)
        sum += buf[i];


    return ((sum % 255) == buf[6]);
}


// ============================================================
// LEER PAQUETE DE 7 BYTES
// ============================================================

bool leer_paquete_valido(
    int fd,
    uint8_t* buf)
{
    int count = 0;


    while (true)
    {
        uint8_t byte;


        int n =
            read(fd, &byte, 1);


        if (n <= 0)
            continue;


        // ----------------------------------------------------
        // Llenar buffer
        // ----------------------------------------------------

        if (count < 7)
        {
            buf[count] = byte;
            count++;
        }
        else
        {
            for (int i = 0; i < 6; i++)
                buf[i] = buf[i + 1];


            buf[6] = byte;
        }


        // ----------------------------------------------------
        // Validar
        // ----------------------------------------------------

        if (count == 7)
        {
            if (paquete_valido(buf))
                return true;
        }
    }
}


// ============================================================
// ENVIAR COMANDO
// ============================================================

bool enviar_comando(
    int fd,
    const char* comando)
{
    size_t len =
        strlen(comando);


    ssize_t written =
        write(fd, comando, len);


    if (written != static_cast<ssize_t>(len))
    {
        perror("write");
        return false;
    }


    tcdrain(fd);


    return true;
}


// ============================================================
// MAIN
// ============================================================

int main(
    int argc,
    char** argv)
{
    rclcpp::init(argc, argv);


    // ========================================================
    // ABRIR PUERTO
    // ========================================================

    int fd =
        open(
            SERIAL_PORT,
            O_RDWR |
            O_NOCTTY
        );


    if (fd < 0)
    {
        perror("open");

        rclcpp::shutdown();

        return 1;
    }


    // ========================================================
    // CONFIGURAR SERIAL
    // ========================================================

    if (configurar_serial(fd) != 0)
    {
        close(fd);

        rclcpp::shutdown();

        return 1;
    }


    // ========================================================
    // CREAR NODE
    // ========================================================

    auto node =
        std::make_shared<SweepNode>();


    // ========================================================
    // DETENER SWEEP
    // ========================================================

    RCLCPP_INFO(
        node->get_logger(),
        "Deteniendo adquisicion..."
    );


    enviar_comando(
        fd,
        "DX\r\n"
    );


    std::this_thread::sleep_for(1s);


    // ========================================================
    // SAMPLE RATE
    // ========================================================

    RCLCPP_INFO(
        node->get_logger(),
        "Configurando sample rate LR03..."
    );


    enviar_comando(
        fd,
        "LR03\r\n"
    );


    std::this_thread::sleep_for(1s);


    // ========================================================
    // VELOCIDAD DEL MOTOR
    // ========================================================

    char motor_cmd[16];


    snprintf(
        motor_cmd,
        sizeof(motor_cmd),
        "MS%02d\r\n",
        MOTOR_HZ
    );


    RCLCPP_INFO(
        node->get_logger(),
        "Configurando motor: MS%02d",
        MOTOR_HZ
    );


    enviar_comando(
        fd,
        motor_cmd
    );


    // ========================================================
    // ESPERAR ESTABILIZACIÓN
    // ========================================================

    RCLCPP_INFO(
        node->get_logger(),
        "Esperando estabilizacion del motor..."
    );


    std::this_thread::sleep_for(8s);


    // ========================================================
    // LIMPIAR BUFFER
    // ========================================================

    tcflush(
        fd,
        TCIFLUSH
    );


    // ========================================================
    // INICIAR DATA
    // ========================================================

    RCLCPP_INFO(
        node->get_logger(),
        "Iniciando adquisicion..."
    );


    enviar_comando(
        fd,
        "DS\r\n"
    );


    std::this_thread::sleep_for(500ms);


    // ========================================================
    // LIMPIAR DATOS QUE HAYAN QUEDADO EN BUFFER
    // ========================================================

    tcflush(
        fd,
        TCIFLUSH
    );


    RCLCPP_INFO(
        node->get_logger(),
        "Adquisicion iniciada."
    );


    // ========================================================
    // BUFFER
    // ========================================================

    uint8_t buf[7];


    // ========================================================
    // SCAN ACTUAL
    // ========================================================

    std::vector<float> current_scan(
        NUM_POINTS,
        std::numeric_limits<float>::infinity()
    );


    std::vector<bool> received(
        NUM_POINTS,
        false
    );


    // ========================================================
    // CONTROL DE SYNC
    // ========================================================

    bool previous_sync = false;


    bool first_rotation = true;


    // ========================================================
    // TIEMPO DE VUELTA
    // ========================================================

    auto previous_rotation_time =
        std::chrono::steady_clock::now();


    // ========================================================
    // LOOP
    // ========================================================

    while (rclcpp::ok())
    {
        // ----------------------------------------------------
        // Leer paquete
        // ----------------------------------------------------

        if (!leer_paquete_valido(
                fd,
                buf))
        {
            continue;
        }


        // ----------------------------------------------------
        // Decodificar
        // ----------------------------------------------------

        uint8_t sync =
            buf[0];


        uint16_t az_raw =
            static_cast<uint16_t>(
                (static_cast<uint16_t>(buf[2]) << 8)
                |
                static_cast<uint16_t>(buf[1])
            );


        uint16_t dist =
            static_cast<uint16_t>(
                (static_cast<uint16_t>(buf[4]) << 8)
                |
                static_cast<uint16_t>(buf[3])
            );


        uint8_t signal =
            buf[5];


        uint8_t checksum =
            buf[6];


        // ----------------------------------------------------
        // Validar checksum nuevamente
        // ----------------------------------------------------

        uint32_t sum = 0;


        for (int i = 0; i < 6; i++)
            sum += buf[i];


        uint8_t calculated_checksum =
            sum % 255;


        if (calculated_checksum != checksum)
        {
            RCLCPP_WARN(
                node->get_logger(),
                "Checksum ERROR"
            );

            continue;
        }


        // ----------------------------------------------------
        // Convertir ángulo
        // ----------------------------------------------------

        float angle =
            static_cast<float>(
                az_raw >> 4
            )
            +
            static_cast<float>(
                az_raw & 15
            ) / 16.0f;


        // ----------------------------------------------------
        // Validar ángulo
        // ----------------------------------------------------

        if (angle < 0.0f ||
            angle >= 360.0f)
        {
            continue;
        }


        // ----------------------------------------------------
        // Detectar flanco SYNC
        // ----------------------------------------------------

        bool sync_now =
            (sync & 0x01);


        bool sync_edge =
            sync_now &&
            !previous_sync;


        previous_sync =
            sync_now;


        // ====================================================
        // SI COMIENZA UNA NUEVA VUELTA
        // ====================================================

        if (sync_edge)
        {
            auto now =
                std::chrono::steady_clock::now();


            // ------------------------------------------------
            // Calcular tiempo de revolución
            // ------------------------------------------------

            std::chrono::duration<double>
                elapsed =
                now -
                previous_rotation_time;


            double scan_time =
                elapsed.count();


            // ------------------------------------------------
            // Primera vuelta
            // ------------------------------------------------

            if (first_rotation)
            {
                RCLCPP_INFO(
                    node->get_logger(),
                    "Primera vuelta detectada. "
                    "No se publica porque puede estar incompleta."
                );


                first_rotation = false;
            }


            // ------------------------------------------------
            // Vueltas posteriores
            // ------------------------------------------------

            else
            {
                // --------------------------------------------
                // Esperamos aproximadamente:
                //
                // 3 Hz = 0.333 s
                //
                // --------------------------------------------

                if (scan_time >= 0.20 &&
                    scan_time <= 0.60)
                {
                    int valid_points = 0;


                    for (int i = 0;
                         i < NUM_POINTS;
                         i++)
                    {
                        if (received[i] &&
                            std::isfinite(
                                current_scan[i]))
                        {
                            valid_points++;
                        }
                    }


                    node->publish_scan(
                        current_scan,
                        scan_time,
                        valid_points
                    );
                }
                else
                {
                    RCLCPP_WARN(
                        node->get_logger(),
                        "Vuelta descartada | "
                        "Tiempo = %.4f s | "
                        "Frecuencia aparente = %.2f Hz",
                        scan_time,
                        1.0 / scan_time
                    );
                }
            }


            // ------------------------------------------------
            // Reiniciar reloj
            // ------------------------------------------------

            previous_rotation_time =
                now;


            // ------------------------------------------------
            // Limpiar scan para nueva vuelta
            // ------------------------------------------------

            std::fill(
                current_scan.begin(),
                current_scan.end(),
                std::numeric_limits<float>::infinity()
            );


            std::fill(
                received.begin(),
                received.end(),
                false
            );
        }


        // ====================================================
        // GUARDAR MEDICIÓN
        // ====================================================

        int index =
            static_cast<int>(
                std::floor(angle)
            );


        if (index < 0 ||
            index >= NUM_POINTS)
        {
            continue;
        }


        // ----------------------------------------------------
        // Distancia
        // ----------------------------------------------------

        float distance =
            static_cast<float>(dist)
            /
            100.0f;


        // ----------------------------------------------------
        // Guardar
        // ----------------------------------------------------

        if (dist == 0 ||
            distance < 0.1f ||
            distance > 40.0f)
        {
            current_scan[index] =
                std::numeric_limits<float>::infinity();
        }
        else
        {
            current_scan[index] =
                distance;
        }


        received[index] = true;


        // ----------------------------------------------------
        // ROS
        // ----------------------------------------------------

        rclcpp::spin_some(node);
    }


    // ========================================================
    // DETENER SENSOR
    // ========================================================

    enviar_comando(
        fd,
        "DX\r\n"
    );


    close(fd);


    rclcpp::shutdown();


    return 0;
}

