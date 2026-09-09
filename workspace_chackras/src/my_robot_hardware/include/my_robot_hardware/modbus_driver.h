#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <modbus/modbus-rtu.h>
#include <getopt.h>
//#include <modbus_driver/modbus_driver.h>

// https://libmodbus.org/


typedef struct __attribute__((packed)) MotorStruct {
    uint16_t setpoint;
    uint16_t sentido;
    float velocidad;
    float corriente;
} MotorStruct_t;


                                  
#define CHACRAS_ADDR    1
#define MOTOR_BASE      0
#define STATUS_BASE     24
#define ARMADO_BASE     28

modbus_t * modbus_init(char * serialport, int baudrate) {
    modbus_t *ctx;

    // Create a new RTU context with proper serial parameters
    ctx = modbus_new_rtu(serialport, baudrate, 'N', 8, 1);
    if (!ctx) {
        fprintf(stderr, "Failed to create the context: %s\n", modbus_strerror(errno));
    }

    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Unable to connect: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        ctx = NULL;
    }

    // Set the Modbus address of the remote slave
    modbus_set_slave(ctx, CHACRAS_ADDR);

    return ctx;
}

int modbus_wr(modbus_t *ctx, int addr, uint16_t * data, int count) {
    int res = 1;

    // Write multiple registers
    int rc = modbus_write_registers(ctx, addr, count, data); 
    if (rc == -1) {
        fprintf(stderr, "Failed to write multiple registers: %s\n", modbus_strerror(errno));
        res = 0;
    } else {
        printf("Successfully wrote %d registers starting from address %d\n", rc, addr);
        res = 0;
    }

    return res;
}

int modbus_rd(modbus_t *ctx, int addr, uint16_t * data, int count) {
    int num;
    num = modbus_read_registers(ctx, addr, count, data);
    if (num != count) {
        fprintf(stderr, "Failed to read: %s\n", modbus_strerror(errno));
        num = 0;
    }

    return num;
}