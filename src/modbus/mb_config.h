#ifndef __MB_CONFIG_H
#define __MB_CONFIG_H

#include "csro_common.h"

#define MODBUS_FC_READ_COILS 0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS 0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS 0x03
#define MODBUS_FC_READ_INPUT_REGISTERS 0x04
#define MODBUS_FC_WRITE_SINGLE_COIL 0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER 0x06
#define MODBUS_FC_READ_EXCEPTION_STATUS 0x07
#define MODBUS_FC_WRITE_MULTIPLE_COILS 0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS 0x10
#define MODBUS_FC_REPORT_SLAVE_ID 0x11
#define MODBUS_FC_MASK_WRITE_REGISTER 0x16
#define MODBUS_FC_WRITE_AND_READ_REGISTERS 0x17

#define MODBUS_BUFFER_LENGTH 1024
#define MODBUS_TIMEOUT 500

#define MODBUS_COIL_MAX 255
#define MODBUS_HOLDING_MAX 255

typedef struct
{
    uint8_t coils[MODBUS_COIL_MAX];
    bool coil_flags[MODBUS_COIL_MAX];

    uint16_t holdings[MODBUS_HOLDING_MAX];
    bool holding_flags[MODBUS_HOLDING_MAX];
} device_regs;

typedef struct
{
    uint8_t uart_num;
    uint8_t slave_id;
    uint8_t func_code;

    uint16_t read_addr;
    uint16_t read_qty;
    uint16_t write_addr;
    uint16_t write_qty;

    uint8_t rx_buf[MODBUS_BUFFER_LENGTH];
    uint16_t rx_len;
    uint8_t tx_buf[MODBUS_BUFFER_LENGTH];
    uint16_t tx_len;
    bool status;

    bool (*master_send_receive)(uint16_t timeout);
    SemaphoreHandle_t reply_sem;
} modbus_master;

typedef struct
{
    uint8_t uart_num;
    uint8_t slave_id;
    uint8_t func_code;

    uint16_t read_addr;
    uint16_t read_qty;
    uint16_t write_addr;
    uint16_t write_qty;

    uint8_t rx_buf[MODBUS_BUFFER_LENGTH];
    uint16_t rx_len;
    uint8_t tx_buf[MODBUS_BUFFER_LENGTH];
    uint16_t tx_len;

    device_regs *regs;
    SemaphoreHandle_t command_sem;
} modbus_slave;

extern modbus_master master_ap;
extern modbus_master master_ac;
extern modbus_slave slave_hmi;
extern device_regs airsys_regs;

void modbus_ap_task(void *param);
void modbus_ac_task(void *param);
void modbus_hmi_task(void *param);

uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);
bool master_read_discs(modbus_master *master, uint8_t addr, uint8_t qty, uint8_t *result);
bool master_read_coils(modbus_master *master, uint8_t addr, uint8_t qty, uint8_t *result);
bool master_read_input_regs(modbus_master *master, uint8_t addr, uint8_t qty, uint16_t *result);
bool master_read_holding_regs(modbus_master *master, uint8_t addr, uint8_t qty, uint16_t *result);
bool master_write_single_coil(modbus_master *master, uint8_t addr, uint8_t value);
bool master_write_single_holding_reg(modbus_master *master, uint8_t addr, uint16_t value);
void slave_handle_command(modbus_slave *slave);

#endif