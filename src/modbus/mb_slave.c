#include "mb_config.h"

static void handle_read_coils(modbus_slave *slave)
{
    slave->read_addr = slave->rx_buf[2] * 256 + slave->rx_buf[3];
    slave->read_qty = slave->rx_buf[4] * 256 + slave->rx_buf[5];
    if (slave->read_addr + slave->read_qty > MODBUS_COIL_MAX)
    {
        slave->tx_buf[slave->tx_len++] = 0x80 + slave->rx_buf[1];
        slave->tx_buf[slave->tx_len++] = 0x02;
    }
    else
    {
        uint8_t byte_count = (slave->read_qty % 8 == 0) ? (slave->read_qty / 8) : (slave->read_qty / 8 + 1);
        slave->tx_buf[slave->tx_len++] = slave->rx_buf[1];
        slave->tx_buf[slave->tx_len++] = byte_count;
        for (size_t i = 0; i < byte_count * 8; i++)
        {
            static uint8_t data = 0;
            if (i % 8 == 0)
            {
                data = 0x00;
            }
            if (i < slave->read_qty)
            {
                data = (airsys_regs.coils[slave->read_addr + i] == true) ? (data >> 1 | 0x80) : (data >> 1 & 0x7F);
            }
            else
            {
                data = data >> 1 & 0x7F;
            }
            if ((i + 1) % 8 == 0)
            {
                slave->tx_buf[slave->tx_len++] = data;
            }
        }
    }
}

static void handle_read_holding_regs(modbus_slave *slave)
{
    slave->read_addr = slave->rx_buf[2] * 256 + slave->rx_buf[3];
    slave->read_qty = slave->rx_buf[4] * 256 + slave->rx_buf[5];
    if (slave->read_addr + slave->read_qty > MODBUS_HOLDING_MAX)
    {
        slave->tx_buf[slave->tx_len++] = 0x80 + slave->rx_buf[1];
        slave->tx_buf[slave->tx_len++] = 0x02;
    }
    else
    {
        slave->tx_buf[slave->tx_len++] = slave->rx_buf[1];
        slave->tx_buf[slave->tx_len++] = slave->read_qty * 2;
        for (size_t i = 0; i < slave->read_qty; i++)
        {
            slave->tx_buf[slave->tx_len++] = airsys_regs.holdings[i + slave->read_addr] >> 8;
            slave->tx_buf[slave->tx_len++] = airsys_regs.holdings[i + slave->read_addr] & 0xFF;
        }
    }
}

static void handle_write_single_coil(modbus_slave *slave)
{
    slave->write_addr = slave->rx_buf[2] * 256 + slave->rx_buf[3];
    if (slave->write_addr + 1 > MODBUS_COIL_MAX)
    {
        slave->tx_buf[slave->tx_len++] = 0x80 + slave->rx_buf[1];
        slave->tx_buf[slave->tx_len++] = 0x02;
    }
    else
    {
        slave->tx_buf[slave->tx_len++] = slave->rx_buf[1];
        slave->tx_buf[slave->tx_len++] = slave->write_addr >> 8;
        slave->tx_buf[slave->tx_len++] = slave->write_addr & 0xFF;
        if (slave->rx_buf[4] == 0xFF && slave->rx_buf[5] == 0x00)
        {
            airsys_regs.coils[slave->write_addr] = true;
            slave->tx_buf[slave->tx_len++] = 0xFF;
        }
        else
        {
            airsys_regs.coils[slave->write_addr] = false;
            slave->tx_buf[slave->tx_len++] = 0x00;
        }
        slave->tx_buf[slave->tx_len++] = 0x00;
        airsys_regs.coil_flags[slave->write_addr] = true;
    }
}

static void handle_write_multi_coils(modbus_slave *slave)
{
    slave->write_addr = slave->rx_buf[2] * 256 + slave->rx_buf[3];
    slave->write_qty = slave->rx_buf[4] * 256 + slave->rx_buf[5];
    if (slave->write_addr + slave->write_qty > MODBUS_COIL_MAX)
    {
        slave->tx_buf[slave->tx_len++] = 0x80 + slave->rx_buf[1];
        slave->tx_buf[slave->tx_len++] = 0x02;
    }
    else
    {
        slave->tx_buf[slave->tx_len++] = slave->rx_buf[1];
        for (size_t i = 0; i < slave->write_qty; i++)
        {
            airsys_regs.coils[slave->write_addr + i] = (slave->rx_buf[7 + i / 8] >> (i % 8)) & 0x01;
            airsys_regs.coil_flags[slave->write_addr + i] = true;
        }
        slave->tx_buf[slave->tx_len++] = slave->write_addr >> 8;
        slave->tx_buf[slave->tx_len++] = slave->write_addr & 0xFF;
        slave->tx_buf[slave->tx_len++] = slave->write_qty >> 8;
        slave->tx_buf[slave->tx_len++] = slave->write_qty & 0xFF;
    }
}

static void handle_write_single_holding_reg(modbus_slave *slave)
{
    slave->write_addr = slave->rx_buf[2] * 256 + slave->rx_buf[3];
    if (slave->write_addr + 1 > MODBUS_HOLDING_MAX)
    {
        slave->tx_buf[slave->tx_len++] = 0x80 + slave->rx_buf[1];
        slave->tx_buf[slave->tx_len++] = 0x02;
    }
    else
    {
        airsys_regs.holdings[slave->write_addr] = slave->rx_buf[4] * 256 + slave->rx_buf[5];
        slave->tx_buf[slave->tx_len++] = slave->rx_buf[1];
        slave->tx_buf[slave->tx_len++] = slave->write_addr >> 8;
        slave->tx_buf[slave->tx_len++] = slave->write_addr & 0xFF;
        slave->tx_buf[slave->tx_len++] = airsys_regs.holdings[slave->write_addr] >> 8;
        slave->tx_buf[slave->tx_len++] = airsys_regs.holdings[slave->write_addr] & 0xFF;
        airsys_regs.holding_flags[slave->write_addr] = true;
    }
}

static void handle_write_multi_holding_regs(modbus_slave *slave)
{
    slave->write_addr = slave->rx_buf[2] * 256 + slave->rx_buf[3];
    slave->write_qty = slave->rx_buf[4] * 256 + slave->rx_buf[5];
    if (slave->write_addr + slave->write_qty > MODBUS_HOLDING_MAX)
    {
        slave->tx_buf[slave->tx_len++] = 0x80 + slave->rx_buf[1];
        slave->tx_buf[slave->tx_len++] = 0x02;
    }
    else
    {
        for (size_t i = 0; i < slave->write_qty; i++)
        {
            airsys_regs.holdings[slave->write_addr + i] = slave->rx_buf[7 + 2 * i] * 256 + slave->rx_buf[8 + 2 * i];
            airsys_regs.holding_flags[slave->write_addr + i] = true;
        }
        slave->tx_buf[slave->tx_len++] = slave->rx_buf[1];
        slave->tx_buf[slave->tx_len++] = slave->write_addr >> 8;
        slave->tx_buf[slave->tx_len++] = slave->write_addr & 0xFF;
        slave->tx_buf[slave->tx_len++] = slave->write_qty >> 8;
        slave->tx_buf[slave->tx_len++] = slave->write_qty & 0xFF;
    }
}

static void handle_unsupport_fucntion(modbus_slave *slave)
{
    slave->tx_buf[slave->tx_len++] = 0x80 + slave->rx_buf[1];
    slave->tx_buf[slave->tx_len++] = 0x01;
}

void slave_handle_command(modbus_slave *slave)
{
    if (slave->rx_len < 8)
    {
        return;
    }
    uint16_t crc = crc16(slave->rx_buf, slave->rx_len - 2);
    if (((crc >> 8) != slave->rx_buf[slave->rx_len - 2]) || ((crc & 0xFF) != slave->rx_buf[slave->rx_len - 1]))
    {
        return;
    }
    if (slave->rx_buf[0] != slave->slave_id)
    {
        return;
    }

    slave->tx_len = 0;
    slave->tx_buf[slave->tx_len++] = slave->slave_id;

    if (slave->rx_buf[1] == MODBUS_FC_READ_COILS)
    {
        handle_read_coils(slave);
    }
    else if (slave->rx_buf[1] == MODBUS_FC_READ_HOLDING_REGISTERS)
    {
        handle_read_holding_regs(slave);
    }
    else if (slave->rx_buf[1] == MODBUS_FC_WRITE_SINGLE_COIL)
    {
        handle_write_single_coil(slave);
    }
    else if (slave->rx_buf[1] == MODBUS_FC_WRITE_MULTIPLE_COILS)
    {
        handle_write_multi_coils(slave);
    }
    else if (slave->rx_buf[1] == MODBUS_FC_WRITE_SINGLE_REGISTER)
    {
        handle_write_single_holding_reg(slave);
    }
    else if (slave->rx_buf[1] == MODBUS_FC_WRITE_MULTIPLE_REGISTERS)
    {
        handle_write_multi_holding_regs(slave);
    }
    else
    {
        handle_unsupport_fucntion(slave);
    }
    crc = crc16(slave->tx_buf, slave->tx_len);
    slave->tx_buf[slave->tx_len++] = crc >> 8;
    slave->tx_buf[slave->tx_len++] = crc & 0xFF;
    uart_write_bytes(slave->uart_num, (const char *)slave->tx_buf, slave->tx_len);
}