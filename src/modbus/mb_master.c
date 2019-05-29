#include "mb_config.h"

static bool master_validate_reply(modbus_master *master)
{
    if (master->rx_len < 5)
    {
        return false;
    }
    uint16_t crc = crc16(master->rx_buf, master->rx_len - 2);
    if (((crc >> 8) != master->rx_buf[master->rx_len - 2]) || ((crc & 0xFF) != master->rx_buf[master->rx_len - 1]))
    {
        return false;
    }
    if ((master->rx_buf[0] != master->slave_id) || (master->rx_buf[1] != master->func_code))
    {
        return false;
    }
    return true;
}

bool master_read_discs(modbus_master *master, uint8_t addr, uint8_t qty, uint8_t *result)
{
    master->func_code = MODBUS_FC_READ_DISCRETE_INPUTS;
    master->read_addr = addr;
    master->read_qty = qty;

    master->tx_len = 0;
    master->tx_buf[master->tx_len++] = master->slave_id;
    master->tx_buf[master->tx_len++] = master->func_code;
    master->tx_buf[master->tx_len++] = master->read_addr >> 8;
    master->tx_buf[master->tx_len++] = master->read_addr & 0xFF;
    master->tx_buf[master->tx_len++] = master->read_qty >> 8;
    master->tx_buf[master->tx_len++] = master->read_qty & 0xFF;
    uint16_t crc = crc16(master->tx_buf, master->tx_len);
    master->tx_buf[master->tx_len++] = crc >> 8;
    master->tx_buf[master->tx_len++] = crc & 0xFF;

    if (master->master_send_receive(MODBUS_TIMEOUT))
    {
        if (master_validate_reply(master))
        {
            if (master->rx_buf[2] != (master->read_qty % 8 == 0 ? master->read_qty / 8 : master->read_qty / 8 + 1))
            {
                return false;
            }
            for (int i = 0; i < master->read_qty; i++)
            {
                result[i] = 0x01 & ((master->rx_buf[3 + i / 8]) >> (i % 8));
            }
            return true;
        }
    }
    return false;
}

bool master_read_coils(modbus_master *master, uint8_t addr, uint8_t qty, uint8_t *result)
{
    master->func_code = MODBUS_FC_READ_COILS;
    master->read_addr = addr;
    master->read_qty = qty;

    master->tx_len = 0;
    master->tx_buf[master->tx_len++] = master->slave_id;
    master->tx_buf[master->tx_len++] = master->func_code;
    master->tx_buf[master->tx_len++] = master->read_addr >> 8;
    master->tx_buf[master->tx_len++] = master->read_addr & 0xFF;
    master->tx_buf[master->tx_len++] = master->read_qty >> 8;
    master->tx_buf[master->tx_len++] = master->read_qty & 0xFF;
    uint16_t crc = crc16(master->tx_buf, master->tx_len);
    master->tx_buf[master->tx_len++] = crc >> 8;
    master->tx_buf[master->tx_len++] = crc & 0xFF;

    if (master->master_send_receive(MODBUS_TIMEOUT))
    {
        if (master_validate_reply(master))
        {
            if (master->rx_buf[2] != (master->read_qty % 8 == 0 ? master->read_qty / 8 : master->read_qty / 8 + 1))
            {
                return false;
            }
            for (int i = 0; i < master->read_qty; i++)
            {
                result[i] = 0x01 & ((master->rx_buf[3 + i / 8]) >> (i % 8));
            }
            return true;
        }
    }
    return false;
}

bool master_read_input_regs(modbus_master *master, uint8_t addr, uint8_t qty, uint16_t *result)
{
    master->func_code = MODBUS_FC_READ_INPUT_REGISTERS;
    master->read_addr = addr;
    master->read_qty = qty;

    master->tx_len = 0;
    master->tx_buf[master->tx_len++] = master->slave_id;
    master->tx_buf[master->tx_len++] = master->func_code;
    master->tx_buf[master->tx_len++] = master->read_addr >> 8;
    master->tx_buf[master->tx_len++] = master->read_addr & 0xFF;
    master->tx_buf[master->tx_len++] = master->read_qty >> 8;
    master->tx_buf[master->tx_len++] = master->read_qty & 0xFF;
    uint16_t crc = crc16(master->tx_buf, master->tx_len);
    master->tx_buf[master->tx_len++] = crc >> 8;
    master->tx_buf[master->tx_len++] = crc & 0xFF;

    if (master->master_send_receive(MODBUS_TIMEOUT))
    {
        if (master_validate_reply(master))
        {
            if (master->rx_buf[2] != master->read_qty * 2)
            {
                return false;
            }
            for (int i = 0; i < master->read_qty; i++)
            {
                result[i] = master->rx_buf[3 + i * 2] * 256 + master->rx_buf[4 + i * 2];
            }
            return true;
        }
    }
    return false;
}

bool master_read_holding_regs(modbus_master *master, uint8_t addr, uint8_t qty, uint16_t *result)
{
    master->func_code = MODBUS_FC_READ_HOLDING_REGISTERS;
    master->read_addr = addr;
    master->read_qty = qty;

    master->tx_len = 0;
    master->tx_buf[master->tx_len++] = master->slave_id;
    master->tx_buf[master->tx_len++] = master->func_code;
    master->tx_buf[master->tx_len++] = master->read_addr >> 8;
    master->tx_buf[master->tx_len++] = master->read_addr & 0xFF;
    master->tx_buf[master->tx_len++] = master->read_qty >> 8;
    master->tx_buf[master->tx_len++] = master->read_qty & 0xFF;
    uint16_t crc = crc16(master->tx_buf, master->tx_len);
    master->tx_buf[master->tx_len++] = crc >> 8;
    master->tx_buf[master->tx_len++] = crc & 0xFF;

    if (master->master_send_receive(MODBUS_TIMEOUT))
    {
        if (master_validate_reply(master))
        {
            if (master->rx_buf[2] != master->read_qty * 2)
            {
                return false;
            }
            for (int i = 0; i < master->read_qty; i++)
            {
                result[i] = master->rx_buf[3 + i * 2] * 256 + master->rx_buf[4 + i * 2];
            }
            return true;
        }
    }
    return false;
}

bool master_write_single_coil(modbus_master *master, uint8_t addr, bool value)
{
    master->func_code = MODBUS_FC_WRITE_SINGLE_COIL;
    master->write_addr = addr;

    master->tx_len = 0;
    master->tx_buf[master->tx_len++] = master->slave_id;
    master->tx_buf[master->tx_len++] = master->func_code;
    master->tx_buf[master->tx_len++] = master->write_addr >> 8;
    master->tx_buf[master->tx_len++] = master->write_addr & 0xFF;
    master->tx_buf[master->tx_len++] = value ? 0xFF : 0x00;
    master->tx_buf[master->tx_len++] = 0x00;
    uint16_t crc = crc16(master->tx_buf, master->tx_len);
    master->tx_buf[master->tx_len++] = crc >> 8;
    master->tx_buf[master->tx_len++] = crc & 0xFF;

    if (master->master_send_receive(MODBUS_TIMEOUT))
    {
        if (master_validate_reply(master))
        {
            if (master->rx_buf[2] != (master->write_addr >> 8) || master->rx_buf[3] != (master->write_addr & 0xFF))
            {
                return false;
            }
            if (master->rx_buf[4] != (value ? 0xFF : 0x00) || master->rx_buf[5] != 0x00)
            {
                return false;
            }
            return true;
        }
    }
    return false;
}

bool master_write_single_holding_reg(modbus_master *master, uint8_t addr, uint16_t value)
{
    master->func_code = MODBUS_FC_WRITE_SINGLE_REGISTER;
    master->write_addr = addr;

    master->tx_len = 0;
    master->tx_buf[master->tx_len++] = master->slave_id;
    master->tx_buf[master->tx_len++] = master->func_code;
    master->tx_buf[master->tx_len++] = master->write_addr >> 8;
    master->tx_buf[master->tx_len++] = master->write_addr & 0xFF;
    master->tx_buf[master->tx_len++] = value >> 8;
    master->tx_buf[master->tx_len++] = value & 0xFF;
    uint16_t crc = crc16(master->tx_buf, master->tx_len);
    master->tx_buf[master->tx_len++] = crc >> 8;
    master->tx_buf[master->tx_len++] = crc & 0xFF;

    if (master->master_send_receive(MODBUS_TIMEOUT))
    {
        if (master_validate_reply(master))
        {
            if (master->rx_buf[2] != (master->write_addr >> 8) || master->rx_buf[3] != (master->write_addr & 0xFF))
            {
                return false;
            }
            if (master->rx_buf[4] != (value >> 8) || master->rx_buf[5] != (value & 0xFF))
            {
                return false;
            }
            return true;
        }
    }
    return false;
}