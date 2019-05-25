#include "mb_config.h"

static bool master_validate_reply(modbus_master *master)
{
    if (master->rx_len < 5)
    {
        return false;
    }
    uint16_t crc = crc16(master->rx_buf, master->rx_len - 2);
    if (((crc & 0xFF) != master->rx_buf[master->rx_len - 2]) || ((crc >> 8) != master->rx_buf[master->rx_len - 1]))
    {
        return false;
    }
    if ((master->rx_buf[0] != master->slave_id) || (master->rx_buf[1] != master->func_code))
    {
        return false;
    }
    return true;
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
    master->tx_buf[master->tx_len++] = crc & 0xFF;
    master->tx_buf[master->tx_len++] = crc >> 8;

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
                printf("%d|", result[i]);
            }
            printf("\r\n");
            return true;
        }
    }
    return false;
}