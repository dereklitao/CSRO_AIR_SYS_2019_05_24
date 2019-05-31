#include "mb_config.h"

modbus_master master_ap;
modbus_master master_ac;
modbus_slave slave_hmi;
device_regs airsys_regs;

void modbus_ap_task(void *param)
{
    while (true)
    {
        master_read_coils(&master_ap, 1, 20, airsys_regs.coils);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void modbus_ac_task(void *param)
{
    while (true)
    {
        master_read_coils(&master_ac, 1, 20, &airsys_regs.coils[100]);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void modbus_hmi_task(void *param)
{
    while (true)
    {
        if (xSemaphoreTake(slave_hmi.command_sem, portMAX_DELAY) == pdTRUE)
        {
            slave_handle_command(&slave_hmi);
            slave_hmi.rx_len = 0;
        }
    }
}