#include "csro_common.h"
#include "modbus/mb_config.h"

#define BUF_SIZE (1024)
#define TXD0_PIN (GPIO_NUM_15)
#define RXD0_PIN (GPIO_NUM_2)
#define RTS0_PIN (GPIO_NUM_0)
#define TXD1_PIN (GPIO_NUM_4)
#define RXD1_PIN (GPIO_NUM_17)
#define RTS1_PIN (GPIO_NUM_16)
#define TXD2_PIN (GPIO_NUM_5)
#define RXD2_PIN (GPIO_NUM_19)
#define RTS2_PIN (GPIO_NUM_18)

portBASE_TYPE HPTaskAwoken = 0;

modbus_master master_ap;
modbus_master master_ac;
modbus_slave slave_hmi;
device_regs airsys_regs;

SemaphoreHandle_t uart1_mutex;

uint8_t ap_coil[30];
uint8_t ac_coil[30];

char message[200];

static void uart_receive_one_byte(uart_port_t uart_num, uint8_t data)
{
    if (uart_num == master_ap.uart_num)
    {
        master_ap.rx_buf[master_ap.rx_len++] = data;
    }
    else if (uart_num == master_ac.uart_num)
    {
        master_ac.rx_buf[master_ac.rx_len++] = data;
    }
    else if (uart_num == slave_hmi.uart_num)
    {
        slave_hmi.rx_buf[slave_hmi.rx_len++] = data;
    }
}

static void uart_receive_complete(uart_port_t uart_num)
{
    if (uart_num == master_ap.uart_num)
    {
        xSemaphoreGiveFromISR(master_ap.reply_sem, &HPTaskAwoken);
        uart_flush(master_ap.uart_num);
    }
    else if (uart_num == master_ac.uart_num)
    {
        xSemaphoreGiveFromISR(master_ac.reply_sem, &HPTaskAwoken);
        uart_flush(master_ac.uart_num);
    }
    else if (uart_num == slave_hmi.uart_num)
    {
        uart_flush(slave_hmi.uart_num);
        slave_handle_command(&slave_hmi);
    }
}

static void modbus_ap_task(void *param)
{
    while (true)
    {
        master_read_coils(&master_ap, 1, 20, ap_coil);

        xSemaphoreTake(uart1_mutex, portMAX_DELAY);
        csro_debug("ap_coil = ");
        for (size_t i = 0; i < 20; i++)
        {
            sprintf(message, "%d|", ap_coil[i]);
            csro_debug(message);
        }
        csro_debug("\r\n");
        xSemaphoreGive(uart1_mutex);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void modbus_ac_task(void *param)
{
    while (true)
    {
        master_read_coils(&master_ac, 1, 20, ac_coil);

        xSemaphoreTake(uart1_mutex, portMAX_DELAY);
        csro_debug("ac_coil = ");
        for (size_t i = 0; i < 20; i++)
        {
            sprintf(message, "%d|", ac_coil[i]);
            csro_debug(message);
        }
        csro_debug("\r\n");
        xSemaphoreGive(uart1_mutex);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static bool master_ap_send_receive(uint16_t timeout)
{
    master_ap.status = false;
    master_ap.rx_len = 0;
    uart_write_bytes(master_ap.uart_num, (const char *)master_ap.tx_buf, master_ap.tx_len);
    if (xSemaphoreTake(master_ap.reply_sem, timeout / portTICK_PERIOD_MS) == pdTRUE)
    {
        master_ap.status = true;
    }
    return master_ap.status;
}

static bool master_ac_send_receive(uint16_t timeout)
{
    master_ac.status = false;
    master_ac.rx_len = 0;
    uart_write_bytes(master_ac.uart_num, (const char *)master_ac.tx_buf, master_ac.tx_len);
    if (xSemaphoreTake(master_ac.reply_sem, timeout / portTICK_PERIOD_MS) == pdTRUE)
    {
        master_ac.status = true;
    }
    return master_ac.status;
}

static void modbus_init(void)
{
    uart1_mutex = xSemaphoreCreateMutex();

    master_ap.uart_num = UART_NUM_0;
    master_ap.slave_id = 1;
    master_ap.master_send_receive = master_ap_send_receive;
    master_ap.reply_sem = xSemaphoreCreateBinary();

    master_ac.uart_num = UART_NUM_2;
    master_ac.slave_id = 2;
    master_ac.master_send_receive = master_ac_send_receive;
    master_ac.reply_sem = xSemaphoreCreateBinary();
}

void csro_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    uart_param_config(UART_NUM_0, &uart_config);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_param_config(UART_NUM_2, &uart_config);

    uart_set_pin(UART_NUM_0, TXD0_PIN, RXD0_PIN, RTS0_PIN, UART_PIN_NO_CHANGE);
    uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, RTS1_PIN, UART_PIN_NO_CHANGE);
    uart_set_pin(UART_NUM_2, TXD2_PIN, RXD2_PIN, RTS2_PIN, UART_PIN_NO_CHANGE);

    uart_handler.receive_one_byte = uart_receive_one_byte;
    uart_handler.receive_complete = uart_receive_complete;

    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);

    uart_set_mode(UART_NUM_0, UART_MODE_RS485_HALF_DUPLEX);
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);
    uart_set_mode(UART_NUM_2, UART_MODE_RS485_HALF_DUPLEX);

    modbus_init();

    xTaskCreate(modbus_ap_task, "modbus_ap_task", 2048, NULL, configMAX_PRIORITIES - 4, NULL);
    xTaskCreate(modbus_ac_task, "modbus_ac_task", 2048, NULL, configMAX_PRIORITIES - 4, NULL);
}

void csro_debug(char *msg)
{
    uart_write_bytes(UART_NUM_1, (const char *)msg, strlen(msg));
}