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

modbus_master master_ap;
portBASE_TYPE HPTaskAwoken = 0;
SemaphoreHandle_t ap_reply_sem;

uint32_t coil[30];

static void uart_receive_one_byte(uart_port_t uart_num, uint8_t data)
{
    if (uart_num == UART_NUM_0)
    {
        master_ap.rx_buf[master_ap.rx_len++] = data;
    }
}

static void uart_receive_complete(uart_port_t uart_num)
{
    if (uart_num == UART_NUM_0)
    {
        xSemaphoreGiveFromISR(ap_reply_sem, &HPTaskAwoken);
    }
}

static void modbus_task(void *param)
{
    while (true)
    {
        master_read_coils(&master_ap, 1, 20, coil);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static bool master_ap_uart0_send_receive(uint16_t timeout)
{
    xSemaphoreTake(ap_reply_sem, 0);
    uart_write_bytes(UART_NUM_0, (const char *)master_ap.tx_buf, master_ap.tx_len);
    if (xSemaphoreTake(ap_reply_sem, timeout) == pdTRUE)
    {
        master_ap.status = true;
    }
    else
    {
        master_ap.status = false;
    }
    return master_ap.status;
}

static void modbus_init(void)
{
    master_ap.slave_id = 1;
    master_ap.master_send_receive = master_ap_uart0_send_receive;
    ap_reply_sem = xSemaphoreCreateBinary();
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

    xTaskCreate(modbus_task, "modbus_task", 2048, NULL, configMAX_PRIORITIES - 4, NULL);
}