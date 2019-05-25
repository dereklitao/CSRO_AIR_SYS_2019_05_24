#ifndef CSRO_COMMON_H_
#define CSRO_COMMON_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_log.h"

void csro_uart_init(void);
void csro_gpio_init(void);

void csro_debug(char *msg);

#endif