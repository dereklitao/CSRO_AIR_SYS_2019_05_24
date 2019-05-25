#include "csro_common.h"

#define LED_PIN (GPIO_NUM_21)
#define RELAY01_PIN (GPIO_NUM_23)
#define RELAY02_PIN (GPIO_NUM_22)
#define RELAY11_PIN (GPIO_NUM_12)
#define RELAY12_PIN (GPIO_NUM_13)

#define GPIO_SELECTED_PIN (1ULL << LED_PIN) | (1ULL << RELAY01_PIN) | (1ULL << RELAY02_PIN) | (1ULL << RELAY11_PIN) | (1ULL << RELAY12_PIN)

static void csro_gpio_task(void *param)
{
    while (true)
    {
        gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void csro_gpio_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_SELECTED_PIN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    xTaskCreate(csro_gpio_task, "csro_gpio_task", 2048, NULL, configMAX_PRIORITIES - 5, NULL);
}