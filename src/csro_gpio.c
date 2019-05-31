#include "csro_common.h"
#include "modbus/mb_config.h"

#define LED_PIN (GPIO_NUM_21)
#define RELAY01_PIN (GPIO_NUM_23)
#define RELAY02_PIN (GPIO_NUM_22)
#define RELAY11_PIN (GPIO_NUM_12)
#define RELAY12_PIN (GPIO_NUM_13)

#define GPIO_SELECTED_PIN (1ULL << LED_PIN) | (1ULL << RELAY01_PIN) | (1ULL << RELAY02_PIN) | (1ULL << RELAY11_PIN) | (1ULL << RELAY12_PIN)

static void led_task(void *param)
{
    static bool led_status = false;
    while (true)
    {
        gpio_set_level(LED_PIN, led_status);
        led_status = !led_status;
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void relay_task(void *param)
{
    while (true)
    {
        gpio_set_level(RELAY01_PIN, airsys_regs.coils[50]);
        gpio_set_level(RELAY02_PIN, airsys_regs.coils[51]);
        gpio_set_level(RELAY11_PIN, airsys_regs.coils[52]);
        gpio_set_level(RELAY12_PIN, airsys_regs.coils[53]);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
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

    xTaskCreate(led_task, "led_task", 2048, NULL, configMAX_PRIORITIES - 5, NULL);
    xTaskCreate(relay_task, "relay_task", 2048, NULL, configMAX_PRIORITIES - 4, NULL);
}