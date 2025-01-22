#include <stdio.h>
#include <string.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi/wifi_manager.h"
#include "esp_netif.h"

// Declaration of the wifi_manager_get_config function
// esp_err_t wifi_manager_get_config(nvs_wifi_config_t *config) {
//     // Dummy implementation for example purposes
//     strcpy(config->wifi_ssid, "example_ssid");
//     strcpy(config->wifi_pass, "example_pass");
//     strcpy(config->mqtt_host, "example_mqtt_host");
//     return ESP_OK;
// }
#include "nvs/nvs_manager.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bmp280.h"
#include "driver/gpio.h"


static const char *TAG = "MAIN";

// Przykładowy pin do "resetu" konfiguracji.
#define RESET_PIN GPIO_NUM_0

#define BUZZER_GPIO 2  // Buzzer connected to GPIO26 (PWM)
#define BUTTON_GPIO 18  // Button connected to GPIO18

void configure_buzzer() {
    ledc_timer_config_t timer_conf = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 2000,   // Set frequency to 2kHz
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .channel = LEDC_CHANNEL_0,
        .duty = 512,  // 50% duty cycle
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&channel_conf);
}

void configure_button() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;  // No interrupt
    io_conf.mode = GPIO_MODE_INPUT;         // Set as input mode
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // Enable pull-up resistor
    gpio_config(&io_conf);
}

/**
 * Bardzo uproszczona pętla sprawdzająca przycisk.
 * Przy dłuższym wciśnięciu wchodzimy w "reset_config".
 */
void check_reset_button(void)
{
    static int press_count = 0;
    int level = gpio_get_level(RESET_PIN);
    if (level == 0) { // zakładamy pull-up i przycisk do GND
        press_count++;
        if (press_count > 50) { // ~0,5 sek
            ESP_LOGW(TAG, "Reset Wi-Fi config triggered!");
            wifi_manager_init(true); // Wymusza AP
        }
    } else {
        press_count = 0;
    }
}

void app_main(void)
{
    // configure_buzzer();
    // configure_button();
    // Konfiguracja pinu przyciskux`
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << RESET_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Uruchamiamy Wi-Fi manager bez resetu
    // - jeśli w NVS jest konfiguracja, przejdzie w STA
    // - jeśli nie ma, przejdzie w AP
    wifi_manager_init(false);

    while (1) {
        check_reset_button();
        // Jeśli chcesz sprawdzić parametry co jakiś czas:
        nvs_wifi_config_t cfg;
        if (wifi_manager_get_config(&cfg) == ESP_OK) {
            //ESP_LOGI(TAG, "Current config: SSID=%s, PASS=%s, MQTT=%s",
                      //cfg.wifi_ssid, cfg.wifi_pass, cfg.mqtt_host);
        }

        // if (gpio_get_level(BUTTON_GPIO) == 0) {  // If button is pressed
        //     ESP_LOGI("APP", "Button pressed, stopping buzzer");
        //     ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);  // Turn off buzzer
        // } else {
        //     ESP_LOGI("APP", "Button not pressed, buzzer running");
        //     ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
        //     ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        // }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
