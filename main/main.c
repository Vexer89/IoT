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

static TaskHandle_t wifi_task_handle = NULL;

// Przykładowy pin do "resetu" konfiguracji.
#define RESET_PIN 0
#define BUTTON_PIN  GPIO_NUM_18   // Dostosuj do rzeczywistego pinu
#define BUZZER_PIN  GPIO_NUM_26  // Dostosuj do rzeczywistego pinu
#define TEMP_THRESHOLD 30.0     // Próg temperatury w stopniach Celsjusza

// void check_temperature_and_alert() {
//     float temperature, pressure;
//     bmp280_read_float(&temperature, &pressure, NULL);

//     if (temperature > TEMP_THRESHOLD) {
//         gpio_set_level(BUZZER_PIN, 1);  // Włącz buzzer
//     } else {
//         gpio_set_level(BUZZER_PIN, 0);  // Wyłącz buzzer
//     }
// }



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
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&channel_conf);
}

void configure_button() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;  // No interrupt
    io_conf.mode = GPIO_MODE_INPUT;         // Set as input mode
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // Enable pull-up resistor
    gpio_config(&io_conf);
}

/**
 * Bardzo uproszczona pętla sprawdzająca przycisk.
 * Przy dłuższym wciśnięciu wchodzimy w "reset_config".
 */

void init_reset_button(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RESET_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

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

void wifi_manager_task(void *pvParameters) {
    bool reset_flag = *((bool *)pvParameters);
    ESP_LOGI("WiFi", "Starting WiFi initialization task...");
    
    wifi_manager_init(reset_flag);  // Inicjalizacja Wi-Fi

    ESP_LOGI("WiFi", "WiFi initialization completed, deleting task...");

    wifi_task_handle = NULL;  // Zerowanie uchwytu po zakończeniu
    vTaskDelete(NULL);  // Zakończenie taska po zakończeniu inicjalizacji

}

void reset_wifi_task(void) {
    if (wifi_task_handle != NULL) {
        ESP_LOGW("WiFi", "Stopping WiFi task...");
        vTaskDelete(wifi_task_handle);
        wifi_task_handle = NULL;
    }

    ESP_LOGI("WiFi", "Restarting WiFi task...");
    bool reset_wifi = true;
    xTaskCreate(wifi_manager_task, "wifi_manager_task", 4096, (void *)&reset_wifi, 5, &wifi_task_handle);
}

void reset_button_task(void *arg) {
    while (1) {
        if (gpio_get_level(RESET_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));  // Debouncing
            if (gpio_get_level(RESET_PIN) == 0) {
                ESP_LOGW("RESET", "Reset button pressed! Restarting Wi-Fi...");
                reset_wifi_task();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Odczekaj 100 ms, aby nie przeciążać CPU
    }
}


void app_main(void)
{

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
    init_reset_button();
    xTaskCreate(reset_button_task, "reset_button_task", 2048, NULL, 10, NULL);

    // Uruchomienie taska Wi-Fi po starcie
    bool reset_wifi = false;
    xTaskCreate(wifi_manager_task, "wifi_manager_task", 4096, (void *)&reset_wifi, 5, &wifi_task_handle);
    ESP_LOGI(TAG, "Inicjalizacja systemu...");
    
    //====================================================BUZZER BUTTON==================================================================
    // // Resetowanie i konfiguracja pinu buzzera jako wyjście
    // gpio_reset_pin(BUZZER_PIN);
    // gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    
    // // Resetowanie i konfiguracja pinu przycisku jako wejście z rezystorem pull-up
    // gpio_reset_pin(BUTTON_PIN);
    // gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    // gpio_pullup_en(BUTTON_PIN);  // Aktywacja pull-up
    // gpio_pulldown_dis(BUTTON_PIN);  // Dezaktywacja pull-down

    // // Domyślnie buzzer włączony
    // gpio_set_level(BUZZER_PIN, 1);
    // ESP_LOGI(TAG, "Buzzer włączony domyślnie.");

    // int button_state = 1;  // Stan przycisku (początkowo nienaciśnięty)
    // int previous_button_state = 1;  // Poprzedni stan przycisku
    //======================================================================================================================
    

    while (1) {


        // check_reset_button();
        // Jeśli chcesz sprawdzić parametry co jakiś czas:
        // nvs_wifi_config_t cfg;
        // if (wifi_manager_get_config(&cfg) == ESP_OK) {
        //     //ESP_LOGI(TAG, "Current config: SSID=%s, PASS=%s, MQTT=%s",
        //               //cfg.wifi_ssid, cfg.wifi_pass, cfg.mqtt_host);
        // }





        // button_state = gpio_get_level(BUTTON_PIN);

        // // Sprawdzenie stanu przycisku (0 oznacza naciśnięty przycisk)
        // if (button_state == 0 && previous_button_state == 1) {
        //     ESP_LOGI(TAG, "Przycisk naciśnięty, wyłączam buzzer.");
        //     gpio_set_level(BUZZER_PIN, 0);  // Wyłącz buzzer
        // }

        // previous_button_state = button_state;  // Aktualizacja poprzedniego stanu
        vTaskDelay(pdMS_TO_TICKS(100));  // Oczekiwanie 100 ms dla stabilności
    
    }
         
}
