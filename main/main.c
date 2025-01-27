#include <stdio.h>
#include <string.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "nvs_flash.h"


//Our libraries
#include "wifi/wifi_manager.h"
#include "nvs/nvs_manager.h"
#include "bmp280/bmp280.h"
#include "mqtt/mqtt_manager.h"
#include "mq2/mq2.h"
#include "ble/ble_gatt_server.h"
#include <esp_wifi_types_generic.h>


static bool stop_signal = false;  // Flaga do zatrzymania działania
static bool is_initialized = false;

static const char *TAG = "MAIN";

// Definicje dla magistrali I2C
#define I2C_MASTER_SCL_IO 22               // Numer pinu GPIO dla linii SCL 
#define I2C_MASTER_SDA_IO 21               // Numer pinu GPIO dla linii SDA 
#define I2C_MASTER_NUM I2C_NUM_0           // Numer portu I2C (I2C0) 
#define I2C_MASTER_FREQ_HZ 100000          // Częstotliwość pracy magistrali I2C (100 kHz) 
#define I2C_MASTER_TX_BUF_DISABLE 0        // Wyłączenie bufora transmisji 
#define I2C_MASTER_RX_BUF_DISABLE 0        // Wyłączenie bufora odbioru 

static TaskHandle_t wifi_task_handle = NULL;

// Przykładowy pin do "resetu" konfiguracji.
#define RESET_PIN 0
#define BUTTON_PIN  GPIO_NUM_18   // Dostosuj do rzeczywistego pinu
#define BUZZER_PIN  GPIO_NUM_26  // Dostosuj do rzeczywistego pinu
#define LED_PIN    GPIO_NUM_25  // Dostosuj do rzeczywistego pinu

// Progi alarmowe
#define TEMP_THRESHOLD 30.0
#define MQ2_THRESHOLD 1500

//#define MQTT_BROKER_URI "mqtt://192.168.144.219:1883"

#define DEVICE_ID "esp32"
#define SERVICE_UUID      0x181A  // UUID serwisu środowiskowego
#define CHARACTERISTIC_UUID 0x2A6E // UUID charakterystyki temperatury

#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523

// Jingle Bells - nuty i czas trwania
int melody[] = {
    NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, 
    NOTE_E4, NOTE_G4, NOTE_C4, NOTE_D4, NOTE_E4,
    NOTE_F4, NOTE_F4, NOTE_F4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, 
    NOTE_E4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_G4
};

int noteDurations[] = {
    300, 300, 600, 300, 300, 600, 
    300, 300, 300, 300, 600,
    300, 300, 300, 300, 300, 300, 300,
    300, 300, 300, 300, 300, 300, 600, 600
};

TaskHandle_t LedBuzzerTaskHandle = NULL;
TaskHandle_t BlinkLedTaskHandle = NULL;

nvs_threshold_config_t tthresholds;

int contains_substring(const char *str, const char *substr) {
    return strstr(str, substr) != NULL;
}

// Funkcja odtwarzania dźwięku
void play_tone(uint32_t frequency, uint32_t duration_ms) {
    ledc_timer_config_t timer_conf = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = frequency,   
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 512,  // 50% duty cycle
        .gpio_num   = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&channel_conf);

    vTaskDelay(pdMS_TO_TICKS(duration_ms));  // Czas trwania tonu

    // Zatrzymanie dźwięku
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}


// Funkcja odtwarzania melodii Jingle Bells
void play_melody() {
    for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
        if (stop_signal) break;
        play_tone(melody[i], noteDurations[i]);
        vTaskDelay(pdMS_TO_TICKS(100));  // Krótka przerwa między nutami
    }
}

// Funkcja migania diody LED
void blink_led_task(void *arg) {
    while (!stop_signal) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

// Task do sterowania LED i buzzera
void led_buzzer_task(void *arg) {
    ESP_LOGI(TAG, "Rozpoczynam miganie LED i odtwarzanie melodii...");

    while (!stop_signal) {
        //gpio_set_level(LED_PIN, 1);  // Włącz LED
        play_melody();  // Odtwarzanie melodii

        //gpio_set_level(LED_PIN, 0);  // Wyłącz LED
        vTaskDelay(pdMS_TO_TICKS(1000));  // Przerwa przed kolejnym cyklem
    }

    ESP_LOGI(TAG, "LED i buzzer zostały wyłączone na stałe.");
    vTaskDelete(NULL);
}


// Task do sprawdzania czujników
void check_sensors_task(void *arg) {
    bmp280_config_t bmp280_config;
    bmp280_config.i2c_port = I2C_NUM_0;
    bmp280_config.i2c_address = 0x76;  // Adres czujnika BMP280
    bmp280_config.oversampling_temp = 1;
    bmp280_config.oversampling_press = 1;
    bmp280_config.mode = BMP280_MODE_NORMAL;

    if (bmp280_init(&bmp280_config) != ESP_OK) {
        ESP_LOGE(TAG, "Nie udało się zainicjalizować BMP280");
        return;
    }

    mq2_init();
    float temperature;
    float pressure;

    while (1) {
        uint16_t gas_value = mq2_read();

        if (bmp280_read_temperature(&bmp280_config, &temperature) != ESP_OK) {
            ESP_LOGE(TAG, "Błąd odczytu temperatury");
            temperature = -1;
        }

        if (bmp280_read_pressure(&bmp280_config, &pressure) != ESP_OK) {
            ESP_LOGE(TAG, "Błąd odczytu ciśnienia");
            pressure = -1;
        }

        ESP_LOGI(TAG, "Temperatura: %.2f °C, Ciśnienie: %.2f hPa, Poziom gazu: %d", temperature, pressure / 100.0, gas_value);
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
        mqtt_manager_send("sensors/temperature", temp_str);
        char pressure_str[16];
        snprintf(pressure_str, sizeof(pressure_str), "%.2f", pressure/100.0);
        mqtt_manager_send("sensors/pressure", pressure_str);
        char gas_str[16];
        snprintf(gas_str, sizeof(gas_str), "%d", gas_value);
        mqtt_manager_send("sensors/smoke", gas_str);

        // Sprawdzenie warunków alarmowych
        if (temperature > tthresholds.temp_threshold || gas_value > tthresholds.smoke_threshold) {
            if (!stop_signal) {
                ESP_LOGW(TAG, "PRZEKROCZONO PROGI! AKTYWACJA ALARMU!");
                stop_signal = false;
                xTaskCreate(led_buzzer_task, "led_buzzer_task", 4096, NULL, 5, &LedBuzzerTaskHandle);
                xTaskCreate(blink_led_task, "blink_led_task", 4096, NULL, 5, &BlinkLedTaskHandle);
                mqtt_manager_send("alarm", "Przekroczono progi bezpieczeństwa!");
                ble_alarm_send();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


//================BLE-task=========================
// void send_alarm_task(void *pvParameters) {
//     while (1) {
//         ESP_LOGI(TAG, "Wysyłanie alarmu...");
//         esp_err_t ret = ble_gatt_server_send_alarm("ALARM: Pożar wykryty!");
//         if (ret == ESP_OK) {
//             ESP_LOGI(TAG, "Alarm został wysłany");
//         } else {
//             ESP_LOGE(TAG, "Nie udało się wysłać alarmu");
//         }
//         vTaskDelay(pdMS_TO_TICKS(10000));  // Co 10 sekund
//     }
// }
//=========================================================


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

    wifi_mode_t mode;
    esp_err_t ret = esp_wifi_get_mode(&mode);
    if (mode == WIFI_MODE_STA) {
            ESP_LOGI("WIFI_MODE", "ESP32 is in Station mode.");
            is_initialized = true;
    }
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

void configure_io_pins() {
    // Konfiguracja pinu LED jako wyjście
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // // Konfiguracja pinu buzzera jako wyjście
    gpio_reset_pin(BUZZER_PIN);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);

    // Konfiguracja pinu przycisku jako wejście z rezystorem pull-up
    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_PIN);
    gpio_pulldown_dis(BUTTON_PIN);
}

// Task do obsługi przycisku
void button_task(void *arg) {   
    while (1) {
        if (gpio_get_level(BUTTON_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));  // Debounce
            if (gpio_get_level(BUTTON_PIN) == 0) {
                ESP_LOGI(TAG, "Przycisk naciśnięty, wyłączam LED i buzzer na stałe.");
                gpio_set_level(LED_PIN, 0);  // Wyłącz LED
                stop_signal = true;  // Ustawienie flagi zatrzymania

                if (LedBuzzerTaskHandle != NULL) {
                    vTaskDelete(LedBuzzerTaskHandle);
                    LedBuzzerTaskHandle = NULL;
                }
                if (BlinkLedTaskHandle != NULL) {
                    vTaskDelete(BlinkLedTaskHandle);
                    BlinkLedTaskHandle = NULL;
                }
                vTaskDelay(pdMS_TO_TICKS(5000));
                stop_signal = false;  // Resetowanie flagi zatrzymania
                
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Oczekiwanie 100 ms
    }
}

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);

    esp_err_t err = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER,
                                       I2C_MASTER_RX_BUF_DISABLE,
                                       I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed: %s\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "I2C configuration success!");
    }
}

// void handle_mqtt_message(const char *topic, const char *message) {

//     // Pobranie aktualnej konfiguracji progów z NVS
//     if (nvs_manager_get_thresholds(&thresholds) != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to load current thresholds from NVS");
//         return;
//     }

//     // Sprawdzenie, czy otrzymano temat do aktualizacji progów temperatury
//     if (contains_substring(topic, "config/threshold/temperature")) {
//         thresholds.temp_threshold = atoi(message);
//         ESP_LOGI(TAG, "Updating temp_threshold to %d", thresholds.temp_threshold);
//     }
//     // Sprawdzenie, czy otrzymano temat do aktualizacji progów dymu
//     else if (contains_substring(topic, "config/threshold/smoke")) {
//         thresholds.smoke_threshold = atoi(message);
//         ESP_LOGI(TAG, "Updating smoke_threshold to %d", thresholds.smoke_threshold);
//     }
//     // Sprawdzenie, czy otrzymano temat resetu Wi-Fi
//     else if (contains_substring(topic, "config/reset")) {
//         ESP_LOGW(TAG, "Received reset command, starting Wi-Fi reset task...");
//         xTaskCreate(reset_wifi_task, "reset_wifi_task", 4096, NULL, 5, NULL);
//         return;
//     } else {
//         ESP_LOGW(TAG, "Unknown topic received: %s", topic);
//         return;
//     }

//     // Zapis nowych progów do NVS
//     if (nvs_manager_set_thresholds(&thresholds) == ESP_OK) {
//         ESP_LOGI(TAG, "Thresholds updated successfully in NVS");
//     } else {
//         ESP_LOGE(TAG, "Failed to save new thresholds to NVS");
//     }
// }

// // Task odbierający wiadomości MQTT
// void mqtt_receive_task(void *pvParameter) {
//     char topic[128];
//     char message[128];
//     const TickType_t wait_time = pdMS_TO_TICKS(15000);  // 5 sekund timeout
//     ESP_LOGI(TAG, "Starting MQTT receive task...");
//     while (1) {
//         esp_err_t ret = mqtt_manager_receive(topic, sizeof(topic), message, sizeof(message), wait_time);

//         if (ret == ESP_OK) {
//             ESP_LOGI(TAG, "Received MQTT message:");
//             ESP_LOGI(TAG, "Topic: %s", topic);
//             ESP_LOGI(TAG, "Message: %s", message);

//             // Obsługa otrzymanej wiadomości
//             handle_mqtt_message(topic, message);
//         } else if (ret == ESP_ERR_TIMEOUT) {
//             ESP_LOGW(TAG, "MQTT receive timeout, waiting...");
//         } else {
//             ESP_LOGE(TAG, "Error receiving MQTT message");
//         }

//         // Sprawdzanie stanu przycisku
//         int button_state = gpio_get_level(BUTTON_PIN);
//         if (button_state == 0) {
//             ESP_LOGW(TAG, "Button pressed, triggering Wi-Fi reset...");
//             xTaskCreate(reset_wifi_task, "reset_wifi_task", 4096, NULL, 5, NULL);
//         }

//         vTaskDelay(pdMS_TO_TICKS(500)); // Opóźnienie między kolejnymi iteracjami
//     }
// }


void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS flash erase required, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "NVS initialized successfully.");
    
    // Inicjalizacja kontrolera BT przed rozpoczęciem Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth controller initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Bluetooth initialized successfully.");

    ble_gatt_server_init();  // Uruchomienie BLE GATT Servera
    //===================================================BMP280==============================================================
    // if (ble_alarm_init() != ESP_OK) {
    //     printf("Błąd inicjalizacji BLE Alarm!\n");
    //     return;
    // }
    // printf("BLE Alarm zainicjalizowany\n");


    i2c_master_init();

        // Konfiguracja czujnika BMP280 
    bmp280_config_t config = {
        .i2c_port = I2C_NUM_0,           // Port I2C używany do komunikacji
        .i2c_address = 0x76,            // Adres I2C czujnika BMP280 
        .mode = BMP280_MODE_NORMAL,     // Tryb pracy czujnika: normalny 
        .oversampling_temp = OS_X2,     // Oversampling temperatury: x2 
        .oversampling_press = OS_X2     // Oversampling ciśnienia: x2 
    };

    //=================================================================================================================

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
    //========================================================WIFI MANAGER==============================================================
    init_reset_button();
    xTaskCreate(reset_button_task, "reset_button_task", 2048, NULL, 10, NULL);

    // Uruchomienie taska Wi-Fi po starcie
    bool reset_wifi = false;
    xTaskCreate(wifi_manager_task, "wifi_manager_task", 4096, (void *)&reset_wifi, 5, &wifi_task_handle);

    ESP_LOGI(TAG, "Inicjalizacja systemu...");

    while (!is_initialized)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    esp_err_t err = nvs_manager_get_thresholds(&tthresholds);
    if (err != ESP_OK) {
        tthresholds.temp_threshold = TEMP_THRESHOLD;
        tthresholds.smoke_threshold = MQ2_THRESHOLD;
    }

    vTaskDelay(pdMS_TO_TICKS(10000));  // Odczekaj 5 sekund na stabilizację Wi-Fi
    
    

    // esp_err_t ret = ble_gatt_server_init();
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Inicjalizacja BLE GATT Server nie powiodła się: %s", esp_err_to_name(ret));
    //     return;
    // }
    // xTaskCreate(&send_alarm_task, "send_alarm_task", 4096, NULL, 5, NULL);

    

    // ============================================Initialize MQTT Manager=======================================================
    
    //===================================================================================================================================
    
    //===============================================Czujniki i Alarm =====================================================

    configure_io_pins();
    xTaskCreate(check_sensors_task, "check_sensors_task", 4096, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    
    //===================================================================================================================================
    ESP_LOGI(TAG, "Starting BLE Alarm System...");
    
    ble_gatt_server_init();
    // esp_err_t ret = ble_alarm_init();
    // if (ret != ESP_OK) {
    //     ESP_LOGE("MAIN", "BLE initialization failed!");
    //     return;
    // }
    // char test_topic[100];
    // snprintf(test_topic, sizeof(test_topic), "test");
    
    
    // // Inicjalizacja czujnika MQ-2
    // mq2_init

    //xTaskCreate(mqtt_receive_task, "mqtt_receive_task", 4096, NULL, 5, NULL);


    while (1) {

        // check_reset_button();
        // Jeśli chcesz sprawdzić parametry co jakiś czas:
        nvs_wifi_config_t cfg;
        if (wifi_manager_get_config(&cfg) == ESP_OK) {
            //ESP_LOGI(TAG, "Current config: SSID=%s, PASS=%s, MQTT=%s",
                      //cfg.wifi_ssid, cfg.wifi_pass, cfg.mqtt_host);
        }



        
        char topic[MQTT_TOPIC_MAX_LEN] = {0};
        char message[MQTT_MSG_MAX_LEN] = {0};

        TickType_t wait_time = pdMS_TO_TICKS(5000);  // Czekaj 5 sekund na wiadomość

        // esp_err_t ret = mqtt_manager_receive(topic, sizeof(topic), message, sizeof(message), wait_time);

        // if (ret == ESP_OK) {
        //     ESP_LOGI(TAG, "Odebrano wiadomość:");
        //     ESP_LOGI(TAG, "Temat: %s", topic);
        //     ESP_LOGI(TAG, "Treść: %s", message);
        // } else if (ret == ESP_ERR_TIMEOUT) {
        //     ESP_LOGW(TAG, "Czas oczekiwania na wiadomość upłynął");
        // } else {
        //     ESP_LOGE(TAG, "Błąd odbioru wiadomości MQTT");
        // }
        // button_state = gpio_get_level(BUTTON_PIN);
        //if(alarm_is_enabled()){
        //ble_alarm_init();
        //ESP_LOGI(TAG, "Sending BLE alarm notification...");
        //ble_alarm_send();
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        
    
    }
         
}
