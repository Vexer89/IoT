#include <stdio.h>
#include <string.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"

// Declaration of the wifi_manager_get_config function
// esp_err_t wifi_manager_get_config(nvs_wifi_config_t *config) {
//     // Dummy implementation for example purposes
//     strcpy(config->wifi_ssid, "example_ssid");
//     strcpy(config->wifi_pass, "example_pass");
//     strcpy(config->mqtt_host, "example_mqtt_host");
//     return ESP_OK;
// }
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

//Our libraries
#include "wifi/wifi_manager.h"
#include "nvs/nvs_manager.h"
#include "bmp280/bmp280.h"
#include "mqtt/mqtt_manager.h"
#include "mq2/mq2.h"
#include "driver/i2c_master.h"


static bool stop_signal = false;  // Flaga do zatrzymania działania


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
#define MQ2_THRESHOLD 2000

//#define MQTT_BROKER_URI "mqtt://192.168.144.219:1883"
#define MQTT_BROKER_URI "mqtt://192.168.55.90:1883"
#define MQTT_USER NULL
#define MQTT_PASSWORD NULL // Replace with actual password if any
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
        gpio_set_level(LED_PIN, 1);  // Włącz LED
        play_melody();  // Odtwarzanie melodii

        gpio_set_level(LED_PIN, 0);  // Wyłącz LED
        vTaskDelay(pdMS_TO_TICKS(1000));  // Przerwa przed kolejnym cyklem
    }

    ESP_LOGI(TAG, "LED i buzzer zostały wyłączone na stałe.");
    vTaskDelete(NULL);
}


// Funkcja sprawdzająca czujniki
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

    while (1) {
        uint16_t gas_value = mq2_read();
        if (bmp280_read_temperature(&bmp280_config, &temperature) != ESP_OK) {
            ESP_LOGE(TAG, "Błąd odczytu temperatury");
            temperature = -1;
        }

        ESP_LOGI(TAG, "Temperatura: %.2f °C, Poziom gazu: %d", temperature, gas_value);

        // Sprawdzenie warunków alarmowych
        if (temperature > TEMP_THRESHOLD || gas_value > MQ2_THRESHOLD) {
            if (!stop_signal) {
                ESP_LOGW(TAG, "PRZEKROCZONO PROGI! AKTYWACJA ALARMU!");
                stop_signal = false;
                //xTaskCreate(led_buzzer_task, "led_buzzer_task", 4096, NULL, 5, NULL);
                //mqtt_manager_send("sensor/alarm", "Przekroczono progi bezpieczeństwa!");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}





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
                vTaskDelete(NULL);  // Zakończenie taska
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


void app_main(void)
{
    //===================================================BMP280==============================================================
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
    

    // ============================================Initialize MQTT Manager===============================================================
    // if (init_mqtt_manager(MQTT_BROKER_URI, MQTT_USER, MQTT_PASSWORD) != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to initialize MQTT Manager");
    //     return;
    // }
    //===================================================================================================================================


    // Konfiguracja pinu jako wyjście
    configure_io_pins();
    // Uruchomienie tasków
    xTaskCreate(check_sensors_task, "check_sensors_task", 4096, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);

    xTaskCreate(led_buzzer_task, "led_buzzer_task", 4096, NULL, 5, NULL);
    // xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    char test_topic[100];
    snprintf(test_topic, sizeof(test_topic), "test");
    
    
    // // Inicjalizacja czujnika MQ-2
    // mq2_init();

    while (1) {

        // uint16_t gas_value = mq2_read();
        
        // if (mq2_detect_gas()) {
        //     ESP_LOGW("APP", "Wykryto niebezpieczny poziom gazu!");
        // } else {
        //     ESP_LOGI("APP", "Poziom gazu w normie.");
        // }

        


        // check_reset_button();
        // Jeśli chcesz sprawdzić parametry co jakiś czas:
        nvs_wifi_config_t cfg;
        if (wifi_manager_get_config(&cfg) == ESP_OK) {
            //ESP_LOGI(TAG, "Current config: SSID=%s, PASS=%s, MQTT=%s",
                      //cfg.wifi_ssid, cfg.wifi_pass, cfg.mqtt_host);
        }



        
        // char topic[MQTT_TOPIC_MAX_LEN] = {0};
        // char message[MQTT_MSG_MAX_LEN] = {0};

        // TickType_t wait_time = pdMS_TO_TICKS(5000);  // Czekaj 5 sekund na wiadomość

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

        // // Sprawdzenie stanu przycisku (0 oznacza naciśnięty przycisk)
        // if (button_state == 0 && previous_button_state == 1) {
        //     ESP_LOGI(TAG, "Przycisk naciśnięty, wyłączam buzzer.");
        //     gpio_set_level(BUZZER_PIN, 0);  // Wyłącz buzzer
        // }



        // previous_button_state = button_state;  // Aktualizacja poprzedniego stanu
        //mqtt_manager_send(test_topic, "TEST");
        vTaskDelay(pdMS_TO_TICKS(3000));  // Oczekiwanie 100 ms dla stabilności
    
    }
         
}
