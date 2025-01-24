#include "mq2.h"
#include "driver/adc.h"
#include "esp_log.h"

#define TAG "MQ2"

// Inicjalizacja czujnika MQ-2
void mq2_init(void) {
    ESP_LOGI(TAG, "Inicjalizacja czujnika MQ-2");

    // Konfiguracja ADC dla odczytu analogowego
    adc1_config_width(ADC_WIDTH_BIT_12);  // Rozdzielczość 12-bitowa
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // Dopasowanie napięcia 0-3.3V
}

// Funkcja do odczytu wartości analogowej z czujnika MQ2
uint16_t mq2_read(void) {
    int adc_value = adc1_get_raw(ADC1_CHANNEL_6);
    ESP_LOGI(TAG, "MQ2 odczytana wartość: %d", adc_value);
    return (uint16_t)adc_value;
}

// Funkcja sprawdzająca czy poziom gazu przekracza próg
bool mq2_detect_gas(void) {
    uint16_t gas_value = mq2_read();
    if (gas_value > MQ2_THRESHOLD) {
        ESP_LOGW(TAG, "Wykryto przekroczenie poziomu gazu! Wartość: %d", gas_value);
        return true;
    } else {
        ESP_LOGI(TAG, "Poziom gazu w normie. Wartość: %d", gas_value);
        return false;
    }
}
