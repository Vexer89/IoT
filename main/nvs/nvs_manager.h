#ifndef NVS_MANAGER_H
#define NVS_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Struktura przechowująca dane konfiguracyjne Wi-Fi
 */
typedef struct {
    char wifi_ssid[32];
    char wifi_pass[64];
    char pairing_token[128];
} nvs_wifi_config_t;

/**
 * @brief Struktura przechowująca konfigurację MQTT
 */
typedef struct {
    char mqtt_broker[64];
    char mqtt_topic[64];
} nvs_mqtt_config_t;

typedef struct{
    int temp_threshold;
    int smoke_threshold;
} nvs_threshold_config_t;


/**
 * @brief Inicjalizuje NVS (jeśli jeszcze nie jest zainicjalizowane).
 * @return esp_err_t – ESP_OK przy powodzeniu
 */
esp_err_t nvs_manager_init(void);

/**
 * @brief Pobiera konfigurację Wi-Fi (SSID, PASS, TOKEN) z NVS.
 * @param[out] cfg Wskaźnik na strukturę, do której zostaną przepisane wartości.
 * @return esp_err_t – ESP_OK przy powodzeniu
 */
esp_err_t nvs_manager_get_wifi_config(nvs_wifi_config_t *cfg);

/**
 * @brief Zapisuje konfigurację Wi-Fi do NVS (SSID, PASS, TOKEN).
 * @param[in] cfg Wskaźnik na strukturę z danymi.
 * @return esp_err_t – ESP_OK przy powodzeniu
 */
esp_err_t nvs_manager_set_wifi_config(const nvs_wifi_config_t *cfg);

/**
 * @brief Pobiera konfigurację MQTT (BROKER, TOPIC) z NVS.
 * @param[out] cfg Wskaźnik na strukturę, do której zostaną przepisane wartości.
 * @return esp_err_t – ESP_OK przy powodzeniu
 */
esp_err_t nvs_manager_get_mqtt_config(nvs_mqtt_config_t *cfg);

/**
 * @brief Zapisuje konfigurację MQTT do NVS (BROKER, TOPIC).
 * @param[in] cfg Wskaźnik na strukturę z danymi.
 * @return esp_err_t – ESP_OK przy powodzeniu
 */
esp_err_t nvs_manager_set_mqtt_config(const nvs_mqtt_config_t *cfg);

esp_err_t nvs_manager_get_thresholds(nvs_threshold_config_t *cfg);

esp_err_t nvs_manager_set_thresholds(const nvs_threshold_config_t *cfg);

/**
 * @brief Czyści zapis konfiguracji Wi-Fi w NVS.
 * @return esp_err_t – ESP_OK przy powodzeniu
 */
esp_err_t nvs_manager_clear_wifi_config(void);

/**
 * @brief Pobiera flagę parowania z NVS.
 * @param[out] paired Wskaźnik na zmienną przechowującą flagę parowania.
 * @return esp_err_t – ESP_OK przy powodzeniu
 */
esp_err_t nvs_manager_get_connection_flag(bool *paired);

/**
 * @brief Ustawia flagę parowania w NVS.
 * @param[in] paired Wartość flagi parowania.
 * @return esp_err_t – ESP_OK przy powodzeniu
 */
esp_err_t nvs_manager_set_connection_flag(bool paired);

#endif // NVS_MANAGER_H
