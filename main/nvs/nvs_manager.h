#ifndef NVS_MANAGER_H
#define NVS_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"

// #ifdef __cplusplus
// extern "C" {
// #endif

/**
 * Podstawowa struktura przechowująca dane konfiguracyjne Wi-Fi
 * i np. host/do serwera MQTT.  
 * Możesz dodać tu inne parametry, np. port MQTT, user/password itp.
 */
typedef struct {
    char wifi_ssid[32];
    char wifi_pass[64];
    char mqtt_host[64];
} nvs_wifi_config_t;

/**
 * @brief Inicjalizuje NVS (jeśli jeszcze nie jest zainicjalizowane).
 * @return esp_err_t – ESP_OK przy powodzeniu
 */
esp_err_t nvs_manager_init(void);

/**
 * @brief Pobiera konfigurację Wi-Fi (SSID, PASS, MQTT) z NVS.
 * @param[out] cfg Wskaźnik na strukturę, do której zostaną przepisane wartości.
 * @return esp_err_t – ESP_OK przy powodzeniu
 */
esp_err_t nvs_manager_get_wifi_config(nvs_wifi_config_t *cfg);

/**
 * @brief Zapisuje konfigurację Wi-Fi do NVS (SSID, PASS, MQTT).
 * @param[in] cfg Wskaźnik na strukturę z danymi.
 * @return esp_err_t – ESP_OK przy powodzeniu
 */
esp_err_t nvs_manager_set_wifi_config(const nvs_wifi_config_t *cfg);

/**
 * @brief Czyści zapis konfiguracji Wi-Fi w NVS.
 * @return esp_err_t – ESP_OK przy powodzeniu
 */
esp_err_t nvs_manager_clear_wifi_config(void);

// #ifdef __cplusplus
// }
// #endif

#endif // NVS_MANAGER_H
