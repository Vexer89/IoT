#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"
#include "../nvs/nvs_manager.h"  // żeby móc korzystać z nvs_wifi_config_t

// #ifdef __cplusplus
// extern "C" {
// #endif

/**
 * @brief Inicjalizuje Wi-Fi zgodnie z danymi w NVS (jeśli istnieją).
 * 
 * - Jeśli `reset_config == true`, to wymusza tryb AP (kasuje konfigurację w NVS).
 * - Jeśli w NVS jest konfiguracja, startuje w trybie STA.
 * - Jeśli w NVS nie ma konfiguracji, startuje w trybie AP.
 * 
 * @param reset_config Jeżeli true, to kasuje obecną konfigurację i uruchamia AP.
 * @return esp_err_t – ESP_OK przy powodzeniu.
 */
esp_err_t wifi_manager_init(bool reset_config);

/**
 * @brief Zwraca aktualnie załadowaną konfigurację (z RAM).
 */
esp_err_t wifi_manager_get_config(nvs_wifi_config_t *out_cfg);

/**
 * @brief Ustawia i zapisuje nową konfigurację (SSID, pass, MQTT) w NVS,
 *        po czym przełącza urządzenie w tryb STA (lub restartuje).
 * 
 * @param new_cfg Wskaźnik do struktury z danymi konfiguracyjnymi.
 * @return esp_err_t 
 */
esp_err_t wifi_manager_set_config(const nvs_wifi_config_t *new_cfg);

// #ifdef __cplusplus
// }
// #endif

#endif // WIFI_MANAGER_H
