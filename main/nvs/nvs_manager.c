#include "nvs_manager.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "NVS_MANAGER";
static bool s_nvs_inited = false;

/**
 * Inicjalizacja NVS – wywołaj raz w aplikacji (np. w wifi_manager_init()).
 */
esp_err_t nvs_manager_init(void)
{
    if (!s_nvs_inited) {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init NVS: %s", esp_err_to_name(ret));
            return ret;
        }
        s_nvs_inited = true;
        ESP_LOGI(TAG, "NVS initialized");
    }
    return ESP_OK;
}

/**
 * Odczyt konfiguracji (SSID, PASS, MQTT) z NVS.
 */
esp_err_t nvs_manager_get_wifi_config(nvs_wifi_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (!s_nvs_inited) {
        ESP_LOGW(TAG, "NVS not inited, call nvs_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        // Jeśli brak kluczy w NVS, nvs_open może zwrócić ERR_NVS_NOT_FOUND.
        // Możemy to logować, ale nie jest to błąd krytyczny.
        ESP_LOGW(TAG, "No wifi_config in NVS or can't open it (%s)", esp_err_to_name(err));
        return err;
    }

    size_t ssid_len = sizeof(cfg->wifi_ssid);
    size_t pass_len = sizeof(cfg->wifi_pass);
    size_t mqtt_len = sizeof(cfg->mqtt_host);

    err = nvs_get_str(handle, "wifi_ssid", cfg->wifi_ssid, &ssid_len);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }
    err = nvs_get_str(handle, "wifi_pass", cfg->wifi_pass, &pass_len);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }
    err = nvs_get_str(handle, "mqtt_host", cfg->mqtt_host, &mqtt_len);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    nvs_close(handle);

    ESP_LOGI(TAG, "Loaded config from NVS: SSID=%s, PASS=%s, MQTT=%s",
             cfg->wifi_ssid, cfg->wifi_pass, cfg->mqtt_host);

    return ESP_OK;
}

/**
 * Zapis konfiguracji do NVS.
 */
esp_err_t nvs_manager_set_wifi_config(const nvs_wifi_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (!s_nvs_inited) {
        ESP_LOGW(TAG, "NVS not inited, call nvs_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open wifi_config namespace: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(handle, "wifi_ssid", cfg->wifi_ssid);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    err = nvs_set_str(handle, "wifi_pass", cfg->wifi_pass);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    err = nvs_set_str(handle, "mqtt_host", cfg->mqtt_host);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved Wi-Fi config to NVS");
    } else {
        ESP_LOGE(TAG, "Error while committing Wi-Fi config: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
    return err;
}

/**
 * Czyszczenie konfiguracji Wi-Fi z NVS.
 */
esp_err_t nvs_manager_clear_wifi_config(void)
{
    if (!s_nvs_inited) {
        ESP_LOGW(TAG, "NVS not inited, call nvs_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open wifi_config namespace: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_erase_all(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase wifi_config namespace: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }
    nvs_close(handle);

    ESP_LOGI(TAG, "Cleared Wi-Fi config from NVS");
    return ESP_OK;
}
