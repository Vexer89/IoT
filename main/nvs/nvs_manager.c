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
        ESP_LOGW(TAG, "NVS not initialized, call nvs_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No wifi_config in NVS or can't open it (%s)", esp_err_to_name(err));
        return err;
    }

    size_t ssid_len = sizeof(cfg->wifi_ssid);
    size_t pass_len = sizeof(cfg->wifi_pass);
    size_t token_len = sizeof(cfg->pairing_token);

    // Pobieranie SSID
    err = nvs_get_str(handle, "wifi_ssid", cfg->wifi_ssid, &ssid_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SSID from NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    // Pobieranie hasła
    err = nvs_get_str(handle, "wifi_pass", cfg->wifi_pass, &pass_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read password from NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    // Pobieranie tokena
    err = nvs_get_str(handle, "pairing_token", cfg->pairing_token, &token_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read pairing token from NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    nvs_close(handle);

    ESP_LOGI(TAG, "Loaded Wi-Fi config: SSID=%s, PASS=%s, TOKEN=%s",
             cfg->wifi_ssid, cfg->wifi_pass, cfg->pairing_token);

    return ESP_OK;
}


esp_err_t nvs_manager_set_wifi_config(const nvs_wifi_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (!s_nvs_inited) {
        ESP_LOGW(TAG, "NVS not initialized, call nvs_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open NVS: %s", esp_err_to_name(err));
        return err;
    }

    nvs_set_str(handle, "wifi_ssid", cfg->wifi_ssid);
    nvs_set_str(handle, "wifi_pass", cfg->wifi_pass);
    nvs_set_str(handle, "pairing_token", cfg->pairing_token);  // Zapis tokena

    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Wi-Fi configuration saved successfully");
    return ESP_OK;
}

esp_err_t nvs_manager_get_mqtt_config(nvs_mqtt_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (!s_nvs_inited) {
        ESP_LOGW(TAG, "NVS not initialized, call nvs_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("mqtt_config", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No mqtt_config in NVS or can't open it (%s)", esp_err_to_name(err));
        return err;
    }

    size_t broker_len = sizeof(cfg->mqtt_broker);
    size_t topic_len = sizeof(cfg->mqtt_topic);

    err = nvs_get_str(handle, "mqtt_broker", cfg->mqtt_broker, &broker_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MQTT broker from NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_get_str(handle, "mqtt_topic", cfg->mqtt_topic, &topic_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MQTT topic from NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    nvs_close(handle);

    ESP_LOGI(TAG, "Loaded MQTT config: BROKER=%s, TOPIC=%s",
             cfg->mqtt_broker, cfg->mqtt_topic);

    return ESP_OK;
}

/**
 * Zapis konfiguracji MQTT do NVS.
 */
esp_err_t nvs_manager_set_mqtt_config(const nvs_mqtt_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (!s_nvs_inited) {
        ESP_LOGW(TAG, "NVS not initialized, call nvs_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("mqtt_config", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open NVS: %s", esp_err_to_name(err));
        return err;
    }

    nvs_set_str(handle, "mqtt_broker", cfg->mqtt_broker);
    nvs_set_str(handle, "mqtt_topic", cfg->mqtt_topic);

    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "MQTT configuration saved successfully");
    return ESP_OK;
}

esp_err_t nvs_manager_get_thresholds(nvs_threshold_config_t *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (!s_nvs_inited) {
        ESP_LOGW(TAG, "NVS not initialized, call nvs_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("t_conf", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No threshold config in NVS or can't open it (%s)", esp_err_to_name(err));
        return err;
    }

    // Pobranie wartości temperatury i dymu
    err = nvs_get_i32(handle, "temp_td", &cfg->temp_threshold);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temp_threshold from NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_get_i32(handle, "smoke_td", &cfg->smoke_threshold);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read smoke_threshold from NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Loaded thresholds: Temp=%d, Smoke=%d",
             cfg->temp_threshold, cfg->smoke_threshold);
    return ESP_OK;
}

/**
 * @brief Zapisuje konfigurację progów do NVS
 */
esp_err_t nvs_manager_set_thresholds(const nvs_threshold_config_t *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (!s_nvs_inited) {
        ESP_LOGW(TAG, "NVS not initialized, call nvs_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("t_conf", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open threshold_config namespace: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_i32(handle, "temp_td", cfg->temp_threshold);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write temp_threshold to NVS");
        nvs_close(handle);
        return err;
    }

    err = nvs_set_i32(handle, "smoke_td", cfg->smoke_threshold);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write smoke_threshold to NVS");
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Thresholds saved successfully: Temp=%d, Smoke=%d",
                 cfg->temp_threshold, cfg->smoke_threshold);
    } else {
        ESP_LOGE(TAG, "Error while committing thresholds to NVS: %s", esp_err_to_name(err));
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
        ESP_LOGW(TAG, "NVS not initialized, call nvs_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err;
    nvs_handle_t handle;

    // Czyszczenie konfiguracji Wi-Fi
    err = nvs_open("wifi_config", NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_erase_all(handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Cleared Wi-Fi config from NVS");
        } else {
            ESP_LOGE(TAG, "Failed to erase Wi-Fi config: %s", esp_err_to_name(err));
            nvs_close(handle);
            return err;
        }
        nvs_close(handle);
    } else {
        ESP_LOGE(TAG, "Could not open wifi_config namespace: %s", esp_err_to_name(err));
        return err;
    }

    // Czyszczenie konfiguracji MQTT
    err = nvs_open("mqtt_config", NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_erase_all(handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Cleared MQTT config from NVS");
        } else {
            ESP_LOGE(TAG, "Failed to erase MQTT config: %s", esp_err_to_name(err));
            nvs_close(handle);
            return err;
        }
        nvs_close(handle);
    } else {
        ESP_LOGE(TAG, "Could not open mqtt_config namespace: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}


esp_err_t nvs_manager_get_connection_flag(bool *connected)
{
    if (!connected) return ESP_ERR_INVALID_ARG;
    
    if (!s_nvs_inited) {
        ESP_LOGW(TAG, "NVS not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READONLY, &handle);
    if (err == ESP_OK) {
        uint8_t value = 0;
        err = nvs_get_u8(handle, "wifi_connected", &value);
        *connected = (value == 1);
        nvs_close(handle);
    } else {
        *connected = false;  // Default to false if not set
    }

    return err;
}

esp_err_t nvs_manager_set_connection_flag(bool connected)
{
    if (!s_nvs_inited) {
        ESP_LOGW(TAG, "NVS not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        uint8_t value = connected ? 1 : 0;
        err = nvs_set_u8(handle, "wifi_connected", value);
        if (err == ESP_OK) {
            nvs_commit(handle);
        }
        nvs_close(handle);
    }

    return err;
}
    
