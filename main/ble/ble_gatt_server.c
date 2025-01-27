#include "ble_gatt_server.h"
#include "esp_log.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "BLE_ALARM";

// Unikalny UUID usługi dla reklamowania
static uint8_t adv_service_uuid128[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 
    0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00
};

// Konfiguracja danych reklamowych BLE
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Parametry reklamowania BLE
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x100,  // Zwiększony interwał (160 ms)
    .adv_int_max        = 0x200,  // Maksymalny interwał (320 ms)
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Struktura przechowująca informacje o GATT
static struct {
    esp_gatt_if_t gatts_if;
    uint16_t service_handle;
    uint16_t char_alarm_handle;
    uint16_t descr_alarm_notif_handle;
    uint16_t conn_id;
} gl_profile;

static uint8_t notify_enabled = 1;

// Obsługa zdarzeń GAP
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising data set, starting advertising...");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "BLE advertising started successfully.");
        } else {
            ESP_LOGE(TAG, "Failed to start advertising, error: %d", param->adv_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE advertising stopped.");
        break;

    default:
        ESP_LOGI(TAG, "Unhandled GAP event: %d", event);
        break;
    }
}

// Obsługa zdarzeń GATTS
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATTS registered, app_id %d", param->reg.app_id);
        gl_profile.gatts_if = gatts_if;

        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = GATTS_SERVICE_UUID_ALARM }
                }
            }
        };
        esp_ble_gatts_create_service(gatts_if, &service_id, 4);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "Service created with handle %d", param->create.service_handle);
        gl_profile.service_handle = param->create.service_handle;
                
        esp_gatt_char_prop_t char_property = ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ;
        esp_bt_uuid_t char_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = { .uuid16 = GATTS_CHAR_UUID_ALARM }
        };

        esp_ble_gatts_add_char(gl_profile.service_handle, &char_uuid, 
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               char_property, NULL, NULL);

        esp_ble_gatts_start_service(gl_profile.service_handle);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        gl_profile.char_alarm_handle = param->add_char.attr_handle;

        esp_bt_uuid_t descr_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = { .uuid16 = GATTS_DESCR_NOTIF_UUID }
        };

        esp_ble_gatts_add_char_descr(gl_profile.service_handle, 
                                     &descr_uuid, 
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 
                                     NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile.descr_alarm_notif_handle = param->add_char_descr.attr_handle;
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "Client connected, conn_id=%d", param->connect.conn_id);
        gl_profile.conn_id = param->connect.conn_id;
        notify_enabled = 1;
        esp_ble_gap_stop_advertising();
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Client disconnected, restarting advertising...");
        esp_ble_gap_start_advertising(&adv_params);
        notify_enabled = 0;
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == gl_profile.descr_alarm_notif_handle && param->write.len == 2) {
            uint16_t descr_value = param->write.value[0] | (param->write.value[1] << 8);
            notify_enabled = (descr_value == 0x0001);
            ESP_LOGI(TAG, "Notifications %s", notify_enabled ? "enabled" : "disabled");
        }
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;

    default:
        break;
    }
}

// Funkcja wysyłania powiadomienia BLE
void ble_alarm_send(void) {
    if (notify_enabled && gl_profile.conn_id != ESP_GATT_IF_NONE) {
        const char *alarm_message = "ALARM TRIGGERED!";
        esp_ble_gatts_send_indicate(
            gl_profile.gatts_if, 
            gl_profile.conn_id, 
            gl_profile.char_alarm_handle, 
            strlen(alarm_message), 
            (uint8_t *)alarm_message, 
            false 
        );
        ESP_LOGI(TAG, "Alarm notification sent: %s", alarm_message);
    } else {
        ESP_LOGW(TAG, "Notifications are not enabled or client not connected.");
    }
}

// Task do okresowego wysyłania powiadomień BLE
void ble_alarm_task(void *param) {
    while (1) {
        ble_alarm_send();
        vTaskDelay(pdMS_TO_TICKS(5000));  // Powiadomienie co 5 sekund
    }
}

void start_ble_alarm_task(void) {
    xTaskCreate(ble_alarm_task, "ble_alarm_task", 2048, NULL, 5, NULL);
}

void ble_gatt_server_init(void) {
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(0);

    esp_ble_gatt_set_local_mtu(500);
    esp_ble_gap_set_device_name("ESP32_ALARM");
    esp_ble_gap_config_adv_data(&adv_data);
    ESP_LOGI(TAG, "BLE GATT Server initialized.");
}
