#include "ble_gatt_server.h"
#include "esp_log.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"

#define GATTS_TAG "BLE_GATT_SERVER"

// UUIDy dla usługi i charakterystyk
#define SERVICE_UUID        0x00FF
#define CHAR_UUID_STATS     0xFF01
#define CHAR_UUID_ALARM     0xFF02

// Deklaracje funkcji
static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param);

// Profil GATT
static struct gatts_profile_inst {
    esp_gatt_if_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    // Dodaj inne potrzebne pola
} gl_profile_tab;

// Rejestracja aplikacji GATT
#define PROFILE_APP_ID 0

// Definicja tabeli serwisów i charakterystyk
static const esp_gatts_attr_db_t gatt_db[] = {
    // Usługa
    [0] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(SERVICE_UUID), (uint8_t*)&SERVICE_UUID}},
    
    // Charakterystyka statystyk
    [1] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read_notify}},
    [2] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&CHAR_UUID_STATS, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(uint8_t), NULL}},
    
    // Charakterystyka alarmów
    [3] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_notify}},
    [4] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&CHAR_UUID_ALARM, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(uint8_t), NULL}},
};

// Inicjalizacja serwera GATT
esp_err_t ble_gatt_server_init(void) {
    esp_err_t ret;

    // Inicjalizacja NVS
    esp_err_t ret = nvs_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE("BLE_GATT_SERVER", "Failed to initialize NVS manager: %s", esp_err_to_name(ret));
        return ret;
}

    // Inicjalizacja BLE
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Rejestracja callbacku GATT
    ret = esp_ble_gatts_register_callback(gatts_profile_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "esp_ble_gatts_register_callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Rejestracja aplikacji GATT
    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "esp_ble_gatts_app_register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

// Funkcja wysyłająca alarm do subskrybentów
esp_err_t ble_gatt_server_send_alarm(const char* alarm_message) {
    if (gl_profile_tab.conn_id == 0) {
        ESP_LOGW(GATTS_TAG, "Brak aktywnego połączenia");
        return ESP_FAIL;
    }

    esp_gatt_notify_param_t notify_param;
    memset(&notify_param, 0, sizeof(notify_param));

    notify_param.conn_id = gl_profile_tab.conn_id;
    notify_param.handle = gl_profile_tab.service_handle + 4; // Handle charakterystyki alarmów
    notify_param.len = strlen(alarm_message);
    notify_param.value = (uint8_t*)alarm_message;
    notify_param.type = ESP_GATT_NOTIFICATION;

    return esp_ble_gatts_send_indicate(gl_profile_tab.gatts_if, notify_param.conn_id,
                                      notify_param.handle, notify_param.len, notify_param.value, false);
}

// Obsługa zdarzeń GATT
static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TAG, "Rejestracja aplikacji, ID %d", param->reg.app_id);
            gl_profile_tab.service_id.is_primary = true;
            gl_profile_tab.service_id.id.inst_id = 0;
            gl_profile_tab.service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab.service_id.id.uuid.uuid.uuid16 = SERVICE_UUID;

            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab.service_id, 4);
            break;

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(GATTS_TAG, "Tworzenie serwisu, handle=%d", param->create.service_handle);
            gl_profile_tab.service_handle = param->create.service_handle;
            esp_ble_gatts_start_service(gl_profile_tab.service_handle);
            break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TAG, "Serwis uruchomiony, handle=%d", param->start.service_handle);
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "Połączono z urządzeniem, ID połączenia=%d", param->connect.conn_id);
            gl_profile_tab.conn_id = param->connect.conn_id;
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "Rozłączono z urządzeniem, ID połączenia=%d", param->disconnect.conn_id);
            gl_profile_tab.conn_id = 0;
            break;

        case ESP_GATTS_WRITE_EVT:
            // Obsługa zapisu, jeśli potrzebna
            break;

        default:
            break;
    }
}
