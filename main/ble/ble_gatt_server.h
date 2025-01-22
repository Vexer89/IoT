#ifndef BLE_GATT_SERVER_H
#define BLE_GATT_SERVER_H

#include "esp_ble_gatts_api.h"

// Funkcja inicjalizująca serwer GATT
esp_err_t ble_gatt_server_init(void);

// Funkcja wysyłająca alarm do subskrybentów
esp_err_t ble_gatt_server_send_alarm(const char* alarm_message);

#endif // BLE_GATT_SERVER_H
