#ifndef BLE_GATT_SERVER_H
#define BLE_GATT_SERVER_H

#include <stdint.h>

#define GATTS_SERVICE_UUID_ALARM   0x01FF
#define GATTS_CHAR_UUID_ALARM      0xFF03
#define GATTS_DESCR_NOTIF_UUID     0x2902

void ble_gatt_server_init(void);
void ble_alarm_send(void);

#endif // BLE_GATT_SERVER_H
