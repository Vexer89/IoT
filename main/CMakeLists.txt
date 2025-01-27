# idf_component_register(SRCS "main.c"
#                     INCLUDE_DIRS ".")

idf_component_register(
    SRCS
        "main.c"
        "bmp280/bmp280.c"
        "nvs/nvs_manager.c"
        "wifi/wifi_manager.c"
        "mqtt/mqtt_manager.c"
        "mq2/mq2.c"
        "ble/ble_gatt_server.c"
    INCLUDE_DIRS
        "."            # katalog 'main/'
        "bmp280"       # katalog z bmp280.h
        "nvs"          # katalog z nvs_manager.h
        "wifi"         # katalog z wifi_manager.h
        "mqtt"         # katalog z mqtt_manager.h
        "mq2"          # katalog z mq2.h
        "ble"          # katalog z ble_gatt_server.h

)

