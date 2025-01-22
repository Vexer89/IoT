# idf_component_register(SRCS "main.c"
#                     INCLUDE_DIRS ".")

idf_component_register(
    SRCS
        "main.c"
        "bmp280/bmp280.c"
        "nvs/nvs_manager.c"
        "wifi/wifi_manager.c"
    INCLUDE_DIRS
        "."            # katalog 'main/'
        "bmp280"       # katalog z bmp280.h
        "nvs"          # katalog z nvs_manager.h
        "wifi"         # katalog z wifi_manager.h
    REQUIRES

)
