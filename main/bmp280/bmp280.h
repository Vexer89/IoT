#ifndef BMP280_H
#define BMP280_H

#include "driver/i2c.h"
#include "driver/spi_master.h"
#include <stdint.h>

/**
 * @brief Tryby pracy czujnika BMP280
 */
typedef enum {
    BMP280_MODE_SLEEP = 0x00, // Tryb uśpienia (czujnik nie wykonuje pomiarów)
    BMP280_MODE_FORCED = 0x01, // Tryb wymuszony (czujnik wykonuje jeden pomiar i wraca do uśpienia)  
    BMP280_MODE_NORMAL = 0x03 // Tryb normalny (ciągłe pomiary z przerwami zgodnymi z konfiguracją)
} bmp280_mode_t;

/**
 * @brief Wartości przeliczników dla różnych poziomów oversampling
 */
typedef enum {
    OS_X1 = 0x01, // 1x oversampling
    OS_X2 = 0x02, // 2x oversampling
    OS_X4 = 0x03, // 4x oversampling
    OS_X8 = 0x04, // 8x oversampling
    OS_X16 = 0x05 // 16x oversampling
} bmp280_oversampling_t;

/**
 * @brief Struktura konfiguracyjna czujnika BMP280
 */
typedef struct {
    i2c_port_t i2c_port; // Numer portu I2C, do którego podłączony jest czujnik
    uint8_t i2c_address; // Adres I2C czujnika
    bmp280_mode_t mode; // Tryb pracy czujnika
    bmp280_oversampling_t oversampling_temp; // Poziom oversampling dla temperatury
    bmp280_oversampling_t oversampling_press; // Poziom oversampling dla ciśnienia
} bmp280_config_t;

/**
 * @brief Inicjalizuje czujnik BMP280 z podaną konfiguracją.
 * 
 * @param config Wskaźnik na strukturę konfiguracji czujnika `bmp280_config_t`.
 * @return 
 *    - ESP_OK: Sukces inicjalizacji.
 *    - ESP_ERR_INVALID_ARG: Nieprawidłowe argumenty wejściowe.
 *    - ESP_FAIL: Błąd komunikacji z czujnikiem.
 */
esp_err_t bmp280_init(bmp280_config_t *config);

/**
 * @brief Odczytuje temperaturę z czujnika BMP280.
 * 
 * @param config Wskaźnik na strukturę konfiguracji czujnika `bmp280_config_t`.
 * @param temperature Wskaźnik na zmienną typu `float`, do której zostanie zapisana wartość temperatury (w stopniach Celsjusza).
 * @return 
 *    - ESP_OK: Sukces odczytu temperatury.
 *    - ESP_FAIL: Błąd podczas odczytu danych z czujnika.
 */
esp_err_t bmp280_read_temperature(bmp280_config_t *config, float *temperature);

/**
 * @brief Odczytuje ciśnienie atmosferyczne z czujnika BMP280.
 * 
 * @param config Wskaźnik na strukturę konfiguracji czujnika `bmp280_config_t`.
 * @param pressure Wskaźnik na zmienną typu `float`, do której zostanie zapisana wartość ciśnienia (w hPa).
 * @return 
 *    - ESP_OK: Sukces odczytu ciśnienia.
 *    - ESP_FAIL: Błąd podczas odczytu danych z czujnika.
 */
esp_err_t bmp280_read_pressure(bmp280_config_t *config, float *pressure);

/**
 * @brief Zmienia tryb pracy czujnika BMP280.
 * 
 * @param config Wskaźnik na strukturę konfiguracji czujnika `bmp280_config_t`.
 * @param mode Tryb pracy czujnika do ustawienia (`bmp280_mode_t`).
 * @return 
 *    - ESP_OK: Tryb został pomyślnie zmieniony.
 *    - ESP_FAIL: Błąd podczas zmiany trybu pracy.
 */
esp_err_t bmp280_change_mode(bmp280_config_t *config, bmp280_mode_t mode);

#endif // BMP280_H
