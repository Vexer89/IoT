#include "bmp280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Definicje rejestrów BMP280
#define BMP280_REG_CHIPID 0xD0         // Rejestr identyfikatora układu
#define BMP280_REG_CTRL_MEAS 0xF4      // Rejestr konfiguracji pomiaru
#define BMP280_REG_CONFIG 0xF5         // Rejestr konfiguracji (czas oczekiwania, filtracja)
#define BMP280_REG_PRESS_MSB 0xF7      // Rejestr zawierający starszy bajt ciśnienia
#define BMP280_REG_TEMP_MSB 0xFA       // Rejestr zawierający starszy bajt temperatury
#define BMP280_CALIB_START 0x88        // Adres początkowy danych kalibracyjnych
#define BMP280_CALIB_LENGTH 24         // Długość danych kalibracyjnych
#define BMP280_REG_STATUS 0xF3         // Rejestr statusu czujnika


static const char *TAG = "BMP280"; // Etykieta logów dla modułu BMP280

/**
 * @brief Struktura zawierająca dane kalibracyjne BMP280
 */
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_data_t;

static bmp280_calib_data_t calib_data; // Struktura przechowująca dane kalibracyjne
static int32_t t_fine; // Przechowuje wartość pośrednią obliczeń temperatury

/**
 * @brief Funkcja zapisująca pojedynczy rejestr BMP280 przez I2C.
 * 
 * @param i2c_num Numer portu I2C.
 * @param i2c_addr Adres urządzenia BMP280.
 * @param reg_addr Adres rejestru, do którego zapisujemy dane.
 * @param data Wartość, którą zapisujemy do rejestru.
 * @return 
 *    - ESP_OK: Operacja zakończona sukcesem.
 *    - ESP_FAIL: Błąd komunikacji.
 */
esp_err_t bmp280_write_register(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start transmisji I2C i zapis rejestru
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);  
    i2c_master_write_byte(cmd, reg_addr, true); 
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    // Wysłanie komendy I2C
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


/**
 * @brief Funkcja odczytująca dane kalibracyjne z czujnika BMP280.
 * 
 * @param i2c_port Numer portu I2C.
 * @param i2c_address Adres urządzenia BMP280.
 * @return 
 *    - ESP_OK: Odczyt zakończony sukcesem.
 *    - ESP_FAIL: Błąd podczas odczytu danych.
 */
static esp_err_t bmp280_read_calib_data(i2c_port_t i2c_port, uint8_t i2c_address) {
    uint8_t calib[BMP280_CALIB_LENGTH];
    uint8_t reg = BMP280_CALIB_START;

    // Odczyt danych kalibracyjnych z czujnika
    esp_err_t ret = i2c_master_write_read_device(i2c_port, i2c_address, &reg, 1, calib, BMP280_CALIB_LENGTH, 1000 / portTICK_PERIOD_MS);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return ret;
    }
    // Parsowanie danych kalibracyjnych z bufora
    calib_data.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    calib_data.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    calib_data.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);
    calib_data.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    calib_data.dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    calib_data.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    calib_data.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    calib_data.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    calib_data.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    calib_data.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    calib_data.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    calib_data.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);

    return ESP_OK;
}

/**
 * @brief Inicjalizacja czujnika BMP280.
 * 
 * @param config Wskaźnik na konfigurację czujnika BMP280.
 * @return ESP_OK w przypadku sukcesu, ESP_FAIL w przypadku błędu.
 */
esp_err_t bmp280_init(bmp280_config_t *config) {

    esp_err_t ret = bmp280_read_calib_data(config->i2c_port, config->i2c_address);
    if (ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, "Calibration data loaded successfully");

    uint8_t ctrl_meas = 0;
     
    ctrl_meas |= (config->oversampling_temp << 5);
    ctrl_meas |= (config->oversampling_press << 2);
    ctrl_meas |= (config->mode << 0);

    bmp280_write_register(config->i2c_port, config->i2c_address, BMP280_REG_CTRL_MEAS, ctrl_meas);

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    return ESP_OK;
}

/**
 * @brief Funkcja odczytująca temperaturę z BMP280.
 * 
 * @param config Konfiguracja czujnika.
 * @param temperature Wskaźnik na zmienną przechowującą temperaturę.
 */
esp_err_t bmp280_read_temperature(bmp280_config_t *config, float *temperature) {

    uint8_t data[3];
    uint8_t reg = BMP280_REG_TEMP_MSB;
    esp_err_t ret = i2c_master_write_read_device(config->i2c_port, config->i2c_address, &reg, 1, data, 3, 1000 / portTICK_PERIOD_MS);


    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature data");
        return ret;
    }

    int32_t adc_T = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;

    t_fine = var1 + var2;
    *temperature = (t_fine * 5 + 128) >> 8;
    *temperature /= 100.0;

    return ESP_OK;
}

/**
 * @brief Funkcja odczytująca ciśnienie z czujnika BMP280.
 *
 * @param config Wskaźnik na konfigurację czujnika BMP280.
 * @param pressure Wskaźnik na zmienną, do której zostanie zapisana wartość ciśnienia.
 * @return 
 *    - ESP_OK: Odczyt zakończony sukcesem.
 *    - ESP_FAIL: Błąd podczas odczytu ciśnienia lub błędy matematyczne.
 */
esp_err_t bmp280_read_pressure(bmp280_config_t *config, float *pressure) {

    uint8_t data[3];
    uint8_t reg = BMP280_REG_PRESS_MSB;
    // Odczyt 3 bajtów danych ciśnienia
    esp_err_t ret = i2c_master_write_read_device(config->i2c_port, config->i2c_address, &reg, 1, data, 3, 1000 / portTICK_PERIOD_MS);


    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read pressure data");
        return ret;
    }

    // Przetwarzanie surowych danych ADC
    int32_t adc_P = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));

    // Obliczenia ciśnienia zgodnie z dokumentacją BMP280
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;

    // Zapobieganie dzieleniu przez zero
    if (var1 == 0) {
        ESP_LOGE(TAG, "Division by zero error");
        return ESP_FAIL;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    // Konwersja wyniku do wartości zmiennoprzecinkowej (w Pascalach)
    *pressure = (float)p / 256.0;

    return ESP_OK;
}

/**
 * @brief Funkcja zmieniająca tryb pracy czujnika BMP280.
 *
 * @param config Wskaźnik na konfigurację czujnika BMP280.
 * @param mode Nowy tryb pracy (sleep, forced, normal).
 * @return 
 *    - ESP_OK: Zmiana trybu zakończona sukcesem.
 *    - ESP_FAIL: Błąd komunikacji z czujnikiem.
 */
esp_err_t bmp280_change_mode(bmp280_config_t *config, bmp280_mode_t mode) {
    // Odtworzenie konfiguracji pomiarów (oversampling) oraz nowego trybu pracy
    uint8_t ctrl_meas = 0;
    ctrl_meas |= (config->oversampling_temp << 5);  // Oversampling dla temperatury
    ctrl_meas |= (config->oversampling_press << 2); // Oversampling dla ciśnienia
    ctrl_meas |= (mode << 0);                       // Ustawienie nowego trybu pracy
    
    // Zapis nowego trybu do struktury konfiguracyjnej
    config->mode = mode;

    // Zapis konfiguracji do rejestru CTRL_MEAS
    return bmp280_write_register(config->i2c_port, config->i2c_address, BMP280_REG_CTRL_MEAS, ctrl_meas);
}