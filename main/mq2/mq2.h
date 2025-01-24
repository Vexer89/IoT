#ifndef MQ2_H
#define MQ2_H

#include <stdint.h>
#include <stdbool.h>

// Definicje pinów do podłączenia czujnika
#define MQ2_ADC_PIN         34  // Pin ADC dla ESP32
#define MQ2_THRESHOLD       2000 // Próg wykrywania gazu

// Funkcja inicjalizująca czujnik MQ2
void mq2_init(void);

// Funkcja do odczytu wartości analogowej z czujnika MQ2
uint16_t mq2_read(void);

// Funkcja sprawdzająca czy poziom gazu przekracza próg
bool mq2_detect_gas(void);

#endif /* MQ2_H */
