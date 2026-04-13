//Library for interfacing with DS18B20 1-Wire temperature sensor
/*P.S.*/

#ifndef DS18B20_H
#define DS18B20_H

#include "stm32l4xx_hal.h"


typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} ds18b20_t;


void ds18b20_init(ds18b20_t *sensor, GPIO_TypeDef *port, uint16_t pin);
uint8_t ds18b20_start_conversion(ds18b20_t *sensor);
float ds18b20_read_temp(ds18b20_t *sensor);

#endif /* DS18B20_H */