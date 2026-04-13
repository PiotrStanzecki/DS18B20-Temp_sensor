#include "ds18b20.h"

#define DS18B20_CMD_SEARCH_ROM        0xF0  // Identify ROM codes of all slave devices 
#define DS18B20_CMD_READ_ROM          0x33  // Read ROM from a single slave device 
#define DS18B20_CMD_MATCH_ROM         0x55  // Address a specific slave device 
#define DS18B20_CMD_SKIP_ROM          0xCC  // Address all devices simultaneously 
#define DS18B20_CMD_ALARM_SEARCH      0xEC  // Identify devices with an active alarm flag 

#define DS18B20_CMD_CONVERT_T         0x44  // Initiate a single temperature conversion 
#define DS18B20_CMD_WRITE_SCRATCHPAD  0x4E  // Write 3 bytes to scratchpad (TH, TL, Config) 
#define DS18B20_CMD_READ_SCRATCHPAD   0xBE  // Read the entire 9-byte scratchpad 
#define DS18B20_CMD_COPY_SCRATCHPAD   0x48  // Copy TH, TL, and Config to EEPROM 
#define DS18B20_CMD_RECALL_E2         0xB8  // Recall TH, TL, and Config from EEPROM 
#define DS18B20_CMD_READ_POWER_SUPPLY 0xB4  

/**
 * @brief  Microsecond delay using the DWT cycle counter.
 * @note   At 80MHz, 1us = 80 cycles.
 * @param  us: Number of microseconds to delay.
 */
static void delay_us(uint32_t us) 
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) 
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

/**
 * @brief  Sends a reset pulse and checks for sensor presence.
 * @details Master pulls bus low for min 480us  
 * then releases and waits for the sensor to pull low (Presence Pulse).
 * @return 1 if sensor detected, 0 if not.
 */
static uint8_t ds18b20_reset(ds18b20_t *sensor) 
{
    uint8_t presence = 0;

    // Master Tx Reset Pulse - Pull low for min 480us
    HAL_GPIO_WritePin(sensor->port, sensor->pin, GPIO_PIN_RESET);
    delay_us(480); 
    
    // Disable interrupts to prevent jitter
    __disable_irq();
    HAL_GPIO_WritePin(sensor->port, sensor->pin, GPIO_PIN_SET);
    
    // DS18B20 waits 15-60us then pulls low
    // We wait 70us to ensure we are inside the 60-240us presence window 
    delay_us(70); 
    
    if (!(HAL_GPIO_ReadPin(sensor->port, sensor->pin))) {
        presence = 1; // Presence pulse detected 
    }
    __enable_irq();
    
    // Wait for reset sequence to complete (min 480us total reset time)
    delay_us(410); 
    
    return presence;
}

/**
 * @brief  Writes a single bit to the 1-Wire bus using time slots
 */
static void ds18b20_write_bit(ds18b20_t *sensor, uint8_t bit) {
    __disable_irq();
    if (bit) {
        // Pull low for <15us , then release
        HAL_GPIO_WritePin(sensor->port, sensor->pin, GPIO_PIN_RESET);
        delay_us(2);
        HAL_GPIO_WritePin(sensor->port, sensor->pin, GPIO_PIN_SET);
        delay_us(60); // Slot must be min 60us 
    } else {
        // Pull low for duration of slot (min 60us) (
        HAL_GPIO_WritePin(sensor->port, sensor->pin, GPIO_PIN_RESET);
        delay_us(60);
        HAL_GPIO_WritePin(sensor->port, sensor->pin, GPIO_PIN_SET);
        delay_us(2); // Recovery time 
    }
    __enable_irq();
}

/**
 * @brief  Reads a single bit from the 1-Wire bus
 */
static uint8_t ds18b20_read_bit(ds18b20_t *sensor) {
    uint8_t bit = 0;
    
    __disable_irq();
    // Master initiates read slot by pulling bus low for min 1us
    HAL_GPIO_WritePin(sensor->port, sensor->pin, GPIO_PIN_RESET);
    delay_us(2);
    
    // Release bus. Output data valid for 15us 
    HAL_GPIO_WritePin(sensor->port, sensor->pin, GPIO_PIN_SET);
    
    // Master must sample the bus within 15us of the start of the slot
    delay_us(8); 
    if (HAL_GPIO_ReadPin(sensor->port, sensor->pin)) {
        bit = 1;
    }
    __enable_irq();
    
    delay_us(50); 
    
    return bit;
}

/**
 * @brief  Writes a byte, LSB first, to the sensor
 */
static void ds18b20_write_byte(ds18b20_t *sensor, uint8_t data) {
    for (int i = 0; i < 8; i++) {
        ds18b20_write_bit(sensor, (data >> i) & 0x01);
    }
}

/**
 * @brief  Reads a byte, LSB first, from the sensor
 */
static uint8_t ds18b20_read_byte(ds18b20_t *sensor) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        if (ds18b20_read_bit(sensor)) {
            data |= (1 << i);
        }
    }
    return data;
}

/**
 * @brief Initializes the sensor struct and the DWT cycle counter.
 */
void ds18b20_init(ds18b20_t *sensor, GPIO_TypeDef *port, uint16_t pin) {
    sensor->port = port;
    sensor->pin = pin;


    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  Tells the sensor to begin measuring temperature.
 * @return 1 if successful, 0 if sensor failed to respond.
 */
uint8_t ds18b20_start_conversion(ds18b20_t *sensor) {
    if (!ds18b20_reset(sensor)) return 0;
    
    // ROM Command - Skip ROM 
    ds18b20_write_byte(sensor, DS18B20_CMD_SKIP_ROM); 
    
    // Function Command - Convert T 
    ds18b20_write_byte(sensor, DS18B20_CMD_CONVERT_T); 
    
    return 1;
}

/**
 * @brief  Reads the temperature data after conversion is done.
 * Must be called at least 750ms after ds18b20_start_conversion.
 * @return float: Temperature in Celsius, or -1000.0f if sensor fails.
 */
float ds18b20_read_temp(ds18b20_t *sensor) {
    if (!ds18b20_reset(sensor)) return -1000.0f;
    
    ds18b20_write_byte(sensor, DS18B20_CMD_SKIP_ROM); // Skip ROM
    ds18b20_write_byte(sensor, DS18B20_CMD_READ_SCRATCHPAD); // Read Scratchpad command
    
    // Byte 0 = Temperature LSB, Byte 1 = Temperature MSB
    uint8_t lsb = ds18b20_read_byte(sensor);
    uint8_t msb = ds18b20_read_byte(sensor);
    
    // Combine to 16-bit number
    int16_t raw_temp = (msb << 8) | lsb;
    
    // 12-bit resolution step is 0.0625 degrees Celsius per bit
    return (float)raw_temp * 0.0625f;
}