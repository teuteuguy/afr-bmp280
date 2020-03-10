/**
 * bmp280.h - Component to work with BMP280
 *
 * Include this header file to use the component.
 *
 * (C) 2019 - Timothee Cruse <timothee.cruse@gmail.com>
 * This code is licensed under the MIT License.
 */

#ifndef _BMP280_H_
#define _BMP280_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "iot_i2c.h"
#include "esp_err.h"

#define BMP280_SUCCESS                  ( 0 )     /**< Operation completed successfully. */
#define BMP280_FAIL                     ( 1 )     /**< Operation failed. */

/**
 * @brief   I2C ADDRESS/BITS/SETTINGS
 */
#define BMP280_ADDRESS      (0x77)  /**< The default I2C address for the sensor. */
#define BMP280_ADDRESS_ALT  (0x76)  /**< Alternative I2C address for the sensor. */
#define BMP280_CHIPID       (0x58)  /**< Default chip ID. */


/**
 * @brief   BMP280 Registers
 */
#define BMP280_REGISTER_DIG_T1 0x88
#define BMP280_REGISTER_DIG_T2 0x8A
#define BMP280_REGISTER_DIG_T3 0x8C
#define BMP280_REGISTER_DIG_P1 0x8E
#define BMP280_REGISTER_DIG_P2 0x90
#define BMP280_REGISTER_DIG_P3 0x92
#define BMP280_REGISTER_DIG_P4 0x94
#define BMP280_REGISTER_DIG_P5 0x96
#define BMP280_REGISTER_DIG_P6 0x98
#define BMP280_REGISTER_DIG_P7 0x9A
#define BMP280_REGISTER_DIG_P8 0x9C
#define BMP280_REGISTER_DIG_P9 0x9E
#define BMP280_REGISTER_CHIPID 0xD0
#define BMP280_REGISTER_VERSION 0xD1
#define BMP280_REGISTER_SOFTRESET 0xE0
#define BMP280_REGISTER_CAL26 0xE1        /**< R calibration 0xE1-0xF0 */
#define BMP280_REGISTER_STATUS 0xF3
#define BMP280_REGISTER_CONTROL 0xF4
#define BMP280_REGISTER_CONFIG 0xF5
#define BMP280_REGISTER_PRESSUREDATA 0xF7
#define BMP280_REGISTER_TEMPDATA 0xFA


/**
 *  Calibration data structure.
 */
typedef struct {
  uint16_t dig_T1; /**< dig_T1 cal register. */
  int16_t dig_T2;  /**< dig_T2 cal register. */
  int16_t dig_T3;  /**< dig_T3 cal register. */

  uint16_t dig_P1; /**< dig_P1 cal register. */
  int16_t dig_P2;  /**< dig_P2 cal register. */
  int16_t dig_P3;  /**< dig_P3 cal register. */
  int16_t dig_P4;  /**< dig_P4 cal register. */
  int16_t dig_P5;  /**< dig_P5 cal register. */
  int16_t dig_P6;  /**< dig_P6 cal register. */
  int16_t dig_P7;  /**< dig_P7 cal register. */
  int16_t dig_P8;  /**< dig_P8 cal register. */
  int16_t dig_P9;  /**< dig_P9 cal register. */
} bmp280_calib_data_t;


/**
 * @brief   Initialize BMP280 Sensor
 *
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
 */
int eBmp280Init( void );

#ifdef __cplusplus
}
#endif

#endif // _BMP280_H_
