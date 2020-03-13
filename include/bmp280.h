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

typedef int32_t bmp280_err_t;

/**
 * @brief   BMP280 FAILURE CODES
 */
#define BMP280_SUCCESS                    ( 0 )     /** Operation completed successfully. */
#define BMP280_FAIL                       ( 1 )     /** Operation failed. */
#define BMP280_CALC_PRESSURE_ERROR        ( 2 )     /** Error calculating pressure. */
#define BMP280_CALC_TEMPERATURE_ERROR     ( 3 )     /** Error calculating temperature. */
#define BMP280_GET_UNCAL_VALUES_ERROR     ( 4 )     /** Error calculating temperature. */
#define BMP280_OTHER_ERROR                ( 5 )     /** Other error. */


/**
 * @brief   I2C ADDRESS/BITS/SETTINGS
 */
#define BMP280_ADDRESS                    ( 0x77 )  /** The default I2C address for the sensor. */
#define BMP280_ADDRESS_ALT                ( 0x76 )  /** Alternative I2C address for the sensor. */
#define BMP280_CHIPID                     ( 0x58 )  /** Default chip ID. */


/**
 * @brief   BMP280 Commands
 */
#define	BMP280_COMMAND_TEMPERATURE        ( 0x2E )
#define	BMP280_COMMAND_PRESSURE0          ( 0x25 )
#define	BMP280_COMMAND_PRESSURE1          ( 0x29 )
#define	BMP280_COMMAND_PRESSURE2          ( 0x2D )
#define	BMP280_COMMAND_PRESSURE3          ( 0x31 )
#define	BMP280_COMMAND_PRESSURE4          ( 0x5D )
#define	BMP280_COMMAND_OVERSAMPLING_MAX   ( 0xF5 )


/**
 * @brief   BMP280 Registers
 */
#define BMP280_REGISTER_DIG_T1            ( 0x88 )
#define BMP280_REGISTER_DIG_T2            ( 0x8A )
#define BMP280_REGISTER_DIG_T3            ( 0x8C )
#define BMP280_REGISTER_DIG_P1            ( 0x8E )
#define BMP280_REGISTER_DIG_P2            ( 0x90 )
#define BMP280_REGISTER_DIG_P3            ( 0x92 )
#define BMP280_REGISTER_DIG_P4            ( 0x94 )
#define BMP280_REGISTER_DIG_P5            ( 0x96 )
#define BMP280_REGISTER_DIG_P6            ( 0x98 )
#define BMP280_REGISTER_DIG_P7            ( 0x9A )
#define BMP280_REGISTER_DIG_P8            ( 0x9C )
#define BMP280_REGISTER_DIG_P9            ( 0x9E )
#define BMP280_REGISTER_CHIPID            ( 0xD0 )
#define BMP280_REGISTER_VERSION           ( 0xD1 )
#define BMP280_REGISTER_SOFTRESET         ( 0xE0 )
#define BMP280_REGISTER_CAL26             ( 0xE1 )  /** R calibration 0xE1-0xF0 */
#define BMP280_REGISTER_STATUS            ( 0xF3 )
#define BMP280_REGISTER_CONTROL           ( 0xF4 )
#define BMP280_REGISTER_CONFIG            ( 0xF5 )
#define BMP280_REGISTER_PRESSUREDATA      ( 0xF7 )  /** 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data. */
#define BMP280_REGISTER_TEMPDATA          ( 0xFA )  /** 0xFA(msb) , 0xFB(lsb) , 0xFC(xlsb) : stores the temperature data. */

/**
 * @brief   Initialize BMP280 Sensor.
 * @note    This assumes that the I2C has already been initialized.
 * @param   handle:   Common-IO I2C Handle
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
 */
bmp280_err_t eBmp280Init( IotI2CHandle_t handle );


/**
 * @brief   Begin a measurement cycle.
 *          Commands BMP280 to start a pressure measurement.
 *          Oversampling: 0 to 4, higher numbers are slower, higher-res outputs.
 * @param   delay: delay in ms to wait
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
 */
bmp280_err_t startMeasurement( uint8_t * delay );

/**
 * @brief   Temperature calculation.
 * @param   T:  stores the temperature value after calculation.
 * @param   uT: the uncalibrated temperature value.
 *
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
 */
bmp280_err_t calcTemperature( double * T, double uT );
  
/**
 * @brief   Pressure calculation.
 * @param   P:  stores the pressure value after calculation.
 * @param   uP: the uncalibrated pressure value.
 *
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
 */
bmp280_err_t calcPressure( double * P, double uP );
  
/**
 * @brief   Converts absolute pressure to sea-level pressure.
 * @param   P:  absolute pressure in mbar.
 * @param   A:  current altitude in meters.
 *
 * @return  sea-level pressure in mbar
 */
double seaLevel( double P, double A );

/**
 * @brief   Convert absolute pressure to altitude (given baseline pressure; sea-level, runway, etc.)
 * @param   P:  absolute pressure in mbar.
 * @param   P0: fixed baseline pressure in mbar.
 *
 * @return  signed altitude in meters
 */
double altitude( double P, double P0 );

/**
 * @brief   Retrieve temperature and pressure.
 * @param   T:  stores the temperature value in degC.
 * @param   P:  stores the pressure value in mBar.
 *
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
 */
bmp280_err_t getTemperatureAndPressure( double * T, double * P );

#ifdef __cplusplus
}
#endif

#endif // _BMP280_H_
