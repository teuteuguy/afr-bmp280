/**
 * @file bmp280.c
 * @brief Component to work with BMP280
 *
 * (C) 2020 - Timothee Cruse <timothee.cruse@gmail.com>
 * This code is licensed under the MIT License.
 */

#include "bmp280.h"
#include "math.h"

#include "common_io_helpers.h"

#define LIBRARY_LOG_LEVEL IOT_LOG_INFO
#define LIBRARY_LOG_NAME  "bmp280"
#include "iot_logging_setup.h"

/*-----------------------------------------------------------*/

static IotI2CHandle_t prvBmp280I2CHandle = NULL;

int16_t oversampling, oversampling_t;
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9; 
int32_t t_fine;

/*-----------------------------------------------------------*/

bmp280_err_t prvReadCalibration( void );
bmp280_err_t prvReadInt( uint8_t address, int16_t * value );
bmp280_err_t prvReadUInt( uint8_t address, uint16_t * value );
bmp280_err_t prvWriteBytes( uint8_t * values, uint8_t length );
bmp280_err_t prvGetUncalibratedPressureAndTemperature( int32_t * uP, int32_t * uT );	

/*-----------------------------------------------------------*/

bmp280_err_t eBmp280Init( IotI2CHandle_t const handle )
{
    prvBmp280I2CHandle = handle;

    uint8_t data = BMP280_REGISTER_CHIPID;
    if ( eReadI2CBytes( prvBmp280I2CHandle, BMP280_ADDRESS_ALT, &data, 1 ) != COMMON_IO_SUCCESS )
    {
        IotLogError( "eBmp280Init: Failed to read the CHIP ID @ %#04x", BMP280_REGISTER_CHIPID );
        return BMP280_FAIL;
    }
    if ( data != BMP280_CHIPID )
    {
        IotLogError( "eBmp280Init: CHIP ID is incorrect %#04x vs. %#04x", data, BMP280_CHIPID );
        return BMP280_FAIL;
    }
    IotLogDebug( "eBmp280Init: CHIP ID is correct. Soft reseting the chip." );

    if ( eBmp280SoftReset() != BMP280_SUCCESS )
    {
        IotLogError( "eBmp280Init: Failed to Soft Reset the chip" );
        return BMP280_FAIL;
    }

    IotLogDebug( "eBmp280Init: Reading calibration" );

    return prvReadCalibration();
}

/*-----------------------------------------------------------*/

bmp280_err_t eBmp280SoftReset( void )
{
    uint8_t data[ 2 ] = { BMP280_REGISTER_SOFTRESET, 0xB6 };
    if ( eWriteI2CBytes( prvBmp280I2CHandle, BMP280_ADDRESS_ALT, data, 1 ) != COMMON_IO_SUCCESS )
    {
        IotLogError( "eBmp280Init: Failed to soft reset the bmp280" );
        return BMP280_FAIL;
    }

    return BMP280_SUCCESS;
}

/*-----------------------------------------------------------*/

/**
 * @brief   Read a signed integer (two bytes) from a BMP280 register
 * @param   address:    register to start reading (plus subsequent register)
 * @param   value:      external variable to store the signed integer (function modifies value)
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
*/
bmp280_err_t prvReadInt( uint8_t address, int16_t * value )
{
    uint8_t data[2];
    data[0] = address;

    if ( eReadI2CBytes( prvBmp280I2CHandle, BMP280_ADDRESS_ALT, data, 2 ) != COMMON_IO_SUCCESS )
    {
        IotLogError( "prvReadInt: Read of int (2 bytes) @ %#04x failed.", address );
        return BMP280_FAIL;
    }

    *value = (int16_t)( ( (uint16_t)data[ 1 ] << 8 ) | (uint16_t)data[ 0 ] );
    IotLogDebug( "prvReadInt: Read  %i @ %#04x", *value, address );
    return BMP280_SUCCESS;
}

/**
 * @brief   Read an unsigned integer (two bytes) from a BMP280 register
 * @param   address:    register to start reading (plus subsequent register)
 * @param   value:      external variable to store data (function modifies value)
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
*/
bmp280_err_t prvReadUInt( uint8_t address, uint16_t * value )
{
    uint8_t data[2];
    data[0] = address;

    if ( eReadI2CBytes( prvBmp280I2CHandle, BMP280_ADDRESS_ALT, data, 2 ) != COMMON_IO_SUCCESS )
    {
        IotLogError( "prvReadUInt: Read of uint (2 bytes) @ %#04x failed.", address );
        return BMP280_FAIL;
    }

    *value = (uint16_t)( ( (uint16_t)data[ 1 ] << 8 ) | (uint16_t)data[ 0 ] );
    IotLogDebug( "prvReadUInt: Read %u @ %#04x", *value, address );
    return BMP280_SUCCESS;
}

/**
 * @brief   Write a number of bytes to a BMP280 register (and consecutive subsequent registers)
 * @param   value:      external array of data to write. Put starting register in values[0].
 * @param   length:     number of bytes to write
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
*/
bmp280_err_t prvWriteBytes( uint8_t * values, uint8_t length )
{
    if ( eWriteI2CBytes( prvBmp280I2CHandle, BMP280_ADDRESS_ALT, values, length ) == COMMON_IO_SUCCESS )
    {
        return BMP280_SUCCESS;
    }
    IotLogError( "prvWriteBytes: Writing of (%u bytes) @ %#04x failed.", length, values[ 0 ] );
    return BMP280_FAIL;
}

/*-----------------------------------------------------------*/

/**
 * @brief   Retrieve calibration data from BMP280
 * @note    The BMP280 includes factory calibration data stored on the device.
 *          Each device has different numbers, these must be retrieved and
 *          used in the calculations when taking measurements.
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
*/
bmp280_err_t prvReadCalibration( void )
{
    if (
        prvReadUInt( BMP280_REGISTER_DIG_T1, &dig_T1 ) == BMP280_SUCCESS &&
        prvReadInt( BMP280_REGISTER_DIG_T2, &dig_T2 ) == BMP280_SUCCESS &&
        prvReadInt( BMP280_REGISTER_DIG_T3, &dig_T3 ) == BMP280_SUCCESS &&
        prvReadUInt( BMP280_REGISTER_DIG_P1, &dig_P1 ) == BMP280_SUCCESS &&
        prvReadInt( BMP280_REGISTER_DIG_P2, &dig_P2 ) == BMP280_SUCCESS &&
        prvReadInt( BMP280_REGISTER_DIG_P3, &dig_P3 ) == BMP280_SUCCESS &&
        prvReadInt( BMP280_REGISTER_DIG_P4, &dig_P4 ) == BMP280_SUCCESS &&
        prvReadInt( BMP280_REGISTER_DIG_P5, &dig_P5 ) == BMP280_SUCCESS &&
        prvReadInt( BMP280_REGISTER_DIG_P6, &dig_P6 ) == BMP280_SUCCESS &&
        prvReadInt( BMP280_REGISTER_DIG_P7, &dig_P7 ) == BMP280_SUCCESS &&
        prvReadInt( BMP280_REGISTER_DIG_P8, &dig_P8 ) == BMP280_SUCCESS &&
        prvReadInt( BMP280_REGISTER_DIG_P9, &dig_P9 ) == BMP280_SUCCESS
    )
    {
        IotLogDebug( "prvReadCalibration: Read calibration values:" );
        IotLogDebug( "prvReadCalibration: dig_T1(%#04x) = %i", BMP280_REGISTER_DIG_T1, dig_T1 );
        IotLogDebug( "prvReadCalibration: dig_T2(%#04x) = %i", BMP280_REGISTER_DIG_T2, dig_T2 );
        IotLogDebug( "prvReadCalibration: dig_T3(%#04x) = %i", BMP280_REGISTER_DIG_T3, dig_T3 );
        IotLogDebug( "prvReadCalibration: dig_P1(%#04x) = %i", BMP280_REGISTER_DIG_P1, dig_P1 );
        IotLogDebug( "prvReadCalibration: dig_P2(%#04x) = %i", BMP280_REGISTER_DIG_P2, dig_P2 );
        IotLogDebug( "prvReadCalibration: dig_P3(%#04x) = %i", BMP280_REGISTER_DIG_P3, dig_P3 );
        IotLogDebug( "prvReadCalibration: dig_P4(%#04x) = %i", BMP280_REGISTER_DIG_P4, dig_P4 );
        IotLogDebug( "prvReadCalibration: dig_P5(%#04x) = %i", BMP280_REGISTER_DIG_P5, dig_P5 );
        IotLogDebug( "prvReadCalibration: dig_P6(%#04x) = %i", BMP280_REGISTER_DIG_P6, dig_P6 );
        IotLogDebug( "prvReadCalibration: dig_P7(%#04x) = %i", BMP280_REGISTER_DIG_P7, dig_P7 );
        IotLogDebug( "prvReadCalibration: dig_P8(%#04x) = %i", BMP280_REGISTER_DIG_P8, dig_P8 );
        IotLogDebug( "prvReadCalibration: dig_P9(%#04x) = %i", BMP280_REGISTER_DIG_P9, dig_P9 );
        return BMP280_SUCCESS;
    }

    IotLogError( "prvReadCalibration: Read of calibration failed.");

    return BMP280_FAIL;
}

/*-----------------------------------------------------------*/

// int16_t iGetOversampling( void )
// {
// 	return oversampling;
// }

// bmp280_err_t eBmp280SetOversampling( int16_t oss )
// {
// 	oversampling = oss;
// 	return BMP280_SUCCESS;
// }

/*-----------------------------------------------------------*/

/**
 * @brief   Begin a measurement cycle.
 *          Oversampling: 0 to 4, higher numbers are slower, higher-res outputs.
 * @param   delay: delay in ms to wait
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
*/
bmp280_err_t eBmp280StartMeasurement( uint8_t * delay )
{
	uint8_t data[2] = { BMP280_REGISTER_CONTROL, 0 };

	switch ( oversampling )
	{
		case 0:
			data[1] = BMP280_COMMAND_PRESSURE0;     
			oversampling_t = 1;
			*delay = 8;			
		    break;
		case 1:
			data[1] = BMP280_COMMAND_PRESSURE1;     
			oversampling_t = 1;
			*delay = 10;			
		    break;
		case 2:
			data[1] = BMP280_COMMAND_PRESSURE2;		
			oversampling_t = 1;
			*delay = 15;
		    break;
		case 3:
			data[1] = BMP280_COMMAND_PRESSURE3;
			oversampling_t = 1;
			*delay = 24;
		    break;
		case 4:
			data[1] = BMP280_COMMAND_PRESSURE4;
			oversampling_t = 1;
			*delay = 45;
		    break;
		case 16:
			data[1] = BMP280_COMMAND_OVERSAMPLING_MAX;
			oversampling_t = 1;
			*delay = 80;	    // I cannot find any data about timings in datasheet for x16 pressure and x16 temperature oversampling
		    break;	            // I guess this is enough time (maybe it can be even smaller ~60ms)
		default:
			data[1] = BMP280_COMMAND_PRESSURE0;
			*delay = 9;
		    break;
	}

	return prvWriteBytes( data, 2 );
}

/*-----------------------------------------------------------*/

/**
 * @brief   Get the uncalibrated pressure and temperature value.
 * @param   uP: stores the uncalibrated pressure value.(20bit)
 * @param   uT: stores the uncalibrated temperature value.(20bit)
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
*/
bmp280_err_t prvGetUncalibratedPressureAndTemperature( int32_t * uP, int32_t * uT )
{
    uint8_t data[6] = { BMP280_REGISTER_PRESSUREDATA, 0, 0, BMP280_REGISTER_TEMPDATA, 0, 0 };

    if ( eReadI2CBytes( prvBmp280I2CHandle, BMP280_ADDRESS_ALT, data, 3 ) == COMMON_IO_SUCCESS && 
        eReadI2CBytes( prvBmp280I2CHandle, BMP280_ADDRESS_ALT, data + 3, 3 ) == COMMON_IO_SUCCESS
    )
    {
        // *uP = (float)( data[0] * 4096 + data[1] * 16 + data[2] / 16 );	//20bit UP
		// *uT = (float)( data[3] * 4096 + data[4] * 16 + data[5] / 16 );	//20bit UT
        // IotLogDebug( "prvGetUncalibratedPressureAndTemperature: uP = %lf", *uP );
        // IotLogDebug( "prvGetUncalibratedPressureAndTemperature: uT = %lf", *uT );
        *uP = ( data[0] << 12 ) + ( data[1] << 4 ) + ( data[2] >> 4 );	//20bit UP
        *uT = ( data[3] << 12 ) + ( data[4] << 4 ) + ( data[5] >> 4 );	//20bit UT
        IotLogDebug( "prvGetUncalibratedPressureAndTemperature: %#04x, %#04x, %#04x, %#04x, %#04x, %#04x", data[0], data[1], data[2], data[3], data[4], data[5]);
        IotLogDebug( "prvGetUncalibratedPressureAndTemperature: uP = %i", *uP );
        IotLogDebug( "prvGetUncalibratedPressureAndTemperature: uT = %i", *uT );
        return BMP280_SUCCESS;
    }

    IotLogError( "prvGetUncalibratedPressureAndTemperature: Getting uncalibrated readings failed." );
    return BMP280_FAIL;
}

/*-----------------------------------------------------------*/

/**
 * @brief   Retrieve temperature and pressure.
 * @param   T:  stores the temperature value in degC.
 * @param   P:  stores the pressure value in mBar.
 *
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
 */
bmp280_err_t eBmp280GetTemperatureAndPressure( float * T, float * P )
{
	int32_t uT, uP;
	if ( prvGetUncalibratedPressureAndTemperature( &uP, &uT ) == BMP280_SUCCESS )
    {
		// calculate the temperature
        if ( eBmp280CalcTemperature( T, uT ) == BMP280_SUCCESS )
        {

            // calculate the pressure
            if ( eBmp280CalcPressure( P, uP ) == BMP280_SUCCESS )
            {
                return BMP280_SUCCESS;
            }

            return BMP280_CALC_PRESSURE_ERROR;

        }

        return BMP280_CALC_TEMPERATURE_ERROR;

	}

    return BMP280_GET_UNCAL_VALUES_ERROR;
}

/*-----------------------------------------------------------*/

/**
 * @brief   Temperature calculation.
 * @param   T:  stores the temperature value after calculation.
 * @param   uT:  the uncalibrated temperature value.
 *
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
 */
bmp280_err_t eBmp280CalcTemperature( float * T, int32_t uT )
{
	// float var1 = ( uT / 16384.0 - dig_T1 / 1024.0 ) * dig_T2;
	// float var2 = ( ( uT / 131072.0 - dig_T1 / 8192.0 ) * ( uT / 131072.0 - dig_T1 / 8192.0 ) ) * dig_T3;
	
    // t_fine = var1 + var2;

	// *T = ( var1 + var2 ) / 5120.0;

    // if ( (*T) > 100 || (*T) < -100 )
    // {
    //     IotLogError( "eBmp280CalcTemperature: Calculating temperature failed: T = %lf", *T );
    //     return BMP280_FAIL;
    // }

    // IotLogDebug( "eBmp280CalcTemperature: Calculated temperature: T = %lf", *T );


    int32_t var1 = ( ( ((int32_t)uT >> 3) - ((int32_t)dig_T1 << 1) ) * ( (int32_t)dig_T2 ) ) >> 11;
    int32_t var2 = ( ( ( ( ((int32_t)uT >> 4) - ( (int32_t)dig_T1 ) ) * ( ((int32_t)uT >> 4) - ( (int32_t)dig_T1 ) ) ) >> 12 ) * ( (int32_t)dig_T3 ) ) >> 14;

    t_fine = var1 + var2;
    *T = (float)( (int32_t)( ( (var1 + var2) * 5 + 128 ) >> 8 ) ) / 100;

    if ( (*T) > 100 || (*T) < -100 )
    {
        IotLogError( "eBmp280CalcTemperature: Calculating temperature failed: T = %f", *T );
        return BMP280_FAIL;
    }

    IotLogDebug( "eBmp280CalcTemperature: Calculated temperature: T = %f", *T );

    return BMP280_SUCCESS;

}

/*-----------------------------------------------------------*/

/**
 * @brief   Pressure calculation.
 * @note    Dependency on calcTemperature before it, in order to get the t_fine value.
 * @param   P:  stores the pressure value after calculation.
 * @param   uT:  the uncalibrated pressure value.
 *
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
 */
bmp280_err_t eBmp280CalcPressure( float * P, int32_t uP )
{
    int64_t var1, var2, p;

    var1 = ( (int64_t)t_fine ) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ( ( var1 * (int64_t)dig_P5 ) << 17 );
    var2 = var2 + ( ( (int64_t)dig_P4 ) << 35 );
    var1 = ( ( var1 * var1 * (int64_t)dig_P3 ) >> 8 ) + ( ( var1 * (int64_t)dig_P2 ) << 12 );
    var1 = ( ( ( ( (int64_t)1 ) << 47 ) + var1 ) ) * ( (int64_t)dig_P1 ) >> 33;
    if ( var1 == 0 )
    {
        IotLogError( "Calculating pressure failed: Protect against / 0" );
        return BMP280_FAIL; // avoid exception caused by division by zero
    }

    p = 1048576 - uP;

    p = ( ( ( p << 31 ) - var2 ) * 3125 ) / var1;
    var1 = ( ( (int64_t)dig_P9 ) * ( p >> 13 ) * ( p >> 13 ) ) >> 25;
    var2 = ( ( (int64_t)dig_P8 ) * p ) >> 19;
    p = ( ( p + var1 + var2 ) >> 8 ) + ( ( (int64_t)dig_P7 ) << 4 );
    *P = ( (float)p ) / 256 / 100;

    // float var1 = ( t_fine / 2.0 ) - 64000.0;
    // float var2 = var1 * ( var1 * dig_P6 / 32768.0 );   // not overflow

	// var2 = var2 + ( var1 * dig_P5 * 2.0 );              // overflow

	// var2 = ( var2 / 4.0 ) + ( ( dig_P4 ) * 65536.0 );

	// var1 = ( dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1 ) / 524288.0;
	// var1 = ( 1.0 + var1 / 32768.0 ) * dig_P1;

	// *P = 1048576.0 - uP;

	// *P = ( (*P) - ( var2 / 4096.0 ) ) * 6250.0 / var1;  //overflow

	// var1 = dig_P9 * (*P) * (*P) / 2147483648.0; 	    //overflow

	// var2 = (*P) * dig_P8 / 32768.0;
	// *P = (*P) + ( var1 + var2 + dig_P7 ) / 16.0;
		
	// *P = (*P) / 100.0;
	
	// if ( (*P) > 1200.0 || (*P) < 800.0 ) 
    // {
    //     IotLogError( "Calculating pressure failed: P = %f", *P );
    //     return BMP280_FAIL;
    // }

    IotLogDebug( "Calculated pressure: P = %f", *P );
	
    return BMP280_SUCCESS;
}

/*-----------------------------------------------------------*/

/**
 * @brief   Converts absolute pressure to sea-level pressure.
 * @param   P:  absolute pressure in mbar.
 * @param   A:  current altitude in meters.
 * @return  sea-level pressure in mbar
 */
float fBmp280GetSeaLevel( float P, float A )
{
    return ( P / pow( 1 - ( A / 44330.0 ) , 5.255 ) );
}

/*-----------------------------------------------------------*/

/**
 * @brief   Convert absolute pressure to altitude (given baseline pressure; sea-level, runway, etc.)
 * @param   P:  absolute pressure in mbar.
 * @param   P0: fixed baseline pressure in mbar.
 *
 * @return  signed altitude in meters
 */

float fBmp280GetAltitude( float P, float P0 )
{
    return ( 44330.0 * ( 1 - pow( P / P0 , 1 / 5.255 ) ) );
}

/*-----------------------------------------------------------*/

