/**
 * bmp280.c - Component to work with BMP280
 *
 * (C) 2020 - Timothee Cruse <timothee.cruse@gmail.com>
 * This code is licensed under the MIT License.
 */

#include "bmp280.h"
#include "math.h"

static const char *TAG = "bmp280";

#define BMP280_CHECK_HANDLER() if ( prvI2CHandle == NULL ) { return BMP280_FAIL; }

/*-----------------------------------------------------------*/

static IotI2CHandle_t prvI2CHandle = NULL;
int16_t oversampling, oversampling_t;
uint8_t dig_T1, dig_T2 , dig_T3 , dig_T4 , dig_P1, dig_P2 , dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9; 
double t_fine;

/*-----------------------------------------------------------*/

/**
 * @brief   Retrieve calibration data from BMP280
 * @note    The BMP280 includes factory calibration data stored on the device.
 *          Each device has different numbers, these must be retrieved and
 *          used in the calculations when taking measurements.
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
*/
bmp280_err_t prvReadCalibration( void );

/**
 * @brief   Read a signed integer (two bytes) from a BMP280 register
 * @param   address:    register to start reading (plus subsequent register)
 * @param   value:      external variable to store the signed integer (function modifies value)
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
*/
bmp280_err_t prvReadInt( uint8_t address, int16_t * value );

/**
 * @brief   Read an unsigned integer (two bytes) from a BMP280 register
 * @param   address:    register to start reading (plus subsequent register)
 * @param   value:      external variable to store data (function modifies value)
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
*/
bmp280_err_t prvReadUInt( uint8_t address, uint16_t * value );

/**
 * @brief   Read an array of bytes a BMP280 register onwards
 * @param   value:      external array to hold data. Put starting register in values[0].
 * @param   length:     number of bytes to read
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
*/
bmp280_err_t prvReadBytes( uint8_t * values, uint8_t length );
			
/**
 * @brief   Write a number of bytes to a BMP280 register (and consecutive subsequent registers)
 * @param   value:      external array of data to write. Put starting register in values[0].
 * @param   length:     number of bytes to write
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
*/
bmp280_err_t prvWriteBytes( uint8_t * values, uint8_t length );
		
// get uncalibrated pressure and temperature value.
bmp280_err_t prvGetUncalibratedPressureAndTemperature( double * uP, double * uT );	

int16_t getOversampling( void );
bmp280_err_t setOversampling( int16_t oss );

/*-----------------------------------------------------------*/


bmp280_err_t eBmp280Init( IotI2CHandle_t handle )
{
    prvI2CHandle = NULL;
    return readCalibration();
}

/*-----------------------------------------------------------*/

bmp280_err_t prvReadInt( uint8_t address, int16_t * value )
{
    uint8_t data[2];
    data[0] = address;
    if ( prvReadBytes( data, 2 ) == BMP280_SUCCESS )
    {
        *value = (int16_t)( ( (uint16_t)data[1] << 8 ) | (uint16_t)data[0] );
        IotLogDebug( "Read %i @ %#04X", *value, address );
        return BMP280_SUCCESS;
    }

    IotLogError( "Read of integer (2 bytes) @ %#04X failed.", address );
    return BMP280_FAIL;
}

bmp280_err_t prvReadUInt( uint8_t address, uint16_t * value )
{
    uint8_t data[2];
    data[0] = address;
    if ( prvReadBytes( data, 2 ) == BMP280_SUCCESS )
    {
        *value = (uint16_t)( ( (uint16_t)data[1] << 8 ) | (uint16_t)data[0] );
        IotLogDebug( "Read %u @ %#04X", *value, address );
        return BMP280_SUCCESS;
    }

    IotLogError( "Read of unsigned integer (2 bytes) @ %#04X failed.", address );
    return BMP280_FAIL;
}

bmp280_err_t prvReadBytes( uint8_t * values, uint8_t length )
{
    // Return value of I2C functions.
    bmp280_err_t res = BMP280_FAIL;

    BMP280_CHECK_HANDLER();

    // Register address on I2C slave device.
    uint16_t ioCtlBuffer = BMP280_ADDRESS;
 
    // Number of read/write bytes.
    uint16_t usReadBytes = 0;
    uint16_t usWriteBytes = 0;

    uint8_t address = values[0];

    // Set slave address.
    res = iot_i2c_ioctl( prvI2CHandle, eI2CSetSlaveAddr, &ioCtlBuffer );
    if ( res != IOT_I2C_SUCCESS )
    {
        IotLogError( "Setting the slave address for %#04X on I2C failed.", ioCtlBuffer );
        return BMP280_FAIL;
    }
    // assert(lRetVal == IOT_I2C_SUCCESS);

    // Write the register address as single byte, in a transaction.
    res = iot_i2c_write_sync( prvI2CHandle, values, 1 );
    if ( res == IOT_I2C_SUCCESS )
    {
        // Get the number of written bytes in last transaction.
        res = iot_i2c_ioctl( prvI2CHandle, eI2CGetTxNoOfbytes, &usWriteBytes );
        if ( res != IOT_I2C_SUCCESS || usWriteBytes != 1 ) 
        {
            IotLogError( "Failed to check the number of written bytes %u vs. %u.", usWriteBytes, 1 );
            return BMP280_FAIL;
        }

        // Read length bytes of data to allocated buffer, in a transaction.
        res = iot_i2c_read_sync( prvI2CHandle, values, length );
        if ( res == IOT_I2C_SUCCESS )
        {
            // Get the number of read bytes in last transaction.
            res = iot_i2c_ioctl( prvI2CHandle, eI2CGetRxNoOfbytes, &usReadBytes ); 
            if ( res != IOT_I2C_SUCCESS || usReadBytes != length ) 
            {
                IotLogError( "Failed to check the number of read bytes %u vs. %u.", usReadBytes, length );
                return BMP280_FAIL;
            }

            IotLogDebug( "Read %u bytes @ %#04X", length, address );

            return BMP280_SUCCESS;
        }
    }
    else
    {
        IotLogError( "Writing %#04X on I2C failed.", address );
    }
    
    return BMP280_FAIL; 
}

bmp280_err_t prvWriteBytes( uint8_t * values, uint8_t length )
{
    // Return value of I2C functions.
    bmp280_err_t res = BMP280_FAIL;

    BMP280_CHECK_HANDLER();

    // Register address on I2C slave device.
    uint16_t ioCtlBuffer = BMP280_ADDRESS;
 
    // Number of read/write bytes.
    uint16_t usReadBytes = 0;
    uint16_t usWriteBytes = 0;

    uint8_t address = values[0];

    // Set slave address.
    res = iot_i2c_ioctl( prvI2CHandle, eI2CSetSlaveAddr, &ioCtlBuffer );
    if ( res != IOT_I2C_SUCCESS )
    {
        IotLogError( "Setting the slave address for %#04X on I2C failed.", ioCtlBuffer );
        return BMP280_FAIL;
    }
    // assert(lRetVal == IOT_I2C_SUCCESS);

    // Write the register address as single byte, in a transaction.
    res = iot_i2c_write_sync( prvI2CHandle, values, length );
    if ( res == IOT_I2C_SUCCESS )
    {
        // Get the number of written bytes in last transaction.
        res = iot_i2c_ioctl( prvI2CHandle, eI2CGetTxNoOfbytes, &usWriteBytes );
        if ( res != IOT_I2C_SUCCESS || usWriteBytes != length )
        {
            IotLogError( "Failed to check the number of written bytes %u vs. %u.", usWriteBytes, 1 );
            return BMP280_FAIL;
        }

        IotLogDebug( "Wrote %u bytes @ %#04X", length, address );

        return BMP280_SUCCESS;
    }
    else
    {
        IotLogError( "Writing %u bytes on I2C failed.", length );
    }
    
    return BMP280_FAIL; 
}

/*-----------------------------------------------------------*/

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
        IotLogDebug( "Read calibration values:" );
        IotLogDebug( "dig_T1(%#04X) = %u", BMP280_REGISTER_DIG_T1, dig_T1 );
        IotLogDebug( "dig_T2(%#04X) = %u", BMP280_REGISTER_DIG_T2, dig_T2 );
        IotLogDebug( "dig_T3(%#04X) = %u", BMP280_REGISTER_DIG_T3, dig_T3 );
        IotLogDebug( "dig_P1(%#04X) = %u", BMP280_REGISTER_DIG_P1, dig_P1 );
        IotLogDebug( "dig_P2(%#04X) = %u", BMP280_REGISTER_DIG_P2, dig_P2 );
        IotLogDebug( "dig_P3(%#04X) = %u", BMP280_REGISTER_DIG_P3, dig_P3 );
        IotLogDebug( "dig_P4(%#04X) = %u", BMP280_REGISTER_DIG_P4, dig_P4 );
        IotLogDebug( "dig_P5(%#04X) = %u", BMP280_REGISTER_DIG_P5, dig_P5 );
        IotLogDebug( "dig_P6(%#04X) = %u", BMP280_REGISTER_DIG_P6, dig_P6 );
        IotLogDebug( "dig_P7(%#04X) = %u", BMP280_REGISTER_DIG_P7, dig_P7 );
        IotLogDebug( "dig_P8(%#04X) = %u", BMP280_REGISTER_DIG_P8, dig_P8 );
        IotLogDebug( "dig_P9(%#04X) = %u", BMP280_REGISTER_DIG_P9, dig_P9 );
        return BMP280_SUCCESS;
    }

    IotLogError( "Read of calibration failed.");

    return BMP280_FAIL;
}

/*-----------------------------------------------------------*/

int16_t getOversampling( void )
{
	return oversampling;
}

bmp280_err_t setOversampling( int16_t oss )
{
	oversampling = oss;
	return BMP280_SUCCESS;
}

/*-----------------------------------------------------------*/

/**
 * @brief   Begin a measurement cycle.
 *          Oversampling: 0 to 4, higher numbers are slower, higher-res outputs.
 * @param   delay: delay in ms to wait
 * @return  BMP280_SUCCESS success
 *          BMP280_FAIL errors found
*/
bmp280_err_t startMeasurement( uint8_t * delay )
{
	uint8_t data[2], result;
	
	data[0] = BMP280_REGISTER_CONTROL;

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
bmp280_err_t prvGetUncalibratedPressureAndTemperature( double * uP, double * uT )
{
    uint8_t data[6];
	bmp280_err_t result;
	
	data[0] = BMP280_REGISTER_PRESSUREDATA; // 0xF7 

	result = prvReadBytes( data, 6 );       // 0xF7; xF8, 0xF9, 0xFA, 0xFB, 0xFC
	if ( result == BMP280_SUCCESS )         // good read
	{
		*uP = (double)( data[0] * 4096 + data[1] * 16 + data[2] / 16 );	//20bit UP
		*uT = (double)( data[3] * 4096 + data[4] * 16 + data[5] / 16 );	//20bit UT
	}
    else
    {
        IotLogError( "Getting uncalibrated readings failed.");
    }
    
    IotLogDebug( "uP = %u", *uP );
    IotLogDebug( "uT = %u", *uT );

	return(result);
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
bmp280_err_t getTemperatureAndPressure( double * T, double * P )
{
	double uT, uP;
	bmp280_err_t result = prvGetUncalibratedPressureAndTemperature( &uP, &uT );
	
    if( result == BMP280_SUCCESS )
    {
		// calculate the temperature
		result = calcTemperature( &T, uT );
		
        if( result == BMP280_SUCCESS )
        {
			// calculate the pressure
			result = calcPressure( &P, uP );
			
            if ( result == BMP280_SUCCESS ) return BMP280_SUCCESS;
            else return BMP280_CALC_PRESSURE_ERROR;
		}
        else 
        {
            return BMP280_CALC_TEMPERATURE_ERROR;
        }
	}
	else
    {
        return BMP280_GET_UNCAL_VALUES_ERROR;
    }
	
	return BMP280_OTHER_ERROR;
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
bmp280_err_t calcTemperature( double * T, double uT )
{		
	double var1 = ( uT / 16384.0 - dig_T1 / 1024.0 ) * dig_T2;
	double var2 = ( ( uT / 131072.0 - dig_T1 / 8192.0 ) * ( uT / 131072.0 - dig_T1 / 8192.0 ) ) * dig_T3;
	
    t_fine = var1 + var2;

	*T = ( var1 + var2 ) / 5120.0;

    if ( (*T) > 100 || (*T) < -100 )
    {
        IotLogError( "Calculating temperature failed: T = %lf", *T );
        return BMP280_FAIL;
    }

    IotLogDebug( "Calculated temperature: T = %lf", *T );

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
bmp280_err_t calcPressure( double * P, double uP )
{
	double var1 = ( t_fine / 2.0 ) - 64000.0;
    double var2 = var1 * ( var1 * dig_P6 / 32768.0 );   // not overflow

	var2 = var2 + ( var1 * dig_P5 * 2.0 );              // overflow

	var2 = ( var2 / 4.0 ) + ( ( dig_P4 ) * 65536.0 );

	var1 = ( dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1 ) / 524288.0;
	var1 = ( 1.0 + var1 / 32768.0 ) * dig_P1;

	*P = 1048576.0 - uP;

	*P = ( (*P) - ( var2 / 4096.0 ) ) * 6250.0 / var1;  //overflow

	var1 = dig_P9 * (*P) * (*P) / 2147483648.0; 	    //overflow

	var2 = (*P) * dig_P8 / 32768.0;
	*P = (*P) + ( var1 + var2 + dig_P7 ) / 16.0;
		
	*P = (*P) / 100.0;
	
	if ( (*P) > 1200.0 || (*P) < 800.0 ) 
    {
        IotLogError( "Calculating pressure failed: P = %lf", *P );
        return BMP280_FAIL;
    }

    IotLogDebug( "Calculated pressure: P = %lf", *P );
	
    return BMP280_SUCCESS;
}

/*-----------------------------------------------------------*/

/**
 * @brief   Converts absolute pressure to sea-level pressure.
 * @param   P:  absolute pressure in mbar.
 * @param   A:  current altitude in meters.
 * @return  sea-level pressure in mbar
 */
double sealevel( double P, double A )
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

double altitude( double P, double P0 )
{
    return ( 44330.0 * ( 1 - pow( P / P0 , 1 / 5.255 ) ) );
}

/*-----------------------------------------------------------*/

