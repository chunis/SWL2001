/*!
 * \file      bmp2_bsp.c
 *
 * \brief     Pressure sensor 
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "stm32l4xx_hal.h"
#include "smtc_hal.h"
#include "bmp2_bsp.h"
#include <math.h>
#include "smtc_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define SEA_LEVEL_PRESSURE	101325.0

/**
 * @brief Get the altitude numbers and calculate average value.
 */
#define ALTITUDE_AVERAGE_NUM 5

/**
 * @brief The maximum altitude restriction 
 */
#define ALTITUDE_MAX_VALUE 3000

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static uint8_t dev_addr;
static struct bmp2_dev dev;
static uint32_t meas_time;


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static BMP2_INTF_RET_TYPE bmp2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t  readStatus = SMTC_HAL_FAILURE;
    
    dev_addr = *(uint8_t*)intf_ptr;
    
    readStatus = hal_i2c_read_buffer( HAL_I2C2_ID, ( dev_addr << 1 | 0x01 ), reg_addr, reg_data, (uint16_t)length );
    if ( readStatus != SMTC_HAL_SUCCESS )
    {
        return BMP2_E_COM_FAIL;
    }
    
    return BMP2_OK;
}

static BMP2_INTF_RET_TYPE bmp2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t  readStatus = SMTC_HAL_FAILURE;
    
    dev_addr = *(uint8_t*)intf_ptr;
    
    readStatus = hal_i2c_write_buffer( HAL_I2C2_ID, ( dev_addr << 1 | 0x00 ), reg_addr, (uint8_t *)reg_data, (uint16_t)length );
    if ( readStatus != SMTC_HAL_SUCCESS )
    {
        return BMP2_E_COM_FAIL;
    }
    
    return BMP2_OK;
}

static void bmp2_delay_us(uint32_t period, void *intf_ptr)
{
    uint32_t time_ms = 0; 
    
    period /= 1000;
    time_ms = smtc_modem_hal_get_time_in_ms( ) + period;
    
    while( smtc_modem_hal_get_time_in_ms( ) < time_ms)
    {        
    }
}

/*!
 * @brief Converts pressure to altitude above sea level (ASL) in meters
 * @param [in] pressure (in Pa)
 */
static double pressure_to_altitude( double pressure )
{
    double sea_level = SEA_LEVEL_PRESSURE;
    return 44330.0 * (1.0 - pow( pressure / sea_level, 0.1903 ));
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
 
int8_t bmp2_init_bsp( void )
{
    int8_t rslt;
    struct bmp2_config conf;
    
    dev_addr = BMP2_I2C_ADDR_PRIM;
    dev.read = bmp2_i2c_read;
    dev.write = bmp2_i2c_write;
    dev.intf = BMP2_I2C_INTF;    
    /* Holds the I2C device addr or SPI chip selection */
    dev.intf_ptr = &dev_addr;
    /* Configure delay in microseconds */
    dev.delay_us = bmp2_delay_us;
    
    rslt = bmp2_init( &dev );
    if (rslt != BMP2_OK)
    {
        return rslt;
    }
    
    bmp2_soft_reset( &dev );
    
    /* Always read the current settings before writing, especially when all the configuration is not modified */
    bmp2_get_config( &conf, &dev );
    
    /* Configuring the over-sampling mode, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP2_FILTER_COEFF_4;
    /* Over-sampling mode is set as high resolution i.e., os_pres = 4x and os_temp = 1x */
    conf.os_mode = BMP2_OS_MODE_STANDARD_RESOLUTION;
    /* Setting the output data rate */
    conf.odr = BMP2_ODR_125_MS;
    conf.os_pres = BMP2_OS_4X;
    conf.os_temp = BMP2_OS_1X;
    rslt = bmp2_set_config( &conf, &dev );
    if (rslt != BMP2_OK)
    {
        return rslt;
    }
    
    rslt = bmp2_set_power_mode( BMP2_POWERMODE_NORMAL, &conf, &dev );
    if (rslt != BMP2_OK)
    {
        return rslt;
    }
    
    rslt = bmp2_compute_meas_time(&meas_time, &conf, &dev);
    if (rslt != BMP2_OK)
    {
        return rslt;
    }
    
    return rslt;
}

void bmp2_get_data( uint16_t *altitude, uint32_t *pressure, uint16_t *temperature )
{
    uint8_t idx = 1;
    struct bmp2_status status;
    struct bmp2_data comp_data;
    double altitude_sum = 0.0, altitude_temp = 0.0;
    double pressure_sum = 0.0, pressure_temp = 0.0;
    double temperature_sum = 0.0, temperature_temp = 0.0;

    while (idx <= ALTITUDE_AVERAGE_NUM)
    {
        bmp2_get_status( &status, &dev );

        if ( status.measuring == BMP2_MEAS_DONE )
        {
            /* Delay between measurements */
            dev.delay_us( meas_time, dev.intf_ptr );

            /* Read compensated data */
            bmp2_get_sensor_data( &comp_data, &dev );

            #ifdef BMP2_64BIT_COMPENSATION
            comp_data.pressure = comp_data.pressure / 256;
            #endif

            #ifdef BMP2_DOUBLE_COMPENSATION
            altitude_temp = pressure_to_altitude( comp_data.pressure );
            altitude_sum += altitude_temp;
            pressure_sum += comp_data.pressure;
            temperature_sum += comp_data.temperature;
            
            #else
            printf("Data[%d]:    Temperature: %ld deg C	Pressure: %lu Pa\n", idx, (long int)comp_data.temperature,
                   (long unsigned int)comp_data.pressure);
            #endif

            idx++;
        }
    }
    altitude_temp = altitude_sum / ALTITUDE_AVERAGE_NUM;
    if ( altitude_temp <= ALTITUDE_MAX_VALUE )
    {
        *altitude = (uint16_t)(altitude_temp * 10);
    }
    
    pressure_temp = pressure_sum / ALTITUDE_AVERAGE_NUM;
    *pressure = (uint32_t)( pressure_temp * 10 );
    
    temperature_temp = temperature_sum / ALTITUDE_AVERAGE_NUM;
    *temperature = (uint16_t)( temperature_temp * 10 );
    
    hal_mcu_trace_print("\n Average Altitude: %.4lf m, Average Pressure: %.4lf Pa, Average Temperature: %.4lf deg C    \n\n",
                           altitude_temp, pressure_temp, temperature_temp );
}

/* --- EOF ------------------------------------------------------------------ */



