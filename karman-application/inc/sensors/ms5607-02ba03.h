/**
 * @file ms5607-02ba03.h
 *
 * Created: 2/1/2017 7:24:32 PM
 * Author: Andrew Kaster
 *
 * @brief Altimeter Driver
 *
 * Driver for the MS5607-02BA03 temperature/pressure module.
 */

#ifndef MS5607_02BA03_H_
#define MS5607_02BA03_H_

#include "SensorTask.h"
#include <ti/drivers/SPI.h>
#include <time.h>

/** Holds the current SPI input/output */
#define ALTIMETER_SPI_BUFF_SIZE (18)

/** Altimeter calibration data */
typedef struct altimeter_cal_s
{
    uint16_t sens;          /**< pressure sensitivity */
    uint16_t offset;        /**< pressure offset */
    uint16_t tcs;           /**< temperature coefficient of pressure sensitivity */
    uint16_t tco;           /**< temp coefficient of pressure offset */
    uint16_t t_ref;         /**< reference temperature */
    uint16_t temp_sens;     /**< temperature coefficient of temperature */
} __attribute__((packed)) ms5607_02ba03_cal_t;

/** Raw ADC Values */
typedef struct altimeter_raw_s
{
    uint32_t dig_press;     /**< d1 */
    uint32_t dig_temp;      /**< d2 */
    int32_t t_diff;         /**< dt */
} ms5607_02ba03_raw_t;

/** Final computed data */
typedef struct altimeter_data_s
{
    int32_t temp;           /**< Temperature in the form YY.XX deg c */
    int32_t pressure;       /**< Pressure in the form YYYYYY.XX millibar (bizzare right?) */
} ms5607_02ba03_data_t;

/** State machine for altimeter */
typedef enum
{
    ENQUEUE_D1_CONVERT, /**< Take Pressure Measurement */
    WAIT_D1_CONVERT,    /**< Wait for SPI transaction to complete */
    WAIT_8ms_D1,        /**< Wait for measurement to complete */
    WAIT_D1_READ,       /**< Read Pressure ADC value */
    ENQUEUE_D2_CONVERT, /**< Take Temperature Measurement */
    WAIT_D2_CONVERT,    /**< Wait for SPI transaction to complete */
    WAIT_8ms_D2,        /**< Wait for measurement to complete */
    WAIT_D2_READ        /**< Read Temp ADC value and convert both measurements */
} ms5607_02ba03_state_t;

/** Control structure for Altimeter */
typedef struct altimeter_control_s
{
    SPI_Handle        *spi_master; /**< Pointer to the task's SPI Master */
    uint8_t    spi_send_buffer[ALTIMETER_SPI_BUFF_SIZE]; /**< Data to be sent out */
    uint8_t    spi_recv_buffer[ALTIMETER_SPI_BUFF_SIZE]; /**< Data received by the device */
    SPI_Transaction     transaction; /**< SPI transaction structure to pass to driver */
    volatile bool       send_complete; /**< set true when spi callback is called */
    ms5607_02ba03_cal_t calibration_vals; /**< PROM calibration values */
    ms5607_02ba03_raw_t raw_vals;         /**< Raw ADC Values */
    ms5607_02ba03_data_t final_vals;      /**< Usable values */
    ms5607_02ba03_state_t get_data_state; /**< state machine */
    struct timespec       time_start;       /**< For keeping track of time */
} ms5607_02ba03_control_t;


void ms5607_02ba03_init(SPI_Handle *spi_master);

sensor_status_t ms5607_02ba03_run(void);

void ms5607_02ba03_get_data(ms5607_02ba03_data_t *out_data);

void ms5607_02ba03_reset(void);

/* 128 bits of calibration */
void ms5607_02ba03_read_prom(void);

bool ms5607_02ba03_d1_convert(void);

bool ms5607_02ba03_d2_convert(void);

/* 24 bits pressure/temperature */
bool ms5607_02ba03_read_data(void);

void ms5607_02ba03_calculate_temp(void);

void ms5607_02ba03_calculate_press(void);

#endif /* MS5607-02BA03_H_ */
