 /**
 * @file ADXL375.h
 *
 * Created: 10/10/2017 8:41 PM
 * Author: Douglas Schonholtz
 *
 * @brief High G Accelerometer Driver
 *
 * Driver for the ADXL375 High G Accelerometer module.
 */ 

#ifndef ADXL375_H_
#define ADXL375_H_

#include <time.h>
#include <ti/drivers/SPI.h>
#include "sensorDefs.h"

/* holds current SPI input/output */
#define HIGH_G_ACCELEROMETER_BUFF_SIZE (8)

/* raw accelerometer data */
typedef struct high_g_accel_raw_s
{
	uint16_t dig_x;     /**< d1 */
	uint16_t dig_y;      /**< d2 */
	uint16_t dig_z;         /**< dt */
} ADXL375_raw_t;

/* calculated accelerometer data */
typedef struct high_g_accel_data_s
{
	uint16_t x;     /**< d1 */
	uint16_t y;      /**< d2 */
	uint16_t z;         /**< dt */
} ADXL375_data_t;

/* high g accelerometer offset data */
typedef struct high_g_accel_offset_s
{
	// Need to represent fractional values
	// struct to do this is TBD
	uint8_t x;     /**< d1 */
	uint8_t y;      /**< d2 */
	uint8_t z;         /**< dt */
} ADXL375_offset_t;

/* high g accelerometer state machine */
typedef enum
{
	ENQUEUE, /**< Send SPI Request */
	XYZ_DATA_CONVERT        /**< Convert the data and return the final result */
} ADXL375_state_t;

/*  control structure for high g accelerometer */
typedef struct high_g_accel_control_s
{
    SPI_Handle       *spi_master; /**< Pointer to the task's SPI Master */
    volatile uint8_t    spi_send_buffer[HIGH_G_ACCELEROMETER_BUFF_SIZE]; /**< Data to be sent out */
    volatile uint8_t    spi_recv_buffer[HIGH_G_ACCELEROMETER_BUFF_SIZE]; /**< Data received by the device */
    SPI_Transaction     transaction; /**< SPI transaction structure to pass to driver */
    volatile bool       send_complete; /**< Boolean to know when transaction complete */
    ADXL375_offset_t offset_vals; /**< PROM calibration values */
    ADXL375_raw_t raw_vals;         /**< Raw ADC Values */
    ADXL375_data_t final_vals;      /**< Usable values */
    ADXL375_state_t get_data_state; /**< state machine */
    uint32_t            time_start;       /**< For keeping track of time */
} ADXL375_control_t;

void ADXL375_init(SPI_Handle *spi_master);

void ADXL375_write_reg_init(uint8_t reg, uint8_t value);
void ADXL375_read_reg_init(uint8_t reg, uint8_t readLen);

bool ADXL375_write_reg(uint8_t reg, uint8_t value);
bool ADXL375_read_reg(uint8_t reg);
bool ADXL375_read_multiple_reg(uint8_t reg, uint8_t readLen);

void ADXL375_reset(void);

sensor_status_t ADXL375_run(void);

//void ADXL375_get_data(ADXL375_data_t *out_data);

void ADXL375_get_offset_values(void);

void enqueue_helper(void);

void xyz_read_helper(void);

void convert_helper(void);

#endif
