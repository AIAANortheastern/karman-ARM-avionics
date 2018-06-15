 /**
 * @file ADXL375.c
 *
 * Created: 10/10/2017 8:41 PM
 * Author: Douglas Schonholtz
 *
 * @brief High G Accelerometer Driver
 *
 * Driver for the ADXL375 High G Accelerometer module.
 */ 
#include <stdlib.h>
#include <time.h>
#include <ti/drivers/SPI.h>
#include <string.h>
#include <unistd.h>

#include "debug_printf.h"
#include "sensorDefs.h"
#include "appDefs.h"
#include "ADXL375.h"
 
/* registers */
#define  HIGH_G_ACCELEROMETER_READ_DATA     (0x32) /* Read register address */
#define  HIGH_G_ACCELEROMETER_DATA_RATE     (0x2C) /* Data rate register address */
#define  HIGH_G_ACCELEROMETER_POWER_REG     (0x2D) /* Power mode register address */
#define  HIGH_G_ACCELEROMETER_OFFSET_REG    (0x1E) /* Offset register address */

/* init values */
#define  HIGH_G_ACCELEROMETER_DATA_RATE_100HZ   (0x0A)
#define  HIGH_G_ACCELEROMETER_POWER_MODE        (0x08)

/* high g accel read/write values */
#define  HIGH_G_ACCELEROMETER_READ_BIT          (0x80)		/* Read/Write bit for spi read request */
#define  HIGH_G_ACCELEROMETER_WRITE_BIT         (0x00)		/* Read/Write bit for spi write request */
#define  HIGH_G_ACCELEROMETER_MULTI_READ_BIT    (0x40)      /* Multi read bit for spi read request */

 /* global High G Accelerometer control structure */
 ADXL375_control_t gHighGAccelerometer;

/**
 * @brief Initialize the High G Accelerometer
 *
 * sets default values in global struct, then resets the High G Accelerometer
 * 
 */
void ADXL375_init(SPI_Handle *spi_master) {
	
	/* assign the spi_master to master struct for high g accelerometer */
	gHighGAccelerometer.spi_master = spi_master;
	
	/* call initialize functions to prepare accelerometer (probably set offsets and add self test) */

	/* set data rate to 100 Hz */
	ADXL375_write_reg_init(HIGH_G_ACCELEROMETER_DATA_RATE, HIGH_G_ACCELEROMETER_DATA_RATE_100HZ);

	/* set power mode */
	ADXL375_write_reg_init(HIGH_G_ACCELEROMETER_POWER_REG, HIGH_G_ACCELEROMETER_POWER_MODE);

	// TODO zero offsets

	// don't need reset based on other driver and datasheet
	//ADXL375_reset();

}

/*
 * @brief Blocking write request
 *
 * Writes a single value to the input register in a blocking transfer.
 */
void ADXL375_write_reg_init(uint8_t reg, uint8_t value) {

    bool transferOK = false;
    gHighGAccelerometer.spi_send_buffer[0] = reg;
    gHighGAccelerometer.spi_send_buffer[1] = value;
    gHighGAccelerometer.transaction.count = 1;
    gHighGAccelerometer.send_complete = false;

    transferOK = sensor_spi_transfer(&(gHighGAccelerometer.transaction), HIGHG_CS);

    if(!transferOK)
    {
        while(1); /* Failed spi transfer in init */
    }

    while(!gHighGAccelerometer.send_complete)
    {
        usleep(50);
    }

}

/*
 * @brief Reads multiple registers
 *
 * Blocking multi-read transfer that can be used in init
 */
void ADXL375_read_reg_init(uint8_t reg, uint8_t readLen) {

    bool transferOK = false;
    gHighGAccelerometer.spi_send_buffer[0] = reg | HIGH_G_ACCELEROMETER_READ_BIT | HIGH_G_ACCELEROMETER_MULTI_READ_BIT;
    gHighGAccelerometer.transaction.count = readLen + 1;
    gHighGAccelerometer.send_complete = false;

    memset((void *)gHighGAccelerometer.spi_recv_buffer, 0, sizeof(gHighGAccelerometer.spi_recv_buffer));

    transferOK = sensor_spi_transfer(&(gHighGAccelerometer.transaction), HIGHG_CS);

    if(!transferOK)
    {
        while(1); /* Failed spi transfer in init */
    }

    while(!gHighGAccelerometer.send_complete)
    {
        usleep(50);
    }

}

/*
 * @brief Writes a value to a register
 *
 * Writes to a single register in a non-blocking transfer
 */
bool ADXL375_write_reg(uint8_t reg, uint8_t value) {

    gHighGAccelerometer.spi_send_buffer[0] = reg;
    gHighGAccelerometer.spi_send_buffer[1] = value;
    gHighGAccelerometer.transaction.count = 2;
    gHighGAccelerometer.send_complete = false;

    return sensor_spi_transfer(&(gHighGAccelerometer.transaction), HIGHG_CS);

}

/*
 * @ brief Reads a singular register
 *
 * Reads in a singular register value. It is non-blocking
 */
bool ADXL375_read_reg(uint8_t reg) {

    gHighGAccelerometer.spi_send_buffer[0] = reg | HIGH_G_ACCELEROMETER_READ_BIT;
    gHighGAccelerometer.transaction.count = 1; // maybe 2?
    gHighGAccelerometer.send_complete = false;

    memset((void *)gHighGAccelerometer.spi_recv_buffer, 0, sizeof(gHighGAccelerometer.spi_recv_buffer));

    return sensor_spi_transfer(&(gHighGAccelerometer.transaction), HIGHG_CS);
}

/*
 * @brief Read multiple registers
 *
 * This function is used to read multiple registers in one go. It is non-blocking, so it should be used
 * during normal operation
 *
 */
bool ADXL375_read_multiple_reg(uint8_t reg, uint8_t readLen) {

    gHighGAccelerometer.spi_send_buffer[0] = reg | HIGH_G_ACCELEROMETER_READ_BIT | HIGH_G_ACCELEROMETER_MULTI_READ_BIT;
    gHighGAccelerometer.transaction.count = readLen + 1;
    gHighGAccelerometer.send_complete = false;

    memset((void *)gHighGAccelerometer.spi_recv_buffer, 0, sizeof(gHighGAccelerometer.spi_recv_buffer));

    return sensor_spi_transfer(&(gHighGAccelerometer.transaction), HIGHG_CS);

}

/**
 * @brief State machine for High G Accelerometer
 * 
 * This function is called repeatedly from SensorTask.c in tasks
 * When this is called it does its current states job and increments
 * this objects state to the next value.
 * Eventually this will return successful and the data will be pulled out of the
 * appropriate global buffers.
 */
sensor_status_t ADXL375_run() {

	/** 
	 *	1. Enqueue read request
	 *	2. read and convert data
	 */

	sensor_status_t return_status;

	return_status = SENSOR_BUSY;

	switch(gHighGAccelerometer.get_data_state) {
		
		case ENQUEUE:
		    if(ADXL375_read_multiple_reg(HIGH_G_ACCELEROMETER_READ_DATA,6)){
                gHighGAccelerometer.get_data_state = XYZ_DATA_CONVERT;
                return_status = SENSOR_WAITING;
                break;
		    }
		case XYZ_DATA_CONVERT:		
			
            xyz_read_helper();
            convert_helper();

            debug_printf("x: %f, y: %f, z: %f\n", gHighGAccelerometer.final_vals.x, gHighGAccelerometer.final_vals.y, gHighGAccelerometer.final_vals.z );

            gHighGAccelerometer.get_data_state = ENQUEUE;
            return_status = SENSOR_COMPLETE;
			break;
	}
	
	return return_status;					   

}
/*
void ADXL375_get_data(ADXL375_data_t *out_data){

    out_data->x = gHighGAccelerometer.final_vals.x;
    out_data->y = gHighGAccelerometer.final_vals.y;
    out_data->z = gHighGAccelerometer.final_vals.z;

}
*/
/** 
 * @brief gets offset values from High G Accelerometer
 * 
 * reads offsets, possibly sets them? 
 * we're not really sure what to do with these yet
 */
void ADXL375_get_offset_values() {
	
    /* read offsets from regs */
    if(ADXL375_read_multiple_reg(HIGH_G_ACCELEROMETER_OFFSET_REG, 3)){

        /* store offsets in global struct */
        gHighGAccelerometer.offset_vals.x = gHighGAccelerometer.spi_recv_buffer[1];
        gHighGAccelerometer.offset_vals.y = gHighGAccelerometer.spi_recv_buffer[2];
        gHighGAccelerometer.offset_vals.z = gHighGAccelerometer.spi_recv_buffer[3];

    }

}

/** 
 * @brief reset function for High G Accelerometer
 * 
 * sends reset request to accelerometer. I'm not sure if it has one since it autosleeps
 */
void ADXL375_reset() {
	


}



/** 
 * @brief moves x,y, & z raw High G Accelerometer data from buffer to control struct
 *
 * reads each byte of raw data into msb or lsb of global control struct raw x, y, or z data
 */
void xyz_read_helper() {
	/* has data  gHighGAccelerometer.spi_recv_buffer */
	/* we want to put data into */
	gHighGAccelerometer.raw_vals.dig_x = (uint16_t)(((uint16_t)gHighGAccelerometer.spi_recv_buffer[2] << 8) | ((uint16_t)gHighGAccelerometer.spi_recv_buffer[1]));
	gHighGAccelerometer.raw_vals.dig_y = (uint16_t)(((uint16_t)gHighGAccelerometer.spi_recv_buffer[4] << 8) | ((uint16_t)gHighGAccelerometer.spi_recv_buffer[3]));
	gHighGAccelerometer.raw_vals.dig_z = (uint16_t)(((uint16_t)gHighGAccelerometer.spi_recv_buffer[6] << 8) | ((uint16_t)gHighGAccelerometer.spi_recv_buffer[5]));
}

/** 
 * @brief converts raw High G Accelerometer data to final values
 * 
 * multiplies raw data by conversion factor to get actual accelerations
 * current factor is 2, actual is 1.96 fix this when board is updated and floating point is viable
 */
void convert_helper() {
	
	gHighGAccelerometer.final_vals.x = 2*gHighGAccelerometer.raw_vals.dig_x;
	gHighGAccelerometer.final_vals.y = 2*gHighGAccelerometer.raw_vals.dig_y;
	gHighGAccelerometer.final_vals.z = 2*gHighGAccelerometer.raw_vals.dig_z;
	
}


