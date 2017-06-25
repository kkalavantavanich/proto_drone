/*
 * I2C.h
 *
 *  Created on: May 15, 2017
 *      Author: Kris
 */

#ifndef I2C_H_
#define I2C_H_
#include <stdint.h>
#include <math.h>

// types //

typedef struct {
	int16_t acc_x, acc_y, acc_z;
	double ang_x, ang_y, ang_z;
	int16_t temperature;
	int16_t gyro_x, gyro_y, gyro_z;
} mpu6050_t;

typedef struct {
	long pressure, temperature;
} bmp085_t;

typedef struct {
	int16_t mag_x, mag_y, mag_z;
} hmc5883_t;

namespace i2c {

/* I2C Master Mode Only. Not Multi-Master aware.
 *
 *
 */

/* I2C Status */
#define I2C_ST_BUS_ERROR  0x00
#define I2C_ST_START 	  0x08
#define I2C_ST_REP_START  0x10

#define I2C_MT_ADDR_ACK   0x18 // SLA+W transmitted. ACK received
#define I2C_MT_ADDR_NACK  0x20 // SLA+W transmitted. NACK received
#define I2C_MT_DATA_ACK   0x28 // Data byte transmitted. ACK received
#define I2C_MT_DATA_NACK  0x30 // Data byte transmitted. NACK received

#define I2C_MR_ADDR_ACK   0x40 // SLA+R transmitted. ACK received
#define I2C_MR_ADDR_NACK  0x48 // SLA+R transmitted. NACK received
#define I2C_MR_DATA_ACK   0x50 // Data byte received. ACK returned
#define I2C_MR_DATA_NACK  0x58 // Data byte received. NACK returned

/* I2C Addresses */
#define I2C_ADDR_COMPASS  0x1E //HMC5883
#define I2C_ADDR_EXT_PIN  0x3F //PCF8574
#define I2C_ADDR_EEPROM   0x50 //AT24C256
#define I2C_ADDR_GYRO_ACC 0x68 //MPU6050
#define I2C_ADDR_BARO     0x77 //BMP085

#define I2C_BITRATE 12 // F_CPU = 16MHz, F_SCL = 400kHz, PRESCALER = 1
#define I2C_INTERNAL_PULLUPS_DISABLE PORTC &= ~(1<<PORTC4) & ~(1<<PORTC5)

#define I2C_WRITE 0
#define I2C_READ 1

void _start(uint8_t address, uint8_t mode);
void _stop(void);
void _send_byte(uint8_t data);
uint8_t _read_byte(bool sendAck = false);
int16_t _read_word(bool sendAck = false, bool isLittleEndian = false);

void initialize();
void check_addresses();
void printData(mpu6050_t);
void printData(hmc5883_t);
void printData(bmp085_t);

void _error();
void _error(const char* message);
void _error(const char* message, uint8_t i2c_address);

namespace ext {
	void write(uint8_t data);
}

namespace eeprom {
	void write(uint16_t internal_address, uint8_t data);
	void write(uint16_t internal_address, uint8_t* data, uint8_t length);
	uint8_t read(uint16_t internal_address);
	void read(uint16_t internal_address, uint8_t array[], uint8_t length);
}

// ======= GYRO & ACC ======== //

#define MPU6050_START_REG_ADDR 0x3B
#define DLPF_CFG 0  // (Digital Low Pass Filter)
#define FS_SEL 0x03 // (Default 3 :: Gyro Full Scale +- 2000 deg/s)
#define AFS_SEL 0x02 // (Default 2 :: Accel Scale +- 8g)
#define GYRO_RESOLUTION 0.060976 // (1/16.4 (deg/s)/LSb @FS_SEL=3 | From MPU6050 Datasheet)
#define ACCEL_RESOLUTION 0.00024414 // (1/4096 g/LSb @AFS_SEL=2 | From MPU6050 Datasheet)

namespace ga {
	void _init();
	double convertFromRawAccel(int16_t raw);
	double convertFromRawGyro(int16_t raw);
	void calibrate();
	mpu6050_t _read();
	mpu6050_t read_smooth();
	void _calculate_angle(mpu6050_t &data);
}

// ======== COMPASS ======== //

#define HMC5883_MA 0x03	// Samples Averaged (default 8)
#define HMC5883_DO 0x04 // Data Output Rate (default 15 Hz)
#define HMC5883_MS 0x00 // Measurement Configuration (default normal)
#define HMC5883_GN 0x01 // Gain (default 0x01)
#define COMPASS_RESOLUTION 0.92 // 0.92 mGauss/LSb
#define COMPASS_OVERFLOW -4096 // From Datasheet
#define MAG_DEC_CORRECTION -0.599 //degrees (2017@Bangkok) Magnetic Declination Correction

namespace cmps {
	void _init();
	double convertFromRawMag(int16_t raw);
	hmc5883_t read();
	unsigned int bearingFrom(hmc5883_t mag_data);
}

// ======== BAROMETER ======== //

#define OSS 3 // oversampling_setting
#define PRESSURE_SEA_LEVEL 101325.0 // Pa

namespace baro {
	typedef struct {
		int16_t ac1, ac2, ac3;
		uint16_t ac4, ac5, ac6;
		int16_t b1, b2;
		int16_t mb, mc, md;
	} bmp085_calibration_params;

	void _init();
	bmp085_t read();
	void _update_calibration_param();
	long get_utemp();
	long get_upres();
	long get_temperature(long uTemp);
	long get_pressure(long uTemp, long uPres);
	long get_altitude(long pressure);
}

} /* namespace i2c */

#endif /* I2C_H_ */
