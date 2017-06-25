/*
 * I2C.cpp
 *
 *  Created on: May 15, 2017
 *      Author: Kris
 */

#include <avr/io.h>
#include <math.h>
#include <util/delay.h>

#include "I2C.h"
#include "USART.h"
#include "Utils.h"

namespace i2c {

/* I2C Private Elementary Functions */

// Don't forget to I2C_initialize()
void _start(uint8_t address, uint8_t mode) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Send Start Condition on Control Register (TWCR)
	while (!(TWCR & (1 << TWINT)))
		; // Wait for start
	if ((TWSR & 0xF8) != I2C_ST_START && (TWSR & 0xF8) != I2C_ST_REP_START)
		_error("I2C_START_ERROR"); // Check Status Register (TWSR)

	TWDR = (address << 1) | mode; // loads address and mode
	TWCR = (1 << TWINT) | (1 << TWEN); // Clear TWINT to start address transmission.
	while (!(TWCR & (1 << TWINT)))
		; // Wait for transmission.

	if ((TWSR & 0xF8) != (mode ? I2C_MR_ADDR_ACK : I2C_MT_ADDR_ACK)) {
		if ((TWSR & 0xF8) == I2C_MR_ADDR_NACK) {
			_error("I2C_MR_ADDRESS_NACK", address);
		} else if ((TWSR & 0xF8) == I2C_MT_ADDR_NACK) {
			_error("I2C_MT_ADDRESS_NACK", address);
		} else
			_error();
	}
}

void _stop() {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

// Sends a byte. Must be called after I2C_start() in Master Transmitter (MT) mode.
void _send_byte(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)))
		;
	if ((TWSR & 0xF8) != I2C_MT_DATA_ACK) {
		if ((TWSR & 0xF8) == I2C_MT_DATA_NACK) {
			_error("I2C_DATA_NACK");
		} else
			_error();
	}
}

// Sends a byte. Must be called after I2C_start() in Master Receiver (MR) mode.
uint8_t _read_byte(bool sendAck) {
	if (sendAck) {
		TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	} else {
		TWCR = (1 << TWINT) | (1 << TWEN);
	}
	while (!(TWCR & (1 << TWINT)))
		;
	if ((TWSR & 0xF8) != I2C_MR_DATA_ACK && (TWSR & 0xF8) != I2C_MR_DATA_NACK) {
		_error();
	}
	return (uint8_t) TWDR;
}

#define LITTLE_ENDIAN true
#define BIG_ENDIAN false

int16_t _read_word(bool sendAck, bool isLittleEndian) {
	uint8_t upper, lower;
	upper = _read_byte(true);
	lower = _read_byte(sendAck);
	if (isLittleEndian) {
		uint8_t temp = upper;
		upper = lower;
		lower = temp;
	}
	int16_t buf = (upper << 8) | lower;
	return buf;
}

/* I2C General Functions */

void initialize() {
	PRR &= ~(1 << PRTWI);
	TWBR = I2C_BITRATE;
	TWSR &= ~(1 << TWPS1) & ~(1 << TWPS0); // set i2c prescaler to 1

	// initialize sensors
	ga::_init();
	cmps::_init();
	baro::_init();
}

void check_addresses() {
	uint8_t buffer;
	println("> Checking Barometer...");
	_start(I2C_ADDR_BARO, I2C_WRITE);
	_stop();

	println("> Checking Compass...");
	_start(I2C_ADDR_COMPASS, I2C_WRITE);
	_send_byte(0x00);
	_start(I2C_ADDR_COMPASS, I2C_READ);
	buffer = _read_byte(true);
	print("MA[1:0] = 0b");
	println((buffer & 0x60) >> 5, 2);
	print("DO[2:0] = 0b");
	println((buffer & 0x1C) >> 2, 2);
	print("MS[1:0] = 0b");
	println(buffer & 0x03, 2);
	buffer = _read_byte(true);
	print("GN[2:0] = 0b");
	println((buffer & 0xe0) >> 5, 2);
	buffer = _read_byte(false);
	print("HS = ");
	println(buffer >> 7, 2);
	print("MD[1:0] = 0b");
	println((buffer & 0x03), 2);
	_stop();

	println("> Checking EEPROM...");
	_start(I2C_ADDR_EEPROM, I2C_WRITE);
	_stop();
	println("> Checking External Pin...");
	_start(I2C_ADDR_EXT_PIN, I2C_WRITE);
	_stop();

	println("> Checking Gyro & Accelerometer...");
	_start(I2C_ADDR_GYRO_ACC, I2C_WRITE);
	_send_byte(0x1A);
	_start(I2C_ADDR_GYRO_ACC, I2C_READ);
	uint8_t dlpf = (_read_byte(true)) & 0x07;
	print("DLPF = 0b");
	println(dlpf, 2);
	uint8_t fs_sel = ((_read_byte(true)) & 0x18) >> 3;
	print("GYRO::FS_SEL = 0b");
	println(fs_sel, 2);
	uint8_t afs_sel = ((_read_byte(true)) & 0x18) >> 3;
	print("ACC::AFS_SEL = 0b");
	println(afs_sel, 2);
	uint8_t fifo = (_read_byte(false));
	print("FIFO_EN = 0b");
	println(fifo, 2);
	_start(I2C_ADDR_GYRO_ACC, I2C_WRITE);
	_send_byte(0x6B);
	_start(I2C_ADDR_GYRO_ACC, I2C_READ);
	print("PWR_MGMT_1 = 0b");
	println(_read_byte(true), 2);
	print("PWR_MGMT_2 = 0b");
	println(_read_byte(false), 2);
	_stop();

	println("Finished checking I2C.");
}

void printData(mpu6050_t data) {
	print("ACC_X = ");
	print(ga::convertFromRawAccel(data.acc_x));
	print("g, ACC_Y = ");
	print(ga::convertFromRawAccel(data.acc_y));
	print("g, ACC_Z = ");
	print(ga::convertFromRawAccel(data.acc_z));
	print("g,\tGYR_X = ");
	print(ga::convertFromRawGyro(data.gyro_x));
	print("deg/s, GYR_Y = ");
	print(ga::convertFromRawGyro(data.gyro_y));
	print("deg/s, GYR_Z = ");
	print(ga::convertFromRawGyro(data.gyro_z));
	print("deg/s,\t ANG_X = ");
	print(ga::convertFromRawGyro(data.ang_x));
	print("deg, ANG_Y = ");
	print(ga::convertFromRawGyro(data.ang_y));
	print("deg, ANG_Z = ");
	print(ga::convertFromRawGyro(data.ang_z));
	print("deg.");
}

void printData(hmc5883_t data) {
	print("MAG_X = ");
	print(cmps::convertFromRawMag(data.mag_x), 0);
	print("mG, MAG_Y = ");
	print(cmps::convertFromRawMag(data.mag_y), 0);
	print("mG, MAG_Z = ");
	print(cmps::convertFromRawMag(data.mag_z), 0);
	print("mG. Bearing: ");
	print(cmps::bearingFrom(data));
}

void printData(bmp085_t data) {
	print("PRES = ");
	print(data.pressure);
	print("Pa, TEMP = ");
	print(data.temperature / 10.0);
	print("C, ALT = ");
	print(baro::get_altitude(data.pressure));
	print("m.");
}

void _error() {
	print("I2C_ERROR::Unspecified Error [TWSR=0x");
	print(TWSR & 0xF8, 16);
	println("]");
}

void _error(const char* message) {
	print("I2C_ERROR::");
	println(message);
}

void _error(const char* message, uint8_t i2c_address) {
	print("I2C_ERROR::");
	print(message);
	print(" [I2C_address=0x");
	print(i2c_address, 16);
	println("]");
}

/* I2C Specific Public Functions */

// ======== EXTERNAL PINS ======== //
namespace ext {
void write(uint8_t data) {
	_start(I2C_ADDR_EXT_PIN, I2C_WRITE);
	print("data = ");
	print(data);
	_send_byte(data);
	_stop();
}
} /* namespace ext */

// ======== EXTERNAL EEPROM ======== //
namespace eeprom {

void write(uint16_t internal_address, uint8_t data) {
	uint8_t upper_address = internal_address >> 8;
	uint8_t lower_address = internal_address & 0xFF;
	_start(I2C_ADDR_EEPROM, I2C_WRITE);
	_send_byte(upper_address);
	_send_byte(lower_address);
	_send_byte(data);
	_stop();
}

// EEPROM Page Write. length must be <= 64. Overwrites the same page
void write(uint16_t internal_address, uint8_t* data, uint8_t length) {
	if (length > 64) {
		_error("EEPROM_PAGE_WRITE::message exceeds 64 bytes.");
		return;
	}
	uint8_t upper_address = internal_address >> 8;
	uint8_t lower_address = internal_address & 0xFF;
	_start(I2C_ADDR_EEPROM, I2C_WRITE);
	_send_byte(upper_address);
	_send_byte(lower_address);
	while (length--) {
		_send_byte(*data++);
	}
	_stop();
}

// EEPROM: Read a byte from given 2-byte address
uint8_t read(uint16_t internal_address) {
	uint8_t buf;
	uint8_t upper_address = internal_address >> 8;
	uint8_t lower_address = internal_address & 0xFF;
	_start(I2C_ADDR_EEPROM, I2C_WRITE);
	_send_byte(upper_address);
	_send_byte(lower_address);
	_start(I2C_ADDR_EEPROM, I2C_READ);
	buf = _read_byte(false);
	_stop();
	return buf;
}

// EEPROM Sequential Read. `length` must be <= 64. Read data is kept into `array[]`.
void read(uint16_t internal_address, uint8_t array[], uint8_t length) {
	uint8_t *ptr = &array[0];
	uint8_t upper_address = internal_address >> 8;
	uint8_t lower_address = internal_address & 0xFF;
	_start(I2C_ADDR_EEPROM, I2C_WRITE);
	_send_byte(upper_address);
	_send_byte(lower_address);
	_start(I2C_ADDR_EEPROM, I2C_READ);
	while (--length) { // omit last loop
		*ptr++ = _read_byte(true);
	}
	*ptr = _read_byte(false);
	_stop();
}

}/* end namespace eeprom */
// ======== GYRO & ACC ======== //

/* == Internal Registers ==
 * 0x3B ACCEL_XOUT[15:8]
 * 0x3C ACCEL_XOUT[7:0]
 * 0x3D ACCEL_YOUT[15:8]
 * 0x3E ACCEL_YOUT[7:0]
 * 0x3F ACCEL_ZOUT[15:8]
 * 0x40 ACCEL_ZOUT[7:0]
 * 0x41 TEMP_OUT[15:8]
 * 0x42 TEMP_OUT[7:0]
 * 0x43 GYRO_XOUT[15:8]
 * 0x44 GYRO_XOUT[7:0]
 * 0x45 GYRO_YOUT[15:8]
 * 0x46 GYRO_YOUT[7:0]
 * 0x47 GYRO_ZOUT[15:8]
 * 0x48 GYRO_ZOUT[7:0]
 */
namespace ga {

int16_t calibration_params[6] = { 0, 0, 0, 0, 0, 0 }; // calibration parameters -- must be added to get correct value(s).
double convertFromRawAccel(int16_t raw) {
	return raw * ACCEL_RESOLUTION;
}

double convertFromRawGyro(int16_t raw) {
	return raw * GYRO_RESOLUTION;
}

void _init() {
	_start(I2C_ADDR_GYRO_ACC, I2C_WRITE);
	_send_byte(0x6B);
	_send_byte(0x80); 						// PWR_MGMT_1 	--DEVICE_RESET
	_stop();
	_delay_ms(50);

	_start(I2C_ADDR_GYRO_ACC, I2C_WRITE);
	_send_byte(0x6B);
	_send_byte(0x03);						// PWR_MGMT_1
	_start(I2C_ADDR_GYRO_ACC, I2C_WRITE);
	_send_byte(0x1A);
	_send_byte(DLPF_CFG);					// CONFIG
	_send_byte(FS_SEL << 3);				// GYRO_CONFIG
	_send_byte(AFS_SEL << 3);				// ACCEL_CONFIG
	_send_byte(0);				// FIFO_EN	 	-- Disable All FIFO by default
	_start(I2C_ADDR_GYRO_ACC, I2C_WRITE);
	_send_byte(0x37);
	_send_byte(0x02);					// INT_PIN_CFG 	-- Enable I2C_BYPASS_EN
	_stop();

	_delay_ms(1);							// wait for warm up
	calibrate();
}

#define I2C_ACC_CALIBRATION_SCALE 8 // How much values to use for calibration (in powers of 2 -- 5 => 32)

// Use to calibrate Accelerometer and Gyroscope. Board must be physically still while calibrating (which should take less than 1 second).
void calibrate() {
	for (int i = 0; i < 6; i++) {
		calibration_params[i] = 0;
	}

	mpu6050_t buf;
	int32_t motion_buffers[6] = { 0, 0, 0, 0, 0, 0 };
	for (int i = 0; i < (1 << (I2C_ACC_CALIBRATION_SCALE + 1)); i++) {
		buf = read();

		// Dismissed first half and use only second half.
		if (i >= (1 << I2C_ACC_CALIBRATION_SCALE)) {
			motion_buffers[0] += buf.acc_x;
			motion_buffers[1] += buf.acc_y;
			motion_buffers[2] += buf.acc_z;
			motion_buffers[3] += buf.gyro_x;
			motion_buffers[4] += buf.gyro_y;
			motion_buffers[5] += buf.gyro_z;
		}
		_delay_us(200);
	}

	for (int i = 0; i < 6; i++) {
		calibration_params[i] =
				-(motion_buffers[i] >> I2C_ACC_CALIBRATION_SCALE);
	}
	calibration_params[2] += 4096; // for acc_z => value must +1.00g
}

mpu6050_t read() {
	mpu6050_t buf;
	_start(I2C_ADDR_GYRO_ACC, I2C_WRITE);
	_send_byte(MPU6050_START_REG_ADDR);
	_start(I2C_ADDR_GYRO_ACC, I2C_READ);

	buf.acc_x = _read_word(true) + calibration_params[0];
	buf.acc_y = _read_word(true) + calibration_params[1];
	buf.acc_z = _read_word(true) + calibration_params[2];
	buf.temperature = _read_word(true);
	buf.gyro_x = _read_word(true) + calibration_params[3];
	buf.gyro_y = _read_word(true) + calibration_params[4];
	buf.gyro_z = _read_word(false) + calibration_params[5];

	_stop();
	_calculate_angle(buf);
	return buf;
}

#define I2C_GA_SMOOTHNESS_SCALE 3 // How much values (smoothness) to use for calculation (in powers of 2 -- 5 => 32).

mpu6050_t read_smooth() {
	mpu6050_t imm;
	int32_t buffers[7] = { 0, 0, 0, 0, 0, 0, 0 };// Buffer used to sum and find average of read values
	for (int i = 0; i < (1 << I2C_GA_SMOOTHNESS_SCALE); i++) {
		imm = read();
		buffers[0] += imm.acc_x;
		buffers[1] += imm.acc_y;
		buffers[2] += imm.acc_z;
		buffers[3] += imm.gyro_x;
		buffers[4] += imm.gyro_y;
		buffers[5] += imm.gyro_z;
		buffers[6] += imm.temperature;
	}

	imm.acc_x = buffers[0] >> I2C_GA_SMOOTHNESS_SCALE;
	imm.acc_y = buffers[1] >> I2C_GA_SMOOTHNESS_SCALE;
	imm.acc_z = buffers[2] >> I2C_GA_SMOOTHNESS_SCALE;
	imm.gyro_x = buffers[3] >> I2C_GA_SMOOTHNESS_SCALE;
	imm.gyro_y = buffers[4] >> I2C_GA_SMOOTHNESS_SCALE;
	imm.gyro_z = buffers[5] >> I2C_GA_SMOOTHNESS_SCALE;
	imm.temperature = buffers[6] >> I2C_GA_SMOOTHNESS_SCALE;
	_calculate_angle(imm);
	return imm;
}

const double PI = (3.14159265358979);
const double RAD_TO_DEG = (180 / PI);
#define ANGLE_MIN_IN -32768
#define ANGLE_MAX_IN 32767
#define ANGLE_MIN_OUT -90
#define ANGLE_MAX_OUT 90

void _calculate_angle(mpu6050_t data) {
	int x_mapped = map(data.acc_x, -32768, 32767, -90, 90);
	int y_mapped = map(data.acc_y, -32768, 32767, -90, 90);
	int z_mapped = map(data.acc_z, -32768, 32767, -90, 90);
	data.ang_x = RAD_TO_DEG * (atan2(-y_mapped, -z_mapped) + PI);
	data.ang_y = RAD_TO_DEG * (atan2(-x_mapped, -z_mapped) + PI);
	data.ang_z = RAD_TO_DEG * (atan2(-y_mapped, -x_mapped) + PI);
}

} /* namespace ga */
// ======== COMPASS ======== //

namespace cmps {
void _init() {
	_start(I2C_ADDR_COMPASS, I2C_WRITE);
	_send_byte(0x00);
	_send_byte((HMC5883_MA << 5) | (HMC5883_DO << 2) | (HMC5883_MS & 0x03)); // CONFIG_1
	_send_byte(HMC5883_GN << 5);									// CONFIG_2
	_send_byte(0x00);													// MODE
	_stop();
}

hmc5883_t read() {
	hmc5883_t buffer;
	_start(I2C_ADDR_COMPASS, I2C_WRITE);
	_send_byte(0x03);
	_start(I2C_ADDR_COMPASS, I2C_READ);

	// Order is X, Z, Y (From Datasheet)
	buffer.mag_x = _read_word(true, BIG_ENDIAN);
	buffer.mag_z = _read_word(true, BIG_ENDIAN);
	buffer.mag_y = _read_word(false, BIG_ENDIAN);
	_stop();

	// Check Overflows
	if (buffer.mag_x == COMPASS_OVERFLOW)
		_error("COMPASS_OVERFLOW::X");
	if (buffer.mag_y == COMPASS_OVERFLOW)
		_error("COMPASS_OVERFLOW::Y");
	if (buffer.mag_z == COMPASS_OVERFLOW)
		_error("COMPASS_OVERFLOW::Z");
	return buffer;
}

double convertFromRawMag(int16_t raw) {
	return raw * COMPASS_RESOLUTION;
}

unsigned int bearingFrom(hmc5883_t mag_data) {
	double heading = atan2(mag_data.mag_y, mag_data.mag_x);
	heading *= 180 / M_PI;
	heading += MAG_DEC_CORRECTION;
	if (heading < 0)
		heading += 360;
	else if (heading > 360)
		heading -= 360;
	return (unsigned int) heading;
}

} /* namespace cmps */
// ======== BAROMETER ======== //

namespace baro {
static bmp085_calibration_params params;

void _init() {
	_update_calibration_param();
}
bmp085_t read() {
	bmp085_t buffer;
	long ut = get_utemp();
	long up = get_upres();
	buffer.temperature = get_temperature(ut);
	buffer.pressure = get_pressure(ut, up);
	return buffer;
}

void _update_calibration_param() {
	_start(I2C_ADDR_BARO, I2C_WRITE);
	_send_byte(0xAA);
	_start(I2C_ADDR_BARO, I2C_READ);
	params.ac1 = _read_word(true);
	params.ac2 = _read_word(true);
	params.ac3 = _read_word(true);
	params.ac4 = _read_word(true);
	params.ac5 = _read_word(true);
	params.ac6 = _read_word(true);
	params.b1 = _read_word(true);
	params.b2 = _read_word(true);
	params.mb = _read_word(true);
	params.mc = _read_word(true);
	params.md = _read_word(false);
	_stop();
}

long get_utemp() {
	int16_t buffer;

	// Set mode to 'read temperature'
	_start(I2C_ADDR_BARO, I2C_WRITE);
	_send_byte(0xF4);
	_send_byte(0x2E);
	_stop();

	// Delay required by Datasheet
	_delay_us(4500);

	// Read Temperature
	_start(I2C_ADDR_BARO, I2C_WRITE);
	_send_byte(0xF6);
	_start(I2C_ADDR_BARO, I2C_READ);
	buffer = _read_word(false);
	_stop();

	return (long) buffer;
}

long get_upres() {
	long buffer;
	long upper, middle, lower;

	// Set mode to 'read pressure'
	_start(I2C_ADDR_BARO, I2C_WRITE);
	_send_byte(0xF4);
	_send_byte(0x34 | (OSS << 6));
	_stop();

	// Delay required by Datasheet (25500 us for uhigh-res)
	_delay_us(25500);

	// Read Pressure
	_start(I2C_ADDR_BARO, I2C_WRITE);
	_send_byte(0xF6);
	_start(I2C_ADDR_BARO, I2C_READ);
	upper = _read_byte(true);
	middle = _read_byte(true);
	lower = _read_byte(false);
	_stop();

//	buffer = ( (upper<<16) + (middle<< 8) + lower);
	buffer = ((upper << 16) + (middle << 8) + lower) >> (8 - OSS);
	return buffer;
}

long get_temperature(long uTemp) {
	// Refer to datasheet
	long x1 = ((uTemp - params.ac6) * params.ac5) >> 15;
	long x2 = ((int32_t) params.mc << 11) / (x1 + params.md);
	long b5 = x1 + x2;
	return (b5 + 8) >> 4;
//	return (b5 * 10 + 8) >> 4;
}

long get_pressure(long uTemp, long uPres) {
	// Refer to datasheet
	int32_t x1, x2, x3, b3, b5, b6, p;
	uint32_t b4, b7;

	x1 = ((uTemp - params.ac6) * params.ac5) >> 15;
	x2 = ((int32_t) params.mc << 11) / (x1 + params.md);
	b5 = x1 + x2;
	b6 = b5 - 4000;

	x1 = (params.b2 * ((b6 * b6) >> 12)) >> 11;
	x2 = (params.ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = ((((params.ac1 * 4) + x3) << OSS) + 2) / 4;

	x1 = (params.ac3 * b6) >> 13;
	x2 = (params.b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (params.ac4 * (uint32_t) (x3 + 32768)) >> 15;
	b7 = ((uint32_t) uPres - b3) * (50000 >> OSS);

	if (b7 < 0x80000000) {
		p = (b7 * 2) / b4;
	} else {
		p = (b7 / b4) * 2;
	}
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p = p + ((x1 + x2 + 3791) >> 4);
	return p;
}

long get_altitude(long pressure) {
	return 44330 * (1 - pow(pressure / PRESSURE_SEA_LEVEL, 1 / 5.255));
}

} /* namespace baro */

} /* namespace i2c */
