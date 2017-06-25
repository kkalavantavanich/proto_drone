#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "USART.h"  // Print
#include "I2C.h" 	// Sensors

//#define DEBUG

/* ==== RX Pins ====
 * PIN2 (D2) = PCINT18 (PCMSK2) = RX 0 - Throttle
 * PIN4 (D4) = PCINT20 (PCMSK2) = RX 1 - Roll
 * PIN5 (D5) = PCINT21 (PCMSK2) = RX 2 - Pitch
 * PIN6 (D6) = PCINT22 (PCMSK2) = RX 3 - Yaw
 * PIN7 (D7) = PCINT23 (PCMSK2) = RX 4 - Aux 1
 * PIN8 (B0) = PCINT0  (PCMSK0) = RX 5 - Aux 2
 */

/* ==== Motor Pins ====
 * PIN11 (B3) = Motor 1 = OC2A
 * PIN3  (D3) = Motor 2 = OC2B
 * PIN10 (B2) = Motor 3 = OC1B
 * PIN9  (B1) = Motor 4 = OC1A
 */
#define LOOPTIME_US 500000

/// TIMER ///

volatile uint32_t timer_ovf_count = 0;
void timer_init() {
	// enable ovf interrupt
	TIMSK0 |= (1 << TOIE0);

	// start timer prescale/8 @ Normal Operation
	TCCR0B |= (1 << CS01);
}

ISR(TIMER0_OVF_vect) {
	++timer_ovf_count;
}

// kronos(): similar to micros() but values are doubled -- unadjusted values are returned. (with F_CPU = 16MHz & prescale/8)
uint32_t kronos() {
	cli();
	uint32_t out = ((uint32_t) (timer_ovf_count << 8)) | TCNT0;
	sei();
	return out;
}

// Strategy //
// 1. Receive with RX
// 2. Process with PID
// 3. Actuate with PWM

//////////////////////////////////////////////////////////////////////////////////////////////////

void rx_init(void) {
	// pinMode(2, 4, 5, 6, 7, 8 -> INPUT_PULLUP)
	PORTD |= (1 << PORTD2) | (1 << PORTD4) | (1 << PORTD5) | (1 << PORTD6)
			| (1 << PORTD7);
	PORTB |= (1 << PORTB0);

	// Enable Interrupts on RX Pins
	PCMSK0 |= (1 << PCINT0);
	PCMSK2 |= (1 << PCINT18) | (1 << PCINT20) | (1 << PCINT21) | (1 << PCINT22) | (1 << PCINT23);
	PCICR  |= (1 << PCIE0)   | (1 << PCIE2);
}

uint32_t rxValues[6] = {3000, 3000, 3000, 3000, 3000, 3000};    //
uint32_t lastTime[6] = {0, 0, 0, 0, 0, 0};						// Last Time Edge Change was detected for RX Pins

// High Priority Control
volatile uint8_t last_pinb_state = PINB;
ISR(PCINT0_vect) {
//	uint8_t pins_state = PINB;
//	uint8_t mask = last_pinb_state ^ pins_state;
	println("PIN_B Interrupt!");
}

// Low Priority Control
volatile uint8_t last_pind_state = PIND;
ISR(PCINT2_vect) {
	uint8_t pins_state = PIND;
	uint8_t mask = last_pind_state ^ pins_state;
	uint32_t currentTime, diffTime;

	currentTime = kronos();
	last_pind_state = pins_state;

	sei();

	if (mask & (1 << 2)) {
		diffTime = currentTime - lastTime[0];
		if (!(pins_state & (1 << 2))) {						// negedge on this pin
			if (diffTime > 1500 && diffTime < 4200) {		// filter only 'good' values
				rxValues[0] = diffTime;
			}
		} else {											// posedge on this pin
			lastTime[0] = currentTime;
		}
	}

	if (mask & (1 << 4)) {
		diffTime = currentTime - lastTime[1];
		if (!(pins_state & (1 << 4))) {						// negedge on this pin
			if (diffTime > 1500 && diffTime < 4200) {		// filter only 'good' values
				rxValues[1] = diffTime;
			}
		} else {											// posedge on this pin
			lastTime[1] = currentTime;
		}
	}

	if (mask & (1 << 5)) {
		diffTime = currentTime - lastTime[2];
		if (!(pins_state & (1 << 5))) {						// negedge on this pin
			if (diffTime > 1500 && diffTime < 4200) {		// filter only 'good' values
				rxValues[2] = diffTime;
			}
		} else {											// posedge on this pin
			lastTime[2] = currentTime;
		}
	}

	if (mask & (1 << 6)) {
		diffTime = currentTime - lastTime[3];
		if (!(pins_state & (1 << 6))) {						// negedge on this pin
			if (diffTime > 1500 && diffTime < 4200) {		// filter only 'good' values
				rxValues[3] = diffTime;
			}
		} else {											// posedge on this pin
			lastTime[3] = currentTime;
		}
	}


}
//////////////////////////////////////////////////////////////////////////////////

//// PWM ////
#define PWM_MIN_INPUT 1500UL
#define PWM_MAX_INPUT 4200UL
#define PWM_MIN_OUTPUT 170UL
#define PWM_MAX_OUTPUT 250UL

#define RANGE_INPUT (PWM_MAX_INPUT - PWM_MIN_INPUT)
#define RANGE_OUTPUT (PWM_MAX_OUTPUT - PWM_MIN_OUTPUT)

void pwm_init() {
	// pinMode(11, 3, 10, 9 -> OUTPUT)
	DDRD |= (1 << PORTD3);
	DDRB |= (1 << PORTB1) | (1 << PORTB2) | (1 << PORTB3);

	// Set Timer 1 as Phase-corrected PWM 8-bit non-inverting
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);

	// Set Timer 2 as Phase-corrected PWM 8-bit non-inverting
	TCCR2A |= (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20);

	// Start Clocks Prescale/64
	TCCR1B |= (1 << CS10) | (1 << CS11);
	TCCR2B |= (1 << CS22);

	// Enable Interrupts
//	TIMSK1 |= (1 << TOIE);
//	TIMSK2 |= (1 << TOIE);
}

void pwm_update() {
	OCR1A = (((rxValues[0] - PWM_MIN_INPUT) * RANGE_OUTPUT) / RANGE_INPUT) + PWM_MIN_OUTPUT;
	OCR1B = (((rxValues[0] - PWM_MIN_INPUT) * RANGE_OUTPUT) / RANGE_INPUT) + PWM_MIN_OUTPUT;
	OCR2A = (((rxValues[0] - PWM_MIN_INPUT) * RANGE_OUTPUT) / RANGE_INPUT) + PWM_MIN_OUTPUT;
	OCR2B = (((rxValues[0] - PWM_MIN_INPUT) * RANGE_OUTPUT) / RANGE_INPUT) + PWM_MIN_OUTPUT;
	print(">> ");
	print(OCR1A, 10);
	print(", ");
	print(OCR1B, 10);
	print(", ");
	print(OCR2A, 10);
	print(", ");
	print(OCR2B, 10);
	println();
}

/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////

int main(void) {
	// Serial.begin(115200)
	UCSR0B |= (1 << TXEN0);
	UBRR0L = 8;
	println("Starting...");

	// no power reduction
	PRR = 0x00;

	// init pulse width modulation (pwm)
	pwm_init();

	// init receiver (rx)
	rx_init();

	// i2c initialize
	i2c::initialize();
	i2c::check_addresses();

	// timer init
	timer_init();

	// global variables in main loop //
	int i = 0;											// loop index
	uint8_t ext_pin_state = 0xFF;						// External Pin

	mpu6050_t motion_data;
	hmc5883_t compass_data;
	bmp085_t baro_data;

	int16_t currentOrientation[3] = {0, 0, 0};			// Calculated from gyro data
	int16_t _gyro_prev[3] = {0, 0, 0};					// previous gyro value
	int16_t _gyro_imm[3] = {0, 0, 0};					// immediate gyro value

	int16_t currentVelocity[3] = {0, 0, 0};				// Calculated from accelerometer
	int16_t _acc_prev[3] = {0, 0, 0};
	int16_t _acc_imm[3]  = {0, 0, 0};

	sei();

	println("Setup Finished.");

	///////////////////////// MAIN LOOP ///////////////////////////
	while (true) {
		// Console Number
		print(++i);
		print(" >> ");

//		print("ext_pin_state = ");
//		print(ext_pin_state, 16);
		// Change External Pin outputs=
//		i2c::ext::write(ext_pin_state);

		// Read data from sensors
		motion_data = i2c::ga::read_smooth();
		compass_data = i2c::cmps::read();
		baro_data = i2c::baro::read();


		// Calculate current orientation
//		_gyro_imm[0] += motion_data.gyro_x * LOOPTIME_US / 1000000;		// Add angle change [deg/s] to immediate.
//		_gyro_imm[1] += motion_data.gyro_y * LOOPTIME_US / 1000000;
//		_gyro_imm[2] += motion_data.gyro_z * LOOPTIME_US / 1000000;

//		currentOrientation[0] = (_gyro_imm[0]);	 // Calculate current orientation
//		currentOrientation[1] = (_gyro_imm[1]);
//		currentOrientation[2] = (_gyro_imm[2]);

//		_gyro_prev[0] = _gyro_imm[0] >> 1;
//		_gyro_prev[1] = _gyro_imm[1] >> 1;
//		_gyro_prev[2] = _gyro_imm[2] >> 1;

		// Calculate current velocity
//		_acc_imm[0] += motion_data.acc_x;
//		_acc_imm[1] += motion_data.acc_y;
//		_acc_imm[2] += motion_data.acc_z;
//
//		currentVelocity[0] = (_acc_imm[0] + _acc_prev[0]) / 3;
//		currentVelocity[1] = (_acc_imm[1] + _acc_prev[1]) / 3;
//		currentVelocity[2] = (_acc_imm[2] + _acc_prev[2]) / 3;
//
//		_acc_prev[0] = _acc_imm[0] >> 1;
//		_acc_prev[1] = _acc_imm[1] >> 1;
//		_acc_prev[2] = _acc_imm[2] >> 1;

//		 Print Data
		i2c::printData(motion_data);
		print(" ");
		i2c::printData(compass_data);
		print(" ");
		i2c::printData(baro_data);
//		print("   ");
//		print("Orient.[");
//		print(i2c::ga::convertFromRawGyro(currentOrientation[0]));
//		print("deg, ");
//		print(i2c::ga::convertFromRawGyro(currentOrientation[1]));
//		print("deg, ");
//		print(i2c::ga::convertFromRawGyro(currentOrientation[2]));
//		print("deg]");



//		println();
//		print(">> ");
//		print(rxValues[0]);
//		print(", ");
//		print(rxValues[1]);
//		print(", ");
//		print(rxValues[2]);
//		print(", ");
//		print(rxValues[3]);

		pwm_update();
		_delay_ms(1000);
		println();
	}
}


////////////////////////////////////////////////////////////////////////////////////////
