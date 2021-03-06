#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <Arduino.h>
#include <math.h>

#include "USART.h"  // Print
#include "I2C.h"    // Sensors
#include "LPF.h"    // Low Pass Filter

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

#define LOOPTIME_US 3000

#define MODE_CALIBRATE 0
#define MODE_RATE 1

/* Drone mode -- controlled by switch-02 (channel 6) */
int drone_mode = MODE_RATE;

// Global Vars //
mpu6050_t motion_data;
hmc5883_t compass_data;
bmp085_t baro_data;
uint32_t prev_kronos, program_time_kr = 1000;

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

// USART //
ISR(USART_RX_vect) {
	/* Get and return received data from buffer */
	// echo function
    // write(UDR0);
	uint8_t data = UDR0;
	switch(data) {
	case 't':
		write(program_time_kr);
		break;
	case 'a':
		write(motion_data.acc_x);
		write(motion_data.acc_y);
		write(motion_data.acc_z);
		write(motion_data.gyro_x);
		write(motion_data.gyro_y);
		write(motion_data.gyro_z);
		write((short) (motion_data.ang_x * 16));
		write((short) (motion_data.ang_y * 16));
		write((short) (motion_data.ang_z * 16));
		break;
	case 'b':
		write(motion_data.acc_x);
		break;
	case 'c':
		write(motion_data.acc_y);
		break;
	case 'd':
		write(motion_data.acc_z);
		break;
	case 'e':
		write(motion_data.gyro_x);
		break;
	case 'f':
		write(motion_data.gyro_y);
		break;
	case 'g':
		write(motion_data.gyro_z);
		break;
	case 'h':
		write((short) (motion_data.ang_x * 16));
		break;
	case 'i':
		write((short) (motion_data.ang_y * 16));
		break;
	case 'j':
		write((short) (motion_data.ang_z * 16));
		break;
	case 'Z':				// checking
		write(0xBEEF);
		break;
	default:
		write("unknown command");
	}
}

// Strategy //
// 1. Receive with RX    => rxValues
// 2. Low pass RX Values => rxlpValues
// 3. Process with PID   => motorValues
// 4. Actuate with PWM   => OCRnX

//////////////////////////////////////////////////////////////////////////////////////////////////

void rx_init(void) {
	// pinMode(2, 4, 5, 6, 7, 8 -> INPUT_PULLUP)
	PORTD |= (1 << PORTD2) | (1 << PORTD4) | (1 << PORTD5) | (1 << PORTD6)
			| (1 << PORTD7);
	PORTB |= (1 << PORTB0);

	// Enable Interrupts on RX Pins
	PCMSK0 |= (1 << PCINT0);
	PCMSK2 |= (1 << PCINT18) | (1 << PCINT20) | (1 << PCINT21) | (1 << PCINT22)
			| (1 << PCINT23);
	PCICR |= (1 << PCIE0) | (1 << PCIE2);
}

uint16_t rxValues[6] = { 3000, 3000, 3000, 3000, 3000, 3000 };
uint16_t rxlpValues[6] = { 3000, 3000, 3000, 3000, 3000, 3000 };
LPF<uint16_t> lpfs[6];

uint32_t lastTime[6] = { 0, 0, 0, 0, 0, 0 }; // Last Time Edge Change was detected for RX Pins

// High Priority Control
volatile uint8_t last_pinb_state = PINB;
ISR(PCINT0_vect) {
	uint8_t pins_state = PINB;
	uint8_t mask = last_pinb_state ^ pins_state;

	uint32_t currentTime, diffTime;

	currentTime = kronos();
	last_pinb_state = pins_state;

	sei();
	if (mask & (1 << 0)) {
		diffTime = currentTime - lastTime[5];
		if (!(pins_state & (1 << 0))) {         // negedge on this pin
			if (diffTime > 1500 && diffTime < 4200) { // filter only 'good' values
				rxValues[5] = diffTime;
			}
		} else {                    // posedge on this pin
			lastTime[5] = currentTime;
		}
	}

	if (rxValues[5] < 2500) {
		drone_mode = MODE_CALIBRATE;
	} else {
		drone_mode = MODE_RATE;
	}
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
		if (!(pins_state & (1 << 2))) {         // negedge on this pin
			if (diffTime > 1500 && diffTime < 4200) { // filter only 'good' values
				rxValues[0] = diffTime;
			}
		} else {                    // posedge on this pin
			lastTime[0] = currentTime;
		}
	}

	if (mask & (1 << 4)) {
		diffTime = currentTime - lastTime[1];
		if (!(pins_state & (1 << 4))) {         // negedge on this pin
			if (diffTime > 1500 && diffTime < 4200) { // filter only 'good' values
				rxValues[1] = diffTime;
			}
		} else {                    // posedge on this pin
			lastTime[1] = currentTime;
		}
	}

	if (mask & (1 << 5)) {
		diffTime = currentTime - lastTime[2];
		if (!(pins_state & (1 << 5))) {         // negedge on this pin
			if (diffTime > 1500 && diffTime < 4200) { // filter only 'good' values
				rxValues[2] = diffTime;
			}
		} else {                    // posedge on this pin
			lastTime[2] = currentTime;
		}
	}

	if (mask & (1 << 6)) {
		diffTime = currentTime - lastTime[3];
		if (!(pins_state & (1 << 6))) {         // negedge on this pin
			if (diffTime > 1500 && diffTime < 4200) { // filter only 'good' values
				rxValues[3] = diffTime;
			}
		} else {                    // posedge on this pin
			lastTime[3] = currentTime;
		}
	}

}

////////////// RX Low Pass ///////////////////////////////

void low_pass_update() {
	for (int i = 0; i < 6; i++) {
		lpfs[i].push(rxValues[i]);
		lpfs[i].get(rxlpValues[i]);
	}
}

//////////////////////////////////////////////////////////////////////////////////

//// PWM ////
#define PWM_MIN_INPUT 2000UL
#define PWM_MAX_INPUT 4000UL
#define PWM_MIN_OUTPUT 15UL
#define PWM_MAX_OUTPUT 70UL
#define RATE_MINDEGSEC (-30L)
#define RATE_MAXDEGSEC 30L

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
//  TCCR1B |= (1 << CS10) | (1 << CS11);
//  TCCR2B |= (1 << CS22);

	// Start Clocks Prescale/128
//    TCCR2B |= (1 << CS22) | (1 << CS20);

	// Start Clocks Prescale/256
	TCCR1B |= (1 << CS12);
	TCCR2B |= (1 << CS22) | (1 << CS21);

	// Enable Interrupts
//  TIMSK1 |= (1 << TOIE);
//  TIMSK2 |= (1 << TOIE);
}

int16_t motorValues[4] = { 0, 0, 0, 0 };
double toy_gyro_x = 0;
double previousPitchRate = 0.0;
double pitch_I = 0;

///////////////////////////////////////////////////////////////////////////////////

void update_motor_values() {
	uint32_t rxValueT = constrain(rxlpValues[0], PWM_MIN_INPUT, PWM_MAX_INPUT);
	uint32_t rxValuesmapT = map(rxValueT, PWM_MIN_INPUT, PWM_MAX_INPUT,
	PWM_MIN_OUTPUT, PWM_MAX_OUTPUT);
	uint32_t rxValueP = constrain(rxlpValues[2], PWM_MIN_INPUT, PWM_MAX_INPUT);
	int32_t rxValuesmapP = map(rxValueP, PWM_MIN_INPUT, PWM_MAX_INPUT,
	RATE_MINDEGSEC, RATE_MAXDEGSEC);

//  print(">> rxValueT=");
//  print(rxValueT, 10);
//  print(", rxVmapT=");
//  print(rxValuesmapT, 10);
//  print(", rxValueP=");
//  print(rxValueP, 10);
//  print(", rxValuemapP=");
//  print(rxValuesmapP, 10);
//  println("");

// PID CONTROL START RATE MODE
// Coefficient
//	double Kp_RateRoll = 1;
//	double Ki_RateRoll = 0.0;
//	double Kd_RateRoll = 0.0;
	double Kp_RatePitch = 0.07;
	double Ki_RatePitch = 0.0;
	double Kd_RatePitch = 0.0;
//	double Kp_RateYaw = 2.2;                      //2.2
//	double Ki_RateYaw = 0.0;
//	double Kd_RateYaw = 0.0;
	double dt_s = 68000 / 1e6;

//

// PITCH PID
//  print("TGX: ");
//  print(toy_gyro_x);
	double currentPitchRate = toy_gyro_x;
	double err_pitch = -rxValuesmapP - currentPitchRate;
	// print(", ERR_PITCH: ");
	// print(err_pitch);
	pitch_I += err_pitch * dt_s;
	pitch_I = constrain(pitch_I, -20, 20);
	double pitch_D = (currentPitchRate - previousPitchRate) / dt_s;
	previousPitchRate = currentPitchRate;
	double control_pitch = Kp_RatePitch * err_pitch - Kd_RatePitch * pitch_D
			+ Ki_RatePitch * pitch_I;
	control_pitch = constrain(control_pitch, -10, 10);

// ROLL PID
// Wait for pitch to success
// YAW PID
// WAIT for pitch to success

	motorValues[0] = rxValuesmapT - control_pitch;
	motorValues[1] = rxValuesmapT + control_pitch;
	motorValues[2] = rxValuesmapT + control_pitch;
	motorValues[3] = rxValuesmapT - control_pitch;
	if (rxValues[0] < 2000) {
		for (int i = 0; i <= 3; i++) {
			motorValues[i] = PWM_MIN_OUTPUT;
		}
	}

//  motorValues[0] = rxValuesmapT;
//  motorValues[1] = rxValuesmapT;
//  motorValues[2] = rxValuesmapT;
//  motorValues[3] = rxValuesmapT;

}

void pwm_update() {
	OCR2A = motorValues[0];
	OCR2B = motorValues[2];
	OCR1B = motorValues[3];
	OCR1A = motorValues[1];
}

/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////

int main(void) {
	// Serial.begin(115200)
	UCSR0B |= (1 << TXEN0) || (1 << RXEN0);
	UBRR0L = 8;
	UCSR0B |= (1 << RXCIE0); 	// enable USART receive interrupt
//	println("Starting...");

	// no power reduction
	PRR = 0x00;

	// init pulse width modulation (pwm)
	pwm_init();

	// init receiver (rx)
	rx_init();

	// i2c initialize
	i2c::initialize();
	//i2c::check_addresses();

	// timer init
	timer_init();
	prev_kronos = kronos();

	// global variables in main loop //
	int i = 0;                      // loop index
//	uint8_t ext_pin_state = 0xFF;           // External Pin

//	int16_t currentOrientation[3] = { 0, 0, 0 };  // Calculated from gyro data
//	int16_t _gyro_prev[3] = { 0, 0, 0 };        // previous gyro value
//	int16_t _gyro_imm[3] = { 0, 0, 0 };         // immediate gyro value
//
//	int16_t currentVelocity[3] = { 0, 0, 0 }; // Calculated from accelerometer
//	int16_t _acc_prev[3] = { 0, 0, 0 };
//	int16_t _acc_imm[3] = { 0, 0, 0 };

	sei();

//	println("Setup Finished.");

	///////////////////////// MAIN LOOP ///////////////////////////
	while (true) {
		// Console Number
//		print(++i);
//		print(" >> ");

		if (drone_mode == MODE_CALIBRATE) {
			// TODO: check throttle minimum
			OCR1A = 0;
			OCR1B = 0;
			OCR2A = 0;
			OCR2B = 0;
			_delay_ms(2000);

			OCR1A = PWM_MAX_OUTPUT;
			OCR1B = PWM_MAX_OUTPUT;
			OCR2A = PWM_MAX_OUTPUT;
			OCR2B = PWM_MAX_OUTPUT;
			_delay_ms(2000);

			OCR1A = PWM_MIN_OUTPUT;
			OCR1B = PWM_MIN_OUTPUT;
			OCR2A = PWM_MIN_OUTPUT;
			OCR2B = PWM_MIN_OUTPUT;
			_delay_ms(2000);

			// TODO: check throttle minimum
			drone_mode = MODE_RATE;
		}

//    print("ext_pin_state = ");
//    print(ext_pin_state, 16);
		// Change External Pin outputs=
//    i2c::ext::write(ext_pin_state);
		low_pass_update();

		// Read data from sensors
		//UCSR0B &= ~(1 << RXCIE0); 	// disable USART receive interrupt
		motion_data = i2c::ga::read_smooth();
		compass_data = i2c::cmps::read();
		baro_data = i2c::baro::read();
		//UCSR0B |= (1 << RXCIE0); 	// enable USART receive interrupt

		toy_gyro_x = i2c::ga::convertFromRawGyro(motion_data.gyro_x);

		// Calculate current orientation
//    _gyro_imm[0] += motion_data.gyro_x * LOOPTIME_US / 1000000;   // Add angle change [deg/s] to immediate.
//    _gyro_imm[1] += motion_data.gyro_y * LOOPTIME_US / 1000000;
//    _gyro_imm[2] += motion_data.gyro_z * LOOPTIME_US / 1000000;

//    currentOrientation[0] = (_gyro_imm[0]);  // Calculate current orientation
//    currentOrientation[1] = (_gyro_imm[1]);
//    currentOrientation[2] = (_gyro_imm[2]);

//    _gyro_prev[0] = _gyro_imm[0] >> 1;
//    _gyro_prev[1] = _gyro_imm[1] >> 1;
//    _gyro_prev[2] = _gyro_imm[2] >> 1;

		// Calculate current velocity
//    _acc_imm[0] += motion_data.acc_x;
//    _acc_imm[1] += motion_data.acc_y;
//    _acc_imm[2] += motion_data.acc_z;
//
//    currentVelocity[0] = (_acc_imm[0] + _acc_prev[0]) / 3;
//    currentVelocity[1] = (_acc_imm[1] + _acc_prev[1]) / 3;
//    currentVelocity[2] = (_acc_imm[2] + _acc_prev[2]) / 3;
//
//    _acc_prev[0] = _acc_imm[0] >> 1;
//    _acc_prev[1] = _acc_imm[1] >> 1;
//    _acc_prev[2] = _acc_imm[2] >> 1;

//     Print Data

		//i2c::printData(motion_data);
//		print(" ");
//    i2c::printData(compass_data);
//		print(" ");
//    i2c::printData(baro_data);
//    print("   ");
//    print("Orient.[");
//    print(i2c::ga::convertFromRawGyro(currentOrientation[0]));
//    print("deg, ");
//    print(i2c::ga::convertFromRawGyro(currentOrientation[1]));
//    print("deg, ");
//    print(i2c::ga::convertFromRawGyro(currentOrientation[2]));
//    print("deg]");

//    println();
//    print(">> ");
//    print(rxValues[0]);
//    print(", ");
//    print(rxValues[1]);
//    print(", ");
//    print(rxValues[2]);
//    print(", ");
//    print(rxValues[3]);

		update_motor_values();
		pwm_update();

//		print(">> Program Time (Kronos): ");
//		print(program_time_kr);
//		println();

//		print(">> OCR values: ");
//		print(OCR2A, 10);
//		print(", ");
//		print(OCR1B, 10);
//		print(", ");
//		print(OCR2B, 10);
//		print(", ");
//		print(OCR1A, 10);
//		println();
//
//		print(">> rxValues: ");
//		for (int i = 0; i < 6; i++) {
//			print(rxValues[i]);
//			print(" ");
//		}
//		println();
//
//		print(">> rxlpValues: ");
//		for (int i = 0; i < 6; i++) {
//			print(rxlpValues[i]);
//			print(" ");
//		}
//		println();
//
//		print(">> drone_mode: ");
//		println(drone_mode);

//      _delay_us(LOOPTIME_US);

		program_time_kr = kronos() - prev_kronos;
		prev_kronos = kronos();
//		println();
	}
}

////////////////////////////////////////////////////////////////////////////////////////
