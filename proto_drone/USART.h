/*
 * USART.h
 *
 *  Created on: May 15, 2017
 *      Author: Kris
 */

#ifndef USART_H_
#define USART_H_
#include <avr/io.h>

/** Serial console connection function prototypes */
void USART_transmit(unsigned char data);
void print(const unsigned char *buffer, uint8_t n);
void print(const char* str);
void print(char c);
void print(int, uint8_t base = 10);
void print(long, uint8_t base = 10);
void print(unsigned long, uint8_t base = 10);
void print(unsigned int, uint8_t base = 10);
void print(double, uint8_t digits = 2);

void println();
void println(const char* str);
void println(unsigned long, uint8_t base = 10);
uint16_t strlen(const char *s); // String must be < 65535 chars

#endif /* USART_H_ */
