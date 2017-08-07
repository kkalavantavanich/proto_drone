/*
 * USART.cpp
 *
 *  Created on: May 15, 2017
 *      Author: Kris
 */

#include "USART.h"


void USART_transmit(const uint8_t data) {
	/* wait for empty transmit buffer */
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void print(const uint8_t *buffer, uint8_t n) {
	while (n--) {
		USART_transmit(*buffer++);
	}
}

void print(const char *str) {
	if (strlen(str) != 0) {
		print((const uint8_t *) str, strlen(str));
	}
}

void print(char c){
	USART_transmit(c);
}

void print(int n, uint8_t base) {
	if (n < 0) {
		USART_transmit('-');
		n = -n;
	}

	char buf[8 * sizeof(int) + 1];
	char *str = &buf[sizeof(buf) - 1];
	*str = '\0';

	if (base < 2) base = 10;

	do {
		char c = n % base;
		n /= base;
		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while (n);

	do {
		USART_transmit(*str);
		++str;
	} while (*str);
}

void print(long n, uint8_t base) {
	if (n < 0) {
		USART_transmit('-');
		n = -n;
	}

	char buf[8 * sizeof(long) + 1];
	char *str = &buf[sizeof(buf) - 1];
	*str = '\0';

	if (base < 2) base = 10;

	do {
		char c = n % base;
		n /= base;
		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while (n);

	do {
		USART_transmit(*str);
		++str;
	} while (*str);
}

void print(unsigned long n, uint8_t base) {
	char buf[8 * sizeof(long) + 1];
	char *str = &buf[sizeof(buf) - 1];
	*str = '\0';

	if (base < 2) base = 10;

	do {
		char c = n % base;
		n /= base;
		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while (n);

	do {
		USART_transmit(*str);
		++str;
	} while (*str);
}

void print(unsigned int n, uint8_t base) {
	char buf[8 * sizeof(int) + 1];
	char *str = &buf[sizeof(buf) - 1];
	*str = '\0';

	if (base < 2) base = 10;

	do {
		char c = n % base;
		n /= base;
		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while (n);

	do {
		USART_transmit(*str);
		++str;
	} while (*str);
}

void print(double number, uint8_t digits)
{
  if (number > 4294967040.0) return print ("ovf");  // constant determined empirically
  if (number <-4294967040.0) return print ("ovf");  // constant determined empirically

  // Handle negative numbers
  if (number < 0.0)
  {
     print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long) number;
  double remainder = number - (double) int_part;
  print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
    print('.');
  }


  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    unsigned int toPrint = (unsigned int)(remainder);
    print(toPrint);
    remainder -= toPrint;
  }
}


void println() {
	USART_transmit('\n');
}

void println(unsigned long n, uint8_t base) {
	print(n, base);
	USART_transmit('\n');
}

void println(const char *str) {
	print(str);
	USART_transmit('\n');
}

uint16_t strlen(const char *s)
{
    char const * const start = s;
    while( *s != 0 )
    {
        ++s;
    }
    return s - start;
}

