/*
 * utils.c
 *
 *  Created on: 20/04/2015
 *      Author: Victor
 */

#include "utils.h"
#include <string.h>

uint8_t startsWith(char *str, char *pre)
{
	if (str == NULL || pre == NULL || strlen(str) < strlen(pre)) return 0;
	return strncmp(pre, str, strlen(pre)) == 0;
}

uint8_t endsWith(char *str, char *suffix) {
	if (str == NULL || suffix == NULL || strlen(str) < strlen(suffix)) return 0;
	return strcmp(&str[strlen(str) - strlen(suffix)], suffix) == 0;
}

/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.
 */
void itoa(int value, char* result, int base) {
	// check that the base if valid
	if (base < 2 || base > 36) { *result = '\0'; }

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while ( value );

	// Apply negative sign
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while(ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr--= *ptr1;
		*ptr1++ = tmp_char;
	}
}
