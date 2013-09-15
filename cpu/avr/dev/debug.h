#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "avr/pgmspace.h"

void debug_print(char *str);

void debug_print8(unsigned char v);
void debug_print16(unsigned short v);

#endif /* __DEBUG_H__ */
