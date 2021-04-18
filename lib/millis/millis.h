#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdarg.h>

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )

#define SET_TCCRA()	(REG_TCCRA = _BV(BIT_WGM))
#define SET_TCCRB()	(REG_TCCRB = CLOCKSEL)

void millis_init(void);
unsigned long millis(void);