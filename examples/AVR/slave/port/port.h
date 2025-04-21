
#ifndef PORT_H_
#define PORT_H_

#include <avr/io.h>
#include <avr/interrupt.h>

/*this file is USER specific , i need it for F_CPU*/
#include "portFile.h"

#define ENTER_CRITICAL_SECTION( )   cli()
#define EXIT_CRITICAL_SECTION( )    sei()

#define RS485_DIR_ENABLED


#ifdef RS485_DIR_ENABLED

	#define RS485_DIR_DDR	DDRB
	#define RS485_DIR_PORT	PORTB
	#define RS485_DIR_PIN	0
	
static inline void dir_init()
{
    RS485_DIR_DDR |= _BV(RS485_DIR_PIN);
    RS485_DIR_PORT &= ~(_BV(RS485_DIR_PIN));
}

static inline void dir_reset()
{
    RS485_DIR_PORT &= ~(_BV(RS485_DIR_PIN));
}

static inline void dir_set()
{
    RS485_DIR_PORT |= _BV(RS485_DIR_PIN);
}
#endif

#endif /* PORT_H_ */
