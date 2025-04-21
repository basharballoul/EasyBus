
#ifndef PORT_H_
#define PORT_H_

#include <avr/io.h>
#include <avr/interrupt.h>

/*comment this if there is no direction control*/
#define RS485_DIR_ENABLED


#ifdef RS485_DIR_ENABLED

	#define RS485_DIR_DDR	DDRC
	#define RS485_DIR_PORT	PORTC
	#define RS485_DIR_PIN	5
	

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

#define ENTER_CRITICAL_SECTION( )   cli()
#define EXIT_CRITICAL_SECTION( )    sei()



#endif /* PORT_H_ */
