
#include "port.h"

#include "EasyBus_master.h"
#include "EasyBus_port.h"

#define TIMER_PRESCALER		(1024UL)
#define TIMER_TICK			(F_CPU /TIMER_PRESCALER)


static uint16_t timer_reload_ocr1a;

BOOL EB_port_timer_init(uint32_t timeout35_10us)
{
	 uint32_t temp = ( ( TIMER_TICK * timeout35_10us  )/ 1E5 );
	/*timer cant handle this value , set appropriate prescaler value*/
	if (temp >= UINT16_MAX)
	{
		return FALSE;
	}
	timer_reload_ocr1a = (uint16_t)temp;
    TCCR1A = 0;
    TCCR1B |= 1 << WGM12;
    EB_port_timer_disable();
	return TRUE;
}


void EB_port_timer_enable()
{
    TCNT1 = 0;
    /*enable COMPA1 interrupt*/
    TIMSK |= 1 << OCIE1A;
    OCR1A = timer_reload_ocr1a;
    /*enable timer*/
    TCCR1B |= (1 << CS10 ) | (1 << CS12);
}

void EB_port_timer_disable()
{
    /*disable timer*/
    TCCR1B &= ~((1 << CS10) | ( 1 << CS12) );
    /*disable COMPA1 interrupt*/
    TIMSK &= ~(1 << OCIE1A);
    /*clear COMPA1 flag*/
    TIFR |= 1 << OCF1A;
}


ISR(TIMER1_COMPA_vect)
{
    EB_timer_expired_handler();
}

