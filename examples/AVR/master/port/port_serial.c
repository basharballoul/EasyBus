

#include "port.h"
#include "EasyBus_master.h"
#include "EasyBus_port.h"


#define UART_BAUD_CALC(baudRate,xtalCpu) (((xtalCpu)+8UL*(baudRate))/(16UL*(baudRate))-1UL)

void EB_port_Serial_init(uint32_t baudrate)
{
    uint8_t UCSRC_TEMP;

    /*set the UBRR */
    UBRR0L = (uint8_t)UART_BAUD_CALC(baudrate  , F_CPU);
    UBRR0H = (uint8_t)(UART_BAUD_CALC(baudrate  , F_CPU) >> 8);

    /*set the data bits to 8 and parity to None and stop bits to one*/
    UCSRC_TEMP = (1 << UCSZ00) | (1 << UCSZ01);

    UCSR0C = (1 << URSEL0) | UCSRC_TEMP;
	
	EB_port_serial_enable(TX_DISABLE , RX_DISABLE);
	
	#ifdef	RS485_DIR_ENABLED
		dir_init();
	#endif
}


void EB_port_serial_enable(TX_CONT_t tx_control, RX_CONT_t rx_control)
{
	#ifdef RS485_DIR_ENABLED
	    UCSR0B |= (1 << TXEN0) | (1 << TXCIE0);
	#else
		UCSR0B |= (1 << TXEN0);
	#endif
	    if (tx_control == TX_ENABLE)
	    {

		    UCSR0B |= (1 << TXEN0) | (1 << UDRIE0);
			#ifdef RS485_DIR_ENABLED
				dir_set();
			#endif
		
	    }
	    else
	    {
		    UCSR0B &= ~(1 << UDRIE0);
	    }

	    if (rx_control == RX_ENABLE)
	    {
		    UCSR0B |= (1 << RXEN0) | (1 << RXCIE0);
	    }
	    else
	    {
		    UCSR0B &= ~((1 << RXEN0) | (1 << RXCIE0));
	    }

}

void EB_port_serial_get_byte(uint8_t *byte)
{
    *byte = UDR0;
}

void EB_port_serial_put_byte(uint8_t byte)
{
    UDR0 = byte;
}





#ifdef RS485_DIR_ENABLED

ISR(USART0_TXC_vect)
{
	dir_reset();
}
#endif


/*USART data register is empty a new char can be send*/
ISR(USART0_UDRE_vect)
{
	EB_serial_tx_handler();
}


/*new byte received*/
ISR(USART0_RXC_vect)
{
	EB_serial_rx_handler();
}
