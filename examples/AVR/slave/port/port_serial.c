
#include "port.h"
#include "EasyBus_slave.h"
#include "EasyBus_port.h"


#define UART_BAUD_CALC(baudRate,xtalCpu) (((xtalCpu)+8UL*(baudRate))/(16UL*(baudRate))-1UL)

void EB_port_Serial_init(uint32_t baudrate)
{
    uint8_t UCSRC_TEMP;

    /*set the UBRR */
    UBRRL = (uint8_t)UART_BAUD_CALC(baudrate  , F_CPU);
    UBRRH = (uint8_t)(UART_BAUD_CALC(baudrate  , F_CPU) >> 8);

    /*set the data bits to 8 and parity to None and stop bits to one*/
    UCSRC_TEMP = (1 << UCSZ0) | (1 << UCSZ1);

    UCSRC = (1 << URSEL) | UCSRC_TEMP;
	
	EB_port_serial_enable(TX_DISABLE , RX_DISABLE);
	
	#ifdef	RS485_DIR_ENABLED
		dir_init();
	#endif
}


void EB_port_serial_enable(TX_CONT_t tx_control, RX_CONT_t rx_control)
{
	
	#ifdef RS485_DIR_ENABLED
	    UCSRB |= (1 << TXEN) | (1 << TXCIE);
	#else
		UCSRB |= (1 << TXEN);
	#endif
	
	    if (tx_control == TX_ENABLE)
	    {
		    /*enable transmitter and UDR0 interrupt */
		    UCSRB |= (1 << TXEN) | (1 << UDRIE);
			#ifdef RS485_DIR_ENABLED
				dir_set();
			#endif
		
	    }
	    else
	    {
		    /*UDRIE0 interrupt */
		    UCSRB &= ~(1 << UDRIE);
	    }

	    if (rx_control == RX_ENABLE)
	    {
		    /*enable Receiver and receive interrupt*/
		    UCSRB |= (1 << RXEN) | (1 << RXCIE);
	    }
	    else
	    {
		    /*disable Receiver and receive interrupt*/
		    UCSRB &= ~((1 << RXEN) | (1 << RXCIE));
	    }

}

void EB_port_serial_get_byte(uint8_t *byte)
{
    *byte = UDR;
}

void EB_port_serial_put_byte(uint8_t byte)
{
    UDR = byte;
}





#ifdef RS485_DIR_ENABLED

ISR(USART_TXC_vect)
{
	dir_reset();
}
#endif


/*USART data register is empty a new char can be send*/
ISR(USART_UDRE_vect)
{
	EB_serial_tx_handler();
}


/*new byte received*/
ISR(USART_RXC_vect)
{
	EB_serial_rx_handler();
}
