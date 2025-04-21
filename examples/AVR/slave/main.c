#include <avr/io.h>
#include <avr/interrupt.h>
#include "EasyBus_slave.h"

#define CMD_1	0x10
#define CMD_2	0x20

EB_exception_t cmd_1_command_handler(uint8_t *msg , uint8_t *lenght);
EB_exception_t cmd_2_command_handler(uint8_t *msg , uint8_t *lenght);

static uint8_t some_var;

int main(void)
{
	/*init the protocol and set the slave address*/	
	EB_slave_init(0x0A , 9600);
	/*register the commands handlers*/
    EB_register_command_handlers(CMD_1 , &cmd_1_command_handler);
    EB_register_command_handlers(CMD_2 , &cmd_2_command_handler);
	
	sei();
	
	while(1)
	{ 	 
	
		EB_poll();	


	}
}

/*
	the command handlers must return a EB_EXCP_NONE to resonse with a normall response , otherwise will return a error message
*/


/*
	this function will be called when a CMD_1 command is received,
	the msg pointer is a poniter the data that arrived with the command, same
	for the lenght, so the address and the commmand not included because we will
	need them in the slave side
*/
EB_exception_t cmd_1_command_handler(uint8_t *msg , uint8_t *lenght)
{
	/// if the care about the data we must copy it to a local buffer, because it will overwriten with 
	/// the response or another message when it will arrive 
	
	/// do something
	msg[0] = 5;	// dummy data
	/*lenght of the response*/
	*lenght = 1;
	return EB_EXCP_NONE;
}

/*this function will be called when a CMD_2 command is received */
EB_exception_t cmd_2_command_handler(uint8_t *msg , uint8_t *lenght)
{
	// for example if this command does not need the return any data to master
	
	*lenght = 0;
	return EB_EXCP_NONE;
	
	/*if something bad happen, for example the data master send in not legal*/
	
	return EB_EXCP_ILLEGAL_DATA;
	
	/*this is the idea you can tweek it as u like */
}
