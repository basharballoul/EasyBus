#include <avr/io.h>
#include <avr/interrupt.h>
#include "EasyBus_master.h"

#define CMD_1	0x10
#define CMD_2	0x20


void cmd_1_notify_func(uint8_t *msg , uint8_t len);
void cmd_2_notify_func(uint8_t *msg , uint8_t len);

const cmd_1_data[] = { 0 , 1 , 2 , 3 , 4 }; // some dummy data

int main(void)
{
	/*init the protocol*/	
	EB_master_init(9600);
	
	/*register the command notification functions*/
	EB_register_notify_func(CMD_1 , &cmd_1_notify_func);
    EB_register_notify_func(CMD_2 , &cmd_2_notify_func);
	
	sei();
	
	while(1)
	{ 	 
	EB_poll();	
	
	if(millis() - t >= 1000)	
	{
		EB_msg_t msg;
   		/*format the message first , slave address 0x0A, Command is 0x10 and no data so data lenght 0 */
	   	EB_make_msg(&msg , 0x0A , CMD_1 , NULL , 0);
   		/*push it into the queue, it will be send when the Bus is free*/
		EB_enqueue_msg(&msg);
		/*format another message, slave address 0x0B, command is 0x20 and there is data with the message */
		EB_make_msg(&msg , 0x0B , CMD_2 , cmd_1_data , 5);
		/*push it into the queue, it will be send when the Bus is free*/
		/*you can check is the queuing is failed*/
		if( EB_enqueue_msg(&msg) != EB_ER_QUEUE_FULL)
		{
			
		}
		t = millis();
	}
	}
}

void cmd_1_notify_func(uint8_t *msg , uint8_t len)
{
	/*check if the response is form the expected slave , because you can send the same command to different slaves*/
	if(msg[EB_MSG_SLAVE_ADDRESS_POS] == 0x0A)	
	{
		/*check if its an error msg*/
		if(EB_IS_IT_ERROR_FRAME(msg) == true)
		{
			/*if yes, check the type of the error*/
			switch(msg[EB_MSG_DATA_POS])
			{
				/*this is all the exception code that protocol support*/
				/*no need to handle all this codes, it's depend on your slave implementation*/
				case EB_EXCP_UNREGISTER_CMD:
					/*slave does't support this command */
					break;
				case EB_EXCP_ILLEGAL_DATA:
					/*illegal data sent by master*/
					break;	
				case EB_EXCP_ACK:
					/*ack , you can use this exception when the command does not need the response data!*/
					break;
				case EB_EXCP_NACK:
					/*same as Ack */
					break;
				case EB_EXCP_SLAVE_BUSY:
					
					break;
				
				case EB_EXCP_SLAVE_NOT_RESPONSING:
					/*this exception is not returned by the slave!, if the master sent the message and no slave response 
					  this exception will return to let you know an timeout accures
					*/
					break;				
			}
		}else
		{
			/*everything is OK*/
			/*do whatever you need with the data you received, len is data lenght received including the slave address and command*/
						
			for(int i = 2 ; i < len ; i++)
			{
				printf("data [%d] = %d\r\n" , i , msg[i]); 
			} 
		}
	}
}

void cmd_2_notify_func(uint8_t *msg , uint8_t len)
{
	/*check if the response is form the expected slave , because you can send the same command to different slaves*/
	if(msg[EB_MSG_SLAVE_ADDRESS_POS] == 0x0B)	
	{
		/*check if its an error msg*/
		if(EB_IS_IT_ERROR_FRAME(msg) == true)
		{
			/*if yes, check the type of the error*/
			switch(msg[EB_MSG_DATA_POS])
			{
				/*this is all the exception code that protocol support*/
				/*no need to handle all this codes, it's depend on your slave implementation*/
				case EB_EXCP_UNREGISTER_CMD:
					/*slave does't support this command */
					break;
				case EB_EXCP_ILLEGAL_DATA:
					/*illegal data sent by master*/
					break;	
				case EB_EXCP_ACK:
					/*ack , you can use this exception when the command does not need the response data!*/
					break;
				case EB_EXCP_NACK:
					/*same as Ack */
					break;
				case EB_EXCP_SLAVE_BUSY:
					
					break;
				
				case EB_EXCP_SLAVE_NOT_RESPONSING:
					/*this exception is not returned by the slave!, if the master sent the message and no slave response 
					  this exception will return to let you know an timeout accures
					*/
					break;				
			}
		}else
		{
			
			/*everything is OK*/
			/*do whatever you need with the data you received, len is data lenght received including the slave address and command*/
			
			for(int i = 2 ; i < len ; i++)
			{
				printf("data [%d] = %d\r\n" , i , msg[i]); 
			} 
		}
	}	
}
