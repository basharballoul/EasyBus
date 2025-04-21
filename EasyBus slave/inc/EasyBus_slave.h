/*
 * EasyBus_slave.h
 * 
 * EasyBus Protocol - Slave-side implementation with command queueing
 *
 * Author: Bashar Balloul
 * License: MIT
 *
 * This file is part of the EasyBus project.
 */
#ifndef EASYBUS_MASTER_H_
#define EASYBUS_MASTER_H_


#include <inttypes.h>



/*
	DEFINE Number of commands the User will use
*/
#define EB_NUM_OF_CMD					5
/*
	DEFINE Serial buffer Max size
*/
#define EB_BUFFER_MAX_SIZE				64


/*
	Message Frame

	****************************************************  ..... ***********************
	|	SLAVE ADDRESS	|  CMD_ID	|	DATA0	| DATA1 |		| DATAN | CRCL | CRCH |
	****************************************************  ..... ***********************
*/



#define EB_MSG_SLAVE_ADDRESS_POS		0
#define EB_MSG_COMMAND_POS				1
#define EB_MSG_DATA_POS					2

#define EB_BROADCAST_ADDRESS			0x00


typedef enum
{
    EB_ER_NOE,				/*no error*/
    EB_ER_NOCMD,			/*illegal command*/
    EB_ER_TOUT,				/*timeout*/
    EB_ER_QUEUE_EMPTY,		/*message queue is empty*/
    EB_ER_QUEUE_FULL,
	EB_ER_TIMER_UNINIT
} EB_error_t;


typedef enum
{
    EB_EXCP_NONE,
    EB_EXCP_UNREGISTER_CMD,
    EB_EXCP_ILLEGAL_DATA,
    EB_EXCP_ACK,
    EB_EXCP_NACK,
    EB_EXCP_SLAVE_BUSY
} EB_exception_t;

typedef EB_exception_t (*func_handler_t)(uint8_t *msg, uint8_t *lenght);



typedef struct
{
    uint8_t cmd;
    func_handler_t cmd_handler;
} EB_msg_cb_handler_t;


typedef struct
{
    uint8_t slave_id;
    uint8_t cmd;
    uint8_t len;
    uint8_t *data;
} EB_msg_t;

EB_error_t EB_slave_init(uint8_t add , uint32_t baudrate);
void EB_poll();
EB_error_t EB_register_command_handlers(uint8_t cmd_id , func_handler_t handler);


EB_error_t EB_send_response(uint8_t slave_address , uint8_t *msg , uint8_t lenght);

void EB_serial_tx_handler();

void EB_serial_rx_handler();

void EB_timer_expired_handler();

#endif /* EASYBUS_MASTER_H_ */