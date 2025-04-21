/*
 * EasyBus_master.h
 * 
 * EasyBus Protocol - Master-side implementation with command queueing
 *
 * Author: Bashar Balloul
 * License: MIT
 *
 * This file is part of the EasyBus project.
 */

#ifndef EASYBUS_MASTER_H_
#define EASYBUS_MASTER_H_


#include <inttypes.h>

/*********************************** [DEFINES] ***********************************/


/*
	DEFINE Max numbers of messages can the messages FIFO store
*/
#define EB_MSG_FIFO_SIZE				5
/*
	DEFINE Number of commands the User will use
*/
#define EB_NUM_OF_CMD					5
/*
	DIFINE Serial buffer Max size
*/
#define EB_BUFFER_MAX_SIZE				64


#define EB_BROADCAST_ADDRESS			0x00

/*
	Message Frame

	****************************************************  ..... ***********************
	|	SLAVE ADDRESS	|  CMD_ID	|	DATA0	| DATA1 |		| DATAN | CRCL | CRCH |
	****************************************************  ..... ***********************
*/
#define EB_MSG_SLAVE_ADDRESS_POS		0
#define EB_MSG_COMMAND_POS				1
#define EB_MSG_DATA_POS					2

#define EB_IS_IT_ERROR_FRAME(MSG)	(( (MSG[EB_MSG_COMMAND_POS]) & 0x80 ) == 0x80 )

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
    EB_EXCP_SLAVE_BUSY,
    EB_EXCP_SLAVE_NOT_RESPONSING
} EB_exception_t;

typedef void (*notify_func_t)(uint8_t *, uint8_t *);



typedef struct
{
    uint8_t cmd;
    notify_func_t notify_func;
} msg_notify_cb_t;


typedef struct
{
    uint8_t slave_id;
    uint8_t cmd;
    uint8_t len;
    uint8_t *data;
} EB_msg_t;

EB_error_t		EB_master_init(uint32_t baudrate);
void			EB_poll();
EB_error_t		EB_register_notify_func(uint8_t cmd_id , notify_func_t handler);
EB_error_t		EB_dequeue_msg(EB_msg_t *msg);
EB_error_t		EB_enqueue_msg(EB_msg_t *msg);
void			EB_make_msg(EB_msg_t *msg, uint8_t slave_id  , uint8_t cmd , uint8_t *data , uint8_t lenght);


/*  */
void EB_serial_tx_handler();

void EB_serial_rx_handler();

void EB_timer_expired_handler();
#endif /* RS485_HANDLER_H_ */