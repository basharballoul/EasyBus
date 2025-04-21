/*
 * EasyBus_master.c
 * 
 * EasyBus Protocol - Master-side implementation with command queueing
 *
 * Author: Bashar Balloul
 * License: MIT
 *
 * This file is part of the EasyBus project.
 */

/*********************************** [system includes] ***********************************/
#include <stddef.h>
/*********************************** [platform includes] ***********************************/
#include "port.h"
/*********************************** [EB includes] ***********************************/
#include "EasyBus_master.h"
#include "EasyBus_port.h"

/*********************************** [DEFINES] ***********************************/
#define EB_MIN_CMD		(1)
#define EB_MAX_CMD		(127)


/*********************************** [TYPE DEFINETIONS] ***********************************/
typedef struct
{
    EB_msg_t buffer[EB_MSG_FIFO_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} msg_fifo_t;

typedef enum
{
    STATE_TX_IDLE,
    STATE_TX_TRAN,
} TX_STATE_T;

typedef enum
{
    STATE_RX_INIT,
    STATE_RX_IDLE,
    STATE_RX_RECEIVE,
    STATE_RX_ERROR
} RX_STATE_T;

typedef enum
{
    EV_READY,
    EV_MSG_SENT,
    EV_WATING_RESPONSE,
    EV_RESPONSE_RECV,
    EV_RESPONSE_TIMEOUT
} event_t;



/*********************************** [static variables] ***********************************/

/*connection management vars */
static BOOL			is_connection_free = TRUE;
static volatile		TX_STATE_T tx_state = STATE_TX_IDLE;
static volatile		RX_STATE_T rx_state = STATE_RX_IDLE;
static uint32_t		t_response_timeout;
static BOOL			wating_response = FALSE;
static EB_msg_t		msg_to_send;
static BOOL volatile		broadcast_message;

/*Notifications callbacks buffer*/
static msg_notify_cb_t notify_functions_callbacks[EB_NUM_OF_CMD];;

/*events management vars*/
static BOOL			is_there_event;
static event_t		stored_event;


/*Serial buffer vars*/
static volatile uint8_t EB_buffer[EB_BUFFER_MAX_SIZE];
static volatile uint16_t tx_frame_lenght;
static volatile uint16_t index;

/*messages buffer*/
static msg_fifo_t msg_fifo_buffer;

/*********************************** [LOCAL FUNCTIONS] ***********************************/


BOOL		EB_event_post(event_t event);
BOOL		EB_event_get(event_t *event);
EB_error_t	EB_get_queue_state();


EB_error_t EB_master_init(uint32_t baudrate)
{
    uint32_t timeout35_10us;
    EB_port_Serial_init(baudrate);

    if (baudrate > 19200)
    {
        /*
        	1750us / 10us = 175
        */
        timeout35_10us = 175;
    }
    else
    {
        /*each char use 10 bit (start and data and stop)*/

        timeout35_10us = (10UL * 700000UL) / (baudrate * 2UL) ;
    }
	
    if (EB_port_timer_init(timeout35_10us) == FALSE)
        return EB_ER_TIMER_UNINIT;

    tx_state = STATE_TX_IDLE;
    is_there_event = FALSE;
    EB_port_timer_enable();
    EB_port_serial_enable(TX_DISABLE , RX_ENABLE);
    return EB_ER_NOE;
}

/*
	register a Notify Function for a specific command, the Notify Function will be called when a message with the iven command arrive
*/
EB_error_t EB_register_notify_func(uint8_t cmd_id , notify_func_t handler)
{	
    /*check if the cmd id is in valid range*/
    EB_error_t ret = EB_ER_NOE;
    uint8_t i;
    if (EB_MIN_CMD < cmd_id && cmd_id <= EB_MAX_CMD)
    {
        /*its critical section, disable interrupts because we dont want to execute a command or handle command not registered completely*/
        ENTER_CRITICAL_SECTION();
        if (handler != NULL)
        {
            /*we want to register the notify function on first available place*/
            for (i = 0 ; i < EB_NUM_OF_CMD ; i++)
            {
                if (notify_functions_callbacks[i].notify_func == NULL || notify_functions_callbacks[i].notify_func == handler)
                {
                    /*found empty place*/
                    notify_functions_callbacks[i].cmd = cmd_id;
                    notify_functions_callbacks[i].notify_func = handler;
                    /*now exit*/
                    break;
                }
            }
            ret = EB_ER_NOCMD;
        }
        else
        {
            /*un-register the Notify function and command for the given command id*/
            for (i = 0 ; i < EB_NUM_OF_CMD ; i++)
            {
                if (notify_functions_callbacks[i].notify_func == NULL)
                {
                    /*found empty place*/
                    notify_functions_callbacks[i].cmd = 0;
                    notify_functions_callbacks[i].notify_func = NULL;
                    /*now exit*/
                    break;
                }
            }

            ret = EB_ER_NOCMD;
        }
    }
    else
    {
        /*command id out of range*/
        ret = EB_ER_NOCMD;
    }
    EXIT_CRITICAL_SECTION();
    return ret;
}

void EB_make_msg(EB_msg_t *msg, uint8_t slave_id  , uint8_t cmd , uint8_t *data , uint8_t lenght)
{

    msg->slave_id = slave_id;
    msg->cmd = cmd;
    msg->len = lenght;
    msg->data = data;


}

EB_error_t EB_get_queue_state()
{
    EB_error_t	err = EB_ER_NOE;
    if (msg_fifo_buffer.count == EB_MSG_FIFO_SIZE)
    {
        /*FIFO is full*/
        err = EB_ER_QUEUE_FULL;
    }
    else if (msg_fifo_buffer.count == 0)
    {
        /*FIFO empty*/
        err = EB_ER_QUEUE_EMPTY;
    }
    return err;
}

EB_error_t EB_enqueue_msg(EB_msg_t *msg)
{

    if (EB_get_queue_state() == EB_ER_QUEUE_FULL)
    {
        /*FIFO is full*/
        return EB_ER_QUEUE_FULL;
    }
    /*insert the msg*/
    msg_fifo_buffer.buffer[msg_fifo_buffer.head] = *msg;
    /*move head*/
    msg_fifo_buffer.head = msg_fifo_buffer.head + 1;

    if (msg_fifo_buffer.head == EB_MSG_FIFO_SIZE)
    {
        /*if the head reach the end of the buffer, wrap it to start*/
        msg_fifo_buffer.head  = 0;
    }
    /*increment msg counter*/
    msg_fifo_buffer.count++;
    return EB_ER_NOE;

}


EB_error_t EB_dequeue_msg(EB_msg_t *msg)
{

    if (EB_get_queue_state() == EB_ER_QUEUE_EMPTY)
    {
        /*buffer is empty*/
        return EB_ER_QUEUE_EMPTY;
    }
    /*get the the first msg is fifo*/
    *msg = msg_fifo_buffer.buffer[msg_fifo_buffer.tail];
    /*move tail forward by 1*/
    msg_fifo_buffer.tail = msg_fifo_buffer.tail + 1;

    if (msg_fifo_buffer.tail == EB_MSG_FIFO_SIZE)
    {
        msg_fifo_buffer.tail = 0;
    }

    msg_fifo_buffer.count--;
    return EB_ER_NOE;

}

void rs485_send_msg(EB_msg_t *msg)
{
    ENTER_CRITICAL_SECTION();
    if (tx_state == STATE_TX_IDLE)
    {
        EB_buffer[EB_MSG_SLAVE_ADDRESS_POS] = msg->slave_id;
        EB_buffer[EB_MSG_COMMAND_POS]	= msg->cmd;
        if (msg->data != NULL)
        {
            /*copy the payload the serial buffer */
            for (int i = 0 ; i < msg->len ; i++)
            {
                EB_buffer[i + EB_MSG_DATA_POS] = msg->data[i];	//4 5
            }
        }
        /*calculate the CRC16 of the message*/
        int16_t crc16 = EB_port_crc16_compute((uint8_t *)EB_buffer, msg->len + 2);

        EB_buffer[msg->len + 2] = (uint8_t)(crc16 & 0x00FF);
        EB_buffer[msg->len + 1 + 2] = (uint8_t)(crc16 >> 8);

        index = 0;
        /*total lenght of the serial buffer*/
        tx_frame_lenght  = msg->len + 4;
        EB_port_serial_enable(TX_ENABLE,  RX_DISABLE);
        tx_state = STATE_TX_TRAN;
    }
    EXIT_CRITICAL_SECTION();
}


/*this function must be called periodically in the main loop */
void EB_poll()
{
    static uint8_t *received_message;
    static uint8_t received_message_lenght;
    event_t cur_event;


    if (EB_get_queue_state() != EB_ER_QUEUE_EMPTY)
    {
        /*there is something is queue*/
        if (rx_state == STATE_RX_IDLE && is_connection_free == TRUE)
        {
            /*if we are not receiving and there is no connection (not waiting any response)*/
            /*trigger an event to send*/
            EB_event_post(EV_READY);
        }
    }
    if (EB_event_get(&cur_event) == TRUE)
    {
        /*there is new event handle it*/
        switch (cur_event)
        {
            case EV_READY:
            {
                if (EB_dequeue_msg(&msg_to_send) != EB_ER_QUEUE_EMPTY)
                {
                    wating_response = FALSE;
                    is_connection_free = FALSE;
                    rs485_send_msg(&msg_to_send);
                }
                else
                {
                    /*FIFO is empty, nothing to send*/
                }
                break;
            }
            /*Msg sent */
            case EV_MSG_SENT:
                /*if it is a broadcast message then dont wait a response*/
                if (msg_to_send.slave_id == 0x00)
                {

                    broadcast_message = 1;
                    EB_port_timer_enable();
                }
                else
                {
                    EB_event_post(EV_WATING_RESPONSE);
                }

                break;

            /*wating the response */
            case EV_WATING_RESPONSE:
                EB_port_serial_enable(TX_DISABLE , RX_ENABLE);
                wating_response = TRUE;
                rx_state = STATE_RX_IDLE;
                t_response_timeout = millis();
                break;

            /*the response from slave has received*/
            case EV_RESPONSE_RECV:
                ENTER_CRITICAL_SECTION();
                wating_response = FALSE;
                /*check the ID of the slave*/
                if (EB_buffer[EB_MSG_SLAVE_ADDRESS_POS] == msg_to_send.slave_id)
                {
                    /*id match , check the CRC*/
                    uint16_t received_crc = (EB_buffer[index - 2]) | ((EB_buffer[index - 1] << 8) & 0xFF00);
                    uint16_t crc_computed = EB_port_crc16_compute((uint8_t *)EB_buffer , index - 2);
                    if (crc_computed == received_crc)
                    {
                        /*crc match */
                        for (uint8_t i = 0 ; i < EB_NUM_OF_CMD ; i++)
                        {

                            if (notify_functions_callbacks[i].cmd ==  0)
                            {
                                is_connection_free = TRUE;
                                break;
                            }
                            else if (notify_functions_callbacks[i].cmd == (0x7f & EB_buffer[EB_MSG_COMMAND_POS]))
                            {
                                /*the slave address should be passed also , because the APP must know which slave is send the response*/
                                received_message = (uint8_t *)EB_buffer;
                                /*exclude the CRC bytes*/
                                received_message_lenght = index - 2;
                                /*the command is registered , execute it*/
                                notify_functions_callbacks[i].notify_func(received_message, &received_message_lenght);
                                is_connection_free = TRUE;
                                break;
                            }
                        }
                    }
                }
                else
                {
                    /*the slave is address not matched, ignore message*/
                    is_connection_free = TRUE;
                }
                EXIT_CRITICAL_SECTION();
                break;
            case EV_RESPONSE_TIMEOUT:
                wating_response = FALSE;
                /*if timeout elapsed , trigger the notify function for the command and pass NULL as msg*/
                for (uint8_t i = 0 ; i < EB_NUM_OF_CMD ; i++)
                {
                    if (notify_functions_callbacks[i].cmd ==  0)
                    {
                        is_connection_free = TRUE;
                        break;
                    }
                    else if (notify_functions_callbacks[i].cmd == EB_buffer[EB_MSG_COMMAND_POS])
                    {
                        /*pass the original message slave address and command id to upper layer to handle the timeout*/
                        received_message_lenght = 2;
                        EB_buffer[EB_MSG_COMMAND_POS] |= 0x80;
                        EB_buffer[EB_MSG_DATA_POS] = EB_EXCP_SLAVE_NOT_RESPONSING;
                        notify_functions_callbacks[i].notify_func((uint8_t *)EB_buffer, &received_message_lenght);
                        is_connection_free = TRUE;
                        break;
                    }
                }
                break;
        }

    }
    /*check if time out */
    if (millis() - t_response_timeout >= 2000 && is_connection_free == FALSE && wating_response == TRUE)
    {
        t_response_timeout = millis();
        EB_event_post(EV_RESPONSE_TIMEOUT);
    }
}


//
void EB_serial_tx_handler()
{

    switch (tx_state)
    {
        case STATE_TX_IDLE:
            //serial_rx_tx_contorl(TX_DISABLE , RX_ENABLE);
            EB_port_serial_enable(TX_DISABLE , RX_ENABLE);
            break;

        case STATE_TX_TRAN:
            if (tx_frame_lenght != 0)
            {
                EB_port_serial_put_byte(EB_buffer[index]);
                index++;
                tx_frame_lenght--;
            }
            else
            {
                index = 0;
                tx_state = STATE_TX_IDLE;
                EB_event_post(EV_MSG_SENT);
                /*disable transmitter*/
                EB_port_serial_enable(TX_DISABLE , RX_ENABLE);
            }

            break;
    }

}


void EB_serial_rx_handler()
{
    uint8_t chr;
    EB_port_serial_get_byte(&chr);
    switch (rx_state)
    {
        case STATE_RX_INIT:
            EB_port_timer_enable();
            break;

        case STATE_RX_IDLE:
            index = 0;
            EB_buffer[index++] = chr;
            rx_state = STATE_RX_RECEIVE;
            EB_port_timer_enable();
            break;

        case STATE_RX_RECEIVE:
            if (index < 255)
            {
                EB_buffer[index++] = chr;
            }
            else
            {
                rx_state = STATE_RX_ERROR;
            }
            EB_port_timer_enable();
            break;


        case STATE_RX_ERROR:
            EB_port_timer_enable();
            break;
    }
}

void EB_timer_expired_handler()
{
    switch (rx_state)
    {
        case STATE_RX_INIT:
        {
            EB_event_post(EV_READY);
        }
        break;
        case STATE_RX_IDLE:

            break;

        case STATE_RX_RECEIVE:
            EB_event_post(EV_RESPONSE_RECV);
            break;

        case  STATE_RX_ERROR:
            ;

            break;
    }

    if (broadcast_message)
    {
        broadcast_message = 0;
        is_connection_free = TRUE;
        EB_port_serial_enable(TX_DISABLE , RX_ENABLE);
        rx_state = STATE_RX_IDLE;
        EB_event_post(EV_READY);
    }
    EB_port_timer_disable();
    rx_state = STATE_RX_IDLE;
}

BOOL EB_event_post(event_t event)
{
    is_there_event = TRUE;
    stored_event = event;
    return TRUE;
}


BOOL EB_event_get(event_t *event)
{
    BOOL new_event = FALSE;
    if (is_there_event == TRUE)
    {
        *event = stored_event;
        is_there_event = FALSE;
        new_event = TRUE;
    }
    return new_event;
}


