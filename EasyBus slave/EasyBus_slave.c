/*
 * EasyBus_slave.c
 * 
 * Author: Bashar Balloul
 * License: MIT
 * 
 * This file is part of the EasyBus project.
 * Slave-side implementation
 * 
 * Inspired by the FreeModbus project (BSD License) by Christian Walter.
 * While no code has been copied, the state machine structure and design principles 
 * have influenced this implementation.
 */


#include <stddef.h>
#include <util/crc16.h>
#include <stddef.h>
/*********************************** [platform includes] ***********************************/
#include "port.h"
/*********************************** [EB includes] ***********************************/
#include "EasyBus_slave.h"
#include "EasyBus_port.h"

/*********************************** [DEFINES] ***********************************/
#define EB_MIN_CMD		(1)
#define EB_MAX_CMD		(127)


#include "seven_segment.h"




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
    EV_MSG_RECEIVED,
    EV_EXECUTE_CMD,
    EV_RESPONSE_SENT
} event_t;


/*********************************** [static variables] ***********************************/
/*connection management vars */
static volatile TX_STATE_T tx_state = STATE_TX_IDLE;
static volatile  RX_STATE_T rx_state = STATE_RX_INIT;


/*Serial buffer vars*/
static volatile uint16_t tx_frame_lenght;
static volatile uint16_t index;
static volatile  uint8_t EB_buffer[EB_BUFFER_MAX_SIZE];
static volatile uint8_t *buffer_ptr;

/*Notifications callbacks buffer*/
static EB_msg_cb_handler_t function_handlers[EB_NUM_OF_CMD];

/*events management vars*/
static event_t stored_event;
static BOOL	   is_there_event;


static uint8_t EB_slave_address;

/*********************************** [LOCAL FUNCTIONS] ***********************************/

BOOL EB_event_post(event_t event);
BOOL EB_event_get(event_t *event);
EB_error_t EB_send_error_response(uint8_t slave_address , EB_exception_t exception);




EB_error_t EB_slave_init(uint8_t add , uint32_t baudrate)
{
    uint32_t timeout35_10us;

    /*set slave address*/
    EB_slave_address = add;

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

    /*init timer*/
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
EB_error_t EB_register_command_handlers(uint8_t cmd_id , func_handler_t handler)
{
    /*check if the cmd id is in valid range*/
    EB_error_t ret = EB_ER_NOE;
    uint8_t i;
    if (0 <= cmd_id && cmd_id <= EB_MAX_CMD)
    {
        /*its critical section, disable interrupts because we dont want to execute a command or handle command not registered completely*/
        ENTER_CRITICAL_SECTION();
        if (handler != NULL)
        {
            /*we want to register the notify function on first available place*/
            for (i = 0 ; i < EB_NUM_OF_CMD ; i++)
            {
                if (function_handlers[i].cmd_handler == NULL || function_handlers[i].cmd_handler == handler)
                {
                    /*found empty place*/
                    function_handlers[i].cmd = cmd_id;
                    function_handlers[i].cmd_handler = handler;
                    /*now exit*/
                    break;
                }

            }
            ret = EB_ER_NOCMD;
        }
        else
        {
            /*un-register the Notify function and command for the given comman id*/
            for (i = 0 ; i < EB_NUM_OF_CMD ; i++)
            {
                if (function_handlers[i].cmd_handler == NULL)
                {
                    /*found empty place*/
                    function_handlers[i].cmd = 0;
                    function_handlers[i].cmd_handler = NULL;
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

void EB_make_msg(EB_msg_t *msg, uint8_t slave_id , uint8_t cmd , uint8_t *data , uint8_t lenght)
{
    msg->slave_id = slave_id;
    msg->cmd = cmd;
    msg->len = lenght;
    msg->data = data;
}




EB_error_t EB_send_response(uint8_t slave_address , uint8_t *msg , uint8_t lenght)
{

    EB_error_t err = EB_ER_NOE;
    ENTER_CRITICAL_SECTION();
    if (rx_state == STATE_RX_IDLE)
    {
        /*msg is a pointer to the main buffer but here pointing to index 1 , So we must return it to index 0*/
        buffer_ptr = (uint8_t *)msg - 2;
        index = 2;
        buffer_ptr[EB_MSG_SLAVE_ADDRESS_POS] = slave_address;
        /*add the response lenght*/
        index += lenght;

        uint16_t crc16 = EB_port_crc16_compute((uint8_t *)buffer_ptr , index);
        buffer_ptr[index++] = (uint8_t)(crc16 & 0xFF);
        buffer_ptr[index++] = (uint8_t)(crc16 >> 8);

        /*message is ready to sent*/
        tx_state = STATE_TX_TRAN;
        EB_port_serial_enable(TX_ENABLE , RX_DISABLE);
    }
    else
    {
        /*add error codes*/
        err = EB_ER_NOCMD;
    }
    EXIT_CRITICAL_SECTION();
    return err;
}


EB_error_t EB_send_error_response(uint8_t slave_address , EB_exception_t exception)
{
    EB_error_t err = EB_ER_NOE;
    ENTER_CRITICAL_SECTION();
    if (rx_state == STATE_RX_IDLE)
    {
        buffer_ptr = (uint8_t *)EB_buffer;
        buffer_ptr[EB_MSG_SLAVE_ADDRESS_POS] = slave_address;
        buffer_ptr[EB_MSG_COMMAND_POS] |= 128;
        buffer_ptr[EB_MSG_DATA_POS] = exception;
        index = 3;
        uint16_t crc16 = EB_port_crc16_compute((uint8_t *)EB_buffer , 3);
        buffer_ptr[index++] = (uint8_t)(crc16 & 0xFF);
        buffer_ptr[index++] = (uint8_t)(crc16 >> 8);
        /*message is ready to sent*/
        tx_state = STATE_TX_TRAN;
        EB_port_serial_enable(TX_ENABLE , RX_DISABLE);
    }
    else
    {
        /*add error codes*/
        err = EB_ER_NOCMD;
    }
    EXIT_CRITICAL_SECTION();
    return err;
}
/*this function must be called periodically in the main loop */
void EB_poll()
{
   

    static uint8_t cmd;
    static uint8_t received_address;
    static uint8_t *received_message;
    static uint8_t received_message_lenght;
    static EB_exception_t exception;
    event_t cur_event;

    if (EB_event_get(&cur_event) == TRUE)
    {
        /*there is new event handle it*/
        switch (cur_event)
        {
            case EV_READY:
            {

            }

            case EV_MSG_RECEIVED:
            {

                /*check if the message is for us or its a broadcast address */
                ENTER_CRITICAL_SECTION();
                if (EB_buffer[EB_MSG_SLAVE_ADDRESS_POS] == EB_slave_address || EB_buffer[EB_MSG_SLAVE_ADDRESS_POS] == EB_BROADCAST_ADDRESS)
                {


                    /*if so , check CRC*/
                    uint16_t received_crc = (EB_buffer[index - 2]) | ((EB_buffer[index - 1] << 8) & 0xFF00);
                    uint16_t crc_computed = EB_port_crc16_compute((uint8_t *)EB_buffer , index - 2);
                    if (crc_computed == received_crc)
                    {
                        /*CRC ok , execute command*/
                        received_address = EB_buffer[EB_MSG_SLAVE_ADDRESS_POS];
                        cmd = EB_buffer[EB_MSG_COMMAND_POS];
                        /*return the message from command, address is already checked*/
                        received_message = (uint8_t *)&EB_buffer[EB_MSG_DATA_POS];
                        /*exclude the CRC and slave address fields and cmd*/
                        received_message_lenght = index - 4;
                        EB_event_post(EV_EXECUTE_CMD);

                    }
                    else
                    {
                        /*bad CRC , let the master know this*/

                    }
                }
                else
                {
                    /*the message is not for us , ignore it*/

                }

                EXIT_CRITICAL_SECTION();

            }
            break;

            /*wating the response */
            case EV_EXECUTE_CMD:
            {
                exception = EB_EXCP_UNREGISTER_CMD;
                for (uint8_t i = 0 ; i < EB_NUM_OF_CMD ; i++)
                {
                    if (function_handlers[i].cmd ==  0)
                    {
                        break;
                    }
                    else if (function_handlers[i].cmd == cmd)
                    {

                        /*the command is registered , execute it*/
                        exception = function_handlers[i].cmd_handler(received_message, &received_message_lenght);
                        break;
                    }
                }
				/*if it's a broadcast address we cant response*/
                if (received_address != EB_BROADCAST_ADDRESS)
                {
                    if (exception != EB_EXCP_NONE)
                    {
                        /*if an exception occurred , then send a error response*/
                        EB_send_error_response(EB_slave_address , exception);
                    }
                    else
                    {
                        EB_send_response(EB_slave_address , received_message , received_message_lenght);
                    }
                }
            }
            break;


            case EV_RESPONSE_SENT:

                break;
        }

    }

}


//
void EB_serial_tx_handler()
{

    switch (tx_state)
    {
        case STATE_TX_IDLE:
            EB_port_serial_enable(TX_DISABLE , RX_ENABLE);
            break;

        case STATE_TX_TRAN:
            if (index != 0)
            {
                EB_port_serial_put_byte(*buffer_ptr);
                buffer_ptr++;
                index--;
            }
            else
            {
                index = 0;
                /*disable transmitter*/
                EB_event_post(EV_RESPONSE_SENT);
                EB_port_serial_enable(TX_DISABLE , RX_ENABLE);
                tx_state = STATE_TX_IDLE;
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
            EB_event_post(EV_MSG_RECEIVED);
            break;

        case  STATE_RX_ERROR:
            ;

            break;
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

