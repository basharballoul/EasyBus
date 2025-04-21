 

**EasyBus** is a custom-designed, efficient, and lightweight communication protocol built specifically for embedded systems. Inspired by **Modbus RTU** in structure , EasyBus supports both **Master** and **Slave** roles, with built-in support for **message queuing**, **command handler registration**, and **exception handling**.

## Features

 * Full support for Master-Slave communication
 * Clean state machine for RX/TX + timers
 * Interrupt-based serial communication
 * Command handler registration (on slaves for handling incoming commands).
 * Command respones notification callbacks (on the master).
 * Queue-based message transmission (on the master)
 * Exception handling for invalid or unregistered commands
 * Broadcast message support
 * No dynamic memory allocation

## Requirements
* Hardware UART
* Timer peripheral

## Protocol Basics

* #### Addressing:
    * Each slave has a unique 1-byte address
    * The broadcast address is: **0x00** All slaves will process the message, but no response is sent

* #### Command ID Range:

    * Valid command IDs: **0x01** to **0x7F** (1–127)

    * **0x80** and above are reserved for exceptions or internal handling

* #### CRC:

    * EasyBus uses **CRC-16**, little-endian format (low byte first)

* #### Frame Size Limits:

    * Max buffer size is user-defined via **EB_BUFFER_MAX_SIZE**

    * Commands must fit within this limit including CRC and header

*** 

## Message Frame Structure

To help you understand the message layout, here’s the frame format used by EasyBus:

### Normall message:
master request message or slave response message

| Field           | Size (Bytes) | Description                     |
|----------------|--------------|---------------------------------|
| Slave Address   | 1            | Target slave device ID          |
| CMD_ID          | 1            | Command identifier              |
| DATA0...DATAN   | Variable     | Optional payload (0 to N bytes) |
| CRC_L           | 1            | CRC16 checksum (low byte)       |
| CRC_H           | 1            | CRC16 checksum (high byte)      |

--- 

 * Slave Address: 1 byte – the target device's address
 * CMD_ID: 1 byte – the command identifier
 * DATA0 to DATAN: Optional payload
 * CRC_L / CRC_H: 2 bytes – CRC16 checksum (low byte first)

### Error Message:
slave response message when an exception occurs

| Field           | Size (Bytes) | Description                     |
|----------------|--------------|---------------------------------|
| Slave Address   | 1            | Target slave device ID          |
| CMD_ID \| 0x80  | 1            | Command identifier + 0x80             |
| Exception code  | 1            | exception code                   |
| CRC_L           | 1            | CRC16 checksum (low byte)       |
| CRC_H           | 1            | CRC16 checksum (high byte)      |

in the master you can check weither it's and error message or not using the MACRO **`EB_IS_IT_ERROR_FRAME(msg)`**.

## Porting Guide
You must implement these functions in your platform layer:
#### Function prototypes for serial and timing
``` C
/*file : EasyBus_port.h*/

uint16_t EB_port_crc16_compute(uint8_t *pucFrame, uint8_t usLen);

void EB_port_Serial_init(uint32_t baudrate);

void EB_port_serial_get_byte(uint8_t *byte);

void EB_port_serial_put_byte(uint8_t byte);

void EB_port_serial_enable(TX_CONT_t tx_control, RX_CONT_t rx_control);

BOOL EB_port_timer_init(uint32_t timeout35_10us);

void EB_port_timer_enable();

void EB_port_timer_disable();

```

#### Interrupts callbacks

``` C 
/*
    this functions is used in porting
*/

/*data resgister empty interrupt callback*/
void EB_serial_tx_handler();
/*new byte received interrupt callback*/
void EB_serial_rx_handler();
/*timer expired interrupt callback */
void EB_timer_expired_handler();
```
See the **examples** folder — we've included a **ready-to-use** porting example.

## How It Works

### Master mode

#### 1. Configuration
Before using **EasyBus** in master mode , you must define a few constants to configure the protocol for your project. These are  placed in **EasyBus_master.h**.

``` C
/*file: EasyBus_master.h*/
/*
* size of the message Queue 
*/

#define EB_MSG_FIFO_SIZE				5
/*
 * Number of commands (Notificatios) you plan to register.
 */
#define EB_NUM_OF_CMD 5

/*
 * Size of the serial communication buffer.
 * This should be large enough to hold the biggest expected message.
 */
#define EB_BUFFER_MAX_SIZE 64
```
#### 2. Main Functions

#### Initialization

Initializes the EasyBus slave with a specific **baudrate**.
``` C
EB_error_t EB_master_init(uint32_t baudrate);
```
#### Polling 

``` C
void EB_poll();
```
This function must called inside the **super loop**.

#### sending message
``` C
/*
make a message for specific slave_id with cmd and the *data payload is optinal. 
*/
void EB_make_msg(EB_msg_t *msg, uint8_t slave_id  , uint8_t cmd , uint8_t *data , uint8_t lenght);

/*
returns EB_ER_QUEUE_FULL if the queue is full , or EB_ER_NOE if the enqueue done.
*/
EB_error_t EB_enqueue_msg(EB_msg_t *msg);
```
see the example for more clear Usage.

#### commands response notification registration

``` C
/*
    register a notify function for thr cmd_id, these notify function will trigger whatever slave is responded.
    
    return EB_ER_NOE if registration in done else return EB_ER_NOCMD
*/
EB_error_t EB_register_notify_func(uint8_t cmd_id , notify_func_t handler);
```

#### Notification function usage
Notification in declared like this:

``` C
void command_notify_name(uint8_t *msg , uint8_t *lenght);
```
where:

* *msg is a pointer to the response received by the slave including the slave address and command id.
* *lenght is a pointer to the response lenght.

here is a example how you can use it:

``` C
void command_notify_name(uint8_t *msg , uint8_t len)
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
		
		}
	}
}
```

**Important Note**: msg points to the EasyBus serial buffer. If you need to keep the data locally, you must copy it, because it will be overwritten when another response arrives.

### Slave mode

#### 1. Configuration
Before using EasyBus in slave mode , you must define a few constants to configure the protocol for your project. These are  placed in EasyBus_slave.h.

``` C
/*file: EasyBus_slave.h*/

/*
 * Number of commands (handlers) you plan to register.
 * A value of 5 allows up to 5 different commands to be handled.
 */
#define EB_NUM_OF_CMD 5

/*
 * Size of the serial communication buffer.
 * This should be large enough to hold the biggest expected message.
 */
#define EB_BUFFER_MAX_SIZE 64
```

#### 2. Main Functions

#### Initialization

Initializes the EasyBus slave with a specific baudrate and slave address.
``` C
EB_error_t EB_slave_init(uint8_t add , uint32_t baudrate);
```
#### Polling 

``` C
void EB_poll();
```
This function must called inside the super loop.

#### Commands response handlers registration

``` C
/*
    register a handlers function for thr cmd_id, these notify function will trigger whatever slave is responded.
    
    return EB_ER_NOE if registration in done else return EB_ER_NOCMD
*/
EB_error_t EB_register_command_handlers(uint8_t cmd_id , func_handler_t handler);

```

#### Commands handlers function usage

Command handlers in declared like this:

``` C
EB_exception_t command_handler_name(uint8_t *msg , uint8_t *lenght);
```
where:

* *msg is a pointer to the command received by the master. the slave address and command id is not included.
* *lenght is a pointer to the command message lenght. also the slave address and command id is not included. 
* the handlers must return an exception type, if it return **EB_EXCP_NONE** a normall response will sent otherwise will return a error response with the exception code to the master.

here is a example how you can use it:

``` C
EB_exception_t cmd_1_command_handler(uint8_t *msg , uint8_t *lenght)
{

	msg[0] = 5;	// dummy data
	/*lenght of the response*/
	*lenght = 1;
	return EB_EXCP_NONE;
}
```
**Imortant Note:** **msg points to the EasyBus serial buffer. If you need to keep the data locally, you must copy it, because it will be overwritten when another response arrives.** 

### License

This project is licensed under the MIT License. Feel free to use, modify, and distribute it in personal or commercial projects.

Developed by **Bashar Balloul**
Inspired by FreeModbus (BSD), but entirely custom and independently designed.

