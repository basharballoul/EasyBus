
/*
 * EasyBus_port.h
 * 
 * Platform Abstraction Layer for EasyBus (hardware-dependent code)
 *
 * Author: Bashar Balloul
 * License: MIT
 *
 * This file is part of the EasyBus project.
 *
 * Implement these functions according to your hardware (UART, timers, CRC).
 */



#ifndef EASYBUS_PORT_H_
#define EASYBUS_PORT_H_




typedef enum
{
	RX_DISABLE,
	RX_ENABLE
}RX_CONT_t;

typedef enum
{
	TX_DISABLE,
	TX_ENABLE
}TX_CONT_t;


uint16_t EB_port_crc16_compute(uint8_t *data, uint8_t len);

void EB_port_Serial_init(uint32_t baudrate);

void EB_port_serial_get_byte(uint8_t *byte);

void EB_port_serial_put_byte(uint8_t byte);

void EB_port_serial_enable(TX_CONT_t tx_control, RX_CONT_t rx_control);

extern uint32_t EB_millis();

BOOL EB_port_timer_init(uint32_t timeout35_10us);

void EB_port_timer_enable();

void EB_port_timer_disable();

extern uint32_t millis();

#endif /* EASYBUS_PORT_H_ */