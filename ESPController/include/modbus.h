
#ifndef MODBUS_H_
#define MODBUS_H_

#include "defines.h"
#include "HAL_ESP32.h"
#include "Rules.h"
#include "settings.h"

#pragma once

// Number of bytes of the largest MODBUS request we make
#define MAX_SEND_RS485_PACKET_LENGTH 36

const uart_port_t rs485_uart_num = UART_NUM_1;

void ConfigureRS485();
void SetupRS485();

[[noreturn]] void service_rs485_transmit_q(void *);


uint16_t calculateModBusCRC(const uint8_t*, uint8_t);
uint8_t SetMobusRegistersFromFloat(uint8_t *cmd, uint8_t ptr, float value);

#endif