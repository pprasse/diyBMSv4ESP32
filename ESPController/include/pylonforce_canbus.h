#ifndef DIYBMS_PYLONFORCE_CANBUS_H_
#define DIYBMS_PYLONFORCE_CANBUS_H_

#include "defines.h"
#include "Rules.h"
#include <driver/twai.h>
#include "currentmon.h"

/* shifts left the '1' over pos times to create a single HIGH bit at location pos. */
//#define BIT(pos) ( 1<<(pos) )

/* Set single bit at pos to '1' by generating a mask
in the proper bit location and ORing x with the mask. */
#define SET_BIT(x, pos) ( (x) |= (BIT(pos)) )


void pylonforce_handle_rx(twai_message_t *);
void pylonforce_handle_tx();


extern uint8_t TotalNumberOfCells();
extern Rules rules;
extern currentmonitoring_struct currentMonitor;
extern diybms_eeprom_settings mysettings;
extern char hostname[16];
extern ControllerState _controller_state;
extern uint32_t canbus_messages_failed_sent;
extern uint32_t canbus_messages_sent;
extern uint32_t canbus_messages_received;

extern void send_ext_canbus_message(uint32_t identifier, uint8_t *buffer, uint8_t length);

#endif