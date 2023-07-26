
#ifndef CURRENTMON_H_
#define CURRENTMON_H_

#include "defines.h"
#include "HAL_ESP32.h"
#include "Rules.h"
#include "settings.h"
#include "modbus.h"

#pragma once

void currentMonInternal_init();
bool CurrentMonitorSetSOC(float newSOC);
bool CurrentMonitorResetDailyAmpHourCounters();
void CurrentMonitorSetBasicSettings(uint16_t shuntmv, uint16_t shuntmaxcur, uint16_t batterycapacity, float fullchargevolt, float tailcurrent, float chargeefficiency);
void CurrentMonitorSetRelaySettingsInternal(currentmonitoring_struct newvalues);
void CurrentMonitorSetRelaySettingsExternal(currentmonitoring_struct newvalues);
void TimeToSoCCalculation();
void CurrentMonitorSetAdvancedSettings(currentmonitoring_struct newvalues);
void ProcessDIYBMSCurrentMonitorRegisterReply(uint8_t length);
void ProcessDIYBMSCurrentMonitorInternal();

void PZEM017_SetShuntType(uint8_t, uint16_t);
void PZEM017_SetDeviceAddress(uint8_t);
void currentMon_ConfigureBasic(uint16_t, uint16_t, uint16_t, float, float, float);
void currentMon_SetSOC(float);
void currentMon_ResetDailyAmpHourCounters();
void currentMon_ConfigureAdvancedExternal(currentmonitoring_struct);
[[noreturn]] void rs485_tx(void *);
[[noreturn]] void rs485_rx(void *);

bool CurrentMonitorCanSOC();


#endif
