/*
 ____  ____  _  _  ____  __  __  ___
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
 )(_) )_)(_  \  /  ) _ < )    ( \__ \
(____/(____) (__) (____/(_/\/\_)(___/

  (c) 2023 Patrick Prasse

This code communicates with various current/voltage sensors using RS485 MODBUS RTU
*/

#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "modbus";

#include "currentmon.h"
#include "CurrentMonitorINA229.h"

extern HAL_ESP32 hal;
extern diybms_eeprom_settings mysettings;
extern currentmonitoring_struct currentMonitor;
extern uint8_t frame[256];
extern TaskHandle_t rs485_tx_task_handle;
extern TaskHandle_t rs485_rx_task_handle;
extern TaskHandle_t service_rs485_transmit_q_task_handle;
extern QueueHandle_t rs485_transmit_q_handle;

extern bool _tft_screen_available;
extern TaskHandle_t updatetftdisplay_task_handle;

CurrentMonitorINA229 currentmon_internal = CurrentMonitorINA229();

uint32_t time100 = 0;
uint32_t time20 = 0;
uint32_t time10 = 0;


void currentMonInternal_init()
{
    if (currentmon_internal.Initialise(hal.VSPI_Ptr(), INA229_CHIPSELECT))
    {
      ESP_LOGI(TAG, "Onboard/internal current monitoring chip available");

      currentmon_internal.Configure(
          mysettings.currentMonitoring_shuntmv,
          mysettings.currentMonitoring_shuntmaxcur,
          mysettings.currentMonitoring_batterycapacity,
          mysettings.currentMonitoring_fullchargevolt,
          mysettings.currentMonitoring_tailcurrent,
          mysettings.currentMonitoring_chargeefficiency,
          mysettings.currentMonitoring_shuntcal,
          mysettings.currentMonitoring_temperaturelimit,
          mysettings.currentMonitoring_overvoltagelimit,
          mysettings.currentMonitoring_undervoltagelimit,
          mysettings.currentMonitoring_overcurrentlimit,
          mysettings.currentMonitoring_undercurrentlimit,
          mysettings.currentMonitoring_overpowerlimit,
          mysettings.currentMonitoring_shunttempcoefficient,
          mysettings.currentMonitoring_tempcompenabled);

      currentmon_internal.GuessSOC();

      currentmon_internal.TakeReadings();
    }
    else
    {
      ESP_LOGI(TAG, "Onboard/internal current monitoring chip not installed");
    }
}


bool CurrentMonitorSetSOC(float newSOC)
{
  if (mysettings.currentMonitoringEnabled == true)
  {
    ESP_LOGI(TAG, "Set SOC");
    if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
    {
      currentMon_SetSOC(newSOC);
      return true;
    }
    else if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)
    {
      auto value = (uint16_t)(newSOC * 100);
      currentmon_internal.SetSOC(value);
      return true;
    }
  }

  return false;
}

bool CurrentMonitorResetDailyAmpHourCounters()
{
  if (mysettings.currentMonitoringEnabled == true)
  {
    ESP_LOGI(TAG, "Reset daily Ah counter");
    if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
    {
      currentMon_ResetDailyAmpHourCounters();
      return true;
    }
    else if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)
    {
      currentmon_internal.ResetDailyAmpHourCounters();
      return true;
    }
  }
  return false;
}

void CurrentMonitorSetBasicSettings(uint16_t shuntmv, uint16_t shuntmaxcur, uint16_t batterycapacity, float fullchargevolt, float tailcurrent, float chargeefficiency)
{

  if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
  {
    currentMon_ConfigureBasic(shuntmv, shuntmaxcur, batterycapacity, fullchargevolt, tailcurrent, chargeefficiency);
  }
  else if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)
  {
    mysettings.currentMonitoring_shuntmv = shuntmv;
    mysettings.currentMonitoring_shuntmaxcur = shuntmaxcur;
    mysettings.currentMonitoring_batterycapacity = batterycapacity;
    mysettings.currentMonitoring_fullchargevolt = (uint16_t)(100.0 * fullchargevolt);
    mysettings.currentMonitoring_tailcurrent = (uint16_t)(100.0 * tailcurrent);
    mysettings.currentMonitoring_chargeefficiency = (uint16_t)(100.0 * chargeefficiency);
    // Reset stored shuntcal to zero to allow automatic calibration
    // this is updated by the CurrentMonitorSetAdvancedSettings call
    mysettings.currentMonitoring_shuntcal = 0;
    ValidateConfiguration(&mysettings);
    SaveConfiguration(&mysettings);

    if (hal.GetVSPIMutex())
    {
      currentmon_internal.Configure(
          mysettings.currentMonitoring_shuntmv,
          mysettings.currentMonitoring_shuntmaxcur,
          mysettings.currentMonitoring_batterycapacity,
          mysettings.currentMonitoring_fullchargevolt,
          mysettings.currentMonitoring_tailcurrent,
          mysettings.currentMonitoring_chargeefficiency,
          mysettings.currentMonitoring_shuntcal,
          mysettings.currentMonitoring_temperaturelimit,
          mysettings.currentMonitoring_overvoltagelimit,
          mysettings.currentMonitoring_undervoltagelimit,
          mysettings.currentMonitoring_overcurrentlimit,
          mysettings.currentMonitoring_undercurrentlimit,
          mysettings.currentMonitoring_overpowerlimit,
          mysettings.currentMonitoring_shunttempcoefficient,
          mysettings.currentMonitoring_tempcompenabled);

      hal.ReleaseVSPIMutex();
    }
  }
  else if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::PZEM_017)
  {
    PZEM017_SetDeviceAddress(mysettings.currentMonitoringModBusAddress);
    PZEM017_SetShuntType(mysettings.currentMonitoringModBusAddress, shuntmaxcur);
  }
}

// Save the current monitor advanced settings back to the internal device
void CurrentMonitorSetRelaySettingsInternal(currentmonitoring_struct newvalues)
{
  // Internal current shunt doesn't support any of the relay trigger values
  // so only TempCompEnabled is stored
  if (hal.GetVSPIMutex())
  {
    mysettings.currentMonitoring_tempcompenabled = newvalues.TempCompEnabled;

    currentmon_internal.Configure(
        mysettings.currentMonitoring_shuntmv,
        mysettings.currentMonitoring_shuntmaxcur,
        mysettings.currentMonitoring_batterycapacity,
        mysettings.currentMonitoring_fullchargevolt,
        mysettings.currentMonitoring_tailcurrent,
        mysettings.currentMonitoring_chargeefficiency,
        mysettings.currentMonitoring_shuntcal,
        mysettings.currentMonitoring_temperaturelimit,
        mysettings.currentMonitoring_overvoltagelimit,
        mysettings.currentMonitoring_undervoltagelimit,
        mysettings.currentMonitoring_overcurrentlimit,
        mysettings.currentMonitoring_undercurrentlimit,
        mysettings.currentMonitoring_overpowerlimit,
        mysettings.currentMonitoring_shunttempcoefficient,
        mysettings.currentMonitoring_tempcompenabled);

    mysettings.currentMonitoring_tempcompenabled = currentmon_internal.calc_tempcompenabled();

    hal.ReleaseVSPIMutex();
  }

  ValidateConfiguration(&mysettings);
  SaveConfiguration(&mysettings);
}

// Save the current monitor advanced settings back to the device over MODBUS/RS485
void CurrentMonitorSetRelaySettingsExternal(currentmonitoring_struct newvalues)
{
  uint8_t flag1 = 0;
  uint8_t flag2 = 0;

  flag1 += newvalues.TempCompEnabled ? B00000010 : 0;

  // Use the previous value for setting the ADCRange4096mV flag
  flag1 += currentMonitor.ADCRange4096mV ? B00000001 : 0;

  // Apply new settings
  flag2 += newvalues.RelayTriggerTemperatureOverLimit ? bit(DIAG_ALRT_FIELD::TMPOL) : 0;
  flag2 += newvalues.RelayTriggerCurrentOverLimit ? bit(DIAG_ALRT_FIELD::SHNTOL) : 0;
  flag2 += newvalues.RelayTriggerCurrentUnderLimit ? bit(DIAG_ALRT_FIELD::SHNTUL) : 0;
  flag2 += newvalues.RelayTriggerVoltageOverlimit ? bit(DIAG_ALRT_FIELD::BUSOL) : 0;
  flag2 += newvalues.RelayTriggerVoltageUnderlimit ? bit(DIAG_ALRT_FIELD::BUSUL) : 0;
  flag2 += newvalues.RelayTriggerPowerOverLimit ? bit(DIAG_ALRT_FIELD::POL) : 0;

  /*
Flag 1
10|Temperature compensation enabled|Read write
9|ADC Range 0=±163.84 mV, 1=±40.96 mV (only 40.96mV supported by diyBMS)|Read only

Flag 2
8|Relay Trigger on TMPOL|Read write
7|Relay Trigger on SHNTOL|Read write
6|Relay Trigger on SHNTUL|Read write
5|Relay Trigger on BUSOL|Read write
4|Relay Trigger on BUSUL|Read write
3|Relay Trigger on POL|Read write
*/

  uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];
  memset(&cmd, 0, sizeof(cmd));

  //	Write Multiple Holding Registers

  // The Slave Address
  cmd[0] = mysettings.currentMonitoringModBusAddress;
  // The Function Code 16
  cmd[1] = 16;
  // Data Address of the first register (9=40010, Various status flags)
  cmd[2] = 0;
  cmd[3] = 9;
  // number of registers to write
  cmd[4] = 0;
  cmd[5] = 1;
  // number of data bytes to follow (2 registers x 2 bytes each = 4 bytes)
  cmd[6] = 2;
  // value to write to register 40010
  cmd[7] = flag1;
  cmd[8] = flag2;

  xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);

  ESP_LOGD(TAG, "Write register 10 = %u %u", flag1, flag2);

  // Zero all data
  // memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));
  currentMonitor.validReadings = false;
}

// Calculate estimated time to various % SoC
void TimeToSoCCalculation()
{
  // Ideally this current value would be averaged over the past few minutes
  float current = currentMonitor.modbus.current;

  time20 = 0;
  time10 = 0;
  time100 = 0;

  // Avoid divide by zero errors
  if (current == 0)
    return;

  // Calculate how "full" the battery is based on SoC %
  float now_capacity_ah = currentMonitor.modbus.batterycapacityamphour / 100.0F * currentMonitor.stateofcharge;

  // Target 100% - only if we are charging
  if (currentMonitor.stateofcharge < 100.0F && current > 0)
  {
    // Gap between now and 100%
    float empty_ah = currentMonitor.modbus.batterycapacityamphour - now_capacity_ah;
    // Use instantaneous current value to predict amp-hour (in number of seconds)
    time100 = (uint32_t)((empty_ah / abs(current)) * 60 * 60);
  }
  else
  {
    time100 = 0;
  }

  // Target 20% - only if we are discharging
  if (currentMonitor.stateofcharge > 20.0 && current < 0)
  {
    // Gap between now and 20%
    float empty_ah = now_capacity_ah - (currentMonitor.modbus.batterycapacityamphour * 0.20F);
    time20 = (uint32_t)((empty_ah / abs(current)) * 60 * 60);
  }
  else
  {
    time20 = 0;
  }

  // Target 10% - only if we are discharging
  if (currentMonitor.stateofcharge > 10.0 && current < 0)
  {
    // Gap between now and 10%
    float empty_ah = now_capacity_ah - (currentMonitor.modbus.batterycapacityamphour * 0.10F);
    time10 = (empty_ah / abs(current)) * 60 * 60;
  }
  else
  {
    time10 = 0;
  }

  // Sensible maximums to avoid silly timespans - limit anything over 10 days remaining...
  const uint32_t limit = 10 * 86400U;

  if (time100 > limit)
  {
    time100 = 0;
  }
  if (time20 > limit)
  {
    time20 = 0;
  }
  if (time10 > limit)
  {
    time10 = 0;
  }
}


// Save the current monitor advanced settings back to the device over MODBUS/RS485
void CurrentMonitorSetAdvancedSettings(currentmonitoring_struct newvalues)
{

  if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
  {
    currentMon_ConfigureAdvancedExternal(newvalues);
  }
  else if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)
  {
    mysettings.currentMonitoring_shuntcal = newvalues.modbus.shuntcal;
    mysettings.currentMonitoring_temperaturelimit = newvalues.modbus.temperaturelimit;
    mysettings.currentMonitoring_overvoltagelimit = 100 * newvalues.modbus.overvoltagelimit;
    mysettings.currentMonitoring_undervoltagelimit = 100 * newvalues.modbus.undervoltagelimit;
    mysettings.currentMonitoring_overcurrentlimit = 100 * newvalues.modbus.overcurrentlimit;
    mysettings.currentMonitoring_undercurrentlimit = 100 * newvalues.modbus.undercurrentlimit;
    mysettings.currentMonitoring_overpowerlimit = newvalues.modbus.overpowerlimit;
    mysettings.currentMonitoring_shunttempcoefficient = newvalues.modbus.shunttempcoefficient;
    ValidateConfiguration(&mysettings);
    SaveConfiguration(&mysettings);

    if (hal.GetVSPIMutex())
    {
      currentmon_internal.Configure(
          mysettings.currentMonitoring_shuntmv,
          mysettings.currentMonitoring_shuntmaxcur,
          mysettings.currentMonitoring_batterycapacity,
          mysettings.currentMonitoring_fullchargevolt,
          mysettings.currentMonitoring_tailcurrent,
          mysettings.currentMonitoring_chargeefficiency,
          mysettings.currentMonitoring_shuntcal,
          mysettings.currentMonitoring_temperaturelimit,
          mysettings.currentMonitoring_overvoltagelimit,
          mysettings.currentMonitoring_undervoltagelimit,
          mysettings.currentMonitoring_overcurrentlimit,
          mysettings.currentMonitoring_undercurrentlimit,
          mysettings.currentMonitoring_overpowerlimit,
          mysettings.currentMonitoring_shunttempcoefficient,
          mysettings.currentMonitoring_tempcompenabled);
      hal.ReleaseVSPIMutex();
    }
  }

  ESP_LOGD(TAG, "Advanced save settings");

  // Zero all data
  memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));
  currentMonitor.validReadings = false;
}

// Swap the two 16 bit words in a 32bit word
static inline unsigned int word16swap32(unsigned int __bsx)
{
  return ((__bsx & 0xffff0000) >> 16) | ((__bsx & 0x0000ffff) << 16);
}

// Extract the current monitor MODBUS registers into our internal STRUCTURE variables
void ProcessDIYBMSCurrentMonitorRegisterReply(uint8_t length)
{
  // ESP_LOGD(TAG, "Modbus len=%i, struct len=%i", length, sizeof(currentmonitor_raw_modbus));

  // ESP_LOG_BUFFER_HEXDUMP(TAG, &frame[3], length, esp_log_level_t::ESP_LOG_DEBUG);

  if (sizeof(currentmonitor_raw_modbus) != length)
  {
    // Abort if the packet sizes are different
    memset(&currentMonitor.modbus, 0, sizeof(currentmonitor_raw_modbus));
    currentMonitor.validReadings = false;
    return;
  }

  // Now byte swap to align to ESP32 endiness, and copy as we go into new structure
  auto *ptr = (uint8_t *)&currentMonitor.modbus;
  for (size_t i = 0; i < length; i += 2)
  {
    uint8_t temp = frame[3 + i];
    ptr[i] = frame[i + 4];
    ptr[i + 1] = temp;
  }

  // Finally, we have to fix the 32 bit fields
  currentMonitor.modbus.milliamphour_out = word16swap32(currentMonitor.modbus.milliamphour_out);
  currentMonitor.modbus.milliamphour_in = word16swap32(currentMonitor.modbus.milliamphour_in);
  currentMonitor.modbus.daily_milliamphour_out = word16swap32(currentMonitor.modbus.daily_milliamphour_out);
  currentMonitor.modbus.daily_milliamphour_in = word16swap32(currentMonitor.modbus.daily_milliamphour_in);
  currentMonitor.modbus.firmwareversion = word16swap32(currentMonitor.modbus.firmwareversion);
  currentMonitor.modbus.firmwaredatetime = word16swap32(currentMonitor.modbus.firmwaredatetime);

  // ESP_LOG_BUFFER_HEXDUMP(TAG, &currentMonitor.modbus, sizeof(currentmonitor_raw_modbus), esp_log_level_t::ESP_LOG_DEBUG);

  currentMonitor.timestamp = esp_timer_get_time();

  // High byte
  auto flag1 = (uint8_t)(currentMonitor.modbus.flags >> 8);
  // Low byte
  auto flag2 = (uint8_t)(currentMonitor.modbus.flags);

  // ESP_LOGD(TAG, "Read relay trigger settings %u %u", flag1, flag2);

  /*
16|TMPOL|Read only
15|SHNTOL|Read only
14|SHNTUL|Read only
13|BUSOL|Read only
12|BUSUL|Read only
11|POL|Read only
10|Temperature compensation enabled|Read write
9|ADC Range 0=±163.84 mV, 1=±40.96 mV (only 40.96mV supported by diyBMS)|Read only
*/

  currentMonitor.TemperatureOverLimit = flag1 & bit(DIAG_ALRT_FIELD::TMPOL);
  currentMonitor.CurrentOverLimit = flag1 & bit(DIAG_ALRT_FIELD::SHNTOL);
  currentMonitor.CurrentUnderLimit = flag1 & bit(DIAG_ALRT_FIELD::SHNTUL);
  currentMonitor.VoltageOverlimit = flag1 & bit(DIAG_ALRT_FIELD::BUSOL);
  currentMonitor.VoltageUnderlimit = flag1 & bit(DIAG_ALRT_FIELD::BUSUL);
  currentMonitor.PowerOverLimit = flag1 & bit(DIAG_ALRT_FIELD::POL);

  currentMonitor.TempCompEnabled = flag1 & B00000010;
  currentMonitor.ADCRange4096mV = flag1 & B00000001;

  /*
8|Relay Trigger on TMPOL|Read write
7|Relay Trigger on SHNTOL|Read write
6|Relay Trigger on SHNTUL|Read write
5|Relay Trigger on BUSOL|Read write
4|Relay Trigger on BUSUL|Read write
3|Relay Trigger on POL|Read write
2|Existing Relay state (0=off)|Read write
1|Factory reset bit (always 0 when read)|Read write
*/
  currentMonitor.RelayTriggerTemperatureOverLimit = flag2 & bit(DIAG_ALRT_FIELD::TMPOL);
  currentMonitor.RelayTriggerCurrentOverLimit = flag2 & bit(DIAG_ALRT_FIELD::SHNTOL);
  currentMonitor.RelayTriggerCurrentUnderLimit = flag2 & bit(DIAG_ALRT_FIELD::SHNTUL);
  currentMonitor.RelayTriggerVoltageOverlimit = flag2 & bit(DIAG_ALRT_FIELD::BUSOL);
  currentMonitor.RelayTriggerVoltageUnderlimit = flag2 & bit(DIAG_ALRT_FIELD::BUSUL);
  currentMonitor.RelayTriggerPowerOverLimit = flag2 & bit(DIAG_ALRT_FIELD::POL);
  currentMonitor.RelayState = flag2 & B00000010;
  // Last bit is for factory reset (always zero)

  currentMonitor.chargeefficiency = ((float)currentMonitor.modbus.raw_chargeefficiency) / 100.0F;
  currentMonitor.stateofcharge = ((float)currentMonitor.modbus.raw_stateofcharge) / 100.0F;

  currentMonitor.validReadings = true;

  TimeToSoCCalculation();
}

// Extract onboard/internal current monitor values into our internal STRUCTURE variables
void ProcessDIYBMSCurrentMonitorInternal()
{
  memset(&currentMonitor.modbus, 0, sizeof(currentmonitor_raw_modbus));
  currentMonitor.validReadings = false;

  currentMonitor.timestamp = esp_timer_get_time();
  currentMonitor.modbus.milliamphour_out = currentmon_internal.calc_milliamphour_out();
  currentMonitor.modbus.milliamphour_in = currentmon_internal.calc_milliamphour_in();
  currentMonitor.modbus.daily_milliamphour_out = currentmon_internal.calc_daily_milliamphour_out();
  currentMonitor.modbus.daily_milliamphour_in = currentmon_internal.calc_daily_milliamphour_in();
  currentMonitor.modbus.firmwareversion = (((uint32_t)GIT_VERSION_B1) << 16) + (uint32_t)GIT_VERSION_B2;
  currentMonitor.modbus.firmwaredatetime = COMPILE_DATE_TIME_UTC_EPOCH;

  /*
16|TMPOL|Read only
15|SHNTOL|Read only
14|SHNTUL|Read only
13|BUSOL|Read only
12|BUSUL|Read only
11|POL|Read only
10|Temperature compensation enabled|Read write
9|ADC Range 0=±163.84 mV, 1=±40.96 mV (only 40.96mV supported by diyBMS)|Read only
*/
  uint16_t flag1 = currentmon_internal.calc_alerts();
  currentMonitor.TemperatureOverLimit = flag1 & bit(DIAG_ALRT_FIELD::TMPOL);
  currentMonitor.CurrentOverLimit = flag1 & bit(DIAG_ALRT_FIELD::SHNTOL);
  currentMonitor.CurrentUnderLimit = flag1 & bit(DIAG_ALRT_FIELD::SHNTUL);
  currentMonitor.VoltageOverlimit = flag1 & bit(DIAG_ALRT_FIELD::BUSOL);
  currentMonitor.VoltageUnderlimit = flag1 & bit(DIAG_ALRT_FIELD::BUSUL);
  currentMonitor.PowerOverLimit = flag1 & bit(DIAG_ALRT_FIELD::POL);

  currentMonitor.TempCompEnabled = currentmon_internal.calc_tempcompenabled();
  currentMonitor.ADCRange4096mV = true;
  // mysettings.currentMonitoring_tempcompenabled = currentmon_internal.calc_tempcompenabled();

  currentMonitor.RelayTriggerTemperatureOverLimit = false;
  currentMonitor.RelayTriggerCurrentOverLimit = false;
  currentMonitor.RelayTriggerCurrentUnderLimit = false;
  currentMonitor.RelayTriggerVoltageOverlimit = false;
  currentMonitor.RelayTriggerVoltageUnderlimit = false;
  currentMonitor.RelayTriggerPowerOverLimit = false;
  currentMonitor.RelayState = false;

  currentMonitor.modbus.temperature = currentmon_internal.calc_temperature();
  currentMonitor.modbus.temperaturelimit = currentmon_internal.calc_temperaturelimit();
  currentMonitor.modbus.shunttempcoefficient = currentmon_internal.calc_shunttempcoefficient();
  currentMonitor.modbus.batterycapacityamphour = currentmon_internal.calc_batterycapacityAh();
  currentMonitor.modbus.shuntmaxcurrent = currentmon_internal.calc_shuntmaxcurrent();
  currentMonitor.modbus.shuntmillivolt = currentmon_internal.calc_shuntmillivolt();
  currentMonitor.modbus.shuntcal = currentmon_internal.calc_shuntcalibration();
  currentMonitor.modbus.modelnumber = 0x229;
  currentMonitor.modbus.power = currentmon_internal.calc_power();
  currentMonitor.modbus.overpowerlimit = currentmon_internal.calc_overpowerlimit();
  currentMonitor.modbus.voltage = currentmon_internal.calc_voltage();
  currentMonitor.modbus.current = currentmon_internal.calc_current();
  currentMonitor.modbus.shuntresistance = currentmon_internal.calc_shuntresistance();
  currentMonitor.modbus.tailcurrentamps = currentmon_internal.calc_tailcurrentamps();
  currentMonitor.chargeefficiency = currentmon_internal.calc_charge_efficiency_factor();
  currentMonitor.stateofcharge = currentmon_internal.calc_state_of_charge();
  currentMonitor.modbus.fullychargedvoltage = currentmon_internal.calc_fullychargedvoltage();
  currentMonitor.modbus.overvoltagelimit = currentmon_internal.calc_overvoltagelimit();
  currentMonitor.modbus.undervoltagelimit = currentmon_internal.calc_undervoltagelimit();
  currentMonitor.modbus.overcurrentlimit = currentmon_internal.calc_overcurrentlimit();
  currentMonitor.modbus.undercurrentlimit = currentmon_internal.calc_undercurrentlimit();

  // uint16_t flags;

  currentMonitor.validReadings = true;
  TimeToSoCCalculation();

  /*
  ESP_LOGD(TAG, "WDog = %u", currentMonitor.modbus.watchdogcounter);
  ESP_LOGD(TAG, "SOC = %i", currentMonitor.stateofcharge);

  ESP_LOGD(TAG, "Volt = %f", currentMonitor.modbus.voltage);
  ESP_LOGD(TAG, "Curr = %f", currentMonitor.modbus.current);
  ESP_LOGD(TAG, "Temp = %i", currentMonitor.modbus.temperature);

  ESP_LOGD(TAG, "Out = %f", currentMonitor.modbus.milliamphour_in);
  ESP_LOGD(TAG, "In = %f", currentMonitor.modbus.milliamphour_out);

  ESP_LOGD(TAG, "Ver = %x", currentMonitor.modbus.firmwareversion);
  ESP_LOGD(TAG, "Date = %u", currentMonitor.modbus.firmwaredatetime);
*/
}


void PZEM017_SetShuntType(uint8_t modbusAddress, uint16_t shuntMaxCurrent)
{
  // Default 100A
  uint8_t shuntType;

  switch (shuntMaxCurrent)
  {
  case 50:
    shuntType = 1;
    break;
  case 200:
    shuntType = 2;
    break;
  case 300:
    shuntType = 3;
    break;
  default:
    // 100amp
    shuntType = 0;
    break;
  }

  uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];
  memset(&cmd, 0, sizeof(cmd));

  cmd[0] = modbusAddress;
  // Function Code 6
  cmd[1] = 0x06;
  // Address of the shunt register (3)
  cmd[3] = 0x03;
  // Value
  cmd[5] = shuntType;

  ESP_LOGD(TAG, "Set PZEM017 max current %uA=%u", shuntMaxCurrent, shuntType);

  // Blocking call
  xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);

  // Zero all data
  memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));
  currentMonitor.validReadings = false;
}

void PZEM017_SetDeviceAddress(uint8_t newAddress)
{
  // First we force the PZEM device to assume the selected MODBUS address using
  // the special "broadcast" address of 0xF8.  Technically the PZEM devices
  // support multiple devices on same RS485 bus, but DIYBMS doesn't....

  uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];
  memset(&cmd, 0, sizeof(cmd));

  // The configuration address (only 1 PZEM device can be connected)
  cmd[0] = 0xf8;
  // Function Code 6
  cmd[1] = 0x06;
  // Register 2
  // cmd[2] = 0;
  cmd[3] = 2;
  // value
  // cmd[4] = 0;
  cmd[5] = newAddress;

  ESP_LOGD(TAG, "Sent PZEM_017 change address");

  // Zero all data
  memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));
  currentMonitor.validReadings = false;
  xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);
}

void currentMon_ConfigureBasic(uint16_t shuntmv, uint16_t shuntmaxcur, uint16_t batterycapacity, float fullchargevolt, float tailcurrent, float chargeefficiency)
{
  auto chargeeff = (uint16_t)(chargeefficiency * 100.0F);

  uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];
  memset(&cmd, 0, sizeof(cmd));

  //	Write Multiple Holding Registers
  uint8_t cmd2[] = {
      // The Slave Address
      mysettings.currentMonitoringModBusAddress,
      // The Function Code 16
      16,
      // Data Address of the first register (zero based so 18 = register 40019)
      0,
      18,
      // number of registers to write
      0,
      8,
      // number of data bytes to follow (2 registers x 2 bytes each = 4 bytes)
      16,
      // value to write to register 40019 |40019|shunt_max_current  (unsigned int16)
      (uint8_t)(shuntmaxcur >> 8),
      (uint8_t)(shuntmaxcur & 0xFF),
      // value to write to register 40020 |40020|shunt_millivolt  (unsigned int16)
      (uint8_t)(shuntmv >> 8),
      (uint8_t)(shuntmv & 0xFF),
      //|40021|Battery Capacity (ah)  (unsigned int16)
      (uint8_t)(batterycapacity >> 8),
      (uint8_t)(batterycapacity & 0xFF),
      //|40022|Fully charged voltage (4 byte double)
      0,
      0,
      0,
      0,
      //|40024|Tail current (Amps) (4 byte double)
      0,
      0,
      0,
      0,
      //|40026|Charge efficiency factor % (unsigned int16) (scale x100 eg. 10000 = 100.00%, 9561 = 95.61%)
      (uint8_t)(chargeeff >> 8),
      (uint8_t)(chargeeff & 0xFF),
  };

  auto ptr = SetMobusRegistersFromFloat(cmd2, 13, fullchargevolt);
  ptr = SetMobusRegistersFromFloat(cmd2, ptr, tailcurrent);

  memcpy(&cmd, &cmd2, sizeof(cmd2));
  xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);

  // Zero all data
  // memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));
  currentMonitor.validReadings = false;
}

void currentMon_SetSOC(float newSOC)
{
  auto value = (uint16_t)(newSOC * 100);

  //	Write Multiple Holding Registers
  uint8_t cmd2[] = {
      // The Slave Address
      mysettings.currentMonitoringModBusAddress,
      // The Function Code 16
      16,
      // Data Address of the first register (zero based so 26 = register 40027)
      0,
      26,
      // number of registers to write (1)
      0,
      1,
      // number of data bytes to follow (1 register x 2 bytes each = 2 bytes)
      2,
      // value to write to register |40027|State of charge % (unsigned int16)
      // (scale x100 eg. 10000 = 100.00%, 8012 = 80.12%, 100 = 1.00%)
      (uint8_t)(value >> 8),
      (uint8_t)(value & 0xFF)};

  xQueueSend(rs485_transmit_q_handle, &cmd2, portMAX_DELAY);
}

void currentMon_ResetDailyAmpHourCounters()
{
  //	Write Multiple Holding Registers
  uint8_t cmd2[] = {
      // The Slave Address
      mysettings.currentMonitoringModBusAddress,
      // The Function Code 16
      16,
      // Data Address of the first register (zero based so 11 = register 40012)
      0,
      11,
      // number of registers to write
      0,
      4,
      // number of data bytes to follow (2 registers x 2 bytes each = 4 bytes)
      8,
      // value to write to register 40012/13 |40012| daily milliamphour_out (uint32_t)
      0,
      0,
      0,
      0,
      // value to write to register 40014/15 |40014| daily milliamphour_in (uint32_t)
      0,
      0,
      0,
      0};

  xQueueSend(rs485_transmit_q_handle, &cmd2, portMAX_DELAY);
}

void currentMon_ConfigureAdvancedExternal(currentmonitoring_struct newvalues)
{
  //	Write Multiple Holding Registers
  uint8_t cmd2[] = {
      // The Slave Address
      mysettings.currentMonitoringModBusAddress,
      // The Function Code 16
      16,
      // Data Address of the first register (|40028|INA_REGISTER::SHUNT_CAL (unsigned int16))
      0,
      27,
      // number of registers to write
      0,
      13,
      // number of data bytes to follow (13 registers x 2 bytes each)
      2 * 13,
      // value to write to register 40028
      // 21 = shuntcal
      (uint8_t)(newvalues.modbus.shuntcal >> 8),
      (uint8_t)(newvalues.modbus.shuntcal & 0xFF),
      // value to write to register 40029
      // temperaturelimit
      (uint8_t)(newvalues.modbus.temperaturelimit >> 8),
      (uint8_t)(newvalues.modbus.temperaturelimit & 0xFF),
      // overvoltagelimit 40030
      0,
      0,
      0,
      0,
      // undervoltagelimit 40032
      0,
      0,
      0,
      0,
      // overcurrentlimit 40034
      0,
      0,
      0,
      0,
      // undercurrentlimit 40029
      0,
      0,
      0,
      0,
      // overpowerlimit 40038
      0,
      0,
      0,
      0,
      // shunttempcoefficient 40
      (uint8_t)(newvalues.modbus.shunttempcoefficient >> 8),
      (uint8_t)(newvalues.modbus.shunttempcoefficient & 0xFF),
  };

  // ESP_LOGD(TAG, "temp limit=%i", newvalues.temperaturelimit);
  // ESP_LOGD(TAG, "shuntcal=%u", newvalues.shuntcal);

  // Register 18 = shunt_max_current
  // Register 19 = shunt_millivolt

  uint8_t ptr = SetMobusRegistersFromFloat(cmd2, 11, newvalues.modbus.overvoltagelimit);
  ptr = SetMobusRegistersFromFloat(cmd2, ptr, newvalues.modbus.undervoltagelimit);
  ptr = SetMobusRegistersFromFloat(cmd2, ptr, newvalues.modbus.overcurrentlimit);
  ptr = SetMobusRegistersFromFloat(cmd2, ptr, newvalues.modbus.undercurrentlimit);
  ptr = SetMobusRegistersFromFloat(cmd2, ptr, newvalues.modbus.overpowerlimit);

  uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];
  memset(&cmd, 0, sizeof(cmd));
  memcpy(&cmd, &cmd2, sizeof(cmd2));
  xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);
}



// RS485 transmit
[[noreturn]] void rs485_tx(void *)
{
  uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];
  memset(&cmd, 0, sizeof(cmd));

  for (;;)
  {
    // Delay 4 seconds between requests
    vTaskDelay(pdMS_TO_TICKS(4000));

    if (mysettings.currentMonitoringEnabled == true)
    {

      if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)
      {
        if (currentmon_internal.Available())
        {
          // Take readings from internal INA229 chip (on controller board)
          if (hal.GetVSPIMutex())
          {
            currentmon_internal.TakeReadings();
            ProcessDIYBMSCurrentMonitorInternal();
            hal.ReleaseVSPIMutex();
          }
        }
        else
        {
          currentMonitor.validReadings = false;
        }
      }
      else if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
      {
        // This is the request we send to diyBMS current monitor, it pulls back 38 registers
        // this is all the registers diyBMS current monitor has
        // Holding Registers = command 3
        cmd[0] = mysettings.currentMonitoringModBusAddress;
        // Input registers - 46 of them (92 bytes + headers + crc = 83 byte reply)
        cmd[1] = 3;
        cmd[5] = 46;
        xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);
      }
      else if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::PZEM_017)
      {
        // ESP_LOGD(TAG, "RS485 TX");
        cmd[0] = mysettings.currentMonitoringModBusAddress;

        if (currentMonitor.modbus.shuntmillivolt == 0)
        {
          ESP_LOGD(TAG, "PZEM_017 Read params");
          cmd[1] = 0x03;
          cmd[5] = 0x04;
        }
        else
        {
          // ESP_LOGD(TAG, "PZEM_017 Read values");
          //  Read the standard voltage/current values
          //   Input registers
          cmd[1] = 0x04;
          // Read 8 registers (0 to 8)
          cmd[5] = 0x08;
        }
        xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);
      }
    } // end if
  }
}

// RS485 receive
[[noreturn]] void rs485_rx(void *)
{
  for (;;)
  {
    // Wait until this task is triggered (sending queue task triggers it)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Delay 50ms for the data to arrive
    vTaskDelay(pdMS_TO_TICKS(50));

    uint16_t len = 0;

    if (hal.GetRS485Mutex())
    {
      // Wait 200ms before timeout
      len = (uint16_t)uart_read_bytes(rs485_uart_num, frame, sizeof(frame), pdMS_TO_TICKS(200));
      hal.ReleaseRS485Mutex();
    }

    // Min packet length of 5 bytes
    if (len > 5)
    {
      uint8_t id = frame[0];

      auto crc = (uint16_t)((frame[len - 2] << 8) | frame[len - 1]); // combine the crc Low & High bytes

      auto temp = calculateModBusCRC(frame, (uint8_t)(len - 2));
      // Swap bytes to match MODBUS ordering
      auto calculatedCRC = (uint16_t)(temp << 8) | (uint16_t)(temp >> 8);

      // ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, esp_log_level_t::ESP_LOG_DEBUG);

      if (calculatedCRC == crc)
      {
        // if the calculated crc matches the recieved crc continue to process data...
        uint8_t RS485Error = frame[1] & B10000000;
        if (RS485Error == 0)
        {
          uint8_t cmd = frame[1] & B01111111;
          uint8_t length = frame[2];

          ESP_LOGD(TAG, "Recv %i bytes, id=%u, cmd=%u", len, id, cmd);
          // ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, esp_log_level_t::ESP_LOG_DEBUG);

          if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::PZEM_017)
          {
            if (cmd == 6 && id == 248)
            {
              ESP_LOGI(TAG, "Reply to broadcast/change address");
            }
            if (cmd == 6 && id == mysettings.currentMonitoringModBusAddress)
            {
              ESP_LOGI(TAG, "Reply to set param");
            }
            else if (cmd == 3 && id == mysettings.currentMonitoringModBusAddress)
            {
              // 75mV shunt (hard coded for PZEM)
              currentMonitor.modbus.shuntmillivolt = 75;

              // Shunt type 0x0000 - 0x0003 (100A/50A/200A/300A)
              switch (((uint32_t)frame[9] << 8 | (uint32_t)frame[10]))
              {
              case 0:
                currentMonitor.modbus.shuntmaxcurrent = 100;
                break;
              case 1:
                currentMonitor.modbus.shuntmaxcurrent = 50;
                break;
              case 2:
                currentMonitor.modbus.shuntmaxcurrent = 200;
                break;
              case 3:
                currentMonitor.modbus.shuntmaxcurrent = 300;
                break;
              default:
                currentMonitor.modbus.shuntmaxcurrent = 0;
              }
            }
            else if (cmd == 4 && id == mysettings.currentMonitoringModBusAddress && len == 21)
            {
              // ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, esp_log_level_t::ESP_LOG_DEBUG);

              // memset(&currentMonitor.modbus, 0, sizeof(currentmonitor_raw_modbus));
              currentMonitor.validReadings = true;
              currentMonitor.timestamp = esp_timer_get_time();
              // voltage in 0.01V
              currentMonitor.modbus.voltage = (float)((uint32_t)frame[3] << 8 | (uint32_t)frame[4]) / (float)100.0;
              // current in 0.01A
              currentMonitor.modbus.current = (float)((uint32_t)frame[5] << 8 | (uint32_t)frame[6]) / (float)100.0;
              // power in 0.1W
              currentMonitor.modbus.power = ((uint32_t)frame[7] << 8 | (uint32_t)frame[8] | (uint32_t)frame[9] << 24 | (uint32_t)frame[10] << 16) / 10.0;
            }
            else
            {
              // Dump out unhandled reply
              ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, esp_log_level_t::ESP_LOG_DEBUG);
            }
          }
          else if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
          {
            if (id == mysettings.currentMonitoringModBusAddress && cmd == 3)
            {
              ProcessDIYBMSCurrentMonitorRegisterReply(length);

              if (_tft_screen_available)
              {
                // Refresh the TFT display
                xTaskNotify(updatetftdisplay_task_handle, 0x00, eNotifyAction::eNoAction);
              }
            }
            else if (id == mysettings.currentMonitoringModBusAddress && cmd == 16)
            {
              ESP_LOGI(TAG, "Write multiple regs, success");
            }
            else
            {
              // Dump out unhandled reply
              ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, esp_log_level_t::ESP_LOG_DEBUG);
            }
          }
        }
        else
        {
          ESP_LOGE(TAG, "RS485 error");
          ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, esp_log_level_t::ESP_LOG_DEBUG);
        }
      }
      else
      {
        ESP_LOGE(TAG, "CRC error");
      }
    }
    else
    {
      // We didn't receive anything on RS485, record error and mark current monitor as invalid
      ESP_LOGE(TAG, "Short packet %i bytes", len);

      // Indicate that the current monitor values are now invalid/unknown
      currentMonitor.validReadings = false;
    }

    // Notify sending queue, to continue
    xTaskNotify(service_rs485_transmit_q_task_handle, 0x00, eNotifyAction::eNoAction);

  } // infinite loop
}


/**
 * Is current monitoring enabled, valid and can it calculate a valid SOC?
*/
bool CurrentMonitorCanSOC()
{
    return mysettings.currentMonitoringEnabled && currentMonitor.validReadings && (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS || mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL);
}