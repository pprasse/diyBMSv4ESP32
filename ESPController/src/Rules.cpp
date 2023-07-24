#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-rules";

#include "Rules.h"

// Its critical these are in the same order as "enum Rule", and occupy the same index position
const char *RuleTextDescription[] = {
    "EmergencyStop",
    "BMSError",
    "CurrentMonitorOverCurrentAmps",
    "ModuleOverVoltage",
    "ModuleUnderVoltage",
    "ModuleOverTemperatureInternal",
    "ModuleUnderTemperatureInternal",
    "ModuleOverTemperatureExternal",
    "ModuleUnderTemperatureExternal",
    "CurrentMonitorOverVoltage",
    "CurrentMonitorUnderVoltage",
    "BankOverVoltage",
    "BankUnderVoltage",
    "BankRange",
    "Timer2",
    "Timer1"};

void Rules::ClearValues()
{
    // Array to hold the total voltage of each bank (in millivolts)
    for (uint8_t r = 0; r < maximum_number_of_banks; r++)
    {
        limitedbankvoltage[r] = 0;
        bankvoltage[r] = 0;
        LowestCellVoltageInBank[r] = 0xFFFF;
        HighestCellVoltageInBank[r] = 0;
    }

    highestBankVoltage = 0;
    address_highestBankVoltage = maximum_number_of_banks+1;
    lowestBankVoltage = 0xFFFFFFFF;
    address_lowestBankVoltage = maximum_number_of_banks+1;
    highestCellVoltage = 0;
    lowestCellVoltage = 0xFFFF;
    highestExternalTemp = -127;
    lowestExternalTemp = 127;
    highestInternalTemp = -127;
    lowestInternalTemp = 127;
    zeroVoltageModuleCount = 0;
    invalidModuleCount = 0;
    moduleHasExternalTempSensor = false;

    address_LowestCellVoltage = maximum_controller_cell_modules + 1;
    address_lowestExternalTemp = maximum_controller_cell_modules + 1;
    address_highestExternalTemp = maximum_controller_cell_modules + 1;
    address_HighestCellVoltage = maximum_controller_cell_modules + 1;
    index_bank_HighestCellVoltage = 0;

    dynamicChargeVoltage = 0;
    dynamicChargeCurrent = 0;
}

// Looking at individual voltages and temperatures and sum up Bank voltages.
void Rules::ProcessCell(uint8_t bank, uint8_t cellNumber, CellModuleInfo *c, uint16_t cellmaxmv)
{
    if (c->valid == false)
    {
        invalidModuleCount++;
        return;
    }

    bankvoltage[bank] += c->voltagemV;
    limitedbankvoltage[bank] += min(c->voltagemV, cellmaxmv);

    // If the voltage of the module is zero, we probably haven't requested it yet (which happens during power up)
    // so keep count so we don't accidentally trigger rules.
    if (c->voltagemV == 0)
    {
        zeroVoltageModuleCount++;
    }

    if (c->voltagemV > HighestCellVoltageInBank[bank])
    {
        HighestCellVoltageInBank[bank] = c->voltagemV;
    }
    if (c->voltagemV < LowestCellVoltageInBank[bank])
    {
        LowestCellVoltageInBank[bank] = c->voltagemV;
    }

    if (c->voltagemV > highestCellVoltage)
    {
        highestCellVoltage = c->voltagemV;
        address_HighestCellVoltage = cellNumber;
        index_bank_HighestCellVoltage = bank;
    }

    if (c->voltagemV < lowestCellVoltage)
    {
        lowestCellVoltage = c->voltagemV;
        address_LowestCellVoltage = cellNumber;
    }

    if (c->externalTemp != -40)
    {
        // Record that we do have at least one external temperature sensor on a module
        moduleHasExternalTempSensor = true;

        if (c->externalTemp > highestExternalTemp)
        {
            highestExternalTemp = c->externalTemp;
            address_highestExternalTemp = cellNumber;
        }

        if (c->externalTemp < lowestExternalTemp)
        {
            lowestExternalTemp = c->externalTemp;
            address_lowestExternalTemp = cellNumber;
        }
    }

    if (c->internalTemp > highestInternalTemp)
    {
        highestInternalTemp = c->internalTemp;
    }

    if (c->externalTemp < lowestInternalTemp)
    {
        lowestInternalTemp = c->internalTemp;
    }
}

uint16_t Rules::VoltageRangeInBank(uint8_t bank)
{
    if (invalidModuleCount > 0)
        return 0;

    return HighestCellVoltageInBank[bank] - LowestCellVoltageInBank[bank];
}

void Rules::ProcessBank(uint8_t bank)
{
    // Combine the voltages - work out the highest and lowest Bank voltages
    if (bankvoltage[bank] > highestBankVoltage)
    {
        highestBankVoltage = bankvoltage[bank];
        address_highestBankVoltage = bank;
    }
    if (bankvoltage[bank] < lowestBankVoltage)
    {
        lowestBankVoltage = bankvoltage[bank];
        address_lowestBankVoltage = bank;
    }

    if (VoltageRangeInBank(bank) > highestBankRange)
    {
        highestBankRange = VoltageRangeInBank(bank);
    }
}

void Rules::SetWarning(InternalWarningCode warncode)
{
    if (warncode > MAXIMUM_InternalWarningCode)
        return;

    // Only set the warning once
    if (WarningCodes[warncode] != InternalWarningCode::NoWarning)
        return;

    WarningCodes[warncode] = warncode;
    numberOfActiveWarnings++;
    ESP_LOGI(TAG, "Set warning %i", warncode);
}

void Rules::SetError(InternalErrorCode err)
{
    if (err > MAXIMUM_InternalErrorCode)
    {
        ESP_LOGE(TAG, "Error %i is > MAXIMUM_InternalErrorCode, setting %i", err, MAXIMUM_InternalErrorCode);
        err = (InternalErrorCode)MAXIMUM_InternalErrorCode;
    }

    // Only set error once
    if (ErrorCodes[err] != InternalErrorCode::NoError)
        return;

    ErrorCodes[err] = err;
    numberOfActiveErrors++;
    rule_outcome[Rule::BMSError] = true;
    ESP_LOGI(TAG, "Set error %i", err);
}

void Rules::RunRules(
    int32_t *value,
    int32_t *hysteresisvalue,
    bool emergencyStop,
    uint16_t mins,
    currentmonitoring_struct *currentMonitor)
{
    if( _controller_state != ControllerState::Running )
    {
        SetError(InternalErrorCode::CommunicationsError);
    }


    // Emergency stop signal...
    rule_outcome[Rule::EmergencyStop] = emergencyStop;

    // Timer 1 and Timer 2
    rule_outcome[Rule::Timer1] = (mins >= value[Rule::Timer1] && mins <= hysteresisvalue[Rule::Timer1]);
    rule_outcome[Rule::Timer2] = (mins >= value[Rule::Timer2] && mins <= hysteresisvalue[Rule::Timer2]);

    if (currentMonitor->validReadings)
    {
        // Currents can be both positive and negative (depending on current flow, we ABS that to get an always POSITIVE number)
        uint32_t integercurrent = (uint32_t)(abs(currentMonitor->modbus.current) + (float)0.5);

        if (integercurrent > value[Rule::CurrentMonitorOverCurrentAmps] && rule_outcome[Rule::CurrentMonitorOverCurrentAmps] == false)
        {
            // CurrentMonitorOverCurrentAmps - TRIGGERED
            rule_outcome[Rule::CurrentMonitorOverCurrentAmps] = true;
        }
        else if (integercurrent < hysteresisvalue[Rule::CurrentMonitorOverCurrentAmps] && rule_outcome[Rule::CurrentMonitorOverCurrentAmps] == true)
        {
            // CurrentMonitorOverCurrentAmps - HYSTERESIS RESET
            rule_outcome[Rule::CurrentMonitorOverCurrentAmps] = false;
        }

        uint32_t integervoltagemV = (uint32_t)((currentMonitor->modbus.voltage * 1000.0) + (float)0.5);

        if (integervoltagemV > value[Rule::CurrentMonitorOverVoltage] && rule_outcome[Rule::CurrentMonitorOverVoltage] == false)
        {
            // Rule - CURRENT MONITOR Bank over voltage (mV)
            rule_outcome[Rule::CurrentMonitorOverVoltage] = true;
        }
        else if (integervoltagemV < hysteresisvalue[Rule::CurrentMonitorOverVoltage] && rule_outcome[Rule::CurrentMonitorOverVoltage] == true)
        {
            // Rule - CURRENT MONITOR Bank over voltage (mV) - HYSTERESIS RESET
            rule_outcome[Rule::CurrentMonitorOverVoltage] = false;
        }

        if (integervoltagemV < value[Rule::CurrentMonitorUnderVoltage] && rule_outcome[Rule::CurrentMonitorUnderVoltage] == false)
        {
            // Rule - CURRENT MONITOR Bank under voltage (mV)
            rule_outcome[Rule::CurrentMonitorUnderVoltage] = true;
        }
        else if (integervoltagemV > hysteresisvalue[Rule::CurrentMonitorUnderVoltage] && rule_outcome[Rule::CurrentMonitorUnderVoltage] == true)
        {
            // Rule - CURRENT MONITOR Bank under voltage (mV) - HYSTERESIS RESET
            rule_outcome[Rule::CurrentMonitorUnderVoltage] = false;
        }
    }
    else
    {
        // We don't have valid current monitor readings, so the rule is ALWAYS false
        rule_outcome[Rule::CurrentMonitorOverCurrentAmps] = false;
        rule_outcome[Rule::CurrentMonitorOverVoltage] = false;
        rule_outcome[Rule::CurrentMonitorUnderVoltage] = false;
    }

    // At least 1 module is zero volt - not a problem whilst we are in stabilizing start up mode
    if (zeroVoltageModuleCount > 0 || invalidModuleCount > 0)
    {
        rule_outcome[Rule::ModuleOverVoltage] = false;
        rule_outcome[Rule::ModuleUnderVoltage] = false;
        rule_outcome[Rule::ModuleOverTemperatureInternal] = false;
        rule_outcome[Rule::ModuleUnderTemperatureInternal] = false;
        rule_outcome[Rule::ModuleOverTemperatureExternal] = false;
        rule_outcome[Rule::ModuleUnderTemperatureExternal] = false;

        // Abort processing any more rules until controller is stable/running state
        return;
    }

    // Only rules which are based on temperature or voltage should go below this point....

    /*
  SERIAL_DEBUG.print("Rule Values: lowP=");
  SERIAL_DEBUG.print(lowestbankvoltage);
  SERIAL_DEBUG.print(" highP=");
  SERIAL_DEBUG.print(highestBankVoltage);

  SERIAL_DEBUG.print(" / lowC=");
  SERIAL_DEBUG.print(lowestCellVoltage);
  SERIAL_DEBUG.print(" highC=");
  SERIAL_DEBUG.print(highestCellVoltage);

  SERIAL_DEBUG.print(" / highTE=");
  SERIAL_DEBUG.print(highestExternalTemp);
  SERIAL_DEBUG.print(" lowTE=");
  SERIAL_DEBUG.print(lowestExternalTemp);
  SERIAL_DEBUG.println("");
*/

    if (highestCellVoltage > value[Rule::ModuleOverVoltage] && rule_outcome[Rule::ModuleOverVoltage] == false)
    {
        // Rule Individual cell over voltage - TRIGGERED
        rule_outcome[Rule::ModuleOverVoltage] = true;
    }
    else if (highestCellVoltage < hysteresisvalue[Rule::ModuleOverVoltage] && rule_outcome[Rule::ModuleOverVoltage] == true)
    {
        // Rule Individual cell over voltage - HYSTERESIS RESET
        rule_outcome[Rule::ModuleOverVoltage] = false;
    }

    if (lowestCellVoltage < value[Rule::ModuleUnderVoltage] && rule_outcome[Rule::ModuleUnderVoltage] == false)
    {
        // Rule Individual cell under voltage (mV) - TRIGGERED
        rule_outcome[Rule::ModuleUnderVoltage] = true;
    }
    else if (lowestCellVoltage > hysteresisvalue[Rule::ModuleUnderVoltage] && rule_outcome[Rule::ModuleUnderVoltage] == true)
    {
        // Rule Individual cell under voltage (mV) - HYSTERESIS RESET
        rule_outcome[Rule::ModuleUnderVoltage] = false;
    }

    // These rules only fire if external temp sensor actually exists
    if (moduleHasExternalTempSensor)
    {
        // Doesn't cater for negative temperatures on rule (int8 vs uint32)
        if ((highestExternalTemp > value[Rule::ModuleOverTemperatureExternal]) && rule_outcome[Rule::ModuleOverTemperatureExternal] == false)
        {
            // Rule Individual cell over temperature (external probe)
            rule_outcome[Rule::ModuleOverTemperatureExternal] = true;
        }
        else if ((highestExternalTemp < hysteresisvalue[Rule::ModuleOverTemperatureExternal]) && rule_outcome[Rule::ModuleOverTemperatureExternal] == true)
        {
            // Rule Individual cell over temperature (external probe) - HYSTERESIS RESET
            rule_outcome[Rule::ModuleOverTemperatureExternal] = false;
        }
        // Doesn't cater for negative temperatures on rule (int8 vs uint32)
        if ((lowestExternalTemp < value[Rule::ModuleUnderTemperatureExternal]) && rule_outcome[Rule::ModuleUnderTemperatureExternal] == false)
        {
            // Rule Individual cell UNDER temperature (external probe)
            rule_outcome[Rule::ModuleUnderTemperatureExternal] = true;
        }
        else if ((lowestExternalTemp > hysteresisvalue[Rule::ModuleUnderTemperatureExternal]) && rule_outcome[Rule::ModuleUnderTemperatureExternal] == true)
        {
            // Rule Individual cell UNDER temperature (external probe) - HYSTERESIS RESET
            rule_outcome[Rule::ModuleUnderTemperatureExternal] = false;
        }
    }
    else
    {
        rule_outcome[Rule::ModuleOverTemperatureExternal] = false;
        rule_outcome[Rule::ModuleUnderTemperatureExternal] = false;
    }

    // Internal temperature monitoring and rules
    // Does not cope with negative temperatures on rule (int8 vs uint32)
    if ((highestInternalTemp > value[Rule::ModuleOverTemperatureInternal]) && rule_outcome[Rule::ModuleOverTemperatureInternal] == false)
    {
        // Rule Individual cell over temperature (Internal probe)
        rule_outcome[Rule::ModuleOverTemperatureInternal] = true;
    }
    else if ((highestInternalTemp < hysteresisvalue[Rule::ModuleOverTemperatureInternal]) && rule_outcome[Rule::ModuleOverTemperatureInternal] == true)
    {
        // Rule Individual cell over temperature (Internal probe) - HYSTERESIS RESET
        rule_outcome[Rule::ModuleOverTemperatureInternal] = false;
    }

    // Doesn't cater for negative temperatures on rule (int8 vs uint32)
    if ((lowestInternalTemp < value[Rule::ModuleUnderTemperatureInternal]) && rule_outcome[Rule::ModuleUnderTemperatureInternal] == false)
    {
        // Rule Individual cell UNDER temperature (Internal probe)
        rule_outcome[Rule::ModuleUnderTemperatureInternal] = true;
    }
    else if ((lowestInternalTemp > hysteresisvalue[Rule::ModuleUnderTemperatureInternal]) && rule_outcome[Rule::ModuleUnderTemperatureInternal] == true)
    {
        // Rule Individual cell UNDER temperature (Internal probe) - HYSTERESIS RESET
        rule_outcome[Rule::ModuleUnderTemperatureInternal] = false;
    }

    // Whole Bank voltages
    if (highestBankVoltage > value[Rule::BankOverVoltage] && rule_outcome[Rule::BankOverVoltage] == false)
    {
        // Rule - Bank over voltage (mV)
        rule_outcome[Rule::BankOverVoltage] = true;
    }
    else if (highestBankVoltage < hysteresisvalue[Rule::BankOverVoltage] && rule_outcome[Rule::BankOverVoltage] == true)
    {
        // Rule - Bank over voltage (mV) - HYSTERESIS RESET
        rule_outcome[Rule::BankOverVoltage] = false;
    }

    if (lowestBankVoltage < value[Rule::BankUnderVoltage] && rule_outcome[Rule::BankUnderVoltage] == false)
    {
        // Rule - Bank under voltage (mV)
        rule_outcome[Rule::BankUnderVoltage] = true;
    }
    else if (lowestBankVoltage > hysteresisvalue[Rule::BankUnderVoltage] && rule_outcome[Rule::BankUnderVoltage] == true)
    {
        // Rule - Bank under voltage (mV) - HYSTERESIS RESET
        rule_outcome[Rule::BankUnderVoltage] = false;
    }

    // While Bank voltages
    if (highestBankRange > value[Rule::BankRange] && rule_outcome[Rule::BankRange] == false)
    {
        // Rule - Bank Range
        rule_outcome[Rule::BankRange] = true;
    }
    else if (highestBankRange < hysteresisvalue[Rule::BankRange] && rule_outcome[Rule::BankRange] == true)
    {
        // Rule - Bank Range - HYSTERESIS RESET
        rule_outcome[Rule::BankRange] = false;
    }

    // Total up the active rules
    active_rule_count = 0;

    for (size_t i = 0; i < RELAY_RULES; i++)
    {
        if (rule_outcome[i] == true)
            active_rule_count++;
    }
}

bool Rules::SharedChargingDischargingRules(diybms_eeprom_settings *mysettings)
{
    if (mysettings->canbusprotocol == CanBusProtocolEmulation::CANBUS_DISABLED)
        return false;

    if (invalidModuleCount > 0)
        return false;
    if (moduleHasExternalTempSensor == false)
        return false;
    // Any errors, stop charge
    if (numberOfActiveErrors > 0)
        return false;

    // Battery high voltage alarm
    if (rule_outcome[Rule::BankOverVoltage])
        return false;
    // Low voltage alarm
    if (rule_outcome[Rule::BankUnderVoltage])
        return false;
    // Battery high temperature alarm
    if (rule_outcome[Rule::ModuleOverTemperatureExternal])
        return false;
    // Battery low temperature alarm
    if (rule_outcome[Rule::ModuleUnderTemperatureExternal])
        return false;

    if( _controller_state != ControllerState::Running )
        return false;

    return true;
}
bool Rules::IsChargeAllowed(diybms_eeprom_settings *mysettings)
{
    if (SharedChargingDischargingRules(mysettings) == false)
        return false;

    if (mysettings->preventcharging == true)
        return false;

    if (lowestExternalTemp<mysettings->chargetemplow | highestExternalTemp> mysettings->chargetemphigh)
    {
        // Stop charge - temperature out of range
        // ESP_LOGW(TAG, "Stop charge - temperature out of range");
        return false;
    }

    // chargevolt = 560
    if ((highestBankVoltage / 100) > mysettings->chargevolt)
        return false;

    // Individual cell over voltage
    if (highestCellVoltage > mysettings->cellmaxmv)
        return false;

    if (numberOfBalancingModules > 0 && mysettings->stopchargebalance == true)
        return false;

    return true;
}
bool Rules::IsDischargeAllowed(diybms_eeprom_settings *mysettings)
{
    if (SharedChargingDischargingRules(mysettings) == false)
        return false;

    if (mysettings->preventdischarge == true)
        return false;

    // Check battery temperature against charge/discharge parameters
    if (lowestExternalTemp<mysettings->dischargetemplow | highestExternalTemp> mysettings->dischargetemphigh)
    {
        // ESP_LOGW(TAG, "Stop discharge - temperature out of range");
        return false;
    }

    if ((lowestBankVoltage / 100) < mysettings->dischargevolt)
        return false;

    // Individual cell under voltage
    if (lowestCellVoltage < mysettings->cellminmv)
        return false;

    return true;
}

// Charge voltage calculated by CalculateDynamicChargeVoltage
// Scale 0.1 = 567 = 56.7V
uint16_t Rules::DynamicChargeVoltage()
{
    return dynamicChargeVoltage;
}
// Charge current calculated by CalculateDynamicChargeCurrent
// Scale 0.1 = 123 = 12.3Amps
int16_t Rules::DynamicChargeCurrent()
{
    return dynamicChargeCurrent;
}

// Apply "dynamic" charge current rules
void Rules::CalculateDynamicChargeCurrent(diybms_eeprom_settings *mysettings, CellModuleInfo *cellarray)
{
    // Remember dynamicChargeCurrent scale is 0.1
    dynamicChargeCurrent = mysettings->chargecurrent;

    if (!mysettings->dynamiccharge || mysettings->canbusprotocol == CanBusProtocolEmulation::CANBUS_DISABLED)
    {
        // Its switched off, use default
        return;
    }

    if (highestCellVoltage < mysettings->kneemv)
    {
        // Voltage of cell is below the knee voltage, so use full current
        return;
    }

    double value1 = mysettings->current_value1 / 10.0F;
    double value2 = mysettings->current_value2 / 10.0F;
    ESP_LOGD(TAG, "value1=%f", value1);
    ESP_LOGD(TAG, "value2=%f", value2);

    // This is always 1 :-)
    double knee_voltage = 0 / 100.0F;
    double at_knee = pow(value1, knee_voltage * pow(knee_voltage, value2));
    ESP_LOGD(TAG, "at_knee=%f", at_knee);

    double target_cell_voltage = (mysettings->cellmaxmv - mysettings->kneemv) / 100.0F;
    double at_target_cell_voltage = pow(value1, target_cell_voltage * pow(target_cell_voltage, value2));
    ESP_LOGD(TAG, "at_target_cell_voltage=%f", at_target_cell_voltage);

    double actual_cell_voltage = (highestCellVoltage - mysettings->kneemv) / 100.0F;
    double at_actual_cell_voltage = pow(value1, actual_cell_voltage * pow(actual_cell_voltage, value2));
    ESP_LOGD(TAG, "at_actual_cell_voltage=%f", at_actual_cell_voltage);

    double percent = 1 - (at_actual_cell_voltage / at_knee) / at_target_cell_voltage;
    ESP_LOGD(TAG, "percent=%f", percent);

    if (percent < 0.01)
    {
        // Catch small values and also negatives, 1% is the lowest we go...
        percent = 0.01;
    }

    // Use lowest of chargecurrent or calculation, just in case some math has gone wrong!
    ESP_LOGD(TAG, "percent=%f", percent);
    dynamicChargeCurrent = min(mysettings->chargecurrent, (uint16_t)round(mysettings->chargecurrent * percent));

    ESP_LOGD(TAG, "dynamicChargeCurrent=%u", dynamicChargeCurrent);
}

// Apply "dynamic" charge voltage rules
// This will always return a charge voltage - its the calling functions responsibility
// to check "IsChargeAllowed" function and take necessary action.
// Thanks to Matthias U (Smurfix) for the ideas and pseudo code https://community.openenergymonitor.org/u/smurfix/
// Output is cached in variable dynamicChargeVoltage as its used in multiple places
void Rules::CalculateDynamicChargeVoltage(diybms_eeprom_settings *mysettings, CellModuleInfo *cellarray)
{
    if (!mysettings->dynamiccharge || mysettings->canbusprotocol == CanBusProtocolEmulation::CANBUS_DISABLED)
    {
        // Its switched off, use default voltage
        dynamicChargeVoltage = mysettings->chargevolt;
        return;
    }

    // Some cells are above the knee voltage....

    // Are any cells at or over the maximum allowed? (panic!)
    if (highestCellVoltage >= mysettings->cellmaxmv)
    {
        ESP_LOGW(TAG, "Cell V>Max");
        // *** Stop charging, we are at or above maximum cell voltage ***

        // Find the lowest "limited" Bank voltage
        uint32_t lowest = 0xFFFFFFFF;
        for (uint8_t r = 0; r < mysettings->totalNumberOfBanks; r++)
        {
            if (limitedbankvoltage[r] < lowest)
            {
                lowest = limitedbankvoltage[r];
            }
        }

        lowest = lowest / 100;
        // ESP_LOGD(TAG, "lowest=%u", lowest);

        // Return MIN of either the "lowest Bank voltage" or the "user specified value"
        dynamicChargeVoltage = min(lowest, (uint32_t)mysettings->chargevolt);
        return;
    }

    // At this point all cell voltages are UNDER the target cellmaxmv

    // This is unlikely to work if the value is changed from 1 (an integer)
    const int16_t UniformDerating = 1;

    // Calculate voltage range
    uint32_t R = min(
        (int16_t)((mysettings->cellmaxmv - highestCellVoltage) * UniformDerating),
        (int16_t)((mysettings->cellmaxspikemv - mysettings->kneemv) / ((float)mysettings->sensitivity / 10.0F)));
    // Avoid future divide by zero errors
    if (R == 0)
    {
        R = 1;
    }
    ESP_LOGD(TAG, "R=%u", R);

    // We use the Bank with the highest cell voltage for these calculations - although hopefully all banks are very similar :-)
    uint32_t S = bankvoltage[index_bank_HighestCellVoltage];
    ESP_LOGD(TAG, "S=%u", S);

    uint32_t HminusR = (uint32_t)highestCellVoltage - R;
    ESP_LOGD(TAG, "HminusR=%u", HminusR);

    uint32_t MminusH = mysettings->cellmaxmv - highestCellVoltage;
    ESP_LOGD(TAG, "MminusH=%u", MminusH);

    // Jump to start of cells in the correct bank.
    uint8_t cellid = index_bank_HighestCellVoltage * mysettings->totalNumberOfSeriesModules;
    for (uint8_t i = 0; i < mysettings->totalNumberOfSeriesModules; i++)
    {
        if (cellarray[i].voltagemV >= HminusR)
        {
            S += ((MminusH) * (cellarray[i].voltagemV - (HminusR)) / R);
        }

        ESP_LOGD(TAG, "id=%u, V=%u, S=%u", cellid, cellarray[i].voltagemV, S);
    }

    // Scale down to 0.1V
    S = S / 100;
    ESP_LOGD(TAG, "S=%u", S);

    // Return MIN of either the above calculation or the "user specified value"
    dynamicChargeVoltage = min(S, (uint32_t)mysettings->chargevolt);
}

// Return SoC value after applying SOCFORCELOW and SOCOVERRIDE settings
// also limits output range between 0 and 100.
// SoC is rounded down to nearest integer
uint16_t Rules::StateOfChargeWithRulesApplied(diybms_eeprom_settings *mysettings, float realSOC)
{
    uint16_t value = floor(realSOC);

    // Deliberately force SoC to be reported as 2%, to trick external CANBUS devices into trickle charging (if they support it)
    if (mysettings->socforcelow)
    {
        value = 2;
    }

    if (mysettings->socoverride)
    {
        if (value > 90)
        {
            // Force inverter SoC reading to 90%, this should force it to continue charging the battery
            // this is helpful when first commissioning as most inverters stop charging at 100% SOC
            // even though the battery may not be full, and the DIYBMS current monitor has not learnt capacity yet.
            // This function should not be left permanently switched on - you could damage the battery.
            value = 90;
        }
        if (value < 25)
        {
            // Force minimum of 25% - some inverters (SoFAR) will force charge a battery lower than
            // this level limiting the charge current to 500W
            value = 25;
        }
    }

    // Limit to 100% maximum, DIYBMS current monitor can go above 100%, so don't confuse inverter/chargers
    if (value > 100)
    {
        value = 100;
    }
    return value;
}