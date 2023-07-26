/*

 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

  (c) 2017 to 2022 Stuart Pittaway

  This is the code for the ESP32 controller - it talks to the V4.X cell modules over isolated serial bus

  This code runs on ESP32 DEVKIT-C and compiles with VS CODE and PLATFORM IO environment.

  Unless you are making code changes, please use the pre-compiled version from GITHUB instead.
*/

#if defined(ESP8266)
#error ESP8266 is not supported by this code
#endif

#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms";

#define CONFIG_DISABLE_HAL_LOCKS 1

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"

#include <Arduino.h>

// #define PACKET_LOGGING_RECEIVE
// #define PACKET_LOGGING_SEND
// #define RULES_LOGGING

#include "FS.h"
#include "LittleFS.h"
#include <ESPmDNS.h>

#include "time.h"
#include <esp_ipc.h>
#include <esp_wifi.h>
#include <esp_bt.h>

#include <esp_event.h>

// Libraries for SD card
#include "SD.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/adc.h"
#include <driver/uart.h>
#include <esp_http_server.h>
#include <esp_sntp.h>
#include <SerialEncoder.h>

#include <ArduinoJson.h>
#include "defines.h"
#include "HAL_ESP32.h"
#include "Rules.h"
#include "settings.h"
#include "avrisp_programmer.h"
#include "tft.h"
#include "influxdb.h"
#include "mqtt.h"
#include "victron_canbus.h"
#include "pylon_canbus.h"
#include "pylonforce_canbus.h"
#include "string_utils.h"

#include <SPI.h>

#include "modbus.h"
#include "currentmon.h"

#include "history.h"



const char *wificonfigfilename = "/diybms/wifi.json";

HAL_ESP32 hal;

volatile bool emergencyStop = false;
bool _sd_card_installed = false;

// Used for WIFI hostname and also sent to Victron/Pylontech Force over CANBUS
char hostname[16];
char ip_string[16]; // xxx.xxx.xxx.xxx

bool wifi_isconnected = false;

History history = History();


CardAction card_action = CardAction::Idle;

// Screen variables in tft.cpp
extern bool _tft_screen_available;
extern uint8_t tftsleep_timer;
extern volatile bool _screen_awake;
extern bool force_tft_wake;
extern TimerHandle_t tftwake_timer;
extern void tftwakeup(TimerHandle_t xTimer);

// HTTPD server handle in webserver.cpp
extern httpd_handle_t _myserver;

wifi_eeprom_settings _wificonfig;

Rules rules;
diybms_eeprom_settings mysettings;
uint8_t TotalNumberOfCells() { return mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules; }

uint32_t canbus_messages_received = 0;
uint32_t canbus_messages_sent = 0;
uint32_t canbus_messages_failed_sent = 0;

bool server_running = false;
RelayState previousRelayState[RELAY_TOTAL];
bool previousRelayPulse[RELAY_TOTAL];

volatile enumInputState InputState[INPUTS_TOTAL];

currentmonitoring_struct currentMonitor;

TimerHandle_t led_off_timer;
TimerHandle_t pulse_relay_off_timer;

TaskHandle_t i2c_task_handle = nullptr;
TaskHandle_t sdcardlog_task_handle = nullptr;
TaskHandle_t sdcardlog_outputs_task_handle = nullptr;
TaskHandle_t avrprog_task_handle = nullptr;
TaskHandle_t enqueue_task_handle = nullptr;
TaskHandle_t transmit_task_handle = nullptr;
TaskHandle_t replyqueue_task_handle = nullptr;
TaskHandle_t lazy_task_handle = nullptr;
TaskHandle_t rule_task_handle = nullptr;

TaskHandle_t voltageandstatussnapshot_task_handle = nullptr;
TaskHandle_t updatetftdisplay_task_handle = nullptr;
TaskHandle_t periodic_task_handle = nullptr;
TaskHandle_t interrupt_task_handle = nullptr;
TaskHandle_t rs485_tx_task_handle = nullptr;
TaskHandle_t rs485_rx_task_handle = nullptr;
TaskHandle_t service_rs485_transmit_q_task_handle = nullptr;
TaskHandle_t canbus_tx_task_handle = nullptr;
TaskHandle_t canbus_rx_task_handle = nullptr;

// This large array holds all the information about the modules
CellModuleInfo cmi[maximum_controller_cell_modules];

avrprogramsettings _avrsettings;

QueueHandle_t rs485_transmit_q_handle;
QueueHandle_t request_q_handle;
QueueHandle_t reply_q_handle;

#include "crc16.h"
#include "settings.h"

#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"
#include "webserver.h"

PacketRequestGenerator prg = PacketRequestGenerator();
PacketReceiveProcessor receiveProc = PacketReceiveProcessor();

// Memory to hold in and out serial buffer
uint8_t SerialPacketReceiveBuffer[2 * sizeof(PacketStruct)];

SerialEncoder myPacketSerial;

uint16_t sequence = 0;

ControllerState _controller_state = ControllerState::Unknown;

void LED(uint8_t bits)
{
  hal.Led(bits);
}

// When triggered, the VOLTAGE and STATUS in the CellModuleInfo structure are accurate and consistant at this point in time.
// Good point to apply rules and update screen/statistics
[[noreturn]] void voltageandstatussnapshot_task(void *)
{
  for (;;)
  {
    // Wait until this task is triggered, when
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (_tft_screen_available)
    {
      // Refresh the TFT display
      xTaskNotify(updatetftdisplay_task_handle, 0x00, eNotifyAction::eNoAction);
    }

  } // end for
}

void mountSDCard()
{
  card_action = CardAction::Idle;

  if (_avrsettings.programmingModeEnabled)
  {
    ESP_LOGW(TAG, "Attempt to mount sd but AVR prog mode enabled");
    return;
  }

  ESP_LOGI(TAG, "Mounting SD card");

  _sd_card_installed = hal.MountSDCard();
}

void unmountSDCard()
{
  card_action = CardAction::Idle;

  if (_sd_card_installed == false)
    return;

  if (_avrsettings.programmingModeEnabled)
  {
    ESP_LOGW(TAG, "Attempt to UNMOUNT SD card but AVR prog mode enabled");
    return;
  }

  ESP_LOGI(TAG, "Unmounting SD card");
  hal.UnmountSDCard();
  _sd_card_installed = false;
}

void wake_up_tft(bool force)
{
  // Wake up the display
  if (tftwake_timer != nullptr)
  {
    force_tft_wake = force;
    if (xTimerStart(tftwake_timer, pdMS_TO_TICKS(10)) != pdPASS)
    {
      ESP_LOGE(TAG, "TFT wake timer error");
    }
  }
}

[[noreturn]] void avrprog_task(void *param)
{
  for (;;)
  {
    // Wait until this task is triggered
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    wake_up_tft(true);

    // TODO: This needs to be passed into this as a parameter
    avrprogramsettings *s;
    s = (avrprogramsettings *)param;

    ESP_LOGD(TAG, "AVR inprogress=%i", s->inProgress);
    ESP_LOGI(TAG, "AVR setting e=%02X h=%02X l=%02X mcu=%08X file=%s", s->efuse, s->hfuse, s->lfuse, s->mcu, s->filename);

    // Now we load the file into program array, from LITTLEFS (SPIFF)
    if (LittleFS.exists(s->filename))
    {
      File binaryfile = LittleFS.open(s->filename);

      s->programsize = binaryfile.size();

      // Reserve the SPI bus for programming purposes
      if (hal.GetVSPIMutex())
      {
        // This will block for the 6 seconds it takes to program ATTINY841...
        // although AVRISP_PROGRAMMER will call the watchdog to prevent reboots

        auto starttime = millis();

        auto isp = AVRISP_PROGRAMMER(hal.VSPI_Ptr(), GPIO_NUM_0, false, VSPI_SCK);

        ESP_LOGI(TAG, "Programming AVR");

        s->progresult = isp.ProgramAVRDevice(&tftdisplay_avrprogrammer_progress, s->mcu, s->programsize, binaryfile, s->lfuse, s->hfuse, s->efuse);
        s->duration = (uint32_t)(millis() - starttime);

        // Return VSPI to default speed/settings
        hal.ConfigureVSPI();

        hal.ReleaseVSPIMutex();

        if (s->progresult == AVRISP_PROGRAMMER_RESULT::SUCCESS)
        {
          ESP_LOGI(TAG, "Success");
        }
        else
        {
          ESP_LOGE(TAG, "Failed %i", s->progresult);
        }

        binaryfile.close();
      }
      else
      {
        s->progresult = AVRISP_PROGRAMMER_RESULT::OTHER_FAILURE;
        ESP_LOGE(TAG, "Unable to obtain Mutex");
      }
    }
    else
    {
      s->progresult = AVRISP_PROGRAMMER_RESULT::OTHER_FAILURE;
      ESP_LOGE(TAG, "AVR file not found %s", s->filename);
    }

    s->inProgress = false;

    // Refresh the display, after programming is complete
    xTaskNotify(updatetftdisplay_task_handle, 0x00, eNotifyAction::eNoAction);
  } // end for
}

// Output a status log to the SD Card in CSV format
[[noreturn]] void sdcardlog_task(void *)
{
  for (;;)
  {
    // Wait X seconds
    for (size_t i = 0; i < mysettings.loggingFrequencySeconds; i++)
    {
      // Delay 1 second
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (_sd_card_installed &&
        !_avrsettings.programmingModeEnabled &&
        mysettings.loggingEnabled &&
        _controller_state == ControllerState::Running &&
        hal.IsVSPIMutexAvailable())
    {
      // ESP_LOGD(TAG, "sdcardlog_task");

      struct tm timeinfo;
      // getLocalTime has delay() functions in it :-(
      if (getLocalTime(&timeinfo, 1))
      {
        timeinfo.tm_year += 1900;
        // Month is 0 to 11 based!
        timeinfo.tm_mon++;

        std::string filename;
        filename.reserve(32);
        filename.append("/data_").append(std::to_string(timeinfo.tm_year)).append(pad_zero(2, (uint16_t)timeinfo.tm_mon)).append(pad_zero(2, (uint16_t)timeinfo.tm_mday)).append(".csv");

        File file;

        // Prevent other devices using the VSPI bus
        if (hal.GetVSPIMutex())
        {
          if (SD.exists(filename.c_str()))
          {
            // Open existing file (assumes there is enough SD card space to log)
            file = SD.open(filename.c_str(), FILE_APPEND);
            ESP_LOGD(TAG, "Open log %s", filename.c_str());
          }
          else
          {
            // Create a new file
            uint64_t freeSpace = SD.totalBytes() - SD.usedBytes();

            // Ensure there is more than 25MB of free space on SD card before creating a file
            if (freeSpace > (uint64_t)(25 * 1024 * 1024))
            {
              // Create the file
              file = SD.open(filename.c_str(), FILE_WRITE);
              if (file)
              {
                ESP_LOGI(TAG, "Create log %s", filename.c_str());

                file.print("DateTime,");

                std::string header;
                header.reserve(150);

                for (auto i = 0; i < TotalNumberOfCells(); i++)
                {
                  std::string n;
                  n = std::to_string(i);

                  header.clear();

                  header.append("VoltagemV_")
                      .append(n)
                      .append(",InternalTemp_")
                      .append(n)
                      .append(",ExternalTemp_")
                      .append(n)
                      .append(",Bypass_")
                      .append(n)
                      .append(",PWM_")
                      .append(n)
                      .append(",BypassOverTemp_")
                      .append(n)
                      .append(",BadPackets_")
                      .append(n)
                      .append(",BalancemAh_")
                      .append(n);

                  if (i < TotalNumberOfCells() - 1)
                  {
                    header.append(",");
                  }
                  else
                  {
                    header.append("\r\n");
                  }

                  file.write((const uint8_t *)header.c_str(), header.length());
                }
              }
            }
            else
            {
              ESP_LOGE(TAG, "SD card has less than 25MiB remaining, logging stopped");
              // We had an error, so switch off logging (this is only in memory so not written perm.)
              mysettings.loggingEnabled = false;
            }
          }

          if (file && mysettings.loggingEnabled)
          {
            std::string dataMessage;
            dataMessage.reserve(150);

            dataMessage.append(pad_zero(4, (uint16_t)timeinfo.tm_year))
                .append("-")
                .append(pad_zero(2, (uint16_t)timeinfo.tm_mon))
                .append("-")
                .append(pad_zero(2, (uint16_t)timeinfo.tm_mday))
                .append(" ")
                .append(pad_zero(2, (uint16_t)timeinfo.tm_hour))
                .append(":")
                .append(pad_zero(2, (uint16_t)timeinfo.tm_min))
                .append(":")
                .append(pad_zero(2, (uint16_t)timeinfo.tm_sec))
                .append(",");

            for (auto i = 0; i < TotalNumberOfCells(); i++)
            {
              // This may output invalid data when controller is first powered up
              dataMessage.append(std::to_string(cmi[i].voltagemV))
                  .append(",")
                  .append(std::to_string(cmi[i].internalTemp))
                  .append(",")
                  .append(std::to_string(cmi[i].externalTemp))
                  .append(",")
                  .append(cmi[i].inBypass ? "Y" : "N")
                  .append(",")
                  .append(std::to_string((int)((float)cmi[i].PWMValue / (float)255.0 * 100)))
                  .append(",")
                  .append(cmi[i].bypassOverTemp ? "Y" : "N")
                  .append(",")
                  .append(std::to_string(cmi[i].badPacketCount))
                  .append(",")
                  .append(std::to_string(cmi[i].BalanceCurrentCount));

              if (i < TotalNumberOfCells() - 1)
              {
                dataMessage.append(",");
              }
              else
              {
                dataMessage.append("\r\n");
              }

              // Write the string out to file on each cell to avoid generating a huge string
              file.write((const uint8_t *)dataMessage.c_str(), dataMessage.length());

              // Start another string
              dataMessage.clear();
            }
            file.close();

            ESP_LOGD(TAG, "Wrote to SD log");
          }
          else
          {
            ESP_LOGE(TAG, "Failed to create/append SD logging file");
          }

          // Now log the current monitor
          if (mysettings.currentMonitoringEnabled)
          {
            std::string cmon_filename;
            cmon_filename.reserve(32);
            cmon_filename.append("/modbus")
                .append(pad_zero(2, mysettings.currentMonitoringModBusAddress))
                .append("_")
                .append(std::to_string(timeinfo.tm_year))
                .append(pad_zero(2, (uint16_t)timeinfo.tm_mon))
                .append(pad_zero(2, (uint16_t)timeinfo.tm_mday))
                .append(".csv");

            File file2;

            if (SD.exists(cmon_filename.c_str()))
            {
              // Open existing file (assumes there is enough SD card space to log)
              file2 = SD.open(cmon_filename.c_str(), FILE_APPEND);
            }
            else
            {
              // Create a new file
              uint64_t freeSpace = SD.totalBytes() - SD.usedBytes();

              // Ensure there is more than 25MB of free space on SD card before creating a file
              if (freeSpace > (uint64_t)(25 * 1024 * 1024))
              {
                // Create the file
                file2 = SD.open(cmon_filename.c_str(), FILE_WRITE);
                if (file)
                {
                  ESP_LOGI(TAG, "Create log %s", cmon_filename.c_str());
                  file.println("DateTime,valid,voltage,current,mAhIn,mAhOut,DailymAhIn,DailymAhOut,power,temperature,relayState");
                }
              }
              else
              {
                ESP_LOGE(TAG, "SD card has less than 25MiB remaining, logging stopped");
                // We had an error, so switch off logging (this is only in memory so not written perm.)
                mysettings.loggingEnabled = false;
              }
            }

            if (file2 && mysettings.loggingEnabled)
            {

              std::string dataMessage;
              dataMessage.reserve(128);

              dataMessage.append(pad_zero(4, (uint16_t)timeinfo.tm_year))
                  .append("-")
                  .append(pad_zero(2, (uint16_t)timeinfo.tm_mon))
                  .append("-")
                  .append(pad_zero(2, (uint16_t)timeinfo.tm_mday))
                  .append(" ")
                  .append(pad_zero(2, (uint16_t)timeinfo.tm_hour))
                  .append(":")
                  .append(pad_zero(2, (uint16_t)timeinfo.tm_min))
                  .append(":")
                  .append(pad_zero(2, (uint16_t)timeinfo.tm_sec))
                  .append(",");

              dataMessage.append(currentMonitor.validReadings ? "1" : "0")
                  .append(",")
                  .append(float_to_string(currentMonitor.modbus.voltage))
                  .append(",")
                  .append(float_to_string(currentMonitor.modbus.current))
                  .append(",")
                  .append(std::to_string(currentMonitor.modbus.milliamphour_in))
                  .append(",")
                  .append(std::to_string(currentMonitor.modbus.milliamphour_out))
                  .append(",")
                  .append(std::to_string(currentMonitor.modbus.daily_milliamphour_in))
                  .append(",")
                  .append(std::to_string(currentMonitor.modbus.daily_milliamphour_out))
                  .append(",")
                  .append(float_to_string(currentMonitor.modbus.power))
                  .append(",")
                  .append(std::to_string(currentMonitor.modbus.temperature))
                  .append(",")
                  .append(currentMonitor.RelayState ? "1" : "0")
                  .append("\r\n");

              file2.write((const uint8_t *)dataMessage.c_str(), dataMessage.length());
              file2.close();

              ESP_LOGD(TAG, "Wrote current monitor data to SD log");
            }
            else
            {
              ESP_LOGE(TAG, "Failed to create/append SD logging file");
            }
          } // end of logging for current monitor
        }
        else
        {
          ESP_LOGE(TAG, "Invalid datetime");
        }

        // Must be the last thing...
        hal.ReleaseVSPIMutex();
      }
    }
  } // end for loop
}

// Writes a status log of the OUTPUT STATUES to the SD Card in CSV format
[[noreturn]] void sdcardlog_outputs_task(void *)
{
  for (;;)
  {
    // Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (_sd_card_installed &&
        !_avrsettings.programmingModeEnabled &&
        mysettings.loggingEnabled &&
        _controller_state == ControllerState::Running &&
        hal.IsVSPIMutexAvailable())
    {
      ESP_LOGD(TAG, "sdcardlog_outputs_task");

      struct tm timeinfo;
      // getLocalTime has delay() functions in it :-(
      if (getLocalTime(&timeinfo, 1))
      {
        timeinfo.tm_year += 1900;
        // Month is 0 to 11 based!
        timeinfo.tm_mon++;

        std::string filename;
        filename.reserve(32);
        filename.append("/output_status_").append(std::to_string(timeinfo.tm_year)).append(pad_zero(2, (uint16_t)timeinfo.tm_mon)).append(pad_zero(2, (uint16_t)timeinfo.tm_mday)).append(".csv");

        File file;

        // Prevent other devices using the VSPI bus
        if (hal.GetVSPIMutex())
        {
          if (SD.exists(filename.c_str()))
          {
            // Open existing file (assumes there is enough SD card space to log)
            file = SD.open(filename.c_str(), FILE_APPEND);
          }
          else
          {
            // Create a new file
            uint64_t freeSpace = SD.totalBytes() - SD.usedBytes();

            // Ensure there is more than 25MB of free space on SD card before creating a file
            if (freeSpace > (uint64_t)(25 * 1024 * 1024))
            {
              // Create the file
              file = SD.open(filename.c_str(), FILE_WRITE);
              if (file)
              {
                ESP_LOGD(TAG, "Create log %s", filename.c_str());

                std::string header;
                header.reserve(128);

                header.append("DateTime,TCA6408,TCA9534,");

                for (uint8_t i = 0; i < RELAY_TOTAL; i++)
                {
                  header.append("Output_").append(std::to_string(i));
                  if (i < RELAY_TOTAL - 1)
                  {
                    header.append(",");
                  }
                }
                header.append("\r\n");
                file.write((const uint8_t *)header.c_str(), header.length());
              }
            }
            else
            {
              ESP_LOGE(TAG, "SD card has less than 25MiB remaining, logging stopped");
            }
          }

          if (file && mysettings.loggingEnabled)
          {

            std::string dataMessage;
            dataMessage.reserve(128);

            dataMessage.append(pad_zero(4, (uint16_t)timeinfo.tm_year))
                .append("-")
                .append(pad_zero(2, (uint16_t)timeinfo.tm_mon))
                .append("-")
                .append(pad_zero(2, (uint16_t)timeinfo.tm_mday))
                .append(" ")
                .append(pad_zero(2, (uint16_t)timeinfo.tm_hour))
                .append(":")
                .append(pad_zero(2, (uint16_t)timeinfo.tm_min))
                .append(":")
                .append(pad_zero(2, (uint16_t)timeinfo.tm_sec))
                .append(",")
                .append(uint8_to_binary_string(hal.LastTCA6408Value()))
                .append(",")
                .append(uint8_to_binary_string(hal.LastTCA9534APWRValue()))
                .append(",");

            for (uint8_t i = 0; i < RELAY_TOTAL; i++)
            {
              // This may output invalid data when controller is first powered up
              dataMessage.append(previousRelayState[i] == RelayState::RELAY_ON ? "Y" : "N");
              if (i < RELAY_TOTAL - 1)
              {
                dataMessage.append(",");
              }
            }
            dataMessage.append("\r\n");
            file.write((const uint8_t *)dataMessage.c_str(), dataMessage.length());
            file.close();

            ESP_LOGD(TAG, "Wrote to SD log");
          }
          else
          {
            ESP_LOGE(TAG, "Failed to create/append SD logging file");
          }
        }
        else
        {
          ESP_LOGE(TAG, "Invalid datetime");
        }

        // Must be the last thing...
        hal.ReleaseVSPIMutex();
      } // end if
    }   // end if
  }     // end for loop
}

// Switch the LED off (triggered by timer on 100ms delay)
void ledoff(const TimerHandle_t)
{
  LED(RGBLED::OFF);
}

void ProcessTCA6408Input_States(uint8_t v)
{
  // P0=A
  InputState[0] = (v & B00000001) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
  // P1=B
  InputState[1] = (v & B00000010) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
  // P2=C
  InputState[2] = (v & B00000100) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
  // P3=D
  InputState[3] = (v & B00001000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
}

void ProcessTCA9534Input_States(uint8_t v)
{
  // P4= J13 PIN 1 = WAKE UP TFT FOR DISPLAYS WITHOUT TOUCH
  // Also SW1 on V4.4 boards
  InputState[4] = (v & B00010000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
  // P6 = spare I/O (on PCB pin)
  // Also SW2 on V4.4 boards
  InputState[5] = (v & B01000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
  // P7 = Emergency Stop
  InputState[6] = (v & B10000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;

  // Emergency Stop (J1) has triggered
  if (InputState[6] == enumInputState::INPUT_LOW)
  {
    emergencyStop = true;
  }

  if (InputState[4] == enumInputState::INPUT_LOW)
  {
    // LEFT BUTTON PUSH
    // Wake screen on pin going low (SW1 on V4.4 boards)
    if (_screen_awake)
    {
      PageBackward();
    }
    else
    {
      wake_up_tft(false);
    }
  }
  if (InputState[5] == enumInputState::INPUT_LOW)
  {
    // RIGHT BUTTON PUSH
    // Wake screen on pin going low (SW1 on V4.4 boards)
    if (_screen_awake)
    {
      PageForward();
    }
    else
    {
      wake_up_tft(false);
    }
  }
}

// Handles interrupt requests raised by ESP32 ISR routines
[[noreturn]] void interrupt_task(void *)
{
  uint32_t ulInterruptStatus;

  for (;;)
  {
    // ULONG_MAX
    xTaskNotifyWait(0, ULONG_MAX, &ulInterruptStatus, portMAX_DELAY);

    if ((ulInterruptStatus & ISRTYPE::TCA6416A) != 0x00)
    {
      ESP_LOGD(TAG, "tca6416a_isr");

      // Emulate both 9534 and 6408 being installed by mimicking the
      // registers and processing the status as needed.
      hal.ReadTCA6416InputRegisters();

      // Read ports
      // The 9534 deals with internal LED outputs and spare IO on J10
      ProcessTCA9534Input_States(hal.LastTCA9534APWRValue());
      // Read ports A/B/C/D inputs (on TCA6408)
      ProcessTCA6408Input_States(hal.LastTCA6408Value());
    }

    if ((ulInterruptStatus & ISRTYPE::TCA6408A) != 0x00)
    {
      ESP_LOGD(TAG, "tca6408_isr");
      // Read ports A/B/C/D inputs (on TCA6408)
      ProcessTCA6408Input_States(hal.ReadTCA6408InputRegisters());
    }

    if ((ulInterruptStatus & ISRTYPE::TCA9534) != 0x00)
    {
      ESP_LOGD(TAG, "tca9534_isr");
      // Read ports
      // The 9534 deals with internal LED outputs and spare IO on J10
      ProcessTCA9534Input_States(hal.ReadTCA9534InputRegisters());
    }
  }
}

void IRAM_ATTR InterruptTrigger(ISRTYPE isrvalue)
{
  if (interrupt_task_handle != nullptr)
  {
    auto wokenTask = pdFALSE;
    xTaskNotifyFromISR(interrupt_task_handle, isrvalue, eNotifyAction::eSetBits, &wokenTask);
    if (wokenTask == pdTRUE)
    {
      portYIELD_FROM_ISR(wokenTask);
    }
  }
}

// Triggered when TCA6416A INT pin goes LOW
// Found on V4.4 controller PCB's onwards
void IRAM_ATTR TCA6416AInterrupt()
{
  InterruptTrigger(ISRTYPE::TCA6416A);
}
// Triggered when TCA6408 INT pin goes LOW
void IRAM_ATTR TCA6408Interrupt()
{
  InterruptTrigger(ISRTYPE::TCA6408A);
}
// Triggered when TCA9534A INT pin goes LOW
void IRAM_ATTR TCA9534AInterrupt()
{
  InterruptTrigger(ISRTYPE::TCA9534);
}

const char *packetType(uint8_t cmd)
{
  switch (cmd)
  {
  case COMMAND::ResetBadPacketCounter:
    return "ResetC";
    break;
  case COMMAND::ReadVoltageAndStatus:
    return "RdVolt";
    break;
  case COMMAND::Identify:
    return "Ident";
    break;
  case COMMAND::ReadTemperature:
    return "RdTemp";
    break;
  case COMMAND::ReadBadPacketCounter:
    return "RdBadPkC";
    break;
  case COMMAND::ReadSettings:
    return "RdSettin";
    break;
  case COMMAND::WriteSettings:
    return "WriteSet";
    break;
  case COMMAND::ReadBalancePowerPWM:
    return "RdBalanc";
    break;
  case COMMAND::Timing:
    return "Timing";
    break;
  case COMMAND::ReadBalanceCurrentCounter:
    return "Current";
    break;
  case COMMAND::ReadPacketReceivedCounter:
    return "PktRvd";
    break;
  default:
    return " ??????   ";
  }
}

void dumpPacketToDebug(char indicator, const PacketStruct *buffer)
{
  // Filter on some commands
  // if ((buffer->command & 0x0F) != COMMAND::Timing)    return;

  ESP_LOGD(TAG, "%c %02X-%02X H:%02X C:%02X SEQ:%04X CRC:%04X %s",
           indicator,
           buffer->start_address,
           buffer->end_address,
           buffer->hops,
           buffer->command,
           buffer->sequence,
           buffer->crc,
           packetType(buffer->command & 0x0F));

  // ESP_LOG_BUFFER_HEX("packet", &(buffer->moduledata[0]), sizeof(buffer->moduledata), ESP_LOG_DEBUG);
}

const char *ControllerStateString(ControllerState value)
{
  switch (value)
  {
  case ControllerState::PowerUp:
    return "PowerUp";
  case ControllerState::NoWifiConfiguration:
    return "NoWIFI";
  case ControllerState::Stabilizing:
    return "Stabilizing";
  case ControllerState::Running:
    return "Running";
  case ControllerState::Unknown:
    return "Unknown";
  }

  return "?";
}

void SetControllerState(ControllerState newState)
{
  if (_controller_state != newState)
  {
    ESP_LOGI(TAG, "** Controller changed state from %s to %s **", ControllerStateString(_controller_state), ControllerStateString(newState));

    _controller_state = newState;

    switch (_controller_state)
    {
    case ControllerState::PowerUp:
      // Purple during start up, don't use the LED as thats not setup at this state
      hal.Led(RGBLED::Purple);
      break;
    case ControllerState::Stabilizing:
      LED(RGBLED::Yellow);
      break;
    case ControllerState::NoWifiConfiguration:
      LED(RGBLED::White);
      break;
    case ControllerState::Running:
      LED(RGBLED::Green);
      break;
    case ControllerState::Unknown:
      // Do nothing
      break;
    }
  }
}

uint16_t minutesSinceMidnight()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    return 0;
  }
  else
  {
    return ((uint16_t)timeinfo.tm_hour * 60) + (uint16_t)timeinfo.tm_min;
  }
}

[[noreturn]] void replyqueue_task(void *)
{
  for (;;)
  {
    if (reply_q_handle != nullptr)
    {
      PacketStruct ps;
      if (xQueueReceive(reply_q_handle, &ps, portMAX_DELAY) == pdPASS)
      {
#if defined(PACKET_LOGGING_RECEIVE)
// Process decoded incoming packet
// dumpPacketToDebug('R', &ps);
#endif

        if (!receiveProc.ProcessReply(&ps))
        {
          // Error blue
          LED(RGBLED::Blue);

          ESP_LOGE(TAG, "Packet Failed");

          // SERIAL_DEBUG.print(F("*FAIL*"));
          // dumpPacketToDebug('F', &ps);
        }
      }
    }
  }
}

void onPacketReceived()
{
  PacketStruct ps;
  memcpy(&ps, SerialPacketReceiveBuffer, sizeof(PacketStruct));

  if ((ps.command & 0x0F) == COMMAND::Timing)
  {
    // Timestamp at the earliest possible moment
    auto t = millis();
    ps.moduledata[2] = (t & 0xFFFF0000) >> 16;
    ps.moduledata[3] = t & (uint32_t)0x0000FFFF;
    // Ensure CRC is correct
    ps.crc = CRC16::CalculateArray((uint8_t *)&ps, sizeof(PacketStruct) - 2);
  }

  if (xQueueSendToBack(reply_q_handle, &ps, (TickType_t)100) != pdPASS)
  {
    ESP_LOGE(TAG, "Reply Q full");
  }

  // ESP_LOGI(TAG,"Reply Q length %i",replyQueue.getCount());
}

[[noreturn]] void transmit_task(void *)
{
  for (;;)
  {
    PacketStruct transmitBuffer;
    if (request_q_handle != nullptr)
    {
      if (xQueueReceive(request_q_handle, &transmitBuffer, portMAX_DELAY) == pdPASS)
      {

        sequence++;
        transmitBuffer.sequence = sequence;

        if (transmitBuffer.command == COMMAND::Timing)
        {
          // Timestamp at the last possible moment
          auto t = millis();
          transmitBuffer.moduledata[0] = (t & 0xFFFF0000) >> 16;
          transmitBuffer.moduledata[1] = t & (uint32_t)0x0000FFFF;
        }

        transmitBuffer.crc = CRC16::CalculateArray((uint8_t *)&transmitBuffer, sizeof(PacketStruct) - 2);
        myPacketSerial.sendBuffer((byte *)&transmitBuffer);

        // Output the packet we just transmitted to debug console
        // #if defined(PACKET_LOGGING_SEND)
        //      dumpPacketToDebug('S', &transmitBuffer);
        // #endif
      }

      // Delay based on comms speed, ensure the first module has time to process and clear the request
      // before sending another packet
      uint16_t delay_ms = 900;

      if (mysettings.baudRate == 9600)
      {
        delay_ms = 450;
      }
      else if (mysettings.baudRate == 5000)
      {
        delay_ms = 700;
      }

      // Delay whilst cell module processes request
      vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
  }
}

// Runs the rules and populates rule_outcome array with true/false for each rule
// Rules based on module parameters/readings like voltage and temperature
// are only processed once every module has returned at least 1 reading/communication
void ProcessRules()
{
  rules.ClearValues();
  rules.ClearWarnings();
  rules.ClearErrors();

  rules.rule_outcome[Rule::BMSError] = false;

  auto totalConfiguredModules = TotalNumberOfCells();
  if (totalConfiguredModules > maximum_controller_cell_modules)
  {
    // System is configured with more than maximum modules - abort!
    rules.SetError(InternalErrorCode::TooManyModules);
  }

  if (receiveProc.totalModulesFound > 0 && receiveProc.totalModulesFound != totalConfiguredModules)
  {
    // Found more or less modules than configured for
    rules.SetError(InternalErrorCode::ModuleCountMismatch);
  }

  // Communications error...
  if (receiveProc.HasCommsTimedOut())
  {
    rules.SetError(InternalErrorCode::CommunicationsError);
  }

  if (rules.rule_outcome[Rule::EmergencyStop])
  {
    // Lowest 3 bits are RGB led GREEN/RED/BLUE
    rules.SetError(InternalErrorCode::ErrorEmergencyStop);
  }

  // Raise error is the current shunt stops responding for over 45 seconds
  if (mysettings.currentMonitoringEnabled)
  {
    auto secondsSinceLastMessage = (int64_t)((esp_timer_get_time() - currentMonitor.timestamp) / 1000000);
    if (secondsSinceLastMessage > 45)
    {
      rules.SetError(InternalErrorCode::CommunicationsError);
    }
  }

  rules.highestBankRange = 0;
  rules.numberOfBalancingModules = 0;
  uint8_t cellid = 0;
  for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
  {
    for (int8_t i = 0; i < mysettings.totalNumberOfSeriesModules; i++)
    {
      rules.ProcessCell(bank, cellid, &cmi[cellid], mysettings.cellmaxmv);

      if (cmi[cellid].valid && cmi[cellid].settingsCached)
      {

        if (cmi[cellid].BypassThresholdmV != mysettings.BypassThresholdmV)
        {
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBypassVoltage);
        }

        if (cmi[cellid].BypassOverTempShutdown != mysettings.BypassOverTempShutdown)
        {
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBypassTemperature);
        }

        if (cmi[cellid].inBypass)
        {
          rules.numberOfBalancingModules++;
        }

        if (cmi[0].settingsCached && cmi[cellid].CodeVersionNumber != cmi[0].CodeVersionNumber)
        {
          // Do all the modules have the same version of code as module zero?
          rules.SetWarning(InternalWarningCode::ModuleInconsistantCodeVersion);
        }

        if (cmi[0].settingsCached && cmi[cellid].BoardVersionNumber != cmi[0].BoardVersionNumber)
        {
          // Do all the modules have the same hardware revision?
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBoardRevision);
        }
      }

      cellid++;
    }
    rules.ProcessBank(bank);
  }

  // Need to call these even if Dynamic is switch off, as it seeds the internal variables with the correct values
  rules.CalculateDynamicChargeVoltage(&mysettings, cmi);
  rules.CalculateDynamicChargeCurrent(&mysettings, cmi);

  if (mysettings.loggingEnabled && !_sd_card_installed && !_avrsettings.programmingModeEnabled)
  {
    rules.SetWarning(InternalWarningCode::LoggingEnabledNoSDCard);
  }

  if (_avrsettings.programmingModeEnabled)
  {
    rules.SetWarning(InternalWarningCode::AVRProgrammingMode);
  }

  if (rules.invalidModuleCount > 0)
  {
    // Some modules are not yet valid
    rules.SetError(InternalErrorCode::WaitingForModulesToReply);
  }

  if (_controller_state == ControllerState::Running && rules.zeroVoltageModuleCount > 0)
  {
    rules.SetError(InternalErrorCode::ZeroVoltModule);
  }

  rules.RunRules(
      mysettings.rulevalue,
      mysettings.rulehysteresis,
      emergencyStop,
      minutesSinceMidnight(),
      &currentMonitor);

  if (rules.moduleHasExternalTempSensor == false)
  {
    // NoExternalTempSensor
    rules.SetWarning(InternalWarningCode::NoExternalTempSensor);
  }

  if (mysettings.canbusprotocol != CanBusProtocolEmulation::CANBUS_DISABLED)
  {
    if (!rules.IsChargeAllowed(&mysettings))
    {
      // Charge prevented
      rules.SetWarning(InternalWarningCode::ChargePrevented);
    }
    if (!rules.IsDischargeAllowed(&mysettings))
    {
      // Discharge prevented
      rules.SetWarning(InternalWarningCode::DischargePrevented);
    }
  }

  if (_controller_state == ControllerState::Stabilizing)
  {
    // Check for zero volt modules - not a problem whilst we are in stabilizing start up mode
    if (rules.zeroVoltageModuleCount == 0 && rules.invalidModuleCount == 0)
    {
      // Every module has been read and they all returned a voltage move to running state
      SetControllerState(ControllerState::Running);
    }
  }

  if (rules.rule_outcome[Rule::EmergencyStop])
  {
    // Lowest 3 bits are RGB led GREEN/RED/BLUE
    LED(RGBLED::Red);
  }

  if (rules.numberOfActiveErrors > 0 || rules.WarningCodes[InternalWarningCode::AVRProgrammingMode] != InternalWarningCode::NoWarning)
  {
    // We have active errors, or AVR programming mode is enabled
    ESP_LOGI(TAG, "Active errors=%u", rules.numberOfActiveErrors);

    // Wake up the screen, this will also trigger it to update the display
    wake_up_tft(true);
  }
}

void pulse_relay_off(const TimerHandle_t)
{
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    if (previousRelayPulse[y])
    {
      // We now need to rapidly turn off the relay after a fixed period of time (pulse mode)
      // However we leave the relay and previousRelayState looking like the relay has triggered (it has!)
      // to prevent multiple pulses being sent on each rule refresh
      hal.SetOutputState(y, RelayState::RELAY_OFF);

      previousRelayPulse[y] = false;
    }
  }

  // Fire task to record state of outputs to SD Card
  xTaskNotify(sdcardlog_outputs_task_handle, 0x00, eNotifyAction::eNoAction);
}

[[noreturn]] void rules_task(void *)
{
  for (;;)
  {
    // 3 seconds
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Run the rules
    ProcessRules();

#if defined(RULES_LOGGING)
    for (int8_t r = 0; r < RELAY_RULES; r++)
    {
      if (rules.rule_outcome[r])
      {
        ESP_LOGD(TAG, "Rule outcome %i=TRUE", r);
      }
    }
#endif

    RelayState relay[RELAY_TOTAL];

    // Set defaults based on configuration
    for (int8_t y = 0; y < RELAY_TOTAL; y++)
    {
      relay[y] = mysettings.rulerelaydefault[y] == RELAY_ON ? RELAY_ON : RELAY_OFF;
    }

    // Test the rules (in reverse order)
    for (int8_t n = RELAY_RULES - 1; n >= 0; n--)
    {
      if (rules.rule_outcome[n] == true)
      {
        for (int8_t y = 0; y < RELAY_TOTAL; y++)
        {
          // Dont change relay if its set to ignore/X
          if (mysettings.rulerelaystate[n][y] != RELAY_X)
          {
            if (mysettings.rulerelaystate[n][y] == RELAY_ON)
            {
              relay[y] = RELAY_ON;
            }
            else
            {
              relay[y] = RELAY_OFF;
            }
          }
        }
      }
    }

    uint8_t changes = 0;
    bool firePulse = false;
    for (int8_t n = 0; n < RELAY_TOTAL; n++)
    {
      if (previousRelayState[n] != relay[n])
      {
        ESP_LOGI(TAG, "Set relay %i=%i", n, relay[n] == RelayState::RELAY_ON ? 1 : 0);
        changes++;

        // This would be better if we worked out the bit pattern first and then
        // just submitted that as a single i2c read/write transaction
        hal.SetOutputState(n, relay[n]);

        // Record the previous state of the relay, to use on the next loop
        // to prevent chatter
        previousRelayState[n] = relay[n];

        if (mysettings.relaytype[n] == RELAY_PULSE)
        {
          previousRelayPulse[n] = true;
          firePulse = true;
          ESP_LOGI(TAG, "Relay %i PULSED", n);
        }
      }
    }

    if (firePulse)
    {
      // Fire timer to switch off LED in a few ms
      if (xTimerStart(pulse_relay_off_timer, 10) != pdPASS)
      {
        ESP_LOGE(TAG, "Pulse timer start error");
      }
    }

    if (changes)
    {
      // Fire task to record state of outputs to SD Card
      xTaskNotify(sdcardlog_outputs_task_handle, 0x00, eNotifyAction::eNoAction);
    }
  }
}

// This task periodically adds requests to the queue
// to schedule reading data from the cell modules
// The actual serial comms is handled by the transmit task
[[noreturn]] void enqueue_task(void *)
{
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(mysettings.interpacketgap));

    LED(RGBLED::Green);

    // Fire timer to switch off LED in a few ms
    if (xTimerStart(led_off_timer, 5) != pdPASS)
    {
      ESP_LOGE(TAG, "Timer start error");
    }

    uint16_t i = 0;
    auto max = TotalNumberOfCells();
    uint8_t startmodule = 0;

    while (i < max)
    {
      uint8_t endmodule = (startmodule + maximum_cell_modules_per_packet) - 1;

      // Limit to number of modules we have configured
      if (endmodule > max)
      {
        endmodule = max - 1;
      }

      // Request voltage, but if queue is full, sleep and try again (other threads will reduce the queue)
      prg.sendCellVoltageRequest(startmodule, endmodule);
      // Same for temperature
      prg.sendCellTemperatureRequest(startmodule, endmodule);

      // If any module is in bypass then request PWM reading for whole bank
      for (uint8_t m = startmodule; m <= endmodule; m++)
      {
        if (cmi[m].inBypass)
        {
          prg.sendReadBalancePowerRequest(startmodule, endmodule);
          // We only need 1 reading for whole bank
          break;
        }
      }

      // Move to the next bank
      startmodule = endmodule + 1;
      i += maximum_cell_modules_per_packet;
    }
  }
}

static int s_retry_num = 0;

void formatCurrentDateTime(char *buf, size_t buf_size)
{
  time_t now;
  time(&now);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  strftime(buf, buf_size, "%c", &timeinfo);
}

static void setTimeZone(long offset, unsigned int daylight)
{
  char cst[17] = {0};
  char cdt[17] = "DST";
  char tz[33] = {0};

  if (offset % 3600)
  {
    sprintf(cst, "UTC%ld:%02ld:%02ld", offset / 3600, abs((offset % 3600) / 60), abs(offset % 60));
  }
  else
  {
    sprintf(cst, "UTC%ld", offset / 3600);
  }
  if (daylight != 3600)
  {
    long tz_dst = offset - daylight;
    if (tz_dst % 3600)
    {
      sprintf(cdt, "DST%ld:%02lu:%02lu", tz_dst / 3600, abs((tz_dst % 3600) / 60), abs(tz_dst % 60));
    }
    else
    {
      sprintf(cdt, "DST%ld", tz_dst / 3600);
    }
  }
  sprintf(tz, "%s%s", cst, cdt);
  setenv("TZ", tz, 1);
  ESP_LOGI(TAG, "Timezone=%s", tz);
  tzset();

  char strftime_buf[64];
  formatCurrentDateTime(strftime_buf, sizeof(strftime_buf));
  ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);
}

void configureSNTP(long gmtOffset_sec, int daylightOffset_sec, const char *server1)
{
  ESP_LOGI(TAG, "Request time from %s", server1);

  if (sntp_enabled())
  {
    sntp_stop();
  }
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, server1);
  // sntp_setservername(1, (char*)server2);
  // sntp_setservername(2, (char*)server3);
  sntp_init();

  // TODO: Fix this with native IDF library
  setTimeZone(-gmtOffset_sec, daylightOffset_sec);
  // setenv("TZ", tz, 1);
  // tzset();
}

static void stopMDNS()
{
  mdns_free();
}

static void startMDNS()
{

  // initialize mDNS service
  esp_err_t err = mdns_init();
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "MDNS Init failed: %d", err);
  }
  else
  {
    mdns_hostname_set(hostname);
    mdns_instance_name_set("diybms");
    mdns_service_add(nullptr, "_http", "_tcp", 80, nullptr, 0);
  }
}

// WIFI Event Handler
static void event_handler(void *, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_BSS_RSSI_LOW)
  {
    ESP_LOGW(TAG, "WiFi signal strength low");
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    wifi_isconnected = false;
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    wifi_isconnected = false;
    if (s_retry_num < 200)
    {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "Retry %i, connect to Wifi AP", s_retry_num);
    }
    else
    {
      //  xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
      ESP_LOGE(TAG, "Connect to the Wifi AP failed");
    }
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP)
  {
    wifi_isconnected = false;

    ESP_LOGI(TAG, "IP_EVENT_STA_LOST_IP");

    // Shut down all TCP/IP reliant services
    if (server_running)
    {
      stop_webserver(_myserver);
      server_running = false;
      _myserver = nullptr;
    }
    stopMqtt();
    stopMDNS();

    esp_wifi_disconnect();

    // Try and reconnect
    esp_wifi_connect();
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    wifi_isconnected = true;
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    // xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

    // Start up all the services after TCP/IP is established
    configureSNTP(mysettings.timeZone * 3600 + mysettings.minutesTimeZone * 60, mysettings.daylight ? 3600 : 0, mysettings.ntpServer);

    if (!server_running)
    {
      StartServer();
      server_running = true;
    }

    connectToMqtt();

    startMDNS();

    snprintf(ip_string, sizeof(ip_string), IPSTR, IP2STR(&event->ip_info.ip));

    ESP_LOGI(TAG, "You can access DIYBMS interface at http://%s.local or http://%s", hostname, ip_string);
  }
}

bool LoadWiFiConfig()
{
  return LoadWIFI(&_wificonfig);
}

void BuildHostname()
{
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8)
  {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  // DIYBMS-00000000
  memset(&hostname, 0, sizeof(hostname));
  snprintf(hostname, sizeof(hostname), "DIYBMS-%08X", chipId);
}

void wifi_init_sta(void)
{
  // Don't do it if already connected....
  if (wifi_isconnected)
    return;

  ESP_LOGD(TAG, "starting wifi_init_sta");

  // Create ESP IDF default event loop to service WIFI events
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // s_wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_netif_init());

  esp_netif_t *netif = esp_netif_create_default_wifi_sta();
  assert(netif);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  cfg.static_tx_buf_num = 0;
  cfg.dynamic_tx_buf_num = 32;
  cfg.tx_buf_type = 1;
  cfg.cache_tx_buf_num = 1;
  cfg.static_rx_buf_num = 4;
  cfg.dynamic_rx_buf_num = 32;

  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // esp_event_handler_instance_t instance_any_id;
  // esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &event_handler,
                                                      nullptr,
                                                      nullptr));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &event_handler,
                                                      nullptr,
                                                      nullptr));

  wifi_config_t wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  strncpy((char *)wifi_config.sta.ssid, _wificonfig.wifi_ssid, sizeof(wifi_config.sta.ssid));
  strncpy((char *)wifi_config.sta.password, _wificonfig.wifi_passphrase, sizeof(wifi_config.sta.password));

  ESP_LOGI(TAG, "WIFI SSID: %s", _wificonfig.wifi_ssid);

  // Avoid issues with GPIO39 interrupt firing all the time - disable power saving
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  // Generates WIFI_EVENT_STA_BSS_RSSI_LOW events when RSS goes low
  ESP_ERROR_CHECK(esp_wifi_set_rssi_threshold(-80));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_ERROR_CHECK_WITHOUT_ABORT(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, hostname));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_set_hostname(netif, hostname));

  ESP_LOGI(TAG, "Hostname: %s", hostname);

  ESP_LOGD(TAG, "wifi_init_sta finished");

  /*
  // Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
  // number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above)
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE,
                                         pdFALSE,
                                         portMAX_DELAY);

  // xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually happened.
  if (bits & WIFI_CONNECTED_BIT)
  {
    ESP_LOGI(TAG, "connected to access point");
  }
  else if (bits & WIFI_FAIL_BIT)
  {
    ESP_LOGI(TAG, "Failed to connect to access point");
  }
  else
  {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }

  // The event will not be processed after unregister
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
  vEventGroupDelete(s_wifi_event_group);
  */
}
void _send_canbus_message(uint32_t identifier, uint8_t *buffer, uint8_t length, uint32_t flags)
{
  twai_message_t message;
  message.identifier = identifier;
  message.flags = flags;
  message.data_length_code = length;

  memcpy(&message.data, buffer, length);

  esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(250));

  // Queue message for transmission
  if (result != ESP_OK)
  {
    ESP_LOGE(TAG, "Fail to queue CANBUS message (0x%x)", result);
    canbus_messages_failed_sent++;
  }
  else
  {
    ESP_LOGD(TAG, "Sent CAN message 0x%x", identifier);
    ESP_LOG_BUFFER_HEXDUMP(TAG, &message.data, message.data_length_code, esp_log_level_t::ESP_LOG_DEBUG);
    canbus_messages_sent++;
  }
}

void send_canbus_message(uint32_t identifier, uint8_t *buffer, uint8_t length)
{
  _send_canbus_message(identifier, buffer, length, TWAI_MSG_FLAG_NONE);
}
void send_ext_canbus_message(uint32_t identifier, uint8_t *buffer, uint8_t length)
{
  _send_canbus_message(identifier, buffer, length, TWAI_MSG_FLAG_EXTD);
}

[[noreturn]] void canbus_tx(void *)
{
  for (;;)
  {
    // Delay 1 second
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (mysettings.canbusprotocol == CanBusProtocolEmulation::CANBUS_PYLONTECH)
    {
      // Pylon Tech Battery Emulation
      // https://github.com/PaulSturbo/DIY-BMS-CAN/blob/main/SEPLOS%20BMS%20CAN%20Protocoll%20V1.0.pdf
      // https://www.setfirelabs.com/green-energy/pylontech-can-reading-can-replication
      // https://github.com/juamiso/PYLON_EMU
      // https://www.studocu.com/row/document/abasyn-university/electronics-engineering/can-bus-protocol-pylon-low-voltage-v1/17205338

      /*
      PYLON TECH battery transmits these values....

      CAN ID – followed by 2 to 8 bytes of data:
      0x351 – 14 02 74 0E 74 0E CC 01 – Battery voltage + current limits
      0x355 – 1A 00 64 00 – State of Health (SOH) / State of Charge (SOC)
      0x356 – 4e 13 02 03 04 05 – Voltage / Current / Temp
      0x359 – 00 00 00 00 0A 50 4E – Protection & Alarm flags
      0x35C – C0 00 – Battery charge request flags
      0x35E – 50 59 4C 4F 4E 20 20 20 – Manufacturer name (“PYLON “)

      If you are watching the bus, you will also see a 0x305 ID message which is output by the inverter once per second.
*/
      pylon_message_351();
      vTaskDelay(pdMS_TO_TICKS(20));

      if (_controller_state == ControllerState::Running)
      {
        pylon_message_355();
        vTaskDelay(pdMS_TO_TICKS(20));
        pylon_message_356();
        vTaskDelay(pdMS_TO_TICKS(20));
      }
      pylon_message_359();
      vTaskDelay(pdMS_TO_TICKS(20));
      pylon_message_35c();
      vTaskDelay(pdMS_TO_TICKS(20));
      pylon_message_35e();
      // Delay a little whilst sending packets to give ESP32 some breathing room and not flood the CANBUS
      // vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (mysettings.canbusprotocol == CanBusProtocolEmulation::CANBUS_PYLONFORCEH2 )
    {
      pylonforce_handle_tx();
    }
    else if (mysettings.canbusprotocol == CanBusProtocolEmulation::CANBUS_VICTRON)
    {
      // minimum CAN-IDs required for the core functionality are 0x351, 0x355, 0x356 and 0x35A.

      // 351 message must be sent at least every 3 seconds - or Victron will stop charge/discharge
      victron_message_351();

      // Delay a little whilst sending packets to give ESP32 some breathing room and not flood the CANBUS
      vTaskDelay(pdMS_TO_TICKS(100));

      // Advertise the diyBMS name on CANBUS
      victron_message_370_371();
      victron_message_35e();
      victron_message_35a();
      victron_message_372();
      victron_message_35f();

      vTaskDelay(pdMS_TO_TICKS(100));

      if (_controller_state == ControllerState::Running)
      {
        victron_message_355();
        victron_message_356();

        vTaskDelay(pdMS_TO_TICKS(100));

        // Detail about individual cells
        victron_message_373();
        victron_message_374_375_376_377();
      }
    }
  }
}

[[noreturn]] void canbus_rx(void *)
{
  for (;;)
  {

    if (mysettings.canbusprotocol != CanBusProtocolEmulation::CANBUS_DISABLED)
    {

      // Wait for message to be received
      twai_message_t message;
      esp_err_t res = twai_receive(&message, pdMS_TO_TICKS(10000));
      if (res == ESP_OK)
      {
        canbus_messages_received++;
        ESP_LOGD(TAG, "CANBUS received message ID: %0x, DLC: %d, flags: %0x",
                 message.identifier, message.data_length_code, message.flags);
        if (!(message.flags & TWAI_MSG_FLAG_RTR))
        {
          ESP_LOG_BUFFER_HEXDUMP(TAG, message.data, message.data_length_code, ESP_LOG_DEBUG);
          if (mysettings.canbusprotocol == CanBusProtocolEmulation::CANBUS_PYLONFORCEH2 )
          {
            pylonforce_handle_rx(&message);
          }
        }
      }
      else if (res == ESP_ERR_TIMEOUT)
      {
        /// ignore the timeout or do something
        ESP_LOGE(TAG, "CANBUS timeout");
      }
    }
    else
    {
      // Canbus is disbled, sleep....
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

[[noreturn]] void periodic_task(void *)
{
  uint8_t countdown_influx = mysettings.influxdb_loggingFreqSeconds;
  uint8_t countdown_mqtt1 = 5;
  uint8_t countdown_mqtt2 = 25;

  for (;;)
  {
    // Delay 1 second
    vTaskDelay(pdMS_TO_TICKS(1000));

    countdown_influx--;
    countdown_mqtt1--;
    countdown_mqtt2--;

    if (tftsleep_timer > 0)
    {
      // Timer can get to 0
      tftsleep_timer--;

      IncreaseDelayCounter();
    }

    // ESP_LOGI(TAG, "mqtt1=%u, mqtt2=%u, influx=%u, tftsleep=%u",countdown_mqtt1,countdown_mqtt2,countdown_influx,tftsleep_timer);

    // 5 seconds
    if (countdown_mqtt1 == 0)
    {
      mqtt1(&currentMonitor, &rules);
      countdown_mqtt1 = 5;
    }

    // 25 seconds
    if (countdown_mqtt2 == 0)
    {
      mqtt2(&receiveProc, &prg, prg.queueLength(), &rules, previousRelayState);
      countdown_mqtt2 = 25;
    }

    // Influxdb - Variable interval
    if (countdown_influx == 0)
    {
      countdown_influx = mysettings.influxdb_loggingFreqSeconds;

      if (mysettings.influxdb_enabled && wifi_isconnected && rules.invalidModuleCount == 0 && _controller_state == ControllerState::Running && rules.rule_outcome[Rule::BMSError] == false)
      {
        ESP_LOGI(TAG, "Influx task");
        influx_task_action();
      }
    }

    if (_screen_awake && tftsleep_timer == 0 &&
        (WhatScreenToDisplay() != ScreenTemplateToDisplay::Error && WhatScreenToDisplay() != ScreenTemplateToDisplay::AVRProgrammer))
    {
      // Screen off
      tftsleep();
    }
  }
}

// Do activities which are not critical to the system like background loading of config, or updating timing results etc.
[[noreturn]] void lazy_tasks(void *)
{
  int year_day = -1;
  time_t snapshot_time = 0;

  for (;;)
  {
    // TODO: Perhaps this should be based on some improved logic - based on number of modules in system?
    //  Delay 5.5 seconds

    // ESP_LOGI(TAG, "Sleep");
    auto delay_ticks = pdMS_TO_TICKS(5500);
    vTaskDelay(delay_ticks);

    // Task 1
    ESP_LOGI(TAG, "Task 1");
    //  Send a "ping" message through the cells to get a round trip time
    prg.sendTimingRequest();

    // Check if clock has rolled over to next day, this won't fire exactly on midnight
    // but should be within 0-20 seconds of it.
    if (sntp_enabled())
    {
      time_t now;
      time(&now);
      struct tm timeinfo;
      localtime_r(&now, &timeinfo);

      // Wait for SNTP to get a valid date/time
      if (timeinfo.tm_year > 70)
      {
        // Has day rolled over?
        if (year_day != timeinfo.tm_yday)
        {
          // Reset the current monitor at midnight (ish)
          CurrentMonitorResetDailyAmpHourCounters();

          for (uint8_t i = 0; i < maximum_controller_cell_modules; i++)
          {
            resetModuleMinMaxVoltage(i);
          }
        }
        year_day = timeinfo.tm_yday;

        // Has clock rolled over for snaps?
        if (snapshot_time != 0 && now > snapshot_time)
        {
          char strftime_buf[64];
          strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
          ESP_LOGI(TAG, "Snap timer: %s", strftime_buf);

          history.SnapshotHistory(now, &rules, &currentMonitor);
          snapshot_time = 0;
        }

        if (snapshot_time == 0)
        {
          // Calculate the next time to do a snapshot in 30 minutes
          snapshot_time = (now - (now % 1800)) + 1800;
        }
      }
    }

    // Sleep between sections to give the ESP a chance to do other stuff
    vTaskDelay(delay_ticks);

    // Task 2
    ESP_LOGI(TAG, "Task 2");
    // uint8_t counter = 0;
    //  Find modules that don't have settings cached and request them
    for (uint8_t m = 0; m < TotalNumberOfCells(); m++)
    {
      if (cmi[m].valid && !cmi[m].settingsCached)
      {
        // This will block if the queue length is reached
        prg.sendGetSettingsRequest(m);
      }
    }

    // Sleep between sections to give the ESP a chance to do other stuff
    vTaskDelay(delay_ticks);

    // Task 3
    //  Send these requests to all banks of modules
    uint8_t i = 0;
    uint8_t max = TotalNumberOfCells();

    uint8_t startmodule = 0;

    while (i < max)
    {
      uint8_t endmodule = (startmodule + maximum_cell_modules_per_packet) - 1;

      // Limit to number of modules we have configured
      if (endmodule > max)
      {
        endmodule = max - 1;
      }

      ESP_LOGD(TAG, "Task 3, s=%i e=%i", startmodule, endmodule);
      prg.sendReadBalanceCurrentCountRequest(startmodule, endmodule);
      prg.sendReadPacketsReceivedRequest(startmodule, endmodule);
      prg.sendReadBadPacketCounter(startmodule, endmodule);

      // Delay per bank/loop
      vTaskDelay(delay_ticks);

      // Move to the next bank
      startmodule = endmodule + 1;
      i += maximum_cell_modules_per_packet;
    } // end while

  } // end for
}

void resetAllRules()
{
  // Clear all rules
  for (int8_t r = 0; r < RELAY_RULES; r++)
  {
    rules.rule_outcome[r] = false;
  }
}

bool CaptureSerialInput(char *buffer, int buffersize, bool OnlyDigits, bool ShowPasswordChar)
{
  int length = 0;
  unsigned long timer = millis() + 30000;

  // Ensure buffer is all zeros
  memset(buffer, 0, buffersize);

  while (true)
  {
    // We should add a timeout in here, and return FALSE when we abort....

    // fgetc is blocking call
    int data = fgetc(stdin);

    if (data == EOF)
    {
      // Ignore
      // Abort after 30 seconds of inactivity
      if (millis() > timer)
        return false;
    }
    else if (data == '\b' || data == '\177')
    { // BS and DEL
      if (length)
      {
        length--;
        fputs("\b \b", stdout);
      }
    }

    else if (data == '\n' || data == '\r')
    {
      if (length > 0)
      {
        fputs("\n", stdout); // output newline

        // Mark end of string
        buffer[length] = '\0';

        // Return to caller
        return true;
      }

      length = 0;
    }
    else if (length < buffersize - 1)
    {
      // Reset timer on serial input
      timer = millis() + 30000;

      if (OnlyDigits && (data < '0' || data > '9'))
      {
        // We need to filter out non-digit characters
      }
      else
      {
        buffer[length++] = (char)data;
        fputc(ShowPasswordChar ? '*' : (char)data, stdout);
      }
    }
  }
}

bool DeleteWiFiConfigFromSDCard()
{
  bool ret = false;
  if (_sd_card_installed)
  {
    ESP_LOGI(TAG, "Delete check for %s", wificonfigfilename);

    if (hal.GetVSPIMutex())
    {
      if (SD.exists(wificonfigfilename))
      {
        ESP_LOGI(TAG, "Deleted file %s", wificonfigfilename);
        ret = SD.remove(wificonfigfilename);
      }

      hal.ReleaseVSPIMutex();
    }
  }

  return ret;
}

[[noreturn]] void TerminalBasedWifiSetup()
{
  SetControllerState(ControllerState::NoWifiConfiguration);

  if (_tft_screen_available)
  {
    // Show the prompt "configure wifi settings" on TFT screen
    if (hal.GetDisplayMutex())
    {
      PrepareTFT_ControlState();
      DrawTFT_ControlState();
      hal.ReleaseDisplayMutex();
      hal.TFTScreenBacklight(true);
    }
  }

  fputs("\n\nDIYBMS CONTROLLER - Scanning Wifi\n\n", stdout);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_scan_start(nullptr, true);

  // Max of 32 stations to return
  uint16_t number = 32;
  wifi_ap_record_t ap_info[32];
  uint16_t ap_count = 0;
  memset(ap_info, 0, sizeof(ap_info));

  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

  // ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);

  fputs("Number\tSSID Name                             RSSI\tChannel\n", stdout);
  for (int i = 0; (i < 32) && (i < ap_count); i++)
  {
    char text[10];
    itoa(i, text, 10);
    fputs(text, stdout);
    fputs(":\t", stdout);
    fputs((char *)ap_info[i].ssid, stdout);

    // Pad out to 38 characters on screen
    size_t len = strlen((char *)ap_info[i].ssid);
    for (; len < 38; len++)
    {
      fputc(' ', stdout);
    }

    itoa(ap_info[i].rssi, text, 10);
    fputs(text, stdout);
    fputc('\t', stdout);
    itoa(ap_info[i].primary, text, 10);
    fputs(text, stdout);

    // New line
    fputc('\n', stdout);
  }

  // Green
  fputs("Enter the NUMBER of the Wifi network to connect to: ", stdout);

  bool result;
  char buffer[64];
  result = CaptureSerialInput(buffer, 3, true, false);
  if (result)
  {
    int index = atoi(buffer);
    fputs("", stdout);
    fputs("Enter the password to use when connecting to '", stdout);
    fputs((char *)ap_info[index].ssid, stdout);
    fputs("':", stdout);

    result = CaptureSerialInput(buffer, sizeof(buffer), false, true);

    if (result)
    {
      wifi_eeprom_settings config;
      memset(&config, 0, sizeof(config));
      strncpy(config.wifi_ssid, (char *)ap_info[index].ssid, sizeof(config.wifi_ssid));
      strncpy(config.wifi_passphrase, buffer, sizeof(config.wifi_passphrase));
      SaveWIFI(&config);
      // Now delete the WIFICONFIG backup from the SDCard to prevent boot loops/resetting defaults
      DeleteWiFiConfigFromSDCard();
    }
  }

  fputc('\n', stdout);
  fputc('\n', stdout);
  for (size_t i = 5; i > 0; i--)
  {
    char n[5];
    itoa(i, n, 10);
    fputs("Rebooting in ", stdout);
    fputs(n, stdout);
    fputs(" seconds\n", stdout);
    delay(1000);
  }

  esp_restart();
}

// CHECK HERE FOR THE PRESENCE OF A /wifi.json CONFIG FILE ON THE SD CARD TO AUTOMATICALLY CONFIGURE WIFI
bool LoadWiFiConfigFromSDCard(bool existingConfigValid)
{
  bool ret = false;
  if (_sd_card_installed)
  {

    ESP_LOGI(TAG, "Checking for %s", wificonfigfilename);

    if (hal.GetVSPIMutex())
    {
      if (SD.exists(wificonfigfilename))
      {
        ESP_LOGD(TAG, "Found file %s", wificonfigfilename);

        StaticJsonDocument<3000> json;
        File file = SD.open(wificonfigfilename);
        DeserializationError error = deserializeJson(json, file);
        file.close();
        // Release Mutex as quickly as possible
        hal.ReleaseVSPIMutex();
        if (error != DeserializationError::Ok)
        {
          ESP_LOGE(TAG, "Error deserialize JSON");
        }
        else
        {
          ESP_LOGD(TAG, "Deserialized %s", wificonfigfilename);

          JsonObject wifi = json["wifi"];

          wifi_eeprom_settings _new_config;

          // Clear config
          memset(&_new_config, 0, sizeof(_new_config));

          String ssid = wifi["ssid"].as<String>();
          String password = wifi["password"].as<String>();
          ssid.toCharArray(_new_config.wifi_ssid, sizeof(_new_config.wifi_ssid));
          password.toCharArray(_new_config.wifi_passphrase, sizeof(_new_config.wifi_passphrase));

          // Our configuration is different, so store the details in EEPROM and flash the LED a few times
          if (existingConfigValid == false || strcmp(_wificonfig.wifi_ssid, _new_config.wifi_ssid) != 0 || strcmp(_wificonfig.wifi_passphrase, _new_config.wifi_passphrase) != 0)
          {
            memcpy(&_wificonfig, &_new_config, sizeof(_new_config));
            ESP_LOGI(TAG, "Wifi config is different, saving");
            SaveWIFI(&_new_config);
            ret = true;
          }
          else
          {
            ESP_LOGI(TAG, "Wifi JSON config is identical, ignoring");
          }
        }
      }

      hal.ReleaseVSPIMutex();
    }
  }
  return ret;
}

static void tft_interrupt_attach(void *)
{
  attachInterrupt(TOUCH_IRQ, TFTScreenTouchInterrupt, FALLING);
}

struct log_level_t
{
  const char *tag;
  esp_log_level_t level;
};

// Default log levels to use for various components.
const std::array<log_level_t, 21> log_levels =
    {
        log_level_t{.tag = "*", .level = ESP_LOG_DEBUG},
        {.tag = "wifi", .level = ESP_LOG_WARN},
        {.tag = "dhcpc", .level = ESP_LOG_WARN},
        {.tag = "diybms", .level = ESP_LOG_DEBUG},
        {.tag = "diybms-avrisp", .level = ESP_LOG_INFO},
        {.tag = "diybms-hal", .level = ESP_LOG_DEBUG},
        {.tag = "diybms-influxdb", .level = ESP_LOG_INFO},
        {.tag = "diybms-rx", .level = ESP_LOG_INFO},
        {.tag = "diybms-tx", .level = ESP_LOG_INFO},
        {.tag = "diybms-rules", .level = ESP_LOG_INFO},
        {.tag = "diybms-softap", .level = ESP_LOG_INFO},
        {.tag = "diybms-tft", .level = ESP_LOG_INFO},
        {.tag = "diybms-victron", .level = ESP_LOG_INFO},
        {.tag = "diybms-webfuncs", .level = ESP_LOG_INFO},
        {.tag = "diybms-webpost", .level = ESP_LOG_INFO},
        {.tag = "diybms-webreq", .level = ESP_LOG_INFO},
        {.tag = "diybms-web", .level = ESP_LOG_INFO},
        {.tag = "diybms-set", .level = ESP_LOG_INFO},
        {.tag = "diybms-mqtt", .level = ESP_LOG_INFO},
        {.tag = "diybms-pylon", .level = ESP_LOG_INFO},
        {.tag = "curmon", .level = ESP_LOG_INFO}};

void consoleConfigurationCheck()
{
  // Allow user to press SPACE BAR key on serial terminal to enter text based WIFI setup
  // Make sure this is always output regardless of the IDF logging level
  fputs("\n\n\nPress SPACE BAR to enter console WiFi configuration....\n\n\n", stdout);
  for (size_t i = 0; i < (5000 / 250); i++)
  {
    fputc('.', stdout);
    auto ch = (uint8_t)fgetc(stdin);
    // SPACE BAR
    if (ch == 32)
    {
      TerminalBasedWifiSetup();
    }
    delay(250);
  }
  fputs("\n\nNo key press detected\n", stdout);
}

// Mark running firmware as ok/valid.
// Believe this is also duplicated into ArduinoESP32 library/runtime
extern "C" void confirmFirmwareIsValid()
{
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t ota_state;
  if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
  {
    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
    {
      // Validate image some how, then call:
      ESP_LOGI(TAG, "esp_ota_mark_app_valid_cancel_rollback");
      esp_ota_mark_app_valid_cancel_rollback();
      // If needed: esp_ota_mark_app_invalid_rollback_and_reboot();
    }
  }
}

void resumeTasksAfterFirmwareUpdateFailure()
{
  vTaskResume(updatetftdisplay_task_handle);
  vTaskResume(avrprog_task_handle);
  vTaskResume(sdcardlog_task_handle);
  vTaskResume(sdcardlog_outputs_task_handle);
  vTaskResume(rs485_tx_task_handle);
  vTaskResume(service_rs485_transmit_q_task_handle);
  vTaskResume(canbus_tx_task_handle);
  vTaskResume(transmit_task_handle);
  vTaskResume(lazy_task_handle);
  vTaskResume(canbus_rx_task_handle);
}
void suspendTasksDuringFirmwareUpdate()
{
  vTaskSuspend(updatetftdisplay_task_handle);
  vTaskSuspend(avrprog_task_handle);
  vTaskSuspend(sdcardlog_task_handle);
  vTaskSuspend(sdcardlog_outputs_task_handle);
  vTaskSuspend(rs485_tx_task_handle);
  vTaskSuspend(service_rs485_transmit_q_task_handle);
  vTaskSuspend(canbus_tx_task_handle);
  vTaskSuspend(transmit_task_handle);
  vTaskSuspend(lazy_task_handle);
  vTaskSuspend(canbus_rx_task_handle);
}

void setup()
{
  esp_chip_info_t chip_info;

  // Configure log levels
  for (auto log : log_levels)
  {
    esp_log_level_set(log.tag, log.level);
  }

  esp_chip_info(&chip_info);

  ESP_LOGI(TAG, R"(


               _          __ 
  _|  o       |_)  |\/|  (_  
 (_|  |  \/   |_)  |  |  __)
         /

CONTROLLER - ver:%s compiled %s
ESP32 Chip model = %u, Rev %u, Cores=%u, Features=%u)",
           GIT_VERSION, COMPILE_DATE_TIME,
           chip_info.model, chip_info.revision, chip_info.cores, chip_info.features);

  // ESP_ERROR_CHECK_WITHOUT_ABORT(esp_bt_controller_disable());
  BuildHostname();
  hal.ConfigurePins();
  hal.ConfigureI2C(TCA6408Interrupt, TCA9534AInterrupt, TCA6416AInterrupt);
  hal.ConfigureVSPI();

  confirmFirmwareIsValid();

  _avrsettings.inProgress = false;
  _avrsettings.programmingModeEnabled = false;

  // See if we can get a sensible reading from the TFT touch chip XPT2046
  // if we can, then a screen is fitted, so enable it
  // Touch is on VSPI bus
  _tft_screen_available = hal.IsScreenAttached();
  SetControllerState(ControllerState::PowerUp);
  init_tft_display();
  hal.Led(0);

  InitializeNVS();

  // Switch CAN chip TJA1051T/3 ON
  hal.CANBUSEnable(true);
  hal.ConfigureCAN();

  if (!LittleFS.begin(false))
  {
    ESP_LOGE(TAG, "LittleFS mount failed, did you upload file system image?");
    hal.Halt(RGBLED::White);
  }

  ESP_LOGI(TAG, "LittleFS mounted, total=%u, used=%u", LittleFS.totalBytes(), LittleFS.usedBytes());

  mountSDCard();

  // consoleConfigurationCheck needs to be after SD card mount, as it attempts to remove wifi.json if it exists
  consoleConfigurationCheck();

  // Retrieve the EEPROM WIFI settings
  bool EepromConfigValid = LoadWiFiConfig();

  if (LoadWiFiConfigFromSDCard(EepromConfigValid))
  {
    // We need to reload the configuration, as it was updated...
    EepromConfigValid = LoadWiFiConfig();
  }

  // Pre configure the array
  memset(&cmi, 0, sizeof(cmi));
  for (uint8_t i = 0; i < maximum_controller_cell_modules; i++)
  {
    clearModuleValues(i);
  }

  history.Clear();

  resetAllRules();

  LoadConfiguration(&mysettings);
  ValidateConfiguration(&mysettings);

  if (!EepromConfigValid)
  {
    // We don't have a valid WIFI configuration, so force terminal based setup
    TerminalBasedWifiSetup();
  }

  // Check and configure internal current monitor (if it exists)
  if (hal.GetVSPIMutex())
  {
    currentMonInternal_init();
    hal.ReleaseVSPIMutex();
  }

  // Serial pins IO2/IO32
  SERIAL_DATA.begin(mysettings.baudRate, SERIAL_8N1, 2, 32); // Serial for comms to modules

  myPacketSerial.begin(&SERIAL_DATA, &onPacketReceived, sizeof(PacketStruct), SerialPacketReceiveBuffer, sizeof(SerialPacketReceiveBuffer));

  SetupRS485();

  // Create queue for transmit, each request could be MAX_SEND_RS485_PACKET_LENGTH bytes long, depth of 3 items
  rs485_transmit_q_handle = xQueueCreate(3, MAX_SEND_RS485_PACKET_LENGTH);
  assert(rs485_transmit_q_handle);

  request_q_handle = xQueueCreate(30, sizeof(PacketStruct));
  assert(request_q_handle);
  prg.setQueueHandle(request_q_handle);

  reply_q_handle = xQueueCreate(4, sizeof(PacketStruct));
  assert(reply_q_handle);

  led_off_timer = xTimerCreate("LEDOFF", pdMS_TO_TICKS(100), pdFALSE, (void *)1, &ledoff);
  assert(led_off_timer);

  pulse_relay_off_timer = xTimerCreate("PULSE", pdMS_TO_TICKS(250), pdFALSE, (void *)2, &pulse_relay_off);
  assert(pulse_relay_off_timer);

  tftwake_timer = xTimerCreate("TFTWAKE", pdMS_TO_TICKS(2), pdFALSE, (void *)3, &tftwakeup);
  assert(tftwake_timer);

  xTaskCreate(voltageandstatussnapshot_task, "snap", 1950, nullptr, 1, &voltageandstatussnapshot_task_handle);
  xTaskCreate(updatetftdisplay_task, "tftupd", 2000, nullptr, 0, &updatetftdisplay_task_handle);
  xTaskCreate(avrprog_task, "avrprog", 2450, &_avrsettings, configMAX_PRIORITIES - 3, &avrprog_task_handle);

  // High priority task
  xTaskCreate(interrupt_task, "int", 2000, nullptr, configMAX_PRIORITIES - 1, &interrupt_task_handle);
  xTaskCreate(sdcardlog_task, "sdlog", 4096, nullptr, 0, &sdcardlog_task_handle);
  xTaskCreate(sdcardlog_outputs_task, "sdout", 3000, nullptr, 0, &sdcardlog_outputs_task_handle);
  xTaskCreate(rs485_tx, "485_TX", 2950, nullptr, 1, &rs485_tx_task_handle);
  xTaskCreate(rs485_rx, "485_RX", 2950, nullptr, 1, &rs485_rx_task_handle);
  xTaskCreate(service_rs485_transmit_q, "485_Q", 2950, nullptr, 1, &service_rs485_transmit_q_task_handle);
  xTaskCreate(canbus_tx, "CAN_Tx", 2950, nullptr, 1, &canbus_tx_task_handle);
  xTaskCreate(canbus_rx, "CAN_Rx", 2950, nullptr, 1, &canbus_rx_task_handle);
  xTaskCreate(transmit_task, "Tx", 2000, nullptr, configMAX_PRIORITIES - 3, &transmit_task_handle);
  xTaskCreate(replyqueue_task, "rxq", 2000, nullptr, configMAX_PRIORITIES - 2, &replyqueue_task_handle);
  xTaskCreate(lazy_tasks, "lazyt", 2500, nullptr, 0, &lazy_task_handle);

  // Set relay defaults
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    previousRelayState[y] = mysettings.rulerelaydefault[y];
    // Set relay defaults
    hal.SetOutputState(y, mysettings.rulerelaydefault[y]);
  }
  // Fire task to record state of outputs to SD Card
  xTaskNotify(sdcardlog_outputs_task_handle, 0x00, eNotifyAction::eNoAction);

  // We have just started...
  SetControllerState(ControllerState::Stabilizing);

  // Only run these after we have wifi...
  xTaskCreate(enqueue_task, "enqueue", 1900, nullptr, configMAX_PRIORITIES / 2, &enqueue_task_handle);
  xTaskCreate(rules_task, "rules", 3500, nullptr, configMAX_PRIORITIES - 5, &rule_task_handle);
  xTaskCreate(periodic_task, "period", 3500, nullptr, 0, &periodic_task_handle);

  // Start the wifi and connect to access point
  wifi_init_sta();

  if (_tft_screen_available)
  {
    ESP_LOGI(TAG, "TFT screen is INSTALLED");
    // Only attach, if device is fitted otherwise false triggers may occur
    // Touch screen IRQ (GPIO_NUM_36) is active LOW (XPT2046 chip)
    ESP_ERROR_CHECK(esp_ipc_call_blocking(PRO_CPU_NUM, tft_interrupt_attach, nullptr));
  }
  else
  {
    ESP_LOGI(TAG, "TFT screen is NOT installed");
  }
}

unsigned long wifitimer = 0;
unsigned long heaptimer = 0;
unsigned long taskinfotimer = 0;

void loop()
{

  if (card_action == CardAction::Mount)
  {
    mountSDCard();
  }
  if (card_action == CardAction::Unmount)
  {
    unmountSDCard();
  }

  if (card_action == CardAction::Remount)
  {
    unmountSDCard();
    mountSDCard();
  }

  unsigned long currentMillis = millis();

  if (_controller_state != ControllerState::NoWifiConfiguration)
  {
    // on first pass wifitimer is zero
    if (currentMillis - wifitimer > 30000)
    {
      // Attempt to connect to WiFi every 30 seconds, this caters for when WiFi drops
      // such as AP reboot

      // wifi_init_sta();
      if (!wifi_isconnected)
      {
        esp_wifi_connect();
      }
      wifitimer = currentMillis;

      // Attempt to connect to MQTT if enabled and not already connected
      connectToMqtt();
    }
  }

  // Call update to receive, decode and process incoming packets
  myPacketSerial.checkInputStream();

  if (currentMillis > heaptimer)
  {
    /*
    size_t total_free_bytes;        Total free bytes in the heap. Equivalent to multi_free_heap_size().
    size_t total_allocated_bytes;   Total bytes allocated to data in the heap.
    size_t largest_free_block;      Size of largest free block in the heap. This is the largest malloc-able size.
    size_t minimum_free_bytes;      Lifetime minimum free heap size. Equivalent to multi_minimum_free_heap_size().
    size_t allocated_blocks;        Number of (variable size) blocks allocated in the heap.
    size_t free_blocks;             Number of (variable size) free blocks in the heap.
    size_t total_blocks;            Total number of (variable size) blocks in the heap.
    */
    multi_heap_info_t heap;
    heap_caps_get_info(&heap, MALLOC_CAP_INTERNAL);

    ESP_LOGD(TAG, "total_free_byte=%u total_allocated_byte=%u largest_free_blk=%u min_free_byte=%u alloc_blk=%u free_blk=%u total_blk=%u",
             heap.total_free_bytes,
             heap.total_allocated_bytes,
             heap.largest_free_block,
             heap.minimum_free_bytes,
             heap.allocated_blocks,
             heap.free_blocks,
             heap.total_blocks);

    // uxTaskGetStackHighWaterMark returns bytes not words on ESP32
    /*
        ESP_LOGD(TAG, "periodic_task_handle high water=%i", uxTaskGetStackHighWaterMark(periodic_task_handle));
        ESP_LOGD(TAG, "rule_task_handle high water=%i", uxTaskGetStackHighWaterMark(rule_task_handle));
        ESP_LOGD(TAG, "enqueue_task_handle high water=%i", uxTaskGetStackHighWaterMark(enqueue_task_handle));
        ESP_LOGD(TAG, "sdcardlog_task_handle high water=%i", uxTaskGetStackHighWaterMark(sdcardlog_task_handle));
        ESP_LOGD(TAG, "sdcardlog_outputs_task_handle high water=%i", uxTaskGetStackHighWaterMark(sdcardlog_outputs_task_handle));
    */
    // Report again in 15 seconds
    heaptimer = currentMillis + 15000;
  }
}
