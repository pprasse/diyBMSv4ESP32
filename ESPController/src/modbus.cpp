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

#include "modbus.h"


extern HAL_ESP32 hal;
extern diybms_eeprom_settings mysettings;
extern const uart_port_t rs485_uart_num;
extern QueueHandle_t rs485_transmit_q_handle;
extern currentmonitoring_struct currentMonitor;
extern TaskHandle_t rs485_tx_task_handle;
extern TaskHandle_t rs485_rx_task_handle;
extern TaskHandle_t service_rs485_transmit_q_task_handle;
extern bool _tft_screen_available;

// holds modbus data
uint8_t frame[256];


// Sets the RS485 serial parameters after they have been changed
void ConfigureRS485()
{

  if (hal.GetRS485Mutex())
  {
    ESP_LOGD(TAG, "Configure RS485");
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_parity(rs485_uart_num, mysettings.rs485parity));
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_stop_bits(rs485_uart_num, mysettings.rs485stopbits));
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_baudrate(rs485_uart_num, mysettings.rs485baudrate));
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_word_length(rs485_uart_num, mysettings.rs485databits));

    hal.ReleaseRS485Mutex();
  }
  else
  {
    ESP_ERROR_CHECK(ESP_FAIL);
  }
}

void SetupRS485()
{
  ESP_LOGD(TAG, "Setup RS485");
  /* TEST RS485 */

  // Zero all data to start with
  memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));

  uart_config_t uart_config = {
      .baud_rate = mysettings.rs485baudrate,
      .data_bits = mysettings.rs485databits,
      .parity = mysettings.rs485parity,
      .stop_bits = mysettings.rs485stopbits,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
  };

  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(rs485_uart_num, &uart_config));

  // Set UART1 pins(TX: IO23, RX: I022, RTS: IO18, CTS: Not used)
  ESP_ERROR_CHECK(uart_set_pin(rs485_uart_num, RS485_TX, RS485_RX, RS485_ENABLE, UART_PIN_NO_CHANGE));

  // Install UART driver (we don't need an event queue here)
  ESP_ERROR_CHECK(uart_driver_install(rs485_uart_num, 256, 256, 0, nullptr, 0));

  // Set RS485 half duplex mode
  ESP_ERROR_CHECK(uart_set_mode(rs485_uart_num, uart_mode_t::UART_MODE_RS485_HALF_DUPLEX));

  ConfigureRS485();
}



[[noreturn]] void service_rs485_transmit_q(void *)
{
  for (;;)
  {
    uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];

    if (rs485_transmit_q_handle != nullptr)
    {
      // Wait for a item in the queue, blocking indefinately
      xQueueReceive(rs485_transmit_q_handle, &cmd, portMAX_DELAY);

      if (hal.GetRS485Mutex())
      {
        // Ensure we have empty receive buffer
        // uart_flush_input(rs485_uart_num);

        // Default of 8 bytes for a modbus request (including CRC)
        size_t packet_length = 8;

        if (cmd[1] == 15 || cmd[1] == 16)
        {
          // Calculate length of this packet, add on extra data
          // Force Multiple Coils (FC=15)
          // https://www.simplymodbus.ca/FC15.htm
          // Preset Multiple Registers (FC=16)
          // https://www.simplymodbus.ca/FC16.htm
          packet_length = 9 + cmd[6];
        }

        // Calculate the MODBUS CRC
        auto temp = calculateModBusCRC(cmd, (uint8_t)(packet_length - 2));
        // Byte swap the Hi and Lo bytes
        auto crc16 = (uint16_t)(temp << 8) | (temp >> 8);
        cmd[packet_length - 2] = (uint8_t)(crc16 >> 8); // split crc into 2 bytes
        cmd[packet_length - 1] = (uint8_t)(crc16 & 0xFF);

        // Send the bytes (actually just put them into the TX FIFO buffer)
        uart_write_bytes(rs485_uart_num, (char *)cmd, packet_length);

        hal.ReleaseRS485Mutex();

        // Notify the receive task that a packet should be on its way
        if (rs485_rx_task_handle != nullptr)
        {
          xTaskNotify(rs485_rx_task_handle, 0x00, eNotifyAction::eNoAction);
        }

        ESP_LOGD(TAG, "Send addr=%u, func=%u, len=%u", cmd[0], cmd[1], packet_length);
        // Debug
        // ESP_LOG_BUFFER_HEXDUMP(TAG, cmd, packet_length, esp_log_level_t::ESP_LOG_DEBUG);

        // Once we have notified the receive task, we pause here to avoid sending
        // another request until the last one has been processed (or we timeout after 2 seconds)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
      }
    }
    else
    {
      ESP_LOGE(TAG, "rs485_transmit_q_handle is NULL");
    }
  }
}


uint16_t calculateModBusCRC(const uint8_t *frame, uint8_t bufferSize)
{
  uint16_t flag;
  uint16_t temp;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < bufferSize; i++)
  {
    temp = temp ^ frame[i];
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }

  return temp;
  /*
  // Reverse byte order.
  uint16_t temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp;
  */
}

uint8_t SetMobusRegistersFromFloat(uint8_t *cmd, uint8_t ptr, float value)
{
  FloatUnionType fut;
  fut.value = value;
  // 4 bytes
  cmd[ptr] = (uint8_t)(fut.word[0] >> 8);
  ptr++;
  cmd[ptr] = (uint8_t)(fut.word[0] & 0xFF);
  ptr++;
  cmd[ptr] = (uint8_t)(fut.word[1] >> 8);
  ptr++;
  cmd[ptr] = (uint8_t)(fut.word[1] & 0xFF);
  ptr++;

  return ptr;
}
