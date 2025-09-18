/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "sl_bt_api.h"
#include "sl_main_init.h"

#include "sl_common.h"
#include "sl_iostream_handles.h"
#include "sli_iostream_uart.h"
#include "sl_iostream.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"

#include "app_log.h"
#include "app_assert.h"

/*******************************************************************************
 * AT-Command API Integration - Constants and Structures
 ******************************************************************************/

// Maximum lengths for AT-Command API
#define MAX_MAC_LENGTH 18
#define MAX_NAME_LENGTH 32
#define MAX_MODBUS_DATA_LENGTH 512
#define MAX_RESPONSE_LENGTH 256
#define MAX_BOUNDED_DEVICES 16

// AT Command buffer size
#define AT_COMMAND_BUFFER_SIZE 1024
#define AT_RESPONSE_BUFFER_SIZE 512

// Command types that BG22 receives from STM32
typedef enum {
    CMD_ADVSTART,
    CMD_ADVSTOP,
    CMD_RFON,
    CMD_RFOFF,
    CMD_LISTBOUNDED,
    CMD_DELBOUNDED,
    CMD_CLEARBOUNDED,
    CMD_SENDMODBUS,
    CMD_UNKNOWN
} bg22_at_command_type_t;

// Notification types that BG22 sends to STM32
typedef enum {
    NOTIFY_CONN_ATTEMPT,
    NOTIFY_CONNECTED,
    NOTIFY_DISCONNECTED,
    NOTIFY_MODBUS,
    NOTIFY_BOUNDED,
    NOTIFY_UNKNOWN
} bg22_notification_type_t;

// Response types
typedef enum {
    RESP_OK,
    RESP_ERROR_ALREADY_ADVERTISING,
    RESP_ERROR_NOT_ADVERTISING,
    RESP_ERROR_RF_ALREADY_ON,
    RESP_ERROR_RF_ALREADY_OFF,
    RESP_ERROR_NO_DEVICES,
    RESP_ERROR_DEVICE_NOT_FOUND,
    RESP_ERROR_NOT_CONNECTED,
    RESP_ERROR_UNKNOWN
} bg22_response_type_t;

// Bonded device structure
typedef struct {
    char mac[MAX_MAC_LENGTH];
    char name[MAX_NAME_LENGTH];
} bonded_device_t;

// AT Command structures
typedef struct {
    bg22_at_command_type_t type;
    union {
        char mac[MAX_MAC_LENGTH];
        struct {
            uint8_t modbus_data[MAX_MODBUS_DATA_LENGTH];  // Binary data
            uint16_t modbus_length;                       // Length field
        } modbus_cmd;
    } params;
} bg22_at_command_t;

typedef struct {
    bg22_notification_type_t type;
    union {
        struct {
            char mac[MAX_MAC_LENGTH];
        } conn_attempt;
        struct {
            char mac[MAX_MAC_LENGTH];
            char name[MAX_NAME_LENGTH];
        } connection;
        struct {
          uint8_t modbus_data[MAX_MODBUS_DATA_LENGTH];
          uint16_t modbus_length;
        } modbus;
        struct {
            bonded_device_t devices[MAX_BOUNDED_DEVICES];
            int count;
        } bounded;
    } data;
} bg22_at_notification_t;

typedef struct {
    bg22_response_type_t type;
    char error_message[MAX_RESPONSE_LENGTH];
} bg22_at_response_t;

// BG22 Extended State Structure (integrating with existing state)
typedef struct {
    bool rf_enabled;
    bool advertising_requested;    // Requested by STM32 via AT commands
    bool connected;
    char connected_device_mac[MAX_MAC_LENGTH];
    char connected_device_name[MAX_NAME_LENGTH];
    bonded_device_t bonded_devices[MAX_BOUNDED_DEVICES];
    int bonded_count;

    // Integration with existing code
    uint8_t advertising_set_handle;
    uint8_t connection_handle;
    uint8_t spp_state;  // STATE_ADVERTISING, STATE_CONNECTED, STATE_SPP_MODE
} bg22_extended_state_t;

/*******************************************************************************
 * Existing SPP Server Constants (unchanged)
 ******************************************************************************/

/* Set here the operation mode of the SPP device: */
#define SPP_SERVER_MODE    0
#define SPP_CLIENT_MODE    1

#define SPP_OPERATION_MODE SPP_SERVER_MODE

#define STATE_ADVERTISING  1
#define STATE_CONNECTED    2
#define STATE_SPP_MODE     3

#define UART_POLL_TIMEOUT  5000

/*Bookkeeping struct for storing amount of received/sent data  */
typedef struct
{
  uint32_t num_pack_sent;
  uint32_t num_bytes_sent;
  uint32_t num_pack_received;
  uint32_t num_bytes_received;
  uint32_t num_writes; /* Total number of send attempts */
} ts_counters;

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

// Existing variables (modified to integrate with new state)
static bg22_extended_state_t g_bg22_state = {0};
static ts_counters counters = {0};

// SPP/BLE variables
static uint8_t max_packet_size = 20;
static uint8_t min_packet_size = 20;
static uint32_t service_handle = 0;
static uint16_t char_handle = 0;

// AT Command processing buffers
static char at_command_buffer[AT_COMMAND_BUFFER_SIZE] = {0};
static uint16_t at_buffer_index = 0;

volatile uint16_t test_var = 0;

/*******************************************************************************
 * Common Local Functions
 ******************************************************************************/

static sl_status_t start_advertising(void);
static void send_spp_data();

/*******************************************************************************
 * AT-Command API Functions (adapted from our parser)
 ******************************************************************************/

static bool is_valid_mac(const char* mac) {
    if (!mac || strlen(mac) != 17) return false;

    for (int i = 0; i < 17; i++) {
        if (i % 3 == 2) {
            if (mac[i] != ':') return false;
        } else {
            if (!((mac[i] >= '0' && mac[i] <= '9') ||
                  (mac[i] >= 'A' && mac[i] <= 'F') ||
                  (mac[i] >= 'a' && mac[i] <= 'f'))) {
                return false;
            }
        }
    }
    return true;
}

static bool is_device_bonded(const char* mac, const bg22_extended_state_t* state) {
    for (int i = 0; i < state->bonded_count; i++) {
        if (strcmp(state->bonded_devices[i].mac, mac) == 0) {
            return true;
        }
    }
    return false;
}

bool bg22_parse_command(const char* input, bg22_at_command_t* cmd) {
  if (!input || !cmd) return false;

      char buffer[512];
      strncpy(buffer, input, sizeof(buffer) - 1);
      buffer[sizeof(buffer) - 1] = '\0';

      // Remove terminator if present
      char* term = strstr(buffer, "\r\n");
      if (term) *term = '\0';

      // Check if it starts with AT
      if (strncmp(buffer, "AT", 2) != 0) return false;

      // Parse different command types
      if (strcmp(buffer, "AT+ADVSTART") == 0) {
          cmd->type = CMD_ADVSTART;
          return true;
      }
      else if (strcmp(buffer, "AT+ADVSTOP") == 0) {
          cmd->type = CMD_ADVSTOP;
          return true;
      }
      else if (strcmp(buffer, "AT+RFON") == 0) {
          cmd->type = CMD_RFON;
          return true;
      }
      else if (strcmp(buffer, "AT+RFOFF") == 0) {
          cmd->type = CMD_RFOFF;
          return true;
      }
      else if (strcmp(buffer, "AT+LISTBOUNDED") == 0) {
          cmd->type = CMD_LISTBOUNDED;
          return true;
      }
      else if (strcmp(buffer, "AT+CLEARBOUNDED") == 0) {
          cmd->type = CMD_CLEARBOUNDED;
          return true;
      }
      else if (strncmp(buffer, "AT+DELBOUNDED=", 14) == 0) {
          cmd->type = CMD_DELBOUNDED;
          const char* mac = buffer + 14;
          if (is_valid_mac(mac)) {
              strncpy(cmd->params.mac, mac, MAX_MAC_LENGTH - 1);
              cmd->params.mac[MAX_MAC_LENGTH - 1] = '\0';
              return true;
          }
          return false;
      }
      else if (strncmp(buffer, "AT+SENDMODBUS=", 14) == 0) {
          cmd->type = CMD_SENDMODBUS;

          // For SENDMODBUS, we expect the data to be in binary format after the '='
          // The data starts right after "AT+SENDMODBUS="
          const char* data_start = buffer + 14;
          size_t remaining_len = strlen(data_start);

          if (remaining_len == 0 || remaining_len > MAX_MODBUS_DATA_LENGTH) {
              return false;
          }

          // Copy binary data directly (no hex conversion)
          memcpy(cmd->params.modbus_cmd.modbus_data, data_start, remaining_len);
          cmd->params.modbus_cmd.modbus_length = remaining_len;

          return true;
      }

    cmd->type = CMD_UNKNOWN;
    return false;
}

int bg22_format_response(const bg22_at_response_t* resp, char* output, size_t output_size) {
    if (!resp || !output || output_size == 0) return -1;

    switch (resp->type) {
        case RESP_OK:
            return snprintf(output, output_size, "OK\r\n");
        case RESP_ERROR_ALREADY_ADVERTISING:
            return snprintf(output, output_size, "ERROR:ALREADY_ADVERTISING\r\n");
        case RESP_ERROR_NOT_ADVERTISING:
            return snprintf(output, output_size, "ERROR:NOT_ADVERTISING\r\n");
        case RESP_ERROR_RF_ALREADY_ON:
            return snprintf(output, output_size, "ERROR:RF_ALREADY_ON\r\n");
        case RESP_ERROR_RF_ALREADY_OFF:
            return snprintf(output, output_size, "ERROR:RF_ALREADY_OFF\r\n");
        case RESP_ERROR_NO_DEVICES:
            return snprintf(output, output_size, "ERROR:NO_DEVICES\r\n");
        case RESP_ERROR_DEVICE_NOT_FOUND:
            return snprintf(output, output_size, "ERROR:DEVICE_NOT_FOUND\r\n");
        case RESP_ERROR_NOT_CONNECTED:
            return snprintf(output, output_size, "ERROR:NOT_CONNECTED\r\n");
        default:
            return snprintf(output, output_size, "ERROR:UNKNOWN\r\n");
    }
}

int bg22_format_notification(const bg22_at_notification_t* notify, char* output, size_t output_size) {
  if (!notify || !output || output_size == 0) return -1;

      int written = 0;
      const char* prefix = NULL;
      const char* suffix = "\r\n";

      switch (notify->type) {
          case NOTIFY_CONN_ATTEMPT: {
              prefix = "+NOTIFY:CONN_ATTEMPT=";
              size_t prefix_len = strlen(prefix);
              size_t mac_len = strlen(notify->data.conn_attempt.mac);
              size_t suffix_len = strlen(suffix);
              size_t total_len = prefix_len + mac_len + suffix_len;

              if (total_len >= output_size) return -1;

              memcpy(output + written, prefix, prefix_len);
              written += prefix_len;
              memcpy(output + written, notify->data.conn_attempt.mac, mac_len);
              written += mac_len;
              memcpy(output + written, suffix, suffix_len);
              written += suffix_len;
              return written;
          }

          case NOTIFY_CONNECTED: {
              prefix = "+NOTIFY:CONNECTED=";
              size_t prefix_len = strlen(prefix);
              size_t mac_len = strlen(notify->data.connection.mac);
              size_t name_len = strlen(notify->data.connection.name);
              size_t suffix_len = strlen(suffix);
              size_t total_len = prefix_len + mac_len + 1 + name_len + suffix_len; // +1 for comma

              if (total_len >= output_size) return -1;

              memcpy(output + written, prefix, prefix_len);
              written += prefix_len;
              memcpy(output + written, notify->data.connection.mac, mac_len);
              written += mac_len;
              output[written++] = ',';
              memcpy(output + written, notify->data.connection.name, name_len);
              written += name_len;
              memcpy(output + written, suffix, suffix_len);
              written += suffix_len;
              return written;
          }

          case NOTIFY_DISCONNECTED: {
              prefix = "+NOTIFY:DISCONNECTED=";
              size_t prefix_len = strlen(prefix);
              size_t mac_len = strlen(notify->data.connection.mac);
              size_t name_len = strlen(notify->data.connection.name);
              size_t suffix_len = strlen(suffix);
              size_t total_len = prefix_len + mac_len + 1 + name_len + suffix_len; // +1 for comma

              if (total_len >= output_size) return -1;

              memcpy(output + written, prefix, prefix_len);
              written += prefix_len;
              memcpy(output + written, notify->data.connection.mac, mac_len);
              written += mac_len;
              output[written++] = ',';
              memcpy(output + written, notify->data.connection.name, name_len);
              written += name_len;
              memcpy(output + written, suffix, suffix_len);
              written += suffix_len;
              return written;
          }

          case NOTIFY_MODBUS: {
              prefix = "+NOTIFY:MODBUS=";
              size_t prefix_len = strlen(prefix);
              size_t suffix_len = strlen(suffix);
              size_t total_len = prefix_len + notify->data.modbus.modbus_length + suffix_len;

              if (total_len >= output_size) return -1;

              // Copy prefix
              memcpy(output + written, prefix, prefix_len);
              written += prefix_len;

              // Copy binary MODBUS data directly (NO hex conversion - preserve raw bytes)
              memcpy(output + written, notify->data.modbus.modbus_data, notify->data.modbus.modbus_length);
              written += notify->data.modbus.modbus_length;

              // Copy suffix
              memcpy(output + written, suffix, suffix_len);
              written += suffix_len;
              return written;
          }

          case NOTIFY_BOUNDED: {
              prefix = "+BOUNDED:";
              size_t prefix_len = strlen(prefix);

              if (prefix_len >= output_size) return -1;

              memcpy(output + written, prefix, prefix_len);
              written += prefix_len;

              for (int i = 0; i < notify->data.bounded.count; i++) {
                  if (i > 0) {
                      if (written >= output_size - 1) return -1;
                      output[written++] = ';';
                  }

                  size_t mac_len = strlen(notify->data.bounded.devices[i].mac);
                  size_t name_len = strlen(notify->data.bounded.devices[i].name);

                  if (written + mac_len + 1 + name_len >= output_size) return -1;

                  memcpy(output + written, notify->data.bounded.devices[i].mac, mac_len);
                  written += mac_len;
                  output[written++] = ',';
                  memcpy(output + written, notify->data.bounded.devices[i].name, name_len);
                  written += name_len;
              }

              size_t suffix_len = strlen(suffix);
              if (written + suffix_len >= output_size) return -1;

              memcpy(output + written, suffix, suffix_len);
              written += suffix_len;
              return written;
          }

          default:
              return -1;
      }
}

void send_at_response_to_stm32(const bg22_at_response_t* response) {
    char tx_buffer[AT_RESPONSE_BUFFER_SIZE];
    int len = bg22_format_response(response, tx_buffer, sizeof(tx_buffer));
    if (len > 0) {
        // Send response to STM32 via UART
        for (int i = 0; i < len; i++) {
            sl_iostream_putchar(sl_iostream_vcom_handle, tx_buffer[i]);
        }
        // app_log("[BG22->STM32] Response: %s", tx_buffer);
    }
}

void send_at_notification_to_stm32(const bg22_at_notification_t* notification) {
    char tx_buffer[AT_RESPONSE_BUFFER_SIZE];
    int len = bg22_format_notification(notification, tx_buffer, sizeof(tx_buffer));
    if (len > 0) {
        // Send notification to STM32 via UART
        for (int i = 0; i < len; i++) {
            sl_iostream_putchar(sl_iostream_vcom_handle, tx_buffer[i]);
        }
         app_log("[BG22->STM32] Notification: %s", tx_buffer);
    }
}

bg22_response_type_t bg22_execute_command(const bg22_at_command_t* cmd, bg22_extended_state_t* state) {
    if (!cmd || !state) return RESP_ERROR_UNKNOWN;

    sl_status_t sc;

    switch (cmd->type) {
        case CMD_ADVSTART:
            if (state->advertising_requested && state->spp_state == STATE_ADVERTISING) {
                return RESP_ERROR_ALREADY_ADVERTISING;
            }
            if (!state->rf_enabled) {
                return RESP_ERROR_RF_ALREADY_OFF;
            }

            state->advertising_requested = true;
            if (state->spp_state != STATE_ADVERTISING) {
                sc = start_advertising();
                if (sc == SL_STATUS_OK) {
//                    app_log("[BG22] BLE advertising started via AT command\r\n");
                    return RESP_OK;
                } else {
                    return RESP_ERROR_UNKNOWN;
                }
            }
            return RESP_OK;

        case CMD_ADVSTOP:
            if (!state->advertising_requested) {
                return RESP_ERROR_NOT_ADVERTISING;
            }

            state->advertising_requested = false;
            sc = sl_bt_advertiser_stop(state->advertising_set_handle);
            if (sc == SL_STATUS_OK) {
//                app_log("[BG22] BLE advertising stopped via AT command\r\n");
                return RESP_OK;
            } else {
                return RESP_ERROR_UNKNOWN;
            }

        case CMD_RFON:
            if (state->rf_enabled) {
                return RESP_ERROR_RF_ALREADY_ON;
            }
            state->rf_enabled = true;
//            app_log("[BG22] RF module enabled via AT command\r\n");
            return RESP_OK;

        case CMD_RFOFF:
            if (!state->rf_enabled) {
                return RESP_ERROR_RF_ALREADY_OFF;
            }

            state->rf_enabled = false;
            state->advertising_requested = false;

            if (state->spp_state == STATE_ADVERTISING) {
                sl_bt_advertiser_stop(state->advertising_set_handle);
            }

            if (state->connected && state->connection_handle != 0xFF) {
                sl_bt_connection_close(state->connection_handle);
            }

//            app_log("[BG22] RF module disabled via AT command\r\n");
            return RESP_OK;

        case CMD_LISTBOUNDED: {
            bg22_at_notification_t notify;
            notify.type = NOTIFY_BOUNDED;
            notify.data.bounded.count = state->bonded_count;
            for (int i = 0; i < state->bonded_count; i++) {
                notify.data.bounded.devices[i] = state->bonded_devices[i];
            }
            send_at_notification_to_stm32(&notify);
            return RESP_OK;
        }

        case CMD_DELBOUNDED: {
            bool found = false;
            for (int i = 0; i < state->bonded_count; i++) {
                if (strcmp(state->bonded_devices[i].mac, cmd->params.mac) == 0) {
                    for (int j = i; j < state->bonded_count - 1; j++) {
                        state->bonded_devices[j] = state->bonded_devices[j + 1];
                    }
                    state->bonded_count--;
                    found = true;
//                    app_log("[BG22] Removed bonded device: %s\r\n", cmd->params.mac);
                    break;
                }
            }
            return found ? RESP_OK : RESP_ERROR_DEVICE_NOT_FOUND;
        }

        case CMD_CLEARBOUNDED:
            if (state->bonded_count == 0) {
                return RESP_ERROR_NO_DEVICES;
            }
            state->bonded_count = 0;
//            app_log("[BG22] Cleared all bonded devices\r\n");
            return RESP_OK;

        case CMD_SENDMODBUS:
            if (!state->connected || state->connection_handle == 0xFF) {
                return RESP_ERROR_NOT_CONNECTED;
            }

            if (state->spp_state == STATE_SPP_MODE) {
                // Send binary MODBUS data directly to BLE client (preserves all bytes including 0x00)
                sl_status_t result = sl_bt_gatt_server_send_notification(
                    state->connection_handle,
                    gattdb_spp_data_tx,
                    cmd->params.modbus_cmd.modbus_length,
                    cmd->params.modbus_cmd.modbus_data
                );

                if (result == SL_STATUS_OK) {
                    // Debug log - show hex representation
                    char hex_debug[MAX_MODBUS_DATA_LENGTH * 2 + 1];
                    for (uint16_t i = 0; i < cmd->params.modbus_cmd.modbus_length; i++) {
                        sprintf(&hex_debug[i * 2], "%02X", cmd->params.modbus_cmd.modbus_data[i]);
                    }
                    hex_debug[cmd->params.modbus_cmd.modbus_length * 2] = '\0';
//                    app_log("[BG22] Sent MODBUS to BLE client (%d bytes): %s\r\n",
//                           cmd->params.modbus_cmd.modbus_length, hex_debug);
                    return RESP_OK;
                } else {
                    return RESP_ERROR_UNKNOWN;
                }
            } else {
                return RESP_ERROR_NOT_CONNECTED;
            }

        default:
            return RESP_ERROR_UNKNOWN;
    }
}

void process_at_command_buffer() {
    if (at_buffer_index == 0) return;

    // Null terminate the buffer
    at_command_buffer[at_buffer_index] = '\0';

    // Look for complete command (ending with \r\n)
    char* cmd_end = strstr(at_command_buffer, "\r\n");
    if (cmd_end == NULL) return;  // Incomplete command, wait for more data

    // Extract the complete command
    *cmd_end = '\0';

    // app_log("[STM32->BG22] Received: %s\r\n", at_command_buffer);

    // Parse and execute the command
    bg22_at_command_t cmd;
    if (bg22_parse_command(at_command_buffer, &cmd)) {
        bg22_response_type_t resp_type = bg22_execute_command(&cmd, &g_bg22_state);

        // Send response
        bg22_at_response_t response;
        response.type = resp_type;
        strcpy(response.error_message, "");

        send_at_response_to_stm32(&response);
    } else {
        // Send error response for invalid command
        bg22_at_response_t response;
        response.type = RESP_ERROR_UNKNOWN;
        strcpy(response.error_message, "INVALID_COMMAND");
        send_at_response_to_stm32(&response);
    }

    // Shift remaining data in buffer
    size_t remaining = at_buffer_index - (cmd_end + 2 - at_command_buffer);
    if (remaining > 0) {
        memmove(at_command_buffer, cmd_end + 2, remaining);
    }
    at_buffer_index = remaining;
}

/*******************************************************************************
 * Existing Functions (adapted to work with integrated state)
 ******************************************************************************/

static void print_stats(ts_counters *p_counters) {
    app_log("Outgoing data:\r\n");
    app_log(" bytes/packets sent: %lu / %lu ", p_counters->num_bytes_sent, p_counters->num_pack_sent);
    app_log(", num writes: %lu\r\n", p_counters->num_writes);
    app_log("(RX buffer overflow is not tracked)\r\n");
    app_log("Incoming data:\r\n");
    app_log(" bytes/packets received: %lu / %lu\r\n", p_counters->num_bytes_received, p_counters->num_pack_received);
}

static void reset_variables() {
    g_bg22_state.connection_handle = 0xFF;
    g_bg22_state.spp_state = STATE_ADVERTISING;
    g_bg22_state.connected = false;
    g_bg22_state.rf_enabled = true;  // Initialize as enabled
    g_bg22_state.advertising_requested = false;

    service_handle = 0;
    char_handle = 0;
    max_packet_size = 20;

    memset(&counters, 0, sizeof(counters));
    at_buffer_index = 0;
}

void my_rx_callback(void *data) {
  // Handle received data here
//  app_log("New data\r\n");
    app_proceed();
}

static sl_status_t start_advertising(void) {
    sl_status_t sc;

    // Create an advertising set if not already created
    if (g_bg22_state.advertising_set_handle == 0xFF) {
        sc = sl_bt_advertiser_create_set(&g_bg22_state.advertising_set_handle);
        app_assert_status(sc);
    }

    // Generate data for advertising
    sc = sl_bt_legacy_advertiser_generate_data(g_bg22_state.advertising_set_handle,
                                               sl_bt_advertiser_general_discoverable);
    app_assert_status(sc);

    // Set advertising interval to 100ms
    sc = sl_bt_advertiser_set_timing(g_bg22_state.advertising_set_handle,
                                     160, 160, 0, 0);
    app_assert_status(sc);

    // Start advertising and enable connections
    sc = sl_bt_legacy_advertiser_start(g_bg22_state.advertising_set_handle,
                                       sl_bt_legacy_advertiser_connectable);
    app_assert_status(sc);

    return sc;
}

static sl_status_t set_device_name(char *name) {
    sl_status_t sc;
    sc = sl_bt_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8_t *)name);
    app_assert_status(sc);
    return sc;
}

// Application Init.
void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////

    sl_status_t status = sli_iostream_uart_subscribe_to_new_data(sl_iostream_uart_vcom_handle,
                                                                 my_rx_callback,
                                                                 NULL);
    app_assert_status(status);

    // Initialize BG22 state
    memset(&g_bg22_state, 0, sizeof(g_bg22_state));
    g_bg22_state.advertising_set_handle = 0xFF;
    g_bg22_state.connection_handle = 0xFF;
    g_bg22_state.rf_enabled = true;  // Start with RF enabled

    app_log("[BG22] Application initialized with AT-Command API\r\n");
}

void app_process_action(void) {
    if (app_is_process_required()) {
        /////////////////////////////////////////////////////////////////////////////
        // Put your additional application code here!                              //
        // This is will run each time app_proceed() is called.                     //
        // Do not call blocking functions from here!                               //
        /////////////////////////////////////////////////////////////////////////////

        // TODO data available, check for management frames

        // Parse : data from the main MCU

        // - Set Device name / MAC Address
        // - Start / Stop BLE
        // - Start / Stop Advertising
        // - Get / Device

        //parse_main_mcu_command(command)


        // Process AT commands from STM32
        char c;
        sl_status_t read_result;

        // Read characters and build AT command buffer
        while ((read_result = sl_iostream_getchar(sl_iostream_vcom_handle, &c)) == SL_STATUS_OK) {
            if (at_buffer_index < AT_COMMAND_BUFFER_SIZE - 1) {
                at_command_buffer[at_buffer_index++] = c;

                // Check for complete command
                if (c == '\n' && at_buffer_index > 1 && at_command_buffer[at_buffer_index-2] == '\r') {
                    process_at_command_buffer();
                    break;  // Process one command at a time
                }
            } else {
                // Buffer overflow, reset
                at_buffer_index = 0;
//                app_log("[BG22] AT command buffer overflow\r\n");
            }
        }

        // Handle SPP data transmission (existing functionality)
        if (STATE_SPP_MODE == g_bg22_state.spp_state) {
            send_spp_data();
        }
    }
}

// Convert MAC address from bd_addr to string format
void format_mac_address(bd_addr *addr, char *mac_str) {
    sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X",
            addr->addr[5], addr->addr[4], addr->addr[3],
            addr->addr[2], addr->addr[1], addr->addr[0]);
}

/**************************************************************************//**
 * Bluetooth stack event handler for SPP Server mode
 *
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt) {
    uint16_t max_mtu_out;
    sl_status_t sc;

    switch (SL_BT_MSG_ID(evt->header)) {
        // -------------------------------
        // This event indicates the device has started and the radio is ready.
        // Do not call any stack command before receiving this boot event!
        case sl_bt_evt_system_boot_id:
            app_log("SPP Role: SPP Server with AT-Command API\r\n");
            reset_variables();
            sc = sl_bt_gatt_server_set_max_mtu(247, &max_mtu_out);
            app_assert_status(sc);

            // Retrieve the device MAC and append its 4 last digits to the device name
            bd_addr address;
            sc = sl_bt_system_get_identity_address(&address, NULL);
            app_assert_status(sc);

            app_log("Device address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                    address.addr[5], address.addr[4], address.addr[3],
                    address.addr[2], address.addr[1], address.addr[0]);


            // TODO Place this code where we receive the final ID

            // Apply received device name by driver or keypad to characteristic
            // Set the device name to "SPP_xxxx" where xxxx is the last 4 digits of
            // the device MAC address.
            char device_name[9];
            sprintf(device_name, "SPP_%02X%02X", address.addr[1], address.addr[0]);

            sc = set_device_name(device_name);
            app_assert_status(sc);

            // Wait for Push Button / Event
            sc = start_advertising();
            app_assert_status(sc);
            g_bg22_state.advertising_requested = false;  // This is automatic startup advertising

            break;

        // -------------------------------
        // This event indicates that a new connection was opened.
        case sl_bt_evt_connection_opened_id: {
            g_bg22_state.connection_handle = evt->data.evt_connection_opened.connection;
            g_bg22_state.connected = true;
            g_bg22_state.spp_state = STATE_CONNECTED;

            // Request connection parameter update.
            // conn.interval min 20ms, max 40ms, slave latency 4 intervals,
            // supervision timeout 2 seconds
            // (These should be compliant with Apple Bluetooth Accessory Design
            // Guidelines, both R7 and R8)

            // Get connected device info and format MAC
            format_mac_address(&evt->data.evt_connection_opened.address, g_bg22_state.connected_device_mac);
            strcpy(g_bg22_state.connected_device_name, "UnknownDevice"); // Will be updated if available

            // app_log("Connection opened from: %s\r\n", g_bg22_state.connected_device_mac);

            // Send notification to STM32
            bg22_at_notification_t notify;
            notify.type = NOTIFY_CONNECTED;
            strcpy(notify.data.connection.mac, g_bg22_state.connected_device_mac);
            strcpy(notify.data.connection.name, g_bg22_state.connected_device_name);
            send_at_notification_to_stm32(&notify);

            // Request connection parameter update
            sl_bt_connection_set_parameters(g_bg22_state.connection_handle, 24, 40, 0, 200, 0, 0xFFFF);
            break;
        }

        case sl_bt_evt_connection_parameters_id:
            // app_log("Conn.parameters: interval %u units\r\n", evt->data.evt_connection_parameters.interval);
            break;

        case sl_bt_evt_gatt_mtu_exchanged_id:
            // Calculate maximum data per one notification / write-without-response,
            // this depends on the MTU. up to ATT_MTU-3 bytes can be sent at once.
            max_packet_size = evt->data.evt_gatt_mtu_exchanged.mtu - 3;

            /* Try to send maximum length packets whenever possible */
            min_packet_size = max_packet_size;
            // app_log("MTU exchanged: %d\r\n", evt->data.evt_gatt_mtu_exchanged.mtu);
            break;

        case sl_bt_evt_connection_closed_id:
            // print_stats(&counters);

            // Send disconnection notification to STM32
            if (g_bg22_state.connected) {
                bg22_at_notification_t notify;
                notify.type = NOTIFY_DISCONNECTED;
                strcpy(notify.data.connection.mac, g_bg22_state.connected_device_mac);
                strcpy(notify.data.connection.name, g_bg22_state.connected_device_name);
                send_at_notification_to_stm32(&notify);
            }

            if (STATE_SPP_MODE == g_bg22_state.spp_state) {
                sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
            }

            // Reset connection state
            g_bg22_state.connected = false;
            g_bg22_state.connection_handle = 0xFF;
            g_bg22_state.spp_state = STATE_ADVERTISING;
            strcpy(g_bg22_state.connected_device_mac, "");
            strcpy(g_bg22_state.connected_device_name, "");

            // Restart advertising if RF is enabled and advertising was requested
            if (g_bg22_state.rf_enabled && g_bg22_state.advertising_requested) {
                sc = sl_bt_legacy_advertiser_start(g_bg22_state.advertising_set_handle,
                                                   sl_bt_legacy_advertiser_connectable);
                app_assert_status(sc);
            }
            break;

        case sl_bt_evt_gatt_server_characteristic_status_id: {
            sl_bt_evt_gatt_server_characteristic_status_t char_status;
            char_status = evt->data.evt_gatt_server_characteristic_status;

            // We consider the tunnel opened only if our client has subscribed to notifications on the TX side
            if (char_status.characteristic == gattdb_spp_data_tx) {
                if (char_status.status_flags == sl_bt_gatt_server_client_config) {
                    // Characteristic client configuration (CCC) for spp_data has been
                    //   changed
                    if (char_status.client_config_flags == sl_bt_gatt_server_notification) {
                        g_bg22_state.spp_state = STATE_SPP_MODE;
                        sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
                        // app_log("SPP Mode ON\r\n");
                    } else {
                        // app_log("SPP Mode OFF\r\n");
                        g_bg22_state.spp_state = STATE_CONNECTED;
                        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
                    }
                }
            }
        }
        break;

        case sl_bt_evt_gatt_server_attribute_value_id: {
            // Data received over BLE (in the RX Characteristic) is sent over UART
            if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_spp_data_rx) {
                if (evt->data.evt_gatt_server_attribute_value.value.len != 0) {
                    // Extract received data
                    uint8_t* received_data = evt->data.evt_gatt_server_attribute_value.value.data;
                    uint16_t data_len = evt->data.evt_gatt_server_attribute_value.value.len;

                    // Send MODBUS notification to STM32 with binary data (using memcpy instead of strcpy)
                    bg22_at_notification_t notify;
                    notify.type = NOTIFY_MODBUS;

                    if (data_len <= MAX_MODBUS_DATA_LENGTH) {
                        // Use memcpy to preserve all bytes including 0x00
                        memcpy(notify.data.modbus.modbus_data, received_data, data_len);
                        notify.data.modbus.modbus_length = data_len;

                        test_var = data_len;
                        send_at_notification_to_stm32(&notify);

                         app_log("[BLE->BG22] MODBUS data received: %s\r\n", notify.data.modbus.modbus_data);

                        // Update counters
                        counters.num_pack_received++;
                        counters.num_bytes_received += data_len;
                    }
                }
            }
        }
        break;

        // -------------------------------
        // Default event handler.
        default:
            break;
    }
}

/*******************************************************************************
 * SPP Data Transmission (existing function, unchanged)
 ******************************************************************************/

static void send_spp_data() {
    uint8_t len = 0;
    uint8_t data[256];
    sl_status_t result, read_result;
    uint32_t timeout = 0;
    char c;

    // Read up to max_packet_size characters from local buffer
    while (len < max_packet_size) {
        read_result = sl_iostream_getchar(sl_iostream_vcom_handle, &c);
        if (SL_STATUS_OK == read_result) {
            data[len++] = (uint8_t)c;
        } else if (len == 0) {
            /* If the first ReadChar() fails then return immediately */
            return;
        } else {
            // Speed optimization: if there are some bytes to be sent but the length
            // is still below the preferred minimum packet size, then wait for
            // additional bytes until timeout. Target is to put as many bytes as
            // possible into each air packet.

            // Conditions for exiting the while loop and proceed to send data:
            if (timeout++ > UART_POLL_TIMEOUT) {
                break;
            } else if (len >= min_packet_size) {
                break;
            }
        }
    }

    if (len > 0) {
    // Stack may return "out-of-memory" (SL_STATUS_NO_MORE_RESOURCE) error if
    //   the local buffer is full -> in that case, just keep trying until the
    //   command succeeds
        do {
            result = sl_bt_gatt_server_send_notification(g_bg22_state.connection_handle,
                                                         gattdb_spp_data_tx,
                                                         len,
                                                         data);
            counters.num_writes++;
        } while (result == SL_STATUS_NO_MORE_RESOURCE);

        if (result != 0) {
            app_log("Unexpected error: %lu\r\n", result);
        } else {
            counters.num_pack_sent++;
            counters.num_bytes_sent += len;
        }
    }
}

/*******************************************************************************
 * Additional Helper Functions for Integration
 ******************************************************************************/

// Function to add a bonded device (called during pairing process)
bool add_bonded_device(const char* mac, const char* name) {
    if (g_bg22_state.bonded_count >= MAX_BOUNDED_DEVICES) {
        return false;  // Maximum devices reached
    }

    // Check if device already exists
    for (int i = 0; i < g_bg22_state.bonded_count; i++) {
        if (strcmp(g_bg22_state.bonded_devices[i].mac, mac) == 0) {
            // Update name if different
            strncpy(g_bg22_state.bonded_devices[i].name, name, MAX_NAME_LENGTH - 1);
            g_bg22_state.bonded_devices[i].name[MAX_NAME_LENGTH - 1] = '\0';
            return true;
        }
    }

    // Add new device
    strncpy(g_bg22_state.bonded_devices[g_bg22_state.bonded_count].mac, mac, MAX_MAC_LENGTH - 1);
    g_bg22_state.bonded_devices[g_bg22_state.bonded_count].mac[MAX_MAC_LENGTH - 1] = '\0';
    strncpy(g_bg22_state.bonded_devices[g_bg22_state.bonded_count].name, name, MAX_NAME_LENGTH - 1);
    g_bg22_state.bonded_devices[g_bg22_state.bonded_count].name[MAX_NAME_LENGTH - 1] = '\0';
    g_bg22_state.bonded_count++;

    // app_log("[BG22] Added bonded device: %s (%s)\r\n", name, mac);
    return true;
}

// Function to handle unknown device connection attempts (for future pairing support)
void handle_unknown_device_connection(const char* mac) {
    if (!is_device_bonded(mac, &g_bg22_state)) {
        // Send connection attempt notification to STM32
        bg22_at_notification_t notify;
        notify.type = NOTIFY_CONN_ATTEMPT;
        strcpy(notify.data.conn_attempt.mac, mac);
        send_at_notification_to_stm32(&notify);

        // app_log("[BG22] Unknown device connection attempt: %s\r\n", mac);
    }
}

// Get current BG22 state (for debugging/monitoring)
const bg22_extended_state_t* get_bg22_state(void) {
    return &g_bg22_state;
}

/*******************************************************************************
 * Debug and Testing Functions
 ******************************************************************************/

void bg22_debug_print_state(void) {
    app_log("=== BG22 State Debug ===\r\n");
    app_log("RF Enabled: %s\r\n", g_bg22_state.rf_enabled ? "Yes" : "No");
    app_log("Advertising Requested: %s\r\n", g_bg22_state.advertising_requested ? "Yes" : "No");
    app_log("Connected: %s\r\n", g_bg22_state.connected ? "Yes" : "No");
    app_log("SPP State: %d\r\n", g_bg22_state.spp_state);
    app_log("Connection Handle: 0x%02X\r\n", g_bg22_state.connection_handle);
    app_log("Advertising Handle: 0x%02X\r\n", g_bg22_state.advertising_set_handle);

    if (g_bg22_state.connected) {
        app_log("Connected Device: %s (%s)\r\n",
                g_bg22_state.connected_device_name,
                g_bg22_state.connected_device_mac);
    }

    app_log("Bonded Devices (%d):\r\n", g_bg22_state.bonded_count);
    for (int i = 0; i < g_bg22_state.bonded_count; i++) {
        app_log("  %d: %s (%s)\r\n", i,
                g_bg22_state.bonded_devices[i].name,
                g_bg22_state.bonded_devices[i].mac);
    }
    app_log("========================\r\n");
}

// Test function to simulate AT commands (for debugging)
void bg22_test_at_commands(void) {
    // app_log("=== Testing AT Commands ===\r\n");

    // Simulate some bonded devices for testing
    add_bonded_device("AA:BB:CC:DD:EE:FF", "TestPhone");
    add_bonded_device("11:22:33:44:55:66", "TestTablet");

    // Test commands
    const char* test_commands[] = {
        "AT+RFON\r\n",
        "AT+ADVSTART\r\n",
        "AT+LISTBOUNDED\r\n",
        "AT+DELBOUNDED=AA:BB:CC:DD:EE:FF\r\n",
        "AT+ADVSTOP\r\n"
    };

    for (int i = 0; i < 5; i++) {
        // app_log("Testing: %s", test_commands[i]);

        // Simulate received command
        strcpy(at_command_buffer, test_commands[i]);
        at_buffer_index = strlen(test_commands[i]);
        process_at_command_buffer();

        // Small delay for readability
        sl_sleeptimer_delay_millisecond(100);
    }
}

/*******************************************************************************
 * Integration Notes and Usage Examples
 ******************************************************************************/

/*
 * INTEGRATION GUIDE:
 *
 * 1. This code replaces your existing app.c file
 * 2. The AT-Command API is now fully integrated with the SPP server
 * 3. Commands from STM32 are processed via UART in app_process_action()
 * 4. BLE events automatically generate notifications to STM32
 * 5. MODBUS data flows bidirectionally through the BLE SPP tunnel
 *
 * KEY FEATURES ADDED:
 * - AT+ADVSTART/ADVSTOP: Control advertising
 * - AT+RFON/RFOFF: Control RF module
 * - AT+LISTBOUNDED: Get bonded devices list
 * - AT+DELBOUNDED/CLEARBOUNDED: Manage bonded devices
 * - AT+SENDMODBUS: Send MODBUS data to BLE client
 * - Automatic notifications for connections, disconnections, and MODBUS data
 *
 * UART COMMUNICATION:
 * - STM32 -> BG22: AT commands (parsed in app_process_action)
 * - BG22 -> STM32: Responses and notifications (sent via UART)
 * - BLE Client -> BG22: MODBUS data (forwarded as notifications to STM32)
 * - BG22 -> BLE Client: MODBUS data (sent via AT+SENDMODBUS)
 *
 * USAGE EXAMPLE FROM STM32:
 * 1. Send "AT+RFON\r\n" -> Response: "OK\r\n"
 * 2. Send "AT+ADVSTART\r\n" -> Response: "OK\r\n"
 * 3. When BLE device connects -> Receive: "+NOTIFY:CONNECTED=AA:BB:CC:DD:EE:FF,MyPhone\r\n"
 * 4. When MODBUS data arrives -> Receive: "+NOTIFY:MODBUS=010300000002C40B\r\n"
 * 5. Send "AT+SENDMODBUS=0103020004\r\n" -> Response: "OK\r\n" (data sent to BLE client)
 */
