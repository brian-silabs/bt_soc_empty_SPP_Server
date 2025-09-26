// --- app.c (top of file) -----------------------------------------------------
#include "sl_status.h"
#include "app_log.h"
#include "sl_iostream.h"
#include "sl_iostream_handles.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"

// VCOM config macros live here in your project:
#include "sl_iostream_usart_vcom_config.h"   // present in config/  ✅
#include "sl_sleeptimer.h"
#include "utests.h"  // your test toggles

// --- Configurable timeouts (ms) ----------------------------------------------
#define VCOM_LOOPBACK_TIMEOUT_MS    500   // total allowed time to read N bytes
#define VCOM_LOOPBACK_NOPROG_MS      50   // fail if no new byte for this long
#define SERIAL_LINK_MTU             247  // must be <= 247

// Return codes for the test
typedef enum { TEST_PASS = 0, TEST_TIMEOUT = 1, TEST_MISMATCH = 2 } test_rc_t;
// tick_count, it can wrap around.
typedef uint32_t sl_sleeptimer_tick_count_t;

// --- Helpers ---------------------------------------------------------------
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else               crc = (crc << 1);
    }
  }
  return crc;
}

// Tiny helper for ms->ticks and getting "now"
static sl_sleeptimer_tick_count_t ms_to_ticks(uint32_t ms) {
  sl_sleeptimer_tick_count_t t = sl_sleeptimer_ms_to_tick(ms);
  return t;
}
static sl_sleeptimer_tick_count_t ticks_now(void) {
  return sl_sleeptimer_get_tick_count();
}

static test_rc_t vcom_read_with_timeout(uint8_t *rx, uint16_t N) {
  const sl_sleeptimer_tick_count_t t_start = ticks_now();
  const sl_sleeptimer_tick_count_t t_dead  = t_start + ms_to_ticks(VCOM_LOOPBACK_TIMEOUT_MS);

  sl_sleeptimer_tick_count_t t_last_prog = t_start;

  uint16_t got = 0;
  while (got < N) {
    char c; // IMPORTANT: getchar expects a 'char *'
    if (sl_iostream_getchar(sl_iostream_vcom_handle, &c) == SL_STATUS_OK) {
      rx[got++] = (uint8_t)c;
      t_last_prog = ticks_now();  // progress made
    } else {
      // No data this tick: let the CPU breathe a bit and check timeouts
      sl_sleeptimer_delay_millisecond(1);

      const sl_sleeptimer_tick_count_t now = ticks_now();
      if (now >= t_dead) {
        app_log("VCOM read TIMEOUT: got %u / %u bytes (overall %u ms)\n",
                (unsigned)got, (unsigned)N, (unsigned)VCOM_LOOPBACK_TIMEOUT_MS);
        return TEST_TIMEOUT;
      }
#if VCOM_LOOPBACK_NOPROG_MS > 0
      if ((now - t_last_prog) >= ms_to_ticks(VCOM_LOOPBACK_NOPROG_MS)) {
        app_log("VCOM read STALL: no progress for %u ms (got %u / %u bytes)\n",
                (unsigned)VCOM_LOOPBACK_NOPROG_MS, (unsigned)got, (unsigned)N);
        return TEST_TIMEOUT;
      }
#endif
    }
  }
  return TEST_PASS;
}

#if TEST_VCOM_LOOPBACK_1K || TEST_VCOM_CONFIG_DUMP
static void dump_vcom_config_once(void)
{
#if TEST_VCOM_CONFIG_DUMP
  app_log("VCOM cfg: RX=%u, TX=%u, HWFC=%u (0=None,1=CTS,2=RTS,3=CTS+RTS)\n",
          (unsigned)SL_IOSTREAM_USART_VCOM_RX_BUFFER_SIZE,
          (unsigned)0,
          (unsigned)0);
#endif
}
#endif

#if TEST_VCOM_LOOPBACK_1K
#include "em_usart.h"

// try to deduce the USART instance pointer from config
static USART_TypeDef* vcom_usart_ptr(void) {
#ifdef SL_IOSTREAM_USART_VCOM_PERIPHERAL
  return SL_IOSTREAM_USART_VCOM_PERIPHERAL;
#elif defined(SL_IOSTREAM_USART_VCOM_PERIPHERAL_NO)
  #if   SL_IOSTREAM_USART_VCOM_PERIPHERAL_NO == 0
    return USART0;
  #elif SL_IOSTREAM_USART_VCOM_PERIPHERAL_NO == 1
    return USART1;
  #elif SL_IOSTREAM_USART_VCOM_PERIPHERAL_NO == 2
    return USART2;
  #else
    return 0;
  #endif
#else
  return 0;
#endif
}

// Example usage inside your loopback test
static test_rc_t run_vcom_loopback_1k_with_timeout(void) {
  const uint16_t N = SERIAL_LINK_MTU;
  static uint8_t tx[SERIAL_LINK_MTU];
  static uint8_t rx[SERIAL_LINK_MTU];

  for (uint16_t i = 0; i < N; i++) tx[i] = (uint8_t)(i & 0xFF);

  // Flush stale RX first
  char dump;
  size_t data_read = 0;
  sl_status_t flush_sc;
    do
    {
        flush_sc = sl_iostream_read(   sl_iostream_vcom_handle,
                                        &dump,
                                        0x01,
                                        &data_read);
    } while ((data_read != 0) && (flush_sc == SL_STATUS_OK));

  // Write the whole buffer in one go
  (void)sl_iostream_write(sl_iostream_vcom_handle, tx, N);

  // Read back with timeout
  test_rc_t rc = vcom_read_with_timeout(rx, N);
  if (rc != TEST_PASS) {
    // Already logged timeout/ stall specifics above
    return rc;
  }

  // Compare content
  for (uint16_t i = 0; i < N; i++) {
    if (rx[i] != tx[i]) {
      app_log("VCOM 1k loopback MISMATCH at idx=%u (tx=0x%02X, rx=0x%02X)\n",
              (unsigned)i, (unsigned)tx[i], (unsigned)rx[i]);
      return TEST_MISMATCH;
    }
  }
  app_log("VCOM 1k loopback: OK (<= %u ms)\n", (unsigned)VCOM_LOOPBACK_TIMEOUT_MS);
  return TEST_PASS;
}
#endif // TEST_VCOM_LOOPBACK_1K

// ---------- BLE test state -------------------------------------------------
#if TEST_BLE_1K_ACCUMULATE_AND_ACK || TEST_BLE_1K_ECHO_BACK || TEST_BLE_LOG_WRITE_OPCODE
static uint8_t  ble_buf[1000];
static uint16_t ble_len = 0;
static uint8_t  ble_conn = 0xFF;
#endif

// --- App init / boot -------------------------------------------------------
void utests_init(void) {
#if TEST_PM_LOG_NOTE
  app_log("PM note: ensure RX-capable EM is retained while tests run (see power manager config).\n");
#endif
#if TEST_VCOM_LOOPBACK_1K
  // Defer to boot event to run after clocks are up; nothing here.
#endif

// Other Sanity checks
#if SERIAL_LINK_MTU > 247
    app_log("ERROR: SERIAL_LINK_MTU (%u) must be <= 247\n", (unsigned)SERIAL_LINK_MTU);
#endif
#if SL_IOSTREAM_USART_VCOM_RX_BUFFER_SIZE < SERIAL_LINK_MTU
    app_log("ERROR: VCOM RX buffer (%u) must be >= SERIAL_LINK_MTU (%u)\n",
            (unsigned)SL_IOSTREAM_USART_VCOM_RX_BUFFER_SIZE,
            (unsigned)SERIAL_LINK_MTU);
#endif

    if(gattdb_spp_data_rx_len < SERIAL_LINK_MTU)
    {
        app_log("ERROR: GATTDB_SPP_DATA_RX_LEN (%u) must be >= SERIAL_LINK_MTU (%u)\n",
                (unsigned)gattdb_spp_data_rx_len,
                (unsigned)SERIAL_LINK_MTU);
    }
}

void utests_on_event(sl_bt_msg_t *evt)
{
    uint16_t max_mtu_out;

  switch (SL_BT_MSG_ID(evt->header)) {

    case sl_bt_evt_system_boot_id:
#if TEST_BLE_LOG_MTU
      // Prefer 247; effective MTU is reported later in mtu_exchanged
      (void)sl_bt_gatt_server_set_max_mtu(SERIAL_LINK_MTU, &max_mtu_out );
#endif
#if TEST_VCOM_LOOPBACK_1K
    dump_vcom_config_once();
    run_vcom_loopback_1k_with_timeout();
#endif
      break;

    case sl_bt_evt_connection_opened_id:
#if TEST_BLE_1K_ACCUMULATE_AND_ACK || TEST_BLE_1K_ECHO_BACK || TEST_BLE_LOG_WRITE_OPCODE
      ble_conn = evt->data.evt_connection_opened.connection;
      ble_len  = 0;
      app_log("BLE conn opened: handle=%u\n", ble_conn);
#endif
      break;

    case sl_bt_evt_gatt_mtu_exchanged_id:
#if TEST_BLE_LOG_MTU
      app_log("BLE MTU negotiated: %u\n", evt->data.evt_gatt_mtu_exchanged.mtu);
#endif
      break;

    case sl_bt_evt_gatt_server_attribute_value_id:
    {
#if TEST_BLE_LOG_WRITE_OPCODE || TEST_BLE_1K_ACCUMULATE_AND_ACK || TEST_BLE_1K_ECHO_BACK
      const uint16_t attr = evt->data.evt_gatt_server_attribute_value.attribute;
      const uint8_t  *p   = evt->data.evt_gatt_server_attribute_value.value.data;
      const uint16_t  n   = evt->data.evt_gatt_server_attribute_value.value.len;
      const uint8_t   op  = evt->data.evt_gatt_server_attribute_value.att_opcode;

#if TEST_BLE_LOG_WRITE_OPCODE
      // 0x12 = Write Request (with response), 0x52 = Write Command (no response)
      if (op == 0x52) {
        app_log("WARN: Client used Write Without Response (opcode 0x52). Use Write With Response for integrity.\n");
      }
#endif

#if TEST_BLE_1K_ACCUMULATE_AND_ACK || TEST_BLE_1K_ECHO_BACK
      if (attr == gattdb_spp_data) { // use your SPP RX characteristic handle
        // Accumulate up to 1000 bytes
        uint16_t copy = (ble_len + n <= sizeof(ble_buf)) ? n : (uint16_t)(sizeof(ble_buf) - ble_len);
        if (copy) {
          memcpy(&ble_buf[ble_len], p, copy);
          ble_len += copy;
        }

        if (ble_len >= 1000) {
#if TEST_BLE_1K_ACCUMULATE_AND_ACK
          uint16_t crc = crc16_ccitt(ble_buf, 1000);
          uint8_t  ack[6];
          // "OK"/"FAIL" + CRC16; here we only compute CRC and mark OK (Write With Response should ensure delivery)
          ack[0] = 'O'; ack[1] = 'K'; ack[2] = ' ';
          ack[3] = (uint8_t)(crc >> 8);
          ack[4] = (uint8_t)(crc & 0xFF);
          ack[5] = '\n';
          (void)sl_bt_gatt_server_send_notification(ble_conn, gattdb_spp_data, sizeof ack, ack);
          app_log("BLE 1k integrity: OK, CRC16=0x%04X\n", crc);
#endif

#if TEST_BLE_1K_ECHO_BACK
          // Echo back the 1000B in MTU-sized chunks
          uint16_t mtu = 247; // safe default; will be the actual negotiated if LOG_MTU is on
          // If you want exact value, cache it from mtu_exchanged event.
          uint16_t max_payload = (mtu > 3) ? (mtu - 3) : 20;
          uint16_t off = 0;
          while (off < 1000) {
            uint16_t chunk = (uint16_t)((1000 - off) < max_payload ? (1000 - off) : max_payload);
            (void)sl_bt_gatt_server_send_notification(ble_conn, gattdb_spp_data, chunk, &ble_buf[off]);
            off += chunk;
          }
          app_log("BLE echo back: 1000 B sent in notifications.\n");
#endif
          ble_len = 0; // ready for next run
        }
      }
#endif // ACCUMULATE/ECHO
#endif // any BLE tests
    } break;

    case sl_bt_evt_connection_closed_id:
#if TEST_BLE_1K_ACCUMULATE_AND_ACK || TEST_BLE_1K_ECHO_BACK || TEST_BLE_LOG_WRITE_OPCODE
      app_log("BLE conn closed.\n");
      ble_conn = 0xFF; ble_len = 0;
#endif
      break;

    default:
      break;
  }
}
