#ifndef UTESTS_H
#define UTESTS_H

#define UTESTS_ENABLED 1 // Set to 1 to enable unit tests, 0 to disable

// ===== Test toggles (0=off, 1=on) =====
#define TEST_VCOM_LOOPBACK_1K 1          // Enable 1 kB internal VCOM loopback
#define TEST_VCOM_CONFIG_DUMP 1          // Log VCOM buffer sizes and flow control mode
#define TEST_BLE_LOG_WRITE_OPCODE 1      // Warn if client uses Write Without Response
#define TEST_BLE_LOG_MTU 1               // Log negotiated MTU on exchange
#define TEST_BLE_1K_ACCUMULATE_AND_ACK 1 // Accumulate 1000 B, CRC16, notify OK/FAIL
#define TEST_BLE_1K_ECHO_BACK 0          // Echo accumulated 1 kB back in MTU-sized chunks
#define TEST_PM_LOG_NOTE 1               // One-time power manager note


void utests_on_event(sl_bt_msg_t *evt);
void utests_init(void);

#endif // UTESTS_H