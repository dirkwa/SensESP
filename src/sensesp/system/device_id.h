#ifndef SENSESP_SYSTEM_DEVICE_ID_H_
#define SENSESP_SYSTEM_DEVICE_ID_H_

#include <Arduino.h>
#include <esp_mac.h>

namespace sensesp {

/**
 * @brief Get the device's base MAC address as a stable hardware identifier.
 *
 * Returns the eFuse base MAC address in standard colon-separated format
 * (e.g. "AA:BB:CC:DD:EE:FF"). This is burned into the chip at the factory
 * and does not change across reboots, firmware updates, hostname changes,
 * or network interface selection — making it suitable as a persistent device
 * identifier comparable to the CAN Name used by N2K devices.
 */
inline String get_device_id() {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  char buf[18];  // "AA:BB:CC:DD:EE:FF\0"
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

}  // namespace sensesp

#endif  // SENSESP_SYSTEM_DEVICE_ID_H_
