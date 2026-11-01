#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum GPSProfile { GPS_GROVE, GPS_CAP, GPS_ARES, GPS_MANUAL, GPS_PROFILE_COUNT };

struct GPSConfig {
  int profile;
  int hardware;    // Keep the hardware behaviour when editing a preset manually.
  int rxPin;
  int txPin;
  int baud;
  int usbBaud;
};

enum GPSState { GPS_OFF, GPS_WAIT, GPS_NO_DATA, GPS_BAD_DATA, GPS_ON, GPS_FIX };
const unsigned long GPS_TIMEOUT = 3000;
const unsigned long GPS_FIX_TIMEOUT = 5000;

/*    Get the default UART settings for a known GPS connection.
*/
inline GPSConfig gpsPreset(int profile) {
  if (profile == GPS_CAP || profile == GPS_ARES)
    return {profile, profile, 15, 13, 115200, 115200};
  return {GPS_GROVE, GPS_GROVE, 1, 2, 9600, 115200};
}

/*    GPS presets must not replace the independent USB console setting.
*/
inline GPSConfig selectGPSPreset(const GPSConfig &current, int profile) {
  GPSConfig selected = gpsPreset(profile);
  selected.usbBaud = current.usbBaud;
  return selected;
}

/*    Parse a decimal setting without silently accepting invalid text or overflow.
*/
inline bool parseGPSNumber(const char* text, int &value) {
  if (!text || !*text) return false;
  int number = 0;
  for (const char* c = text; *c; c++) {
    if (*c < '0' || *c > '9' || number > (2000000 - (*c - '0')) / 10)
      return false;
    number = number * 10 + (*c - '0');
  }
  value = number;
  return true;
}

/*    Allow exposed UART pins without taking over onboard peripherals.
*/
inline bool validGPSPin(int pin, bool adv, int hardware) {
  if (pin == 1 || pin == 2) return true;
  if (!adv) return false;
  if (pin == 13 || pin == 15) return true;
  // Both expansion boards reserve G3/G4/G5/G6 for IR or radio control.
  return hardware == GPS_GROVE && (pin == 3 || pin == 4 || pin == 5 || pin == 6);
}

/*    Validate a complete configuration before changing the active UART.
*/
inline const char* gpsConfigError(const GPSConfig &config, bool adv) {
  if (config.profile < 0 || config.profile >= GPS_PROFILE_COUNT
      || config.hardware < GPS_GROVE || config.hardware > GPS_ARES)
    return "Invalid profile";
  if (!adv && config.hardware != GPS_GROVE) return "ADV hardware required";
  if (!validGPSPin(config.rxPin, adv, config.hardware)
      || !validGPSPin(config.txPin, adv, config.hardware))
    return "Pin unavailable/reserved";
  if (config.rxPin == config.txPin) return "RX and TX must differ";
  if (config.baud < 1200 || config.baud > 1000000) return "Baud: 1200..1000000";
  if (config.usbBaud < 1200 || config.usbBaud > 1000000) return "USB baud: 1200..1000000";
  // G13 is an output to GPS RX and nRF CSN on Ares, never a GPS input.
  if (config.hardware == GPS_ARES && config.rxPin == 13)
    return "Ares G13 is TX / CSN";
  if (config.profile != GPS_MANUAL) {
    GPSConfig preset = gpsPreset(config.profile);
    if (config.hardware != preset.hardware || config.rxPin != preset.rxPin
        || config.txPin != preset.txPin || config.baud != preset.baud)
      return "Preset values mismatch";
  }
  return nullptr;
}

/*    Read old and new configuration files without applying partial settings.
*/
struct GPSConfigReader {
  GPSConfig config = {GPS_MANUAL, GPS_GROVE, 1, 2, 9600, 115200};
  unsigned int fields = 0;
  bool valid = true;

  void set(const char* key, const char* value) {
    const char* keys[] = {"gpsRxPin", "gpsTxPin", "gpsBaud", "gpsProfile", "gpsHardware", "usbBaud"};
    int* values[] = {&config.rxPin, &config.txPin, &config.baud, &config.profile, &config.hardware, &config.usbBaud};
    for (int i = 0; i < 6; i++) {
      if (strcmp(key, keys[i]) != 0) continue;
      if ((fields & (1U << i)) || !parseGPSNumber(value, *values[i])) valid = false;
      fields |= 1U << i;
      break;
    }
  }

  const char* error(bool adv) {
    if (!valid || (fields & 7) != 7) return "Invalid/incomplete SD config";
    if ((fields & 8) && !(fields & 16) && config.profile != GPS_MANUAL)
      config.hardware = config.profile;
    return gpsConfigError(config, adv);
  }
};

/*    Track key-down edges independently of the number of held keys.
*/
struct GPSKeyEdges {
  bool previous[128] = {};
  bool pressed[128] = {};

  void update(const char* keys) {
    bool current[128] = {};
    for (const char* c = keys; *c; c++) {
      unsigned char key = static_cast<unsigned char>(*c);
      if (key < 128) current[key] = true;
    }
    for (int i = 0; i < 128; i++) {
      pressed[i] = current[i] && !previous[i];
      previous[i] = current[i];
    }
  }
};

/*    Accept only complete NMEA frames with a matching checksum.
*/
inline bool validNMEASentence(const char* line, size_t length) {
  if (length < 10 || line[0] != '$' || line[6] != ',' || line[length - 3] != '*') return false;
  for (size_t i = 1; i <= 5; i++) {
    if (!((line[i] >= 'A' && line[i] <= 'Z') || (line[i] >= '0' && line[i] <= '9')))
      return false;
  }
  uint8_t checksum = 0;
  for (size_t i = 1; i < length - 3; i++) {
    if (line[i] == '*') return false;
    checksum ^= static_cast<uint8_t>(line[i]);
  }
  const char* digits = "0123456789ABCDEF";
  char high = line[length - 2];
  char low = line[length - 1];
  if (high >= 'a' && high <= 'f') high -= 'a' - 'A';
  if (low >= 'a' && low <= 'f') low -= 'a' - 'A';
  return high == digits[checksum >> 4] && low == digits[checksum & 15];
}

/*    Distinguish UART traffic, valid NMEA and a current position fix.
*/
inline GPSState gpsReceptionState(bool running, bool received, bool valid,
    bool fix, uint32_t now, uint32_t started, uint32_t lastByte, uint32_t lastValid) {
  if (!running) return GPS_OFF;
  if (!received) return now - started < GPS_TIMEOUT ? GPS_WAIT : GPS_NO_DATA;
  if (now - lastByte >= GPS_TIMEOUT) return GPS_NO_DATA;
  if (!valid || now - lastValid >= GPS_TIMEOUT) return GPS_BAD_DATA;
  return fix ? GPS_FIX : GPS_ON;
}
