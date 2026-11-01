#include "../GpsConfig.h"
#include <assert.h>
#include <initializer_list>
#include <stdio.h>
#include <string.h>

int main() {
  GPSConfig grove = gpsPreset(GPS_GROVE);
  GPSConfig cap = gpsPreset(GPS_CAP);
  GPSConfig ares = gpsPreset(GPS_ARES);
  assert(!gpsConfigError(grove, false));
  assert(!gpsConfigError(cap, true));
  assert(!gpsConfigError(ares, true));
  assert(gpsConfigError(cap, false));
  assert(cap.rxPin == 15 && cap.txPin == 13 && cap.baud == 115200);
  cap.usbBaud = 57600;
  assert(!gpsConfigError(cap, true));
  GPSConfig switched = selectGPSPreset(cap, GPS_ARES);
  assert(switched.usbBaud == 57600 && switched.baud == 115200);
  switched.usbBaud = 0;
  assert(gpsConfigError(switched, true));

  // A baud override must preserve the Ares electrical behaviour.
  ares.profile = GPS_MANUAL;
  ares.baud = 9600;
  assert(!gpsConfigError(ares, true));
  ares.rxPin = 13;
  ares.txPin = 15;
  assert(gpsConfigError(ares, true));
  ares.rxPin = 4;
  assert(gpsConfigError(ares, true));
  grove.profile = GPS_MANUAL;
  grove.rxPin = grove.txPin;
  assert(gpsConfigError(grove, false));
  for (int pin : {8, 9, 11, 12, 14, 19, 20, 33, 39, 40, 99}) {
    cap.profile = GPS_MANUAL;
    cap.rxPin = pin;
    assert(gpsConfigError(cap, true));
  }

  int number = 123;
  assert(parseGPSNumber("115200", number) && number == 115200);
  for (const char* text : {"", "-1", "9600x", "abc", "2147483648", "999999999999"})
    assert(!parseGPSNumber(text, number));
  assert(parseGPSNumber("0009600", number) && number == 9600);
  grove = gpsPreset(GPS_GROVE);
  grove.profile = GPS_MANUAL;
  grove.baud = 0;
  assert(gpsConfigError(grove, false));

  GPSConfigReader legacy;
  legacy.set("gpsRxPin", "15");
  legacy.set("gpsTxPin", "13");
  legacy.set("gpsBaud", "115200");
  assert(!legacy.error(true));
  assert(legacy.config.usbBaud == 115200);
  assert(legacy.config.profile == GPS_MANUAL && legacy.config.hardware == GPS_GROVE);
  assert(legacy.error(false));
  GPSConfigReader saved = legacy;
  saved.set("gpsProfile", "2");
  assert(!saved.error(true) && saved.config.hardware == GPS_ARES);
  GPSConfigReader usb = saved;
  usb.set("usbBaud", "57600");
  assert(!usb.error(true) && usb.config.usbBaud == 57600);
  GPSConfigReader invalidUSB = saved;
  invalidUSB.set("usbBaud", "0");
  assert(invalidUSB.error(true));
  GPSConfigReader manual = legacy;
  manual.set("gpsProfile", "3");
  manual.set("gpsHardware", "2");
  assert(!manual.error(true) && manual.config.hardware == GPS_ARES);
  GPSConfigReader incomplete;
  incomplete.set("gpsBaud", "9600");
  assert(incomplete.error(true));
  GPSConfigReader malformed = legacy;
  malformed.set("gpsRxPin", "abc");
  assert(malformed.error(true));
  GPSConfigReader unknown = legacy;
  unknown.set("gpsProfile", "99");
  assert(unknown.error(true));

  GPSKeyEdges keys;
  keys.update("c");
  assert(keys.pressed['c']);
  keys.update("c");
  assert(!keys.pressed['c']);
  keys.update("c.");
  assert(!keys.pressed['c'] && keys.pressed['.']);
  keys.update("c;"); // Same number of held keys, different arrow.
  assert(!keys.pressed['c'] && keys.pressed[';']);
  keys.update("");
  keys.update("c");
  assert(keys.pressed['c']);
  keys.update("\n\b");
  assert(keys.pressed['\n'] && keys.pressed['\b']);
  keys.update("\n\b");
  assert(!keys.pressed['\n'] && !keys.pressed['\b']);

  const char* valid = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
  assert(validNMEASentence(valid, strlen(valid)));
  assert(!validNMEASentence("garbage", 7));
  assert(!validNMEASentence("$GPGGA*00", 9));
  char corrupted[128];
  strcpy(corrupted, valid);
  corrupted[10] = '9';
  assert(!validNMEASentence(corrupted, strlen(corrupted)));
  assert(!validNMEASentence(valid, strlen(valid) - 1));
  assert(!validNMEASentence("$GP*00*00", 9));

  // Starting after a long uptime must still get a full grace period.
  assert(gpsReceptionState(true, false, false, false, 100001, 100000, 0, 0) == GPS_WAIT);
  assert(gpsReceptionState(true, false, false, false, 103000, 100000, 0, 0) == GPS_NO_DATA);
  assert(gpsReceptionState(true, true, false, false, 100010, 100000, 100010, 0) == GPS_BAD_DATA);
  assert(gpsReceptionState(true, true, true, false, 100010, 100000, 100010, 100010) == GPS_ON);
  assert(gpsReceptionState(true, true, true, true, 100010, 100000, 100010, 100010) == GPS_FIX);
  assert(gpsReceptionState(true, true, true, true, 103010, 100000, 100010, 100010) == GPS_NO_DATA);
  assert(gpsReceptionState(true, true, true, false, 103010, 100000, 103010, 100010) == GPS_BAD_DATA);
  assert(gpsReceptionState(false, true, true, true, 100010, 100000, 100010, 100010) == GPS_OFF);
  // Unsigned elapsed times must survive the millis() rollover.
  assert(gpsReceptionState(true, false, false, false, 10, UINT32_MAX - 10, 0, 0) == GPS_WAIT);
  puts("GPS configuration, checksum and reception tests passed.");
}
