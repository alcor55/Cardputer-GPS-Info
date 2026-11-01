#include "../GpsMetrics.h"
#include <TinyGPS++.h>
#include <assert.h>

unsigned long testMillis = 1000;
unsigned long millis() { return testMillis; }

void feed(GPSNavigationMetrics &metrics, TinyGPSPlus &gps, const std::string &body) {
  uint8_t checksum = 0;
  for (char c : body) checksum ^= c;
  char suffix[4];
  snprintf(suffix, sizeof(suffix), "*%02X", checksum);
  std::string line = "$" + body + suffix;
  NMEAFields fields(line.c_str());
  metrics.process(fields, testMillis);
  for (char c : metrics.decoderFrame(fields)) gps.encode(c);
  gps.encode('\r');
  gps.encode('\n');
}

int main() {
  GPSNavigationMetrics metrics;
  TinyGPSPlus gps;
  feed(metrics, gps, "GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
  feed(metrics, gps, "GNRMC,123519,A,4807.038,N,01131.000,E,10.0,84.4,011126,,");
  assert(std::abs(gps.speed.kmph() - 18.52) < 0.001);
  assert(std::abs(gps.location.lat() - 48.1173) < 0.000001);
  assert(gps.satellites.value() == 8 && metrics.inFix.fresh(testMillis, true));
  testMillis += 1000;
  feed(metrics, gps, "GNRMC,123520,A,4807.038,N,01131.000,E,,,011126,,");
  assert(!metrics.speed.fresh(testMillis, true) && !metrics.course.fresh(testMillis, true));
  // A single-constellation navigation talker must be decoded as well.
  feed(metrics, gps, "GAGGA,123520,4807.038,N,01131.000,E,1,09,,,M,46.9,M,,");
  assert(gps.satellites.value() == 9);
  assert(!metrics.altitude.fresh(testMillis, true) && !metrics.hdop.fresh(testMillis, true));
  // Each metric is independent: a later position must not expose old altitude.
  feed(metrics, gps, "GNGGA,123521,,,,,1,09,0.9,123.4,M,46.9,M,,");
  assert(!metrics.position.fresh(testMillis, true));
  feed(metrics, gps, "GNRMC,123521,A,4807.038,N,01131.000,E,0.0,0.0,011126,,");
  assert(metrics.position.fresh(testMillis, true));
  assert(metrics.altitude.fresh(testMillis, true));
  assert(std::abs(gps.altitude.meters() - 123.4) < 0.001);
  assert(metrics.speed.fresh(testMillis, true) && gps.speed.kmph() == 0);
  testMillis += 60000;
  assert(!metrics.altitude.fresh(testMillis, true) && !metrics.inFix.fresh(testMillis, true));
  // Starting with empty navigation fields must not expose staged/default values.
  GPSNavigationMetrics emptyMetrics;
  TinyGPSPlus emptyGPS;
  feed(emptyMetrics, emptyGPS, "GNGGA,,,,,,0,,,,,,,,");
  feed(emptyMetrics, emptyGPS, "GNRMC,,V,,,,,,,,,,");
  assert(!emptyMetrics.position.fresh(testMillis, true));
  assert(!emptyMetrics.time.fresh(testMillis, true));
  assert(!emptyMetrics.date.fresh(testMillis, true));
  assert(!emptyMetrics.inFix.fresh(testMillis, true));
  puts("TinyGPS++ decoding, single-constellation frames and empty-field regression tests passed.");
}
