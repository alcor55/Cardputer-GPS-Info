#include "../GpsMetrics.h"
#include <assert.h>
#include <stdio.h>

std::string frame(const std::string &body) {
  unsigned char checksum = 0;
  for (char c : body) checksum ^= c;
  char suffix[4];
  snprintf(suffix, sizeof(suffix), "*%02X", checksum);
  return "$" + body + suffix;
}

void process(GPSSatelliteMetrics &metrics, const std::string &body, uint32_t now) {
  std::string sentence = frame(body);
  NMEAFields fields(sentence.c_str());
  assert(fields.count > 0);
  metrics.process(fields, now);
  metrics.refresh(now, true, true);
}

std::string gsa(const std::string &talker, const std::vector<int> &ids, int mode = 3, int system = 0) {
  std::vector<std::string> fields(18);
  fields[0] = talker + "GSA";
  fields[1] = "A";
  fields[2] = std::to_string(mode);
  for (size_t i = 0; i < ids.size(); i++) fields[3 + i] = std::to_string(ids[i]);
  fields[15] = "1.0"; fields[16] = "0.6"; fields[17] = "0.8";
  if (system) fields.push_back(std::to_string(system));
  std::string result;
  for (const auto &field : fields) { if (!result.empty()) result += ','; result += field; }
  return result;
}

int countUsed(const GPSSatelliteMetrics &metrics, bool historical) {
  int result = 0;
  for (const auto &sat : metrics.satellites) if (historical ? sat.usedEver : sat.used) result++;
  return result;
}

int countVisible(const GPSSatelliteMetrics &metrics) {
  int result = 0;
  for (const auto &sat : metrics.satellites) if (sat.visible) result++;
  return result;
}

void testSatelliteHistory() {
  GPSSatelliteMetrics metrics;
  // GSA may precede GSV, and local IDs can overlap between constellations.
  process(metrics, gsa("GP", {7}), 100);
  assert(metrics.satellites.size() == 1 && countUsed(metrics, true) == 1);
  assert(!metrics.satellites[0].hasPosition);
  process(metrics, "GPGSV,1,1,01,07,30,120,40", 110);
  process(metrics, "GAGSV,1,1,01,07,45,100,35", 120);
  assert(metrics.satellites.size() == 2);
  assert(countUsed(metrics, false) == 1 && countUsed(metrics, true) == 1);
  process(metrics, gsa("GP", {}, 1), 130);
  assert(countUsed(metrics, false) == 0 && countUsed(metrics, true) == 1);
  process(metrics, gsa("GN", {7}, 3, 3), 140);
  assert(countUsed(metrics, true) == 2);
  metrics.refresh(20000, true, true);
  assert(countVisible(metrics) == 0 && countUsed(metrics, false) == 0);
  assert(countUsed(metrics, true) == 2 && metrics.satellites.size() == 2);
  assert(!metrics.freshViews(20000, true));
  metrics.refresh(20001, false, false);
  assert(countUsed(metrics, true) == 2);
}

void testGSAEpochs() {
  GPSSatelliteMetrics metrics;
  process(metrics, gsa("GN", {1, 2}, 3, 1), 100);
  process(metrics, gsa("GN", {3, 4}, 3, 1), 110);
  assert(countUsed(metrics, false) == 4);
  process(metrics, gsa("GN", {2}, 3, 1), 1100);
  assert(countUsed(metrics, false) == 1 && countUsed(metrics, true) == 4);
  // UTC boundaries also separate cycles at update rates above 4 Hz.
  process(metrics, "GNGGA,123519.00,4807.038,N,01131.000,E,1,01,0.9,545.4,M,46.9,M,,", 1110);
  process(metrics, gsa("GN", {3}, 3, 1), 1120);
  assert(countUsed(metrics, false) == 1);
  assert(metrics.satellites[2].used);
  // A continuous GSA-only stream must not accumulate a current set forever.
  GPSSatelliteMetrics rapid;
  process(rapid, gsa("GP", {1}), 100);
  process(rapid, gsa("GP", {2}), 200);
  process(rapid, gsa("GP", {3}), 400);
  assert(countUsed(rapid, false) == 1 && countUsed(rapid, true) == 3);
}

void testSignalUnion() {
  GPSSatelliteMetrics metrics;
  process(metrics, "GPGSV,1,1,02,01,30,120,40,02,40,150,35,1", 100);
  process(metrics, "GPGSV,1,1,01,02,40,150,38,5", 200);
  assert(metrics.satellites.size() == 2 && countVisible(metrics) == 2);
  assert(metrics.satellites[1].snr == 38);
  metrics.refresh(15100, true, true);
  assert(countVisible(metrics) == 1);
  process(metrics, "GPGSV,1,1,00,5", 15110);
  assert(countVisible(metrics) == 0 && metrics.freshViews(15110, true));
  metrics.refresh(15120, false, false);
  assert(countVisible(metrics) == 0 && !metrics.freshViews(15120, false));
}

void testIncompleteGSV() {
  GPSSatelliteMetrics metrics;
  process(metrics, "GPGSV,2,2,05,05,30,120,40", 100);
  assert(metrics.satellites.empty() && !metrics.freshViews(100, true));
  process(metrics, "GPGSV,2,1,05,01,30,120,40,02,40,150,35,03,45,180,30,04,50,200,25", 110);
  assert(metrics.satellites.empty());
  process(metrics, "GPGSV,2,2,05,05,30,120,40", 120);
  assert(metrics.satellites.size() == 5 && countVisible(metrics) == 5);
  // A broken next cycle must not replace the last complete snapshot.
  process(metrics, "GPGSV,2,1,05,01,30,120,40,02,40,150,35,03,45,180,30,04,50,200,25", 130);
  process(metrics, "GPGSV,2,2,04,05,30,120,40", 140);
  assert(countVisible(metrics) == 5);
  process(metrics, "GPGSV,1,1,00", 150);
  assert(countVisible(metrics) == 0 && metrics.satellites.size() == 5);
  process(metrics, "GPGSV,1,1,01,00,30,120,40", 160);
  assert(metrics.satellites.size() == 5);
}

void testIdentity() {
  GPSSatelliteMetrics metrics;
  process(metrics, "GPGSV,1,1,02,01,30,120,40,33,40,150,35", 100);
  process(metrics, "GLGSV,1,1,01,65,30,120,40", 110);
  process(metrics, gsa("GN", {1}, 3, 2), 120);
  process(metrics, "GQGSV,1,1,01,01,30,120,40", 130);
  assert(metrics.satellites.size() == 4);
  assert(metrics.satellites[1].system == "SBAS" && metrics.satellites[1].id == 120);
  assert(metrics.satellites[2].system == "GLONASS" && metrics.satellites[2].used);
  assert(metrics.satellites[3].system == "QZSS");
  // A GN local ID has no unique constellation. Do not create a duplicate.
  process(metrics, "GNGSV,1,1,01,01,30,120,40", 140);
  process(metrics, gsa("GN", {1}), 150);
  assert(metrics.satellites.size() == 4);
  assert(metrics.seenUncertain && metrics.usedUncertain && metrics.uncertainViews(150, true));
  assert(countUsed(metrics, true) == 1);
  process(metrics, "GBGSV,1,1,01,01,,,35,B", 160);
  assert(metrics.satellites.size() == 5 && !metrics.satellites.back().hasPosition);
  assert(metrics.satellites.back().system == "BeiDou");
  process(metrics, "GBGSV,1,1,01,01,30,120,40,B", 170);
  assert(metrics.satellites.back().hasPosition);
  process(metrics, "GBGSV,1,1,01,01,,,35,B", 180);
  assert(!metrics.satellites.back().hasPosition);
}

void navigate(GPSNavigationMetrics &metrics, const std::string &body, uint32_t now) {
  std::string sentence = frame(body);
  NMEAFields fields(sentence.c_str());
  assert(fields.count > 0);
  metrics.process(fields, now);
}

void testMetricValidity() {
  GPSNavigationMetrics metrics;
  assert(!metrics.speed.fresh(0, true));
  navigate(metrics, "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,", 100);
  navigate(metrics, "GPRMC,123519,A,4807.038,N,01131.000,E,10.0,84.4,011126,,", 110);
  assert(metrics.position.fresh(110, true) && metrics.altitude.fresh(110, true));
  assert(metrics.speed.fresh(110, true) && metrics.course.fresh(110, true));
  assert(metrics.hdop.fresh(110, true) && metrics.inFix.fresh(110, true));
  assert(metrics.date.fresh(110, true) && metrics.time.fresh(110, true));
  assert(!metrics.speed.fresh(60000, true) && !metrics.inFix.fresh(60000, true));
  assert(!metrics.date.fresh(110, false));
  // Empty fields must invalidate TinyGPS++ values even when it recommits them.
  navigate(metrics, "GPRMC,123520,A,4807.038,N,01131.000,E,,,011126,,", 200);
  assert(!metrics.speed.fresh(200, true) && !metrics.course.fresh(200, true));
  navigate(metrics, "GPGGA,123520,4807.038,N,01131.000,E,1,08,,,M,46.9,M,,", 210);
  assert(!metrics.hdop.fresh(210, true) && !metrics.altitude.fresh(210, true));
  navigate(metrics, "GPRMC,123521,A,4807.038,N,01131.000,E,0.0,0.0,011126,,", 220);
  assert(metrics.speed.fresh(220, true) && metrics.course.fresh(220, true));
  navigate(metrics, "GPRMC,123522,V,,,,,,,011126,,", 230);
  assert(!metrics.hasFix && !metrics.position.fresh(230, true));
  assert(metrics.time.fresh(230, true) && metrics.date.fresh(230, true));
  navigate(metrics, "GPRMC,256199,A,4860.000,N,01131.000,E,1,400,311126,,", 240);
  assert(!metrics.time.fresh(240, true) && !metrics.date.fresh(240, true));
  assert(!metrics.position.fresh(240, true) && !metrics.course.fresh(240, true));
  navigate(metrics, "GPRMC,123521,A,4807.038,N,01131.000,E,0.0,0.0,011126,,,E", 250);
  assert(!metrics.hasFix); // Dead reckoning is not a satellite fix.
  GPSMetricStamp stamp;
  stamp.set(true, UINT32_MAX - 10);
  assert(stamp.fresh(10, true) && !stamp.fresh(6000, true));
}

void testDecoderFrames() {
  GPSNavigationMetrics metrics;
  std::string sentence = frame("GAGGA,123519,4807.038,N,01131.000,E,1,08,,,M,46.9,M,,");
  NMEAFields fields(sentence.c_str());
  metrics.process(fields, 100);
  std::string normalized = metrics.decoderFrame(fields);
  assert(validNMEASentence(normalized.c_str(), normalized.length()));
  NMEAFields decoded(normalized.c_str());
  assert(!strcmp(decoded.field(0), "$GPGGA"));
  assert(!strcmp(decoded.field(8), "0") && !strcmp(decoded.field(9), "0"));
  assert(!metrics.hdop.fresh(100, true) && !metrics.altitude.fresh(100, true));
  sentence = frame("GNRMC,123519,A,4807.038,N,01131.000,E,,,011126,,");
  NMEAFields rmc(sentence.c_str());
  metrics.process(rmc, 110);
  normalized = metrics.decoderFrame(rmc);
  NMEAFields decodedRMC(normalized.c_str());
  assert(!strcmp(decodedRMC.field(7), "0") && !strcmp(decodedRMC.field(8), "0"));
  assert(!metrics.speed.fresh(110, true) && !metrics.course.fresh(110, true));
}

int main() {
  testSatelliteHistory();
  testGSAEpochs();
  testSignalUnion();
  testIncompleteGSV();
  testIdentity();
  testMetricValidity();
  testDecoderFrames();
  puts("Satellite identity, history, GSV sequences, bands and metric freshness tests passed.");
}
