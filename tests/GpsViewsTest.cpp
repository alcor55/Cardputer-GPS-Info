#include "../GpsViews.h"
#include <assert.h>
#include <stdio.h>

void observe(GPSSatelliteMetrics &metrics, const std::string &body) {
  unsigned char checksum = 0;
  for (char c : body) checksum ^= c;
  char suffix[4];
  snprintf(suffix, sizeof(suffix), "*%02X", checksum);
  std::string line = "$" + body + suffix;
  NMEAFields fields(line.c_str());
  metrics.process(fields, 100);
  metrics.refresh(100, true, true);
}

int main() {
  GPSSatelliteMetrics metrics;
  assert(!constellationCounts(metrics, 100, true).available);
  observe(metrics, "GPGSV,1,1,02,01,30,120,40,33,40,150,35,1");
  observe(metrics, "GPGSV,1,1,01,01,30,120,40,5");
  observe(metrics, "GLGSV,1,1,01,65,30,120,40");
  observe(metrics, "GAGSV,1,1,01,01,30,120,40");
  observe(metrics, "GBGSV,1,1,01,01,30,120,40");
  observe(metrics, "GQGSV,1,1,01,01,30,120,40");
  observe(metrics, "GIGSV,1,1,01,01,30,120,40");
  GPSConstellationCounts counts = constellationCounts(metrics, 100, true);
  assert(counts.available && !counts.partial && counts.total == 7);
  for (int count : counts.counts) assert(count == 1);
  assert(constellationBarScale(counts) == 10);
  counts.counts[3] = 63;
  assert(constellationBarScale(counts) == 80);
  observe(metrics, "GNGSV,1,1,01,01,30,120,40");
  counts = constellationCounts(metrics, 100, true);
  assert(counts.partial && counts.total == 7);
  assert(!constellationCounts(metrics, 100, false).available);
  metrics.refresh(20000, true, true);
  assert(!constellationCounts(metrics, 20000, true).available);
  GPSSatelliteMetrics empty;
  observe(empty, "GPGSV,1,1,00");
  counts = constellationCounts(empty, 100, true);
  assert(counts.available && counts.total == 0 && !counts.partial);
  puts("Constellation bar counts, multiband deduplication and missing-data tests passed.");
}
