#pragma once

#include "GpsMetrics.h"

struct GPSConstellationCounts {
  int counts[7] = {};
  int total = 0;
  bool available = false;
  bool partial = false;
};

inline const char* satelliteSystemCode(const std::string &system) {
  if (system == "GPS") return "Gp";
  if (system == "GLONASS") return "Gl";
  if (system == "Galileo") return "Ga";
  if (system == "BeiDou") return "Bd";
  if (system == "QZSS") return "Qz";
  if (system == "SBAS") return "Sb";
  if (system == "NavIC") return "In";
  return "?";
}

/*    Use the same identified, currently visible satellites as the main table.
*/
inline GPSConstellationCounts constellationCounts(const GPSSatelliteMetrics &metrics,
    uint32_t now, bool stream) {
  GPSConstellationCounts result;
  result.available = metrics.freshViews(now, stream);
  result.partial = metrics.uncertainViews(now, stream);
  if (!result.available) return result;
  const char* names[] = {"GPS", "GLONASS", "Galileo", "BeiDou", "QZSS", "SBAS", "NavIC"};
  for (const auto &sat : metrics.satellites) {
    if (!sat.visible) continue;
    for (int i = 0; i < 7; i++) {
      if (sat.system == names[i]) { result.counts[i]++; result.total++; break; }
    }
  }
  return result;
}

/*    Keep a readable axis while adapting to different satellite populations.
*/
inline int constellationBarScale(const GPSConstellationCounts &counts) {
  int maximum = 0;
  for (int count : counts.counts) if (count > maximum) maximum = count;
  if (maximum <= 10) return 10;
  if (maximum <= 20) return 20;
  return ((maximum + 19) / 20) * 20;
}
