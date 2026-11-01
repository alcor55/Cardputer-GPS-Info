#pragma once

#include "GpsConfig.h"
#include <algorithm>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

const uint32_t GPS_METRIC_TIMEOUT = 5000;
const uint32_t GPS_SAT_TIMEOUT = 15000;

inline bool knownGNSSTalker(const char* type) {
  if (strlen(type) != 6 || type[0] != '$') return false;
  std::string talker(type + 1, 2);
  return talker == "GP" || talker == "GN" || talker == "GL" || talker == "GA"
      || talker == "GB" || talker == "BD" || talker == "GQ" || talker == "GI" || talker == "QZ";
}

/*    Split one checked frame without including its checksum in data fields.
*/
struct NMEAFields {
  char data[256] = {};
  const char* values[32] = {};
  int count = 0;

  explicit NMEAFields(const char* line) {
    size_t length = strlen(line);
    if (length >= sizeof(data) || !validNMEASentence(line, length)) return;
    memcpy(data, line, length - 3);
    values[count++] = data;
    for (char* c = data; *c; c++) {
      if (*c != ',') continue;
      *c = '\0';
      if (count == 32) { count = 0; return; }
      values[count++] = c + 1;
    }
  }

  const char* field(int index) const {
    return index >= 0 && index < count ? values[index] : "";
  }

  bool number(int index, double &value, double low, double high) const {
    const char* text = field(index);
    if (!*text || strlen(text) > 14) return false;
    // NMEA decimal fields do not use exponent notation or whitespace.
    bool digit = false, point = false;
    for (const char* c = text; *c; c++) {
      if (c == text && *c == '-') continue;
      if (*c == '.' && !point) { point = true; continue; }
      if (*c < '0' || *c > '9') return false;
      digit = true;
    }
    if (!digit) return false;
    value = strtod(text, nullptr);
    return std::isfinite(value) && value >= low && value <= high;
  }

  bool integer(int index, int &value, int low, int high) const {
    return parseGPSNumber(field(index), value) && value >= low && value <= high;
  }
};

struct GPSMetricStamp {
  bool valid = false;
  uint32_t updated = 0;

  void set(bool available, uint32_t now) { valid = available; updated = now; }
  bool fresh(uint32_t now, bool running) const {
    return running && valid && now - updated < GPS_METRIC_TIMEOUT;
  }
};

/*    Track field presence separately: TinyGPS++ can recommit an old value when
*     a new sentence leaves that field empty. Age alone cannot detect this.
*/
struct GPSNavigationMetrics {
  GPSMetricStamp position, altitude, speed, course, date, time, hdop, inFix;
  bool hasFix = false;
  uint32_t fixUpdated = 0;

  bool validTime(const char* text) const {
    size_t length = strlen(text);
    if (length < 6) return false;
    for (size_t i = 0; i < length; i++) {
      if (i == 6 && text[i] == '.' && length > 7) continue;
      if (i >= 6 && text[6] != '.') return false;
      if (text[i] < '0' || text[i] > '9') return false;
    }
    int hour = (text[0] - '0') * 10 + text[1] - '0';
    int minute = (text[2] - '0') * 10 + text[3] - '0';
    int second = (text[4] - '0') * 10 + text[5] - '0';
    return hour < 24 && minute < 60 && second <= 60;
  }

  bool validDate(const char* text) const {
    if (strlen(text) != 6) return false;
    for (int i = 0; i < 6; i++) if (text[i] < '0' || text[i] > '9') return false;
    int day = (text[0] - '0') * 10 + text[1] - '0';
    int month = (text[2] - '0') * 10 + text[3] - '0';
    int year = 2000 + (text[4] - '0') * 10 + text[5] - '0';
    const int days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    return month >= 1 && month <= 12 && day >= 1
        && day <= days[month - 1] + (month == 2 && year % 4 == 0);
  }

  bool validPosition(const NMEAFields &fields, int latitude, int longitude) const {
    double lat = 0, lon = 0;
    if (!fields.number(latitude, lat, 0, 9000) || !fields.number(longitude, lon, 0, 18000))
      return false;
    const char* north = fields.field(latitude + 1);
    const char* east = fields.field(longitude + 1);
    return fmod(lat, 100) < 60 && fmod(lon, 100) < 60
        && (!strcmp(north, "N") || !strcmp(north, "S"))
        && (!strcmp(east, "E") || !strcmp(east, "W"));
  }

  void process(const NMEAFields &fields, uint32_t now) {
    const char* type = fields.field(0);
    if (!knownGNSSTalker(type)) return;
    double number = 0;
    int integer = 0;
    if (!strcmp(type + 3, "GGA")) {
      bool quality = fields.integer(6, integer, 0, 8) && integer != 7;
      hasFix = quality && integer > 0 && integer != 6 && integer != 8;
      fixUpdated = now;
      position.set(hasFix && validPosition(fields, 2, 4), now);
      altitude.set(hasFix && fields.number(9, number, -20000, 100000)
          && !strcmp(fields.field(10), "M"), now);
      hdop.set(hasFix && fields.number(8, number, 0.01, 1000), now);
      inFix.set(quality && fields.integer(7, integer, 0, 256), now);
      time.set(validTime(fields.field(1)), now);
    } else if (!strcmp(type + 3, "RMC")) {
      const char* mode = fields.field(12);
      hasFix = !strcmp(fields.field(2), "A") && strcmp(mode, "N")
          && strcmp(mode, "E") && strcmp(mode, "M") && strcmp(mode, "S");
      fixUpdated = now;
      position.set(hasFix && validPosition(fields, 3, 5), now);
      speed.set(hasFix && fields.number(7, number, 0, 10000), now);
      course.set(hasFix && fields.number(8, number, 0, 359.999999), now);
      date.set(validDate(fields.field(9)), now);
      time.set(validTime(fields.field(1)), now);
    }
  }

  /*    Normalize navigation talkers for TinyGPS++ and initialize missing
  *     numeric staging fields. Only the original validity stamps control UI.
  */
  std::string decoderFrame(const NMEAFields &fields) const {
    const char* type = fields.field(0);
    if (!knownGNSSTalker(type)) return "";
    bool gga = !strcmp(type + 3, "GGA");
    if (!gga && strcmp(type + 3, "RMC")) return "";
    std::string line = gga ? "$GPGGA" : "$GPRMC";
    for (int i = 1; i < (gga ? 15 : 13); i++) {
      const char* value = fields.field(i);
      if (i == 1 && !time.valid) value = "000000";
      if (gga) {
        if (i == 6) value = hasFix ? "1" : "0";
        if (i == 7 && !inFix.valid) value = "0";
        if (i == 8 && !hdop.valid) value = "0";
        if (i == 9 && !altitude.valid) value = "0";
        if (!position.valid && (i == 2 || i == 4)) value = "0";
        if (!position.valid && i == 3) value = "N";
        if (!position.valid && i == 5) value = "E";
      } else {
        if (i == 2) value = hasFix ? "A" : "V";
        if (i == 7 && !speed.valid) value = "0";
        if (i == 8 && !course.valid) value = "0";
        if (i == 9 && !date.valid) value = "010100";
        if (!position.valid && (i == 3 || i == 5)) value = "0";
        if (!position.valid && i == 4) value = "N";
        if (!position.valid && i == 6) value = "E";
      }
      line += ',';
      line += value;
    }
    uint8_t checksum = 0;
    for (size_t i = 1; i < line.length(); i++) checksum ^= line[i];
    char suffix[4];
    snprintf(suffix, sizeof(suffix), "*%02X", checksum);
    return line + suffix;
  }
};

struct SatData {
  std::string system;
  int id = 0;
  int elevation = 0;
  int azimuth = 0;
  int snr = -1;          // -1 means no signal measurement was reported.
  bool used = false;    // Used in the current fix.
  bool usedEver = false;
  bool visible = false;
  bool hasPosition = false;
  uint32_t positionUpdated = 0;
};

struct SatelliteKey {
  std::string system;
  int id;
  bool operator==(const SatelliteKey &other) const {
    return system == other.system && id == other.id;
  }
};

struct GSVSequenceState {
  std::string talker;
  std::string signal;
  int totalMsgs = 0;
  int lastMsgNum = 0;
  int declared = 0;
  int entries = 0;
  bool pending = false;
  bool complete = false;
  bool uncertain = false;
  bool pendingUncertain = false;
  uint32_t started = 0;
  uint32_t updated = 0;
  std::vector<SatData> staging;
  std::vector<SatData> published;
};

struct GSASequenceState {
  std::string system;
  uint32_t epoch = 0;
  uint32_t updated = 0;
  std::vector<SatelliteKey> current;
};

/*    Keep satellites unique across bands and separate live observations from
*     session history. Ambiguous mixed-system IDs are never guessed.
*/
class GPSSatelliteMetrics {
public:
  std::vector<SatData> satellites;
  bool seenUncertain = false;
  bool usedUncertain = false;
  std::vector<GSVSequenceState> views;
  std::vector<GSASequenceState> solutions;
  uint32_t epoch = 1;
  uint32_t lastGSA = 0;
  uint32_t epochStarted = 0;
  bool receivedGSA = false;
  std::string utc;

  std::string talkerSystem(const std::string &talker) const {
    if (talker == "GP") return "GPS";
    if (talker == "GL") return "GLONASS";
    if (talker == "GA") return "Galileo";
    if (talker == "BD" || talker == "GB") return "BeiDou";
    if (talker == "GQ" || talker == "QZ") return "QZSS";
    if (talker == "GI") return "NavIC";
    return "";
  }

  std::string systemID(int id) const {
    const char* names[] = {"", "GPS", "GLONASS", "Galileo", "BeiDou", "QZSS", "NavIC"};
    return id >= 1 && id <= 6 ? names[id] : "";
  }

  SatelliteKey identify(std::string system, int id) const {
    if (system.empty()) {
      // Local IDs overlap across GPS, Galileo and BeiDou. Only global ranges
      // can be resolved without a talker or a GSA system identifier.
      if (id >= 65 && id <= 96) system = "GLONASS";
      else if (id >= 193 && id <= 199) system = "QZSS";
      else if (id >= 201 && id <= 263) system = "BeiDou";
      else if (id >= 301 && id <= 336) system = "Galileo";
      else if (id >= 120 && id <= 158) system = "SBAS";
      else return {"", 0};
    }
    if (system == "GPS" && id >= 33 && id <= 64) return {"SBAS", id + 87};
    if (system == "GPS" && id >= 120 && id <= 158) return {"SBAS", id};
    if (system == "GPS" && id >= 193 && id <= 199) return {"QZSS", id - 192};
    if (system == "GLONASS" && id >= 65 && id <= 96) id -= 64;
    if (system == "Galileo" && id >= 301 && id <= 336) id -= 300;
    if (system == "BeiDou" && id >= 201 && id <= 263) id -= 200;
    if (system == "BeiDou" && id >= 101 && id <= 163) id -= 100;
    if (system == "QZSS" && id >= 193 && id <= 199) id -= 192;
    int maximum = system == "GPS" ? 32 : system == "GLONASS" ? 32
        : system == "Galileo" ? 36 : system == "BeiDou" ? 63 : 7;
    if (system == "SBAS") return id >= 120 && id <= 158 ? SatelliteKey{system, id} : SatelliteKey{"", 0};
    return id >= 1 && id <= maximum ? SatelliteKey{system, id} : SatelliteKey{"", 0};
  }

  SatData* remember(const SatelliteKey &key) {
    for (auto &sat : satellites) if (sat.system == key.system && sat.id == key.id) return &sat;
    if (key.system.empty() || satellites.size() >= 256) return nullptr;
    SatData sat;
    sat.system = key.system;
    sat.id = key.id;
    satellites.push_back(sat);
    return &satellites.back();
  }

  void processGSV(const NMEAFields &fields, const std::string &talker, uint32_t now) {
    int total = 0, number = 0, declared = 0;
    if (!fields.integer(1, total, 1, 32) || !fields.integer(2, number, 1, total)
        || !fields.integer(3, declared, 0, 128)) return;
    int extra = fields.count - 4;
    if (extra < 0 || (extra % 4 != 0 && extra % 4 != 1)) return;
    int groups = extra / 4;
    if (groups > 4) return;
    std::string signal = extra % 4 ? fields.field(fields.count - 1) : "";
    if (!signal.empty() && (signal.length() != 1 || !strchr("0123456789ABCDEFabcdef", signal[0]))) return;
    if (!signal.empty() && signal[0] >= 'a' && signal[0] <= 'f') signal[0] -= 'a' - 'A';
    GSVSequenceState* state = nullptr;
    for (auto &view : views) if (view.talker == talker && view.signal == signal) state = &view;
    if (!state) {
      if (views.size() >= 64) return;
      views.emplace_back();
      state = &views.back();
      state->talker = talker;
      state->signal = signal;
    }
    if (number == 1) {
      state->staging.clear();
      state->totalMsgs = total;
      state->declared = declared;
      state->lastMsgNum = state->entries = 0;
      state->started = now;
      state->pending = true;
      state->pendingUncertain = false;
    }
    if (!state->pending || total != state->totalMsgs || declared != state->declared
        || number != state->lastMsgNum + 1 || now - state->started >= GPS_SAT_TIMEOUT) {
      state->pending = false;
      return;
    }
    for (int group = 0; group < groups; group++) {
      int base = 4 + group * 4, id = 0, elevation = 0, azimuth = 0, snr = -1;
      if (!fields.integer(base, id, 1, 999)
          || (*fields.field(base + 1) && !fields.integer(base + 1, elevation, 0, 90))
          || (*fields.field(base + 2) && !fields.integer(base + 2, azimuth, 0, 359))
          || (*fields.field(base + 3) && !fields.integer(base + 3, snr, 0, 99))) {
        state->pending = false;
        return;
      }
      state->entries++;
      SatelliteKey key = identify(talkerSystem(talker), id);
      if (key.system.empty()) { state->pendingUncertain = true; continue; }
      for (const auto &sat : state->staging) {
        if (sat.system == key.system && sat.id == key.id) { state->pending = false; return; }
      }
      SatData sat;
      sat.system = key.system;
      sat.id = key.id;
      sat.elevation = elevation;
      sat.azimuth = azimuth;
      sat.hasPosition = *fields.field(base + 1) && *fields.field(base + 2);
      sat.snr = snr;
      state->staging.push_back(sat);
    }
    state->lastMsgNum = number;
    if (number != total) return;
    state->pending = false;
    if (state->entries != declared) return;
    state->published = state->staging;
    state->updated = now;
    state->complete = true;
    state->uncertain = state->pendingUncertain;
    seenUncertain |= state->uncertain;
    for (const auto &sat : state->published) remember({sat.system, sat.id});
  }

  void processGSA(const NMEAFields &fields, const std::string &talker, uint32_t now) {
    if (fields.count < 18) return;
    int mode = 0;
    if (!fields.integer(2, mode, 1, 3)) return;
    std::string system = talkerSystem(talker);
    if (talker == "GN" && *fields.field(18)) {
      int id = 0;
      if (!fields.integer(18, id, 1, 6)) { seenUncertain = usedUncertain = true; return; }
      system = systemID(id);
    }
    if (!receivedGSA || now - lastGSA >= 250 || now - epochStarted >= 250) {
      epoch++;
      epochStarted = now;
    }
    receivedGSA = true;
    lastGSA = now;
    GSASequenceState* state = nullptr;
    std::string group = system.empty() ? "GN" : system;
    for (auto &solution : solutions) if (solution.system == group) state = &solution;
    if (!state) {
      solutions.emplace_back();
      state = &solutions.back();
      state->system = group;
    }
    if (state->epoch != epoch || mode == 1) state->current.clear();
    state->epoch = epoch;
    state->updated = now;
    if (mode == 1) return;
    for (int i = 3; i <= 14; i++) {
      if (!*fields.field(i)) continue;
      int id = 0;
      if (!fields.integer(i, id, 1, 999)) { seenUncertain = usedUncertain = true; continue; }
      SatelliteKey key = identify(system, id);
      if (key.system.empty()) { seenUncertain = usedUncertain = true; continue; }
      if (std::find(state->current.begin(), state->current.end(), key) == state->current.end())
        state->current.push_back(key);
      SatData* sat = remember(key);
      if (sat) sat->usedEver = true;
    }
  }

  void process(const NMEAFields &fields, uint32_t now) {
    std::string type = fields.field(0);
    if (!knownGNSSTalker(type.c_str())) return;
    std::string talker = type.substr(1, 2), sentence = type.substr(3);
    if (sentence == "GSV") processGSV(fields, talker, now);
    else if (sentence == "GSA") processGSA(fields, talker, now);
    else if ((sentence == "GGA" || sentence == "RMC") && *fields.field(1)) {
      if (utc != fields.field(1)) { utc = fields.field(1); epoch++; epochStarted = now; }
    }
  }

  bool freshViews(uint32_t now, bool running) const {
    for (const auto &view : views)
      if (running && view.complete && now - view.updated < GPS_SAT_TIMEOUT) return true;
    return false;
  }

  bool uncertainViews(uint32_t now, bool running) const {
    for (const auto &view : views)
      if (running && view.complete && view.uncertain && now - view.updated < GPS_SAT_TIMEOUT) return true;
    return false;
  }

  void refresh(uint32_t now, bool running, bool fix) {
    for (auto &sat : satellites) {
      sat.visible = sat.used = false;
      sat.snr = -1;
    }
    if (!running) return;
    for (const auto &view : views) {
      if (!view.complete || now - view.updated >= GPS_SAT_TIMEOUT) continue;
      for (const auto &observation : view.published) {
        SatData* sat = remember({observation.system, observation.id});
        if (!sat) continue;
        if (!sat->visible) sat->hasPosition = false;
        sat->visible = true;
        if (observation.hasPosition && (!sat->hasPosition
            || now - view.updated <= now - sat->positionUpdated)) {
          sat->elevation = observation.elevation;
          sat->azimuth = observation.azimuth;
          sat->hasPosition = true;
          sat->positionUpdated = view.updated;
        }
        sat->snr = std::max(sat->snr, observation.snr);
      }
    }
    if (!fix) return;
    for (const auto &solution : solutions) {
      if (now - solution.updated >= GPS_METRIC_TIMEOUT) continue;
      for (const auto &key : solution.current) {
        SatData* sat = remember(key);
        if (sat) sat->used = true;
      }
    }
  }
};
