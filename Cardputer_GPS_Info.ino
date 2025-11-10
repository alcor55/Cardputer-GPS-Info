#include <M5Cardputer.h>
#include <vector>
#include <algorithm>
#include <TinyGPSPlus.h>

HardwareSerial GPS_Serial(1); // Use UART1 for GPS.
TinyGPSPlus gps;

struct SatData {
  String system;   // "GPS", "GLONASS", "Galileo", "BeiDou".
  int id;
  int elevation;   // 0-90°.
  int azimuth;     // 0-359°.
  int snr;         // 0-99.
  bool used;       // used in the fix.
  bool visible;    // visible in the last cycle.
};
std::vector<SatData> satellites;

struct GSVSequenceState {
    String system;
    int totalMsgs = 0;
    int lastMsgNum = 0;
    std::vector<int> currentVisible;
};
GSVSequenceState gsvStates[5];
int gsvCount = 0;

bool gpsSerial = false;
bool debugSerial = false;
bool nmeaSerial = false;
bool satListSerial = false;
bool hidePlotId = true;
bool hidePlotSystem = true;
bool openMenu = false;
bool helpMenu = false;
bool infoMenu = false;
bool configsMenu = false;
int configsMenuSel = 0;
String tempPins[2] = {"", ""};  // 0 Tx, 1 Rx.
int gpsTxPin = 15; // GPS TX pin (Cardputer GPIO2)
int gpsRxPin = 13; // GPS RX pin (Cardputer GPIO1)

// Cardputer (1, 1.1, ADV) Display.
const int screenW = 240;
const int screenH = 135;

/*    Open or close the GPS UART serial console.
*/
void initGPSSerial(bool should_I) {
  if (should_I == true)
    GPS_Serial.begin(115200, SERIAL_8N1, gpsTxPin, gpsRxPin); // Start GPS UART.
  else
    GPS_Serial.end();
}

/*    Read the GPS seria and compose the NMEA sentence.
*/
void serialGPSRead() {
  static String nmeaLine = "";
  while (GPS_Serial.available()) {
    char c = GPS_Serial.read();
    gps.encode(c);
    if (c == '\n') {
      nmeaDispatcher(nmeaLine);
      nmeaLine = "";
    } else if (c != '\r')
      nmeaLine += c;
  }
}

/*    Read the NMEA sentence and dispatch to parsers.
*/
void nmeaDispatcher(const String &nmeaLine) {
  if (nmeaSerial)
    Serial.println(nmeaLine);
  // Trim line endings.
  String line = nmeaLine;
  line.trim();
  // Define NMEA handlers.
  struct NMEAHandler { 
    const char* prefix; 
    void (*parser)(const String&); 
  };
  static NMEAHandler handlers[] = {
    {"$GPGSV", parseGSV},
    {"$GLGSV", parseGSV},
    {"$GAGSV", parseGSV},
    {"$BDGSV", parseGSV},
    {"$GNGSV", parseGSV},
    {"$GPGSA", parseGSA},
    {"$GLGSA", parseGSA},
    {"$GAGSA", parseGSA},
    {"$BDGSA", parseGSA},
    {"$GNGSA", parseGSA}
    //{"$GPGGA", parseGGA}
  };
  // Dispatch to the correct parser.
  for (auto &h : handlers) {
    if (line.startsWith(h.prefix)) {
      h.parser(line);
      break;
    }
  }
}

/*    Parse NMEA 0183 GSA sentence. (GNSS DOP and Active Satellites).
*     Mode (2D/3D), IDs of used satellites, PDOP/HDOP/VDOP.
*/
void parseGSA(const String &line) {
  int fieldNum = 0, lastIndex = 0;
  for (int i = 0; i <= line.length(); i++) {
    if (i == line.length() || line[i] == ',' || line[i] == '*') {
      String val = line.substring(lastIndex, i);
      lastIndex = i + 1;
      fieldNum++;
      if (fieldNum >= 4 && fieldNum <= 15 && val.length() > 0) {
        int id = val.toInt();
        for (auto &sat : satellites) {
          if (sat.id == id) sat.used = true;
        }
      }
    }
  }
}

/*    Parse NMEA 0183 GSV sentence. (GNSS Satellites in View).
*     Info on all visible satellites (ID, elevation, azimuth, SNR).
*/
void parseGSV(const String &line) {
  String system;
  if (line.startsWith("$GPGSV")) system = "GPS";
  else if (line.startsWith("$GLGSV")) system = "GLONASS";
  else if (line.startsWith("$GAGSV")) system = "Galileo";
  else if (line.startsWith("$BDGSV")) system = "BeiDou";
  else if (line.startsWith("$GNGSV")) system = "Mixed";
  else return;
  GSVSequenceState* state = getGSVState(system);
  if (!state) return;
  std::vector<String> fields;
  int lastIndex = 0;
  for (int i = 0; i <= line.length(); i++) {
    if (i == line.length() || line[i] == ',' || line[i] == '*') {
      fields.push_back(line.substring(lastIndex, i));
      lastIndex = i + 1;
    }
  }
  if (fields.size() < 4) return;
  int totalMsgs = fields[1].toInt(); 
  int msgNum    = fields[2].toInt();
  if (msgNum == 1 || state->totalMsgs != totalMsgs) {
    state->currentVisible.clear();
    state->totalMsgs = totalMsgs;
  }
  // Pars sats.
  for (size_t i = 4; i + 3 < fields.size(); i += 4) {
    SatData sat;
    sat.system = system;
    sat.id = fields[i].toInt();
    // Inverted BeiDou.
    if (system == "BeiDou") {
      sat.azimuth   = fields[i + 1].toInt();
      sat.elevation = fields[i + 2].toInt();
    } else {
      sat.elevation = fields[i + 1].toInt();
      sat.azimuth   = fields[i + 2].toInt();
    }
    sat.snr = fields[i + 3].toInt();
    sat.used = false;
    storeSatellite(sat);
    state->currentVisible.push_back(sat.id);
  }
  state->lastMsgNum = msgNum;
  if (msgNum == totalMsgs) {
    for (auto &s : satellites) {
      if (s.system == system) {
        s.visible = (std::find(state->currentVisible.begin(),state->currentVisible.end(),s.id) != state->currentVisible.end());
      }
    }
  }
}

/*    Stores GNSS satellite sequence states.
*/
GSVSequenceState* getGSVState(const String& system) {
  for (int i = 0; i < gsvCount; i++) {
    if (gsvStates[i].system == system)
      return &gsvStates[i];
  }
  if (gsvCount < 5) {
    gsvStates[gsvCount].system = system;
    return &gsvStates[gsvCount++];
  }
  return nullptr;
}

/*    Store satellite in a list.
*/
void storeSatellite(const SatData &sat) {
  for (auto &s : satellites) {
    if (s.system == sat.system && s.id == sat.id) {
      s.elevation = sat.elevation;
      s.azimuth   = sat.azimuth;
      s.snr       = sat.snr;
      return;
    }
  }
  satellites.push_back(sat);
}

/*    Open or close the USB serial console.
*/
void initDebugSerial(bool should_I) {
  if (should_I && !debugSerial) {
    debugSerial = true;
    Serial.begin( 115200 );
    Serial.setTimeout( 2000 );
    while( !Serial ){}  // Wait for serial to initialize.
    delay( 100 );
    Serial.println( "\n\n\n Initialited Cardputer GPS INFO serial console!" );
  } else if (!should_I && debugSerial) {
    debugSerial = false;
    Serial.end();
  }
}

/*    Print in the USB Serial Console a List with all seen satellites.
*/
void serialConsoleSatsList() {
  if (!satListSerial) return;
  static uint32_t lastSerialSatsPrint = 0;
  if (millis() - lastSerialSatsPrint > 5000) {
    lastSerialSatsPrint = millis();
    if (satellites.empty()) return;
    // Sort by system and ID.
    std::sort(satellites.begin(), satellites.end(), [](const SatData &a, const SatData &b) {
        if (a.system != b.system) return a.system < b.system;
        return a.id < b.id;
    });
    Serial.println("------------------------------------");
    Serial.println("System    ID  Ele  Azi  SNR  Usd Vis");
    Serial.println("------------------------------------");
    for (auto &sat : satellites) {
      Serial.printf("%-8s %3d %4d %4d %4d   %c   %c\n", sat.system.c_str(), sat.id, sat.elevation, sat.azimuth, sat.snr, sat.used ? 'Y' : 'N', sat.visible ? 'V' : 'X');
      //                                                                                                                                         V = visibile, X = lost
    }
    Serial.println("------------------------------------");
  }
}

/*    Manage display elements drawing.
*/
void updateScreen(bool force = false) {
  static uint32_t lastDisplay = 0;
  if (force || millis() - lastDisplay > 1000) // 1sec update or force it.
  {
    lastDisplay = millis();
    if (openMenu) return;
    if (force) {
      M5Cardputer.Display.fillScreen(TFT_BLACK);
      drawHeader();
      drawStatus();
    }
    // Satellites datas.
    drawSatelliteDataTab();
    // Satellites plot.
    drawSkyPlot();
  }
}

/*    Draw the satellites sky plot graph.
*/
void drawSkyPlot() {
  int x = 143;
  int y = 27;
  int w = 96;
  int h = w-1;
  int half_side = h * 0.5;
  int cx = x + half_side;
  int cy = y + half_side;
  int r = half_side;
  M5Cardputer.Display.drawRect(x-1, y-1, w+2, h+2, TFT_DARKGREY);
  M5Cardputer.Display.fillRect(x, y, w, h, TFT_BLACK);
  // Ref circles.
  M5Cardputer.Display.drawCircle(cx, cy, r, TFT_WHITE);
  M5Cardputer.Display.drawCircle(cx, cy, r * 0.66, TFT_DARKGREY);
  M5Cardputer.Display.drawCircle(cx, cy, r * 0.33, TFT_DARKGREY);
  M5Cardputer.Display.drawLine(cx - r, cy, cx + r, cy, TFT_DARKGREY);
  M5Cardputer.Display.drawLine(cx, cy - r, cx, cy + r, TFT_DARKGREY);
  // Cardinals label.
  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.setTextColor(TFT_LIGHTGREY);
  M5Cardputer.Display.setCursor(cx - 3, cy - r + 4);  M5Cardputer.Display.print("N");
  M5Cardputer.Display.setCursor(cx - 3, cy + r - 10); M5Cardputer.Display.print("S");
  M5Cardputer.Display.setCursor(cx + r - 10, cy - 3); M5Cardputer.Display.print("E");
  M5Cardputer.Display.setCursor(cx - r + 4, cy - 3);  M5Cardputer.Display.print("W");
  // Satellites.
  for (auto &sat : satellites) {
    float elev = constrain(sat.elevation, 0.0, 90.0);
    float az   = fmod(sat.azimuth + 360.0, 360.0);
    float rad = (90.0 - elev) / 90.0 * r;
    float radAz = radians(az);
    float sx = cx + rad * sin(radAz);
    float sy = cy - rad * cos(radAz);
    uint16_t color = TFT_RED;
    if (sat.used)
      color = TFT_GREEN;
    else if
      (sat.visible) color = TFT_YELLOW;
    M5Cardputer.Display.fillCircle(sx, sy, 2, color); // Satellite dot.
    if (!hidePlotId)
    {
      M5Cardputer.Display.setTextColor(color);  // Satellite label.
      M5Cardputer.Display.setTextSize(0);
      M5Cardputer.Display.setCursor(sx + 5, sy - 3);
      M5Cardputer.Display.printf("%d", sat.id);
    }
    if (!hidePlotSystem)
    {
      M5Cardputer.Display.setTextColor(color);  // Satellite label.
      M5Cardputer.Display.setTextSize(0);
      M5Cardputer.Display.setCursor(sx + 5, sy - 3);
      const char* sys;
      if (sat.system == "GPS") sys = "Gp";
      else if (sat.system == "GLONASS") sys = "Gl";
      else if (sat.system == "Galileo") sys = "Ga";
      else if (sat.system == "BeiDou") sys = "Bd";
      else sys = "?";
      M5Cardputer.Display.printf("%s", sys);
    }
  }
}

/*    Draw the satellites main data table.
*/
void drawSatelliteDataTab(){
  int x = 1;
  int y = 26;
  // Column 1.
  // Labels.
  const char* c1labels[] = { "Lat", "Lng", "Alt", "Spd", "Crs", "Date", "Time", "HDOP" };
  // Values.
  char c1values[8][20];
  if (gps.location.isValid()) {sprintf(c1values[0], "%.6f", gps.location.lat());} else {sprintf(c1values[0], "NoFix");}
  if (gps.location.isValid()) {sprintf(c1values[1], "%.6f", gps.location.lng());} else {sprintf(c1values[1], "NoFix");}
  if (gps.altitude.isValid()) {sprintf(c1values[2], "%.2f", gps.altitude.meters());} else {sprintf(c1values[2], "NoData");}
  sprintf(c1values[3], "%.1f", gps.speed.kmph());
  sprintf(c1values[4], "%.1f", gps.course.deg());
  if (gps.date.isValid()) {sprintf(c1values[5], "%02d/%02d/%02d", gps.date.day(), gps.date.month(), gps.date.year() % 100);} else {sprintf(c1values[5], "NoData");}
  if (gps.time.isValid()) {sprintf(c1values[6], "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());} else {sprintf(c1values[6], "NoData");}
  sprintf(c1values[7], "%.1f", gps.hdop.hdop());
  // Draw.
  for (int i = 0; i < 8; i++) {
    M5Cardputer.Display.fillRect(x, y, 90, 13, TFT_BLACK);
    M5Cardputer.Display.drawRect(x, y, 90, 13, TFT_DARKGREY);
    M5Cardputer.Display.setTextColor(TFT_WHITE);
    M5Cardputer.Display.setCursor(x + 4, y + 3);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.printf("%s: %s", c1labels[i], c1values[i]);
    y += 12;
  }
  // Column 2.
  y = 26;
  x += 89;
  // Labels.
  const char* c2labels[] = { "Seen", "Visb", "Used", "InFx", "GPS", "Gln", "Gal", "BDo" };
  // Values.
  char c2values[8][12];
  int totalAll = satellites.size(); // Ever seen.
  int totalUsed = 0;                // Ever used in fix.
  int totalVisible = 0;             // Now visible.
  int gpsVisible = 0;
  int glonassVisible = 0;
  int galileoVisible = 0;
  int beidouVisible = 0;
  for (auto &sat : satellites) {
    if (sat.used) totalUsed++;
    if (sat.visible) {
      totalVisible++;
      if (sat.system == "GPS") gpsVisible++;
      else if (sat.system == "GLONASS") glonassVisible++;
      else if (sat.system == "Galileo") galileoVisible++;
      else if (sat.system == "BeiDou") beidouVisible++;
    }
  }
  sprintf(c2values[0], "%d", totalAll);
  sprintf(c2values[1], "%d", totalVisible);
  sprintf(c2values[2], "%d", totalUsed);
  if (gps.satellites.isValid()) {sprintf(c2values[3], "%d", gps.satellites.value());} else {sprintf(c2values[3], "0");} // Now in the fix.
  sprintf(c2values[4], "%d", gpsVisible);
  sprintf(c2values[5], "%d", glonassVisible);
  sprintf(c2values[6], "%d", galileoVisible);
  sprintf(c2values[7], "%d", beidouVisible);
  // Draw.
  for (int i = 0; i < 8; i++) {
    M5Cardputer.Display.fillRect(x, y, 53, 13, TFT_BLACK);
    M5Cardputer.Display.drawRect(x, y, 53, 13, TFT_DARKGREY);
    M5Cardputer.Display.setTextColor(TFT_WHITE);
    M5Cardputer.Display.setCursor(x + 4, y + 3);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.printf("%s: %s", c2labels[i], c2values[i]);
    y += 12;
  }
}

/*    Draw main features key map.
*/
void drawHeader(){
  int x = 1;
  int y = 1;
  int w = screenW;
  int h = 13;
  M5Cardputer.Display.setTextSize(1);
  // Title.
  M5Cardputer.Display.fillRect(x, y, w, h, TFT_GREEN); // bordo nel colore del testo
  M5Cardputer.Display.setTextColor(TFT_BLACK);      // testo e sfondo
  M5Cardputer.Display.setCursor(x + 4, y + 3);
  M5Cardputer.Display.printf("%-1s", "      -= Cardputer GPS Info =-");
  // Key map.
  M5Cardputer.Display.drawRect(x, y+h-1, w, h, TFT_GREEN);
  M5Cardputer.Display.setTextColor(TFT_GREEN);
  M5Cardputer.Display.setCursor(x + 4, y+h + 3);
  M5Cardputer.Display.printf("%-1s", "[s]Start/Stop. [c]Config. [h]Help.");
  Serial.printf("drawHeader");
}

/*    Draw app features status.
*/
void drawStatus(){
  int x = 1;
  int y = 122;
  int w = screenW;
  int h = 13;
  char statusChar[64];
  statusChar[0] = '\0';
  snprintf(statusChar + strlen(statusChar),sizeof(statusChar) - strlen(statusChar),"GPS:%s |", gpsSerial ? "Start" : "Stop");
  snprintf(statusChar + strlen(statusChar),sizeof(statusChar) - strlen(statusChar)," Cnsol:%s |", debugSerial ? "Start" : "Stop");
  snprintf(statusChar + strlen(statusChar),sizeof(statusChar) - strlen(statusChar)," Tx:%d | Rx:%d", gpsTxPin, gpsRxPin);
  M5Cardputer.Display.fillRect(x, y, w, h, TFT_DARKGREY);
  M5Cardputer.Display.setTextColor(TFT_WHITE);
  M5Cardputer.Display.setCursor(x + 4, y + 3);
  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.printf("%-1s", statusChar);
  Serial.printf("drawStatus");
}

/*    Draw configuration popup.
*/
void drawConfig(bool should_I) {
  if (should_I == true) {
    if (gpsSerial) { // If active, stop it.
      gpsSerial = false;
      initGPSSerial(false);
    }
    M5Cardputer.Display.fillRect(18, 18, 204, 99, TFT_BLACK);
    M5Cardputer.Display.drawRect(20, 20, 200, 95, TFT_GREEN);
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setCursor(25, 25);
    M5Cardputer.Display.println("Configurations:");
    M5Cardputer.Display.setCursor(25, 35);
    M5Cardputer.Display.println("Nav: [Left/Right]. Val: [0-9].");
    M5Cardputer.Display.setCursor(25, 45);
    M5Cardputer.Display.println("Ext:[c]. Sav:[ok].");
    M5Cardputer.Display.setCursor(30, 70);
    M5Cardputer.Display.printf("TX pin (std:15 act:%d): %s %s", gpsTxPin, tempPins[0].c_str(), configsMenuSel == 0 ? "<" : " ");
    M5Cardputer.Display.setCursor(30, 80);
    M5Cardputer.Display.printf("RX pin (std:13 act:%d): %s %s", gpsRxPin, tempPins[1].c_str(), configsMenuSel == 1 ? "<" : " ");
  }
  else
    updateScreen(true); // Forced update.
}

/*    Draw info popup.
*/
void drawInfo(bool should_I) {
  if (should_I == true) {
    openMenu = true;
    const char* helpText[] = {
      "Cardputer GPS Info",
      "V 1.0 by alcor55",
      "",
      "Github:",
      "https://github.com/alcor55",
      "/Cardputer-GPS-Info"
    };
    int count = sizeof(helpText) / sizeof(helpText[0]);
    M5Cardputer.Display.fillRect(18, 18, 204, 99, TFT_BLACK);
    M5Cardputer.Display.drawRect(20, 20, 200, 95, TFT_GREEN);
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setCursor(25, 24);
    int y = 24;
    for (int i = 0; i < count; i++) {
      M5Cardputer.Display.setCursor(25, y);
      M5Cardputer.Display.println(helpText[i]);
      y += 10;
    }
  } else {
    openMenu = false;
    updateScreen(true); // Forced update.
  }
}

/*    Draw help popup.
*/
void drawHelp(bool should_I) {
  if (should_I == true) {
    openMenu = true;
    const char* helpText[] = {
      "[s] Start/Stop the GPS (serial).",
      "[c] Configuration menu.",
      "[h] Help menu (this).",
      "[i] Info menu.",
      "[l] Print in the usb serial",
      " console the satellites list.",
      "[n] Print in the usb serial",
      " console the NMEA sentences.",
      "[p] Show/hide ID on skyplot.",
      "[o] Show/hide System on skyplot."
    };
    int count = sizeof(helpText) / sizeof(helpText[0]);
    M5Cardputer.Display.fillRect(10, 10, screenW-20, screenH-20, TFT_BLACK);
    M5Cardputer.Display.drawRect(12, 12, screenW-24, screenH-24, TFT_GREEN);
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5Cardputer.Display.setTextSize(1);
    int y = 19;
    for (int i = 0; i < count; i++) {
      String line = helpText[i];
      int endKey = line.indexOf(']') + 1;
      if (endKey > 0) {
        String key = line.substring(0, endKey);
        String desc = line.substring(endKey);
        M5Cardputer.Display.setTextColor(TFT_GREEN, TFT_BLACK);
        M5Cardputer.Display.setCursor(20, y);
        M5Cardputer.Display.print(key);
        M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5Cardputer.Display.print(desc);
      } else {
        M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5Cardputer.Display.setCursor(25, y);
        M5Cardputer.Display.print(line);
      }
      y += 10;
    }
  } else {
    openMenu = false;
    updateScreen(true); // Forced update.
  }
}

/*    Handle keyboard data inputs.
*/
void handleKeys() {
  if (configsMenu) {
    if (M5Cardputer.Keyboard.isChange()) {
      if (M5Cardputer.Keyboard.isPressed()) {
        Keyboard_Class::KeysState status = M5Cardputer.Keyboard.keysState();
        for (auto c : status.word) {
          // Arrow selection vertical up.
          if (c == ';' || c == '.')
            configsMenuSel = (configsMenuSel == 0) ? 1 : 0;
          // Numbers 0-9.
          if (c >= 48 && c <= 57) {
            // Max 2 digit.
            if (tempPins[configsMenuSel].length() < 2)
              tempPins[configsMenuSel] += c;
          }
        }
        // Delete.
        if (status.del && tempPins[configsMenuSel].length() > 0)
          tempPins[configsMenuSel].remove(tempPins[configsMenuSel].length() - 1);
        // Store.
        if (status.enter) {
          if (tempPins[0].length() > 0)
            gpsTxPin = tempPins[0].toInt();
          if (tempPins[1].length() > 0)
            gpsRxPin = tempPins[1].toInt();
          tempPins[0] = tempPins[1] = "";
          configsMenu = false;
          updateScreen(true);
          return;
        }
        drawConfig(true);
      }
    }
  }
}

/*    Handle app functions.
*/
void handleControls() {
  // Handle S key for Start/Stop.
  if (M5Cardputer.Keyboard.isKeyPressed('s')) {
    gpsSerial = !gpsSerial; // Invert status.
    initGPSSerial(gpsSerial);
    delay(300); // Debounce.
    drawStatus();
  }
  // Handle C key for Config.
  if (M5Cardputer.Keyboard.isKeyPressed('c')) {
    configsMenu = !configsMenu; // Invert status.
    drawConfig(configsMenu);
    delay(300); // Debounce.
  }
  // Handle H key for Help.
  if (M5Cardputer.Keyboard.isKeyPressed('h')) {
    helpMenu = !helpMenu; // Invert status.
    drawHelp(helpMenu);
    delay(300); // Debounce.
  }
  // Handle I key for Info.
  if (M5Cardputer.Keyboard.isKeyPressed('i')) {
    infoMenu = !infoMenu; // Invert status.
    drawInfo(infoMenu);
    delay(300); // Debounce.
  }
  // Handle P key to hide id from plot.
  if (M5Cardputer.Keyboard.isKeyPressed('p')) {
    hidePlotId = !hidePlotId; // Invert status.
    delay(300); // Debounce.
  }
  // Handle O key to hide system from plot.
  if (M5Cardputer.Keyboard.isKeyPressed('o')) {
    hidePlotSystem = !hidePlotSystem; // Invert status.
    delay(300); // Debounce.
  }
  // Handle L key for Serial satellite list.
  if (M5Cardputer.Keyboard.isKeyPressed('l')) {
    nmeaSerial = false;
    satListSerial = !satListSerial; // Invert status.
    initDebugSerial(satListSerial);
    delay(300); // Debounce.
    drawStatus();
  }
  // Handle N key for serial NMEA sentences dump.
  if (M5Cardputer.Keyboard.isKeyPressed('n')) {
    satListSerial = false;
    nmeaSerial = !nmeaSerial; // Invert status.
    initDebugSerial(nmeaSerial);
    delay(300); // Debounce.
    drawStatus();
  }
}

/*    Standard setup function.
*/
void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg); // Init Cardputer.
  M5Cardputer.Display.setBrightness(32); // Set brightness 0-255.
  updateScreen(true); // Forced first update.
}

/*    Standard loop.
*/
void loop() {
  M5Cardputer.update(); // Update Cardputer state (keyboard, buttons, sensors etc...).
  if (gpsSerial){
    serialGPSRead();
    updateScreen();
    serialConsoleSatsList();
  }
  handleControls();
  handleKeys();
  delay(10); // Small delay to avoid busy loop.
}

/*
  That's all folks!
*/