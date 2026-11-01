#include <M5Cardputer.h>
#include <vector>
#include <algorithm>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <FS.h>
#include <SPI.h>
#include <memory>
#include <utility/Keyboard/KeyboardReader/TCA8418.h>
#include "GpsConfig.h"
#include "GpsMetrics.h"
#include "GpsViews.h"
#include "GpsPopup.h"

#define APP_VERSION "1.2.0"

#define CONFIG_FILE "/cpGpsInfo.conf"
bool sdAvailable = false;
bool sdErr = false;

HardwareSerial GPS_Serial(1); // Use UART1 for GPS.
TinyGPSPlus gps;
M5Canvas popupCanvas(&M5Cardputer.Display);
bool popupCanvasReady = false;
bool popupCanvasAttempted = false;

GPSSatelliteMetrics satelliteMetrics;
GPSNavigationMetrics navigationMetrics;
std::vector<SatData> &satellites = satelliteMetrics.satellites;

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
bool skyMenu = false;
bool barsMenu = false;
int configsMenuSel = 0;
String configsTmp[4] = {"", "", "", ""};  // 0 Rx, 1 Tx, 2 GPS baud, 3 USB baud.
GPSConfig gpsConfig = gpsPreset(GPS_GROVE);
GPSConfig configsDraft = gpsConfig;
String configMessage = "";
bool boardAdv = false;
bool configLoadErr = false;
String pressedKeys = "";
GPSKeyEdges keyEdges;
bool enterPressed = false;
bool deletePressed = false;

int gpsRxPin = 1; // Cardputer Rx pin <- GPS Tx pin.
int gpsTxPin = 2; // Cardputer Tx pin <- GPS Rx pin.
int gpsBaud = 9600;

GPSState gpsSerialState = GPS_OFF;
unsigned long gpsStartedMillis = 0;
unsigned long lastGpsByteMillis = 0;
unsigned long lastValidGpsMillis = 0;
bool gpsReceived = false;
bool gpsValid = false;
String nmeaLine = "";

// Cardputer (1, 1.1, ADV) Display.
const int screenW = 240;
const int screenH = 135;

// Explicit declarations for render targets that Arduino's preprocessor skips.
void drawMainScreen(lgfx::LGFXBase &display);
void drawSatelliteDataTab(lgfx::LGFXBase &display);

/*    Keep Ares radios deselected and the IR transmitter off.
*     Call only after initializing the Cardputer display and keyboard.
*/
void prepareGPSHardware() {
  if (!boardAdv) return;
  if (gpsConfig.hardware == GPS_CAP) {
    digitalWrite(5, HIGH); // SX1262 NSS, shared SPI bus with microSD.
    pinMode(5, OUTPUT);
    return;
  }
  if (gpsConfig.hardware != GPS_ARES) return;
  digitalWrite(6, HIGH); // CC1101 CSN, shared SPI bus with microSD.
  pinMode(6, OUTPUT);
  digitalWrite(13, HIGH); // GPS RX / nRF CSN; no UART writes while using SPI.
  pinMode(13, OUTPUT);
  pinMode(4, INPUT); // Shared CC1101 GDO0 / nRF CE, external pull-down.
  digitalWrite(3, LOW); // Ares IR TX stays off.
  pinMode(3, OUTPUT);
  pinMode(5, INPUT_PULLUP); // Ares IR receiver.
}

/*    Open or close the GPS UART serial console.
*/
void initGPSSerial(bool should_I) {
  GPS_Serial.end();
  gpsSerial = false;
  gpsSerialState = GPS_OFF;
  nmeaLine = "";
  satelliteMetrics.refresh(millis(), false, false);
  prepareGPSHardware();
  if (should_I == true) {
    const char* error = gpsConfigError(gpsConfig, boardAdv);
    if (error) {
      configMessage = error;
      return;
    }
    gps = TinyGPSPlus();
    satelliteMetrics = GPSSatelliteMetrics();
    navigationMetrics = GPSNavigationMetrics();
    gpsReceived = gpsValid = false;
    gpsStartedMillis = lastGpsByteMillis = lastValidGpsMillis = millis();
    GPS_Serial.setRxBufferSize(2048);
    GPS_Serial.begin(gpsBaud, SERIAL_8N1, gpsRxPin, gpsTxPin);
    gpsSerial = true;
    gpsSerialState = GPS_WAIT;
  }
}

/*    Report live metrics only while the checked NMEA stream is current.
*/
bool currentGPSStream() {
  return gpsSerial && gpsValid && millis() - lastValidGpsMillis < GPS_TIMEOUT;
}

/*    Report a fix only while its position and source fields are current.
*/
bool currentGPSFix() {
  return currentGPSStream() && navigationMetrics.hasFix
      && navigationMetrics.position.fresh(millis(), gpsSerial)
      && millis() - navigationMetrics.fixUpdated < GPS_FIX_TIMEOUT
      && gps.location.isValid() && gps.location.age() < GPS_FIX_TIMEOUT;
}

/*    Read bounded NMEA frames and reject corrupted or wrong-baud traffic.
*/
void serialGPSRead() {
  GPSState prevState = gpsSerialState;
  // Limit work per loop so noise or a busy UART cannot starve the keyboard.
  int remaining = 2048;
  while (GPS_Serial.available() && remaining-- > 0) {
    char c = GPS_Serial.read();
    lastGpsByteMillis = millis();
    gpsReceived = true;
    if (c == '$') nmeaLine = "$";
    else if (c == '\r' || c == '\n') {
      if (nmeaLine.length() > 0) {
        if (nmeaSerial) Serial.println(nmeaLine);
        if (validNMEASentence(nmeaLine.c_str(), nmeaLine.length())) {
          lastValidGpsMillis = millis();
          gpsValid = true;
          nmeaDispatcher(nmeaLine);
        }
      }
      nmeaLine = "";
    } else if (nmeaLine.length() > 0) {
      if (c >= 32 && c <= 126 && nmeaLine.length() < 255) nmeaLine += c;
      else nmeaLine = ""; // Resynchronize at the next '$'.
    }
  }
  gpsSerialState = gpsReceptionState(gpsSerial, gpsReceived, gpsValid,
      currentGPSFix(), millis(), gpsStartedMillis, lastGpsByteMillis, lastValidGpsMillis);
  satelliteMetrics.refresh(millis(), currentGPSStream(), currentGPSFix());
  if (gpsSerialState != prevState) drawStatus();
}

/*    Update satellite history and metric validity from one checked frame.
*/
void nmeaDispatcher(const String &line) {
  NMEAFields fields(line.c_str());
  if (fields.count == 0 || !knownGNSSTalker(fields.field(0))) return;
  navigationMetrics.process(fields, millis());
  satelliteMetrics.process(fields, millis());
  std::string normalized = navigationMetrics.decoderFrame(fields);
  if (normalized.empty()) return;
  for (size_t i = 0; i < normalized.length(); i++) gps.encode(normalized[i]);
  gps.encode('\r');
  gps.encode('\n');
}

/*    Open or close the USB serial console.
*/
void initDebugSerial(bool should_I) {
  if (should_I && !debugSerial) {
    debugSerial = true;
    Serial.begin( gpsConfig.usbBaud );
    Serial.setTimeout( 2000 );
    // USB output must not block keyboard handling without a connected host.
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
    Serial.println("System    ID  Ele  Azi  CN0  Now Ever Vis");
    Serial.println("------------------------------------");
    for (auto &sat : satellites) {
      Serial.printf("%-8s %3d %4d %4d %4d   %c    %c   %c\n", sat.system.c_str(), sat.id, sat.elevation, sat.azimuth, sat.snr, sat.used ? 'Y' : 'N', sat.usedEver ? 'Y' : 'N', sat.visible ? 'V' : 'X');
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
    if (openMenu) {
      if (skyMenu) drawSkyPopup();
      else if (barsMenu) drawConstellationBars();
      return;
    }
    if (force) {
      drawMainScreen(M5Cardputer.Display);
      return;
    }
    // Satellites datas.
    drawSatelliteDataTab(M5Cardputer.Display);
    // Satellites plot.
    drawSkyPlot();
  }
}

/*    Buffer live popups to avoid clearing the LCD during each refresh.
*/
lgfx::LGFXBase &popupDisplay() {
  if (!popupCanvasAttempted) {
    popupCanvasAttempted = true;
    popupCanvas.setPsram(false);
    popupCanvas.setColorDepth(8);
    popupCanvasReady = popupCanvas.createSprite(screenW, screenH) != nullptr;
  }
  if (popupCanvasReady) return popupCanvas;
  return M5Cardputer.Display;
}

/*    Create a measured popup over the main screen; callers draw only contents.
*/
lgfx::LGFXBase &createPopup(int x, int y, int width, int height) {
  lgfx::LGFXBase &display = popupDisplay();
  PopupRect frame = {x, y, width, height};
  preparePopup(display, frame, drawMainScreen, TFT_BLACK, TFT_GREEN);
  display.setTextSize(1);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  return display;
}

/*    Present the composed screen only after the popup contents are complete.
*/
void finishPopup() {
  popupDisplay().clearClipRect();
  if (popupCanvasReady) popupCanvas.pushSprite(0, 0);
}

/*    Draw the satellites sky plot inside the requested popup or table area.
*/
void drawSkyPlotArea(lgfx::LGFXBase &display, int x, int y, int w, int h, bool fullScreen) {
  int half_side = (std::min(w, h) - 1) / 2;
  int cx = x + w / 2;
  int cy = y + h / 2;
  int r = half_side;
  int32_t clipX, clipY, clipW, clipH;
  display.getClipRect(&clipX, &clipY, &clipW, &clipH);
  int32_t left = std::max<int32_t>(clipX, x-1);
  int32_t top = std::max<int32_t>(clipY, y-1);
  int32_t right = std::min<int32_t>(clipX + clipW, x+w+1);
  int32_t bottom = std::min<int32_t>(clipY + clipH, y+h+1);
  display.setClipRect(left, top, std::max<int32_t>(0, right-left), std::max<int32_t>(0, bottom-top));
  if (!fullScreen) display.drawRect(x-1, y-1, w+2, h+2, TFT_DARKGREY);
  display.fillRect(x, y, w, h, TFT_BLACK);
  // Ref circles.
  display.drawCircle(cx, cy, r, TFT_WHITE);
  display.drawCircle(cx, cy, r * 0.66, TFT_DARKGREY);
  display.drawCircle(cx, cy, r * 0.33, TFT_DARKGREY);
  display.drawLine(cx - r, cy, cx + r, cy, TFT_DARKGREY);
  display.drawLine(cx, cy - r, cx, cy + r, TFT_DARKGREY);
  // Cardinals label.
  display.setTextSize(1);
  display.setTextColor(TFT_LIGHTGREY);
  display.setCursor(cx - 3, cy - r + 4);  display.print("N");
  display.setCursor(cx - 3, cy + r - 10); display.print("S");
  display.setCursor(cx + r - 10, cy - 3); display.print("E");
  display.setCursor(cx - r + 4, cy - 3);  display.print("W");
  // Satellites with no reported position must not appear at a false azimuth.
  for (auto &sat : satellites) {
    if (!sat.hasPosition) continue;
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
    display.fillCircle(sx, sy, 2, color); // Satellite dot.
    String label = "";
    if (!hidePlotSystem) label = satelliteSystemCode(sat.system);
    if (!hidePlotId) label += String(sat.id);
    if (label.length() > 0) {
      display.setTextSize(1);
      int textX = std::max(x + 2, std::min(static_cast<int>(sx) + 5,
          x + w - 2 - static_cast<int>(display.textWidth(label))));
      int textY = std::max(y + 1, std::min(static_cast<int>(sy) - 3, y + h - 9));
      display.setTextColor(TFT_BLACK);
      display.setCursor(textX - 1, textY - 1);
      display.print(label);
      display.setTextColor(color);
      display.setCursor(textX, textY);
      display.print(label);
    }
  }
  display.setClipRect(clipX, clipY, clipW, clipH);
}

/*    Draw the compact skyplot on the main screen.
*/
void drawSkyPlot() {
  drawSkyPlotArea(M5Cardputer.Display, 143, 27, 96, 95, false);
}

/*    Draw only the enlarged skyplot in a square popup at the top right.
*/
void drawSkyPopup() {
  const int margin = 5;
  const int side = screenH - margin * 2;
  const int x = screenW - margin - side;
  const int y = margin;
  lgfx::LGFXBase &display = createPopup(x, y, side, side);
  drawSkyPlotArea(display, x + 4, y + 4, side - 8, side - 8, true);
  finishPopup();
}

/*    Draw one bar per constellation, including the satellites counted as Other.
*/
void drawConstellationBars() {
  lgfx::LGFXBase &display = createPopup(5, 5, screenW-10, screenH-10);
  GPSConstellationCounts counts = constellationCounts(satelliteMetrics, millis(), currentGPSStream());
  display.setTextSize(1);
  display.setTextColor(TFT_GREEN, TFT_BLACK);
  display.setCursor(11, 11);
  display.print("Visible satellites");
  display.setCursor(177, 11);
  display.print("[b]Close");
  display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  display.setCursor(11, 23);
  if (counts.available) display.printf("Total:%d%s  [f]Skyplot", counts.total, counts.partial ? "?" : "");
  else display.print("No current GSV data  [f]Skyplot");
  const int left = 33, top = 45, bottom = 106, slot = 27;
  int scale = constellationBarScale(counts);
  for (int tick = 0; tick <= 2; tick++) {
    int value = tick * scale / 2;
    int y = bottom - value * (bottom - top) / scale;
    display.drawLine(left-2, y, 229, y, TFT_DARKGREY);
    display.setCursor(11, y - 3);
    display.printf("%d", value);
  }
  const char* labels[] = {"GPS", "GLO", "GAL", "BDS", "QZS", "SBAS", "NAV"};
  const uint16_t colors[] = {TFT_CYAN, TFT_MAGENTA, TFT_GREEN, TFT_YELLOW, TFT_ORANGE, TFT_LIGHTGREY, TFT_BLUE};
  for (int i = 0; i < 7; i++) {
    int x = left + i * slot;
    int height = counts.counts[i] * (bottom - top) / scale;
    display.setTextColor(colors[i], TFT_BLACK);
    display.setCursor(x, 35);
    if (counts.available) display.printf("%d%s", counts.counts[i], counts.partial ? "?" : "");
    else display.print("--");
    if (height > 0) display.fillRect(x + 3, bottom - height, 16, height, colors[i]);
    else display.drawLine(x + 3, bottom - 1, x + 18, bottom - 1, colors[i]);
    display.setCursor(x, 115);
    display.print(labels[i]);
  }
  finishPopup();
}

/*    Fit a coordinate inside its cell without overwriting satellite counts.
*/
void formatGPSCoordinate(lgfx::LGFXBase &display, char* output, size_t size, const char* label, double value) {
  for (int precision = 6; precision >= 0; precision--) {
    snprintf(output, size, "%.*f", precision, value);
    if (display.textWidth(String(label) + ":" + output) <= 84) break;
  }
}

/*    Draw the satellites main data table.
*/
void drawSatelliteDataTab(lgfx::LGFXBase &display){
  int x = 1;
  int y = 26;
  uint32_t now = millis();
  bool stream = currentGPSStream();
  bool fix = currentGPSFix();
  display.setTextSize(1);
  // Column 1. Numeric zero is displayed only when a valid field reports zero.
  const char* c1labels[] = { "Lat", "Lng", "Alt", "Spd", "Crs", "Date", "UTC", "HDOP" };
  char c1values[8][20];
  for (auto &value : c1values) snprintf(value, sizeof(value), "NoData");
  if (fix) {
    formatGPSCoordinate(display, c1values[0], sizeof(c1values[0]), "Lat", gps.location.lat());
    formatGPSCoordinate(display, c1values[1], sizeof(c1values[1]), "Lng", gps.location.lng());
  } else {
    snprintf(c1values[0], sizeof(c1values[0]), "NoFix");
    snprintf(c1values[1], sizeof(c1values[1]), "NoFix");
  }
  if (fix && navigationMetrics.altitude.fresh(now, stream) && gps.altitude.isValid())
    snprintf(c1values[2], sizeof(c1values[2]), "%.2f", gps.altitude.meters());
  if (fix && navigationMetrics.speed.fresh(now, stream) && gps.speed.isValid())
    snprintf(c1values[3], sizeof(c1values[3]), "%.1f", gps.speed.kmph());
  if (fix && navigationMetrics.course.fresh(now, stream) && gps.course.isValid())
    snprintf(c1values[4], sizeof(c1values[4]), "%.1f", gps.course.deg());
  if (navigationMetrics.date.fresh(now, stream) && gps.date.isValid())
    snprintf(c1values[5], sizeof(c1values[5]), "%02d/%02d/%02d", gps.date.day(), gps.date.month(), gps.date.year() % 100);
  if (navigationMetrics.time.fresh(now, stream) && gps.time.isValid())
    snprintf(c1values[6], sizeof(c1values[6]), "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  if (fix && navigationMetrics.hdop.fresh(now, stream) && gps.hdop.isValid())
    snprintf(c1values[7], sizeof(c1values[7]), "%.1f", gps.hdop.hdop());
  for (int i = 0; i < 8; i++) {
    display.fillRect(x, y, 90, 13, TFT_BLACK);
    display.drawRect(x, y, 90, 13, TFT_DARKGREY);
    display.setTextColor(TFT_WHITE);
    display.setCursor(x + 4, y + 3);
    display.printf("%s:%s", c1labels[i], c1values[i]);
    y += 12;
  }
  // Column 2. Seen/Used are session history; all other counts are current.
  y = 26;
  x += 89;
  const char* c2labels[] = { "Seen", "Visb", "Used", "InFx", "GPS", "Gln", "Gal", "BDo" };
  char c2values[8][12];
  for (auto &value : c2values) snprintf(value, sizeof(value), "--");
  int totalUsed = 0, totalVisible = 0;
  int gpsVisible = 0, glonassVisible = 0, galileoVisible = 0, beidouVisible = 0;
  for (const auto &sat : satellites) {
    if (sat.usedEver) totalUsed++;
    if (!sat.visible) continue;
    totalVisible++;
    if (sat.system == "GPS") gpsVisible++;
    else if (sat.system == "GLONASS") glonassVisible++;
    else if (sat.system == "Galileo") galileoVisible++;
    else if (sat.system == "BeiDou") beidouVisible++;
  }
  snprintf(c2values[0], sizeof(c2values[0]), "%u%s", static_cast<unsigned int>(satellites.size()), satelliteMetrics.seenUncertain ? "?" : "");
  snprintf(c2values[2], sizeof(c2values[2]), "%d%s", totalUsed, satelliteMetrics.usedUncertain ? "?" : "");
  if (navigationMetrics.inFix.fresh(now, stream) && gps.satellites.isValid())
    snprintf(c2values[3], sizeof(c2values[3]), "%lu", navigationMetrics.hasFix ? static_cast<unsigned long>(gps.satellites.value()) : 0UL);
  if (satelliteMetrics.freshViews(now, stream)) {
    const char* partial = satelliteMetrics.uncertainViews(now, stream) ? "?" : "";
    snprintf(c2values[1], sizeof(c2values[1]), "%d%s", totalVisible, partial);
    snprintf(c2values[4], sizeof(c2values[4]), "%d%s", gpsVisible, partial);
    snprintf(c2values[5], sizeof(c2values[5]), "%d%s", glonassVisible, partial);
    snprintf(c2values[6], sizeof(c2values[6]), "%d%s", galileoVisible, partial);
    snprintf(c2values[7], sizeof(c2values[7]), "%d%s", beidouVisible, partial);
  }
  for (int i = 0; i < 8; i++) {
    display.fillRect(x, y, 53, 13, TFT_BLACK);
    display.drawRect(x, y, 53, 13, TFT_DARKGREY);
    display.setTextColor(TFT_WHITE);
    display.setCursor(x + 4, y + 3);
    String text = String(c2labels[i]) + ":" + c2values[i];
    if (display.textWidth(text) > 47) display.setTextSize(0.85f);
    display.print(text);
    display.setTextSize(1);
    y += 12;
  }
}

/*    Draw main features key map.
*/
void drawHeader(lgfx::LGFXBase &display){
  int x = 1;
  int y = 1;
  int w = screenW;
  int h = 13;
  display.setTextSize(1);
  // Title.
  display.fillRect(x, y, w, h, TFT_GREEN); // bordo nel colore del testo
  display.setTextColor(TFT_BLACK);      // testo e sfondo
  display.setCursor(x + 4, y + 3);
  display.printf("%-1s", "      -= Cardputer GPS Info =-");
  // Key map.
  display.drawRect(x, y+h-1, w, h, TFT_GREEN);
  display.setTextColor(TFT_GREEN);
  display.setCursor(x + 4, y+h + 3);
  display.printf("%-1s", "[s]Start/Stop. [c]Config. [h]Help.");
}

/*    Draw app features status.
*/
void drawStatusArea(lgfx::LGFXBase &display){
  int x = 1;
  int y = 122;
  int w = screenW;
  int h = 13;
  char statusChar[64];
  statusChar[0] = '\0';
  const char* gpsStr = "Off";
  if (gpsSerialState == GPS_WAIT) gpsStr = "Wait";
  else if (gpsSerialState == GPS_NO_DATA) gpsStr = "NoRx";
  else if (gpsSerialState == GPS_BAD_DATA) gpsStr = "NMEA?";
  else if (gpsSerialState == GPS_ON) gpsStr = "NMEA";
  else if (gpsSerialState == GPS_FIX) gpsStr = "Fix";
  snprintf(statusChar + strlen(statusChar), sizeof(statusChar) - strlen(statusChar),"GP:%s ", gpsStr);
  snprintf(statusChar + strlen(statusChar),sizeof(statusChar) - strlen(statusChar),"Rx:%d Tx:%d ", gpsRxPin, gpsTxPin);
  snprintf(statusChar + strlen(statusChar),sizeof(statusChar) - strlen(statusChar),"Bd:%d ", gpsBaud);
  snprintf(statusChar + strlen(statusChar),sizeof(statusChar) - strlen(statusChar),"USB:%s", debugSerial ? "On" : "Off");
  display.fillRect(x, y, w, h, TFT_DARKGREY);
  display.setTextColor(TFT_WHITE);
  display.setCursor(x + 4, y + 3);
  display.setTextSize(1);
  display.printf("%-1s", statusChar);
}

/*    Update the status bar only when it is not covered by a popup.
*/
void drawStatus() {
  if (!openMenu) drawStatusArea(M5Cardputer.Display);
}

/*    Redraw the real main screen behind each popup, including after switches.
*/
void drawMainScreen(lgfx::LGFXBase &display) {
  display.fillScreen(TFT_BLACK);
  drawHeader(display);
  drawStatusArea(display);
  drawSatelliteDataTab(display);
  drawSkyPlotArea(display, 143, 27, 96, 95, false);
}

/*    Get a short label for the selected GPS connection.
*/
const char* gpsProfileName(int profile) {
  const char* names[] = {"Grove GPS", "ADV Cap LoRa-1262", "ADV Ares PCB v0.8", "Manual"};
  return profile >= 0 && profile < GPS_PROFILE_COUNT ? names[profile] : "Invalid";
}

/*    Copy validated settings to the active configuration.
*/
void applyGPSConfig(const GPSConfig &config) {
  if (boardAdv && gpsConfig.hardware == GPS_CAP && config.hardware != GPS_CAP)
    pinMode(5, INPUT);
  if (boardAdv && gpsConfig.hardware == GPS_ARES && config.hardware != GPS_ARES) {
    // Release the pins owned by the old hardware profile.
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    pinMode(5, INPUT);
    pinMode(6, INPUT);
    pinMode(13, INPUT);
  }
  bool restartUSB = debugSerial && gpsConfig.usbBaud != config.usbBaud;
  if (restartUSB) initDebugSerial(false);
  gpsConfig = config;
  if (restartUSB) initDebugSerial(true);
  gpsRxPin = config.rxPin;
  gpsTxPin = config.txPin;
  gpsBaud = config.baud;
  prepareGPSHardware();
}

/*    Draw configuration popup without changing UART or menu state.
*/
void drawConfig(bool should_I) {
  if (!should_I) {
    updateScreen(true);
    return;
  }
  lgfx::LGFXBase &display = createPopup(2, 2, screenW-4, screenH-4);
  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setTextSize(1);
  display.setCursor(8, 8);
  display.printf("Config SD:%s %s", sdAvailable ? "Yes" : "No",
      sdErr ? "ERR" : "");
  display.setCursor(8, 20);
  display.print("Up/Down:row Left/Right:profile");
  display.setCursor(8, 32);
  display.print("[m]Manual [c]Close [Enter]Save");
  display.setCursor(8, 44);
  display.printf("%c Profile: %s", configsMenuSel == 0 ? '>' : ' ',
      gpsProfileName(configsDraft.profile));
  const char* labels[] = {"RX <- GPS TX", "TX -> GPS RX", "GPS baud", "USB baud"};
  for (int i = 0; i < 4; i++) {
    display.setCursor(8, 56 + i * 12);
    display.printf("%c %s: %s", configsMenuSel == i + 1 ? '>' : ' ',
        labels[i], configsTmp[i].c_str());
  }
  display.setCursor(8, 104);
  if (configsDraft.profile == GPS_MANUAL)
    display.printf("Base: %s", gpsProfileName(configsDraft.hardware));
  else
    display.print("GPS:[m]Manual  USB:always editable");
  display.setCursor(8, 117);
  display.setTextColor(TFT_YELLOW, TFT_BLACK);
  display.print(configMessage.substring(0, 37));
  finishPopup();
}

/*    Store configuration on microSD, with visible write-error reporting.
*     The UART is stopped before saving: Ares G13 must remain CSN high.
*/
void saveConfig() {
  if (!sdAvailable) {
    // Ares may need its radio chip selects initialized before mounting SD.
    SD.end();
    sdAvailable = SD.begin(12, SPI, 4000000);
  }
  if (!sdAvailable) {
    configMessage = "Applied in RAM; no SD card";
    return;
  }
  File file = SD.open(CONFIG_FILE, FILE_WRITE);
  if (!file) {
    sdErr = true;
    configMessage = "SD write failed; applied in RAM";
    return;
  }
  String contents = "gpsProfile=" + String(gpsConfig.profile) + "\n";
  contents += "gpsHardware=" + String(gpsConfig.hardware) + "\n";
  contents += "gpsRxPin=" + String(gpsRxPin) + "\n";
  contents += "gpsTxPin=" + String(gpsTxPin) + "\n";
  contents += "gpsBaud=" + String(gpsBaud) + "\n";
  contents += "usbBaud=" + String(gpsConfig.usbBaud) + "\n";
  size_t written = file.print(contents);
  file.flush();
  sdErr = written != contents.length() || file.getWriteError();
  file.close();
  configMessage = sdErr ? "SD write failed; applied in RAM" : "Saved. [c]Close, [s]Start GPS";
}

/*    Load legacy or profile-based configuration as one validated transaction.
*/
void loadConfig() {
  if (!sdAvailable || !SD.exists(CONFIG_FILE)) return;
  File file = SD.open(CONFIG_FILE);
  if (!file) {
    sdErr = true;
    configLoadErr = true;
    return;
  }
  GPSConfigReader reader;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("#")) continue;
    int separator = line.indexOf('=');
    if (separator < 0) { reader.valid = false; continue; }
    String key = line.substring(0, separator);
    String value = line.substring(separator + 1);
    key.trim();
    value.trim();
    reader.set(key.c_str(), value.c_str());
  }
  file.close();
  // Preserve legacy values as Manual; never guess Ares from UART pins alone.
  const char* error = reader.error(boardAdv);
  if (error) {
    configLoadErr = true;
    configMessage = error;
    return;
  }
  applyGPSConfig(reader.config);
}

/*    Draw info popup.
*/
void drawInfo(bool should_I) {
  if (should_I == true) {
    openMenu = true;
    const char* helpText[] = {
      "Cardputer GPS Info",
      "V " APP_VERSION " by alcor55",
      "",
      "Github:",
      "https://github.com/alcor55",
      "/Cardputer-GPS-Info"
    };
    int count = sizeof(helpText) / sizeof(helpText[0]);
    lgfx::LGFXBase &display = createPopup(20, 20, 200, 95);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setTextSize(1);
    display.setCursor(25, 24);
    int y = 24;
    for (int i = 0; i < count; i++) {
      display.setCursor(25, y);
      display.println(helpText[i]);
      y += 10;
    }
    finishPopup();
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
      "[l] USB serial satellite list.",
      "[n] USB serial NMEA sentences.",
      "[f] Enlarged skyplot popup.",
      "[b] Visible satellites by GNSS.",
      "[p] Show/hide ID on skyplot.",
      "[o] Show/hide System on skyplot."
    };
    int count = sizeof(helpText) / sizeof(helpText[0]);
    lgfx::LGFXBase &display = createPopup(12, 12, screenW-24, screenH-24);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setTextSize(1);
    int y = 19;
    for (int i = 0; i < count; i++) {
      String line = helpText[i];
      int endKey = line.indexOf(']') + 1;
      if (endKey > 0) {
        String key = line.substring(0, endKey);
        String desc = line.substring(endKey);
        display.setTextColor(TFT_GREEN, TFT_BLACK);
        display.setCursor(20, y);
        display.print(key);
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        display.print(desc);
      } else {
        display.setTextColor(TFT_WHITE, TFT_BLACK);
        display.setCursor(25, y);
        display.print(line);
      }
      y += 10;
    }
    finishPopup();
  } else {
    openMenu = false;
    updateScreen(true); // Forced update.
  }
}

/*    Capture key-down edges, including same-size keyboard chord changes.
*/
void readKeyboardPresses() {
  Keyboard_Class::KeysState status = M5Cardputer.Keyboard.keysState();
  String currentKeys = "";
  for (auto c : status.word) currentKeys += c;
  if (status.enter) currentKeys += '\n';
  if (status.del) currentKeys += '\b';
  pressedKeys = "";
  keyEdges.update(currentKeys.c_str());
  for (int i = 1; i < 128; i++) {
    if (keyEdges.pressed[i]) pressedKeys += static_cast<char>(i);
  }
  enterPressed = pressedKeys.indexOf('\n') >= 0;
  deletePressed = pressedKeys.indexOf('\b') >= 0;
}

/*    Fill configuration fields after choosing a profile.
*/
void fillConfigFields() {
  configsTmp[0] = String(configsDraft.rxPin);
  configsTmp[1] = String(configsDraft.txPin);
  configsTmp[2] = String(configsDraft.baud);
  configsTmp[3] = String(configsDraft.usbBaud);
}

/*    Handle keyboard data inputs in the configuration menu.
*/
void handleKeys() {
  if (!configsMenu) return;
  bool changed = false;
  for (size_t i = 0; i < pressedKeys.length(); i++) {
    char c = pressedKeys[i];
    if (c == 'm') {
      configsDraft.profile = GPS_MANUAL;
      configMessage = "Manual: hardware base retained";
      changed = true;
    } else if (c == ';') { configsMenuSel = (configsMenuSel + 4) % 5; changed = true; }
    else if (c == '.') { configsMenuSel = (configsMenuSel + 1) % 5; changed = true; }
    else if (configsMenuSel == 0 && (c == ',' || c == '/')) {
      int profile = (configsDraft.profile + (c == ',' ? GPS_PROFILE_COUNT - 1 : 1))
          % GPS_PROFILE_COUNT;
      if (profile == GPS_MANUAL) configsDraft.profile = GPS_MANUAL;
      else configsDraft = selectGPSPreset(configsDraft, profile);
      String usbDraft = configsTmp[3];
      fillConfigFields();
      configsTmp[3] = usbDraft;
      configMessage = "";
      changed = true;
    } else if (configsMenuSel > 0 && c >= '0' && c <= '9'
        && (configsDraft.profile == GPS_MANUAL || configsMenuSel == 4)) {
      String &field = configsTmp[configsMenuSel - 1];
      if (field.length() < 7) field += c;
      configMessage = "";
      changed = true;
    }
  }
  if (deletePressed && configsMenuSel > 0 && (configsDraft.profile == GPS_MANUAL || configsMenuSel == 4)) {
    String &field = configsTmp[configsMenuSel - 1];
    if (field.length() > 0) field.remove(field.length() - 1);
    changed = true;
  }
  if (enterPressed) {
    GPSConfig candidate = configsDraft;
    bool valid = parseGPSNumber(configsTmp[0].c_str(), candidate.rxPin)
        && parseGPSNumber(configsTmp[1].c_str(), candidate.txPin)
        && parseGPSNumber(configsTmp[2].c_str(), candidate.baud)
        && parseGPSNumber(configsTmp[3].c_str(), candidate.usbBaud);
    const char* error = gpsConfigError(candidate, boardAdv);
    if (!valid || error) configMessage = valid ? error : "Enter valid decimal values";
    else {
      applyGPSConfig(candidate);
      configLoadErr = false;
      saveConfig();
    }
    changed = true;
  }
  if (changed) drawConfig(true);
}

/*    Close one popup before opening another, without changing the GPS session.
*/
void closePopup() {
  helpMenu = infoMenu = configsMenu = skyMenu = barsMenu = openMenu = false;
}

/*    Handle app functions once per key press, with one modal menu at a time.
*/
void handleControls() {
  if (pressedKeys.indexOf('c') >= 0) {
    bool close = configsMenu;
    closePopup();
    if (close) updateScreen(true);
    else {
      configsMenu = openMenu = true;
      initGPSSerial(false);
      configsDraft = gpsConfig;
      fillConfigFields();
      configsMenuSel = 0;
      if (!configLoadErr) configMessage = sdAvailable ? "" : "No SD: settings stay in RAM";
      drawConfig(true);
    }
    return;
  }
  if (configsMenu) return;
  if (pressedKeys.indexOf('h') >= 0) {
    bool close = helpMenu;
    closePopup();
    helpMenu = !close;
    drawHelp(helpMenu);
    return;
  }
  if (pressedKeys.indexOf('i') >= 0) {
    bool close = infoMenu;
    closePopup();
    infoMenu = !close;
    drawInfo(infoMenu);
    return;
  }
  if (pressedKeys.indexOf('f') >= 0) {
    bool close = skyMenu;
    closePopup();
    if (close) updateScreen(true);
    else {
      skyMenu = openMenu = true;
      drawSkyPopup();
    }
    return;
  }
  if (pressedKeys.indexOf('b') >= 0) {
    bool close = barsMenu;
    closePopup();
    if (close) updateScreen(true);
    else {
      barsMenu = openMenu = true;
      drawConstellationBars();
    }
    return;
  }
  if (skyMenu) {
    if (pressedKeys.indexOf('p') >= 0) { hidePlotId = !hidePlotId; drawSkyPopup(); }
    if (pressedKeys.indexOf('o') >= 0) { hidePlotSystem = !hidePlotSystem; drawSkyPopup(); }
  }
  if (deletePressed && (skyMenu || barsMenu)) {
    closePopup();
    updateScreen(true);
    return;
  }
  if (openMenu) return;
  if (pressedKeys.indexOf('s') >= 0) {
    initGPSSerial(!gpsSerial);
    updateScreen(true);
  }
  if (pressedKeys.indexOf('p') >= 0) hidePlotId = !hidePlotId;
  if (pressedKeys.indexOf('o') >= 0) hidePlotSystem = !hidePlotSystem;
  if (pressedKeys.indexOf('l') >= 0) {
    nmeaSerial = false;
    satListSerial = !satListSerial;
    initDebugSerial(satListSerial);
    drawStatus();
  }
  if (pressedKeys.indexOf('n') >= 0) {
    satListSerial = false;
    nmeaSerial = !nmeaSerial;
    initDebugSerial(nmeaSerial);
    drawStatus();
  }
}

/*    Detect the ADV keyboard before enabling any GPIO matrix reader.
*     Ares pull-ups on G5/G6 can confuse the display library's board detection.
*/
void initCardputerKeyboard() {
  boardAdv = M5.getBoard() == m5::board_t::board_M5CardputerADV;
  if (!boardAdv) {
    m5::In_I2C.release();
    m5::In_I2C.begin(I2C_NUM_1, 8, 9);
    boardAdv = m5::In_I2C.scanID(0x34);
    if (!boardAdv) m5::In_I2C.release();
  }
  if (boardAdv) {
    m5::In_I2C.begin(I2C_NUM_1, 8, 9);
    std::unique_ptr<KeyboardReader> reader(new TCA8418KeyboardReader(11));
    M5Cardputer.Keyboard.begin(std::move(reader));
  } else
    M5Cardputer.Keyboard.begin();
}

/*    Standard setup function.
*/
void setup() {
  auto cfg = M5.config();
  cfg.internal_mic = false;
  cfg.internal_spk = false;
  cfg.internal_imu = false;
  cfg.internal_rtc = false;
  M5Cardputer.begin(cfg, false); // Initialize keyboard after checking ADV hardware.
  initCardputerKeyboard();
  M5Cardputer.Display.setBrightness(32); // Set brightness 0-255.
  if (boardAdv) {
    // Bootstrap SD before the saved profile is known. Weak pull-ups keep
    // radio chip selects high without driving Ares IR RX or Cap BUSY outputs.
    pinMode(5, INPUT_PULLUP);
    pinMode(6, INPUT_PULLUP);
    pinMode(13, INPUT_PULLUP);
    pinMode(4, INPUT);
  }
  SPI.begin(40, 39, 14, 12); // Cardputer microSD: SCK, MISO, MOSI, CS.
  if (SD.begin(12, SPI, 4000000)) {
    sdAvailable = true;
    loadConfig();
  } else
    sdAvailable = false;
  if (configLoadErr || !sdAvailable || !SD.exists(CONFIG_FILE)) {
    configsMenu = openMenu = true;
    configsDraft = gpsConfig;
    fillConfigFields();
    if (configMessage.length() == 0)
      configMessage = configLoadErr ? "SD read error; defaults kept" : "Choose profile, then press Enter";
    drawConfig(true);
  } else
    updateScreen(true); // Forced first update.
}

/*    Standard loop.
*/
void loop() {
  M5Cardputer.update(); // Update Cardputer state (keyboard, buttons, sensors etc...).
  // begin(cfg, false) leaves automatic keyboard polling disabled.
  M5Cardputer.Keyboard.updateKeyList();
  M5Cardputer.Keyboard.updateKeysState();
  readKeyboardPresses();
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
