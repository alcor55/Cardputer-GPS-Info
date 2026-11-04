"""Run the sketch startup functions against simulated board peripherals.

This checks driver selection and startup flow; it does not emulate ESP32 hardware.
"""

from pathlib import Path
import subprocess
import tempfile


def sketch_function(sketch, signature):
    start = sketch.index(signature)
    position = sketch.index("{", start) + 1
    depth = 1
    while depth:
        if sketch[position] == "{":
            depth += 1
        elif sketch[position] == "}":
            depth -= 1
        position += 1
    return sketch[start:position]


sketch = (Path(__file__).resolve().parent.parent / "Cardputer_GPS_Info.ino").read_text()
code = r'''
#include <cassert>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#define APP_VERSION "test"
#define CONFIG_FILE "/cpGpsInfo.conf"
constexpr int I2C_NUM_1 = 1, INPUT_PULLUP = 1, INPUT = 0;
namespace m5 {
  enum class board_t { board_unknown, board_M5Cardputer, board_M5CardputerADV };
  struct Bus {
    bool available = true;
    std::vector<bool> replies;
    unsigned int scans = 0;
    bool released = true;
    bool begin(int, int, int) { released = false; return available; }
    void release() { released = true; }
    bool scanID(int address) {
      assert(address == 0x34 && !released);
      return scans < replies.size() ? replies[scans++] : false;
    }
  } In_I2C;
}
struct KeyboardReader { virtual ~KeyboardReader() {} virtual int kind() = 0; };
struct TCA8418KeyboardReader : KeyboardReader {
  TCA8418KeyboardReader(int pin) { assert(pin == 11); }
  int kind() override { return 2; }
};
struct IOMatrixKeyboardReader : KeyboardReader { int kind() override { return 1; } };
struct Keyboard {
  int reader = 0;
  void begin(std::unique_ptr<KeyboardReader> r) {
    reader = r->kind();
    assert(reader == 2 || m5::In_I2C.released);
  }
};
struct Config { bool internal_mic, internal_spk, internal_imu, internal_rtc; };
struct Board {
  m5::board_t reported = m5::board_t::board_M5Cardputer;
  m5::board_t getBoard() { return reported; }
  Config config() { return {}; }
} M5;
struct Cardputer {
  struct Keyboard Keyboard;
  struct Display { void setBrightness(int) {} } Display;
  void begin(Config, bool keyboard) { assert(!keyboard); }
} M5Cardputer;
struct Console {
  void begin(int) {}
  void println(const char*) {}
  template<typename... Args> void printf(const char*, Args...) {}
} Serial;
struct Spi { void begin(int, int, int, int) {} } SPI;
struct Sd {
  bool present = false, config = false;
  bool begin(int, Spi&, int) { return present; }
  bool exists(const char*) { return config; }
} SD;
bool boardAdv = false, sdAvailable = false, configLoadErr = false;
bool mainScreen = false, loaded = false;
std::string configMessage;
void delay(int) {}
void pinMode(int, int) {}
void drawBootStage(const char*) {}
void loadConfig() { loaded = true; }
void updateScreen(bool force) { assert(force); mainScreen = true; }
'''
code += sketch_function(sketch, "void initCardputerKeyboard()")
code += sketch_function(sketch, "void setup()")
code += r'''
int main() {
  for (auto reported : {m5::board_t::board_unknown,
      m5::board_t::board_M5Cardputer, m5::board_t::board_M5CardputerADV}) {
    for (bool adv : {false, true}) {
      M5.reported = reported;
      m5::In_I2C = {};
      m5::In_I2C.replies = {adv, adv};
      initCardputerKeyboard();
      assert(boardAdv == adv);
      assert(M5Cardputer.Keyboard.reader == (adv ? 2 : 1));
    }
  }
  m5::In_I2C = {};
  m5::In_I2C.replies = {false, true};
  initCardputerKeyboard();
  assert(boardAdv && m5::In_I2C.scans == 2);
  m5::In_I2C = {};
  m5::In_I2C.available = false;
  initCardputerKeyboard();
  assert(!boardAdv && m5::In_I2C.scans == 0);
  for (bool sd : {false, true}) {
    for (bool config : {false, true}) {
      for (bool invalid : {false, true}) {
        SD.present = sd;
        SD.config = config;
        configLoadErr = invalid;
        configMessage.clear();
        loaded = mainScreen = false;
        setup();
        assert(mainScreen && sdAvailable == sd && loaded == sd);
      }
    }
  }
  puts("Cardputer startup: PASS (driver selection, retry, bus failure, SD/config states)");
}
'''
with tempfile.TemporaryDirectory(prefix="cardputer-startup-test-") as directory:
    source = Path(directory) / "startup.cpp"
    executable = Path(directory) / "startup"
    source.write_text(code)
    subprocess.run(["c++", "-std=c++14", "-Wall", "-Wextra", "-Werror",
        str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
