#include "../GpsPopup.h"
#include <algorithm>
#include <assert.h>
#include <stdio.h>
#include <vector>

struct TestDisplay {
  const int width = 240, height = 135;
  int left = 0, top = 0, right = 240, bottom = 135;
  int backgrounds = 0;
  std::vector<uint16_t> pixels = std::vector<uint16_t>(240 * 135);

  void clearClipRect() { left = top = 0; right = width; bottom = height; }
  void setClipRect(int x, int y, int w, int h) {
    left = std::max(0, x); top = std::max(0, y);
    right = std::min(width, x + w); bottom = std::min(height, y + h);
  }
  void fillRect(int x, int y, int w, int h, uint16_t color) {
    for (int row = std::max(top, y); row < std::min(bottom, y + h); row++)
      for (int col = std::max(left, x); col < std::min(right, x + w); col++)
        pixels[row * width + col] = color;
  }
  void drawRect(int x, int y, int w, int h, uint16_t color) {
    fillRect(x, y, w, 1, color);
    fillRect(x, y + h - 1, w, 1, color);
    fillRect(x, y, 1, h, color);
    fillRect(x + w - 1, y, 1, h, color);
  }
};

uint16_t backgroundPixel(int x, int y) { return 10 + y * 240 + x; }

void mainScreen(TestDisplay &display) {
  display.backgrounds++;
  for (int y = 0; y < display.height; y++)
    for (int x = 0; x < display.width; x++)
      display.fillRect(x, y, 1, 1, backgroundPixel(x, y));
}

void verifyPopup(TestDisplay &display, const PopupRect &frame) {
  assert(preparePopup(display, frame, mainScreen, 0, 1));
  // Deliberately oversized contents must not paint the frame or main screen.
  display.fillRect(0, 0, display.width, display.height, 2);
  display.clearClipRect();
  for (int y = 0; y < display.height; y++) {
    for (int x = 0; x < display.width; x++) {
      bool outer = x >= frame.x - 2 && x < frame.x + frame.width + 2
          && y >= frame.y - 2 && y < frame.y + frame.height + 2;
      bool window = x >= frame.x && x < frame.x + frame.width
          && y >= frame.y && y < frame.y + frame.height;
      bool inside = x > frame.x && x < frame.x + frame.width - 1
          && y > frame.y && y < frame.y + frame.height - 1;
      uint16_t expected = inside ? 2 : window ? 1 : outer ? 0 : backgroundPixel(x, y);
      assert(display.pixels[y * display.width + x] == expected);
    }
  }
}

int main() {
  TestDisplay display;
  verifyPopup(display, {5, 5, 230, 125});      // Bars: main screen remains in the margin.
  verifyPopup(display, {110, 5, 125, 125});    // Skyplot: left-hand main screen remains visible.
  verifyPopup(display, {20, 20, 200, 95});     // Info replaces the previous popup completely.
  verifyPopup(display, {12, 12, 216, 111});    // Help.
  verifyPopup(display, {2, 2, 236, 131});      // Configuration, including screen-edge clipping.
  assert(display.backgrounds == 5);
  assert(!preparePopup(display, {0, 0, 2, 10}, mainScreen, 0, 1));
  assert(display.backgrounds == 5);
  puts("Popup background preservation, black/green borders, content clipping and switching tests passed.");
}
