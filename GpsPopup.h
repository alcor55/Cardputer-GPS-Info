#pragma once

#include <stdint.h>

struct PopupRect {
  int x;
  int y;
  int width;
  int height;
};

/*    Compose a popup over the main screen, then clip its separately drawn
*     contents inside the green frame. The surrounding black border is 2 px.
*/
template <typename Display, typename Background>
bool preparePopup(Display &display, const PopupRect &frame, Background drawBackground,
    uint16_t black, uint16_t green) {
  if (frame.width < 3 || frame.height < 3) return false;
  display.clearClipRect();
  drawBackground(display);
  display.fillRect(frame.x - 2, frame.y - 2, frame.width + 4, frame.height + 4, black);
  display.drawRect(frame.x, frame.y, frame.width, frame.height, green);
  display.setClipRect(frame.x + 1, frame.y + 1, frame.width - 2, frame.height - 2);
  return true;
}
