#pragma once

// Minimal Arduino compatibility layer for testing the unmodified TinyGPS++ parser.
#include <stdint.h>
#include <cmath>

using byte = uint8_t;
unsigned long millis();
inline double radians(double value) { return value * 3.14159265358979323846 / 180; }
inline double degrees(double value) { return value * 180 / 3.14159265358979323846; }
inline double sq(double value) { return value * value; }
#define TWO_PI 6.28318530717958647692
