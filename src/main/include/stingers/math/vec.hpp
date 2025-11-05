/**
*   Geometric vector declarations for FRC #8193's FRC 2026 season chassis.
*
*   Copyright (C) 2025 Frederick Ziola, et al. (New Lothrop Robotics)
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.

*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.

*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
*   Contact us: robotics@newlothrop.k12.mi.us
*/

#pragma once

#include <cmath>

namespace stingers::math {

struct Vec2 {
  float x, y;

  inline float length() const { return sqrtf(x*x+y*y); }

  inline Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  inline Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  inline Vec2 operator*(float f) const { return {x * f, y * f}; }
  inline Vec2 operator/(float f) const { return {x / f, y / f}; }
  inline Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
  inline Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
  inline Vec2& operator*=(const float& o) { x *= o; y *= o; return *this; }

  inline static float dot(const Vec2& a, const Vec2& b) { return a.x*b.x+a.y*b.y; }
  inline static float cross(const Vec2& a, const Vec2& b) { return a.x*b.y-a.y*b.x; }
};

inline Vec2 operator*(float a, const Vec2& b) {
  return b*a;
}
}
