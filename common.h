
#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>

// Whether to include and communicate through serial
//#define USE_SERIAL

#include <Tiny4kOLED_common.h>
#ifndef TINY4KOLED_H
extern SSD1306Device oled;
#endif

#include "timekeep.h"

typedef uint8_t byte;

template<typename T>
class Vec3 {
public:
  unsigned long magsq() const {
    return ((unsigned long) this->x) * this->x + ((unsigned long) this->y) * this->y + ((unsigned long) this->z) * this-> z;
  }

  long dot(const Vec3& other) const {
    return ((long) this->x) * other.x + ((long) this->y) * other.y + ((long) this->z) * other.z;
  }

  void add_mut(const Vec3& other) {
    this->x += other.x;
    this->y += other.y;
    this->z += other.z;
  }
  
  void sub_mut(const Vec3& other) {
    this->x -= other.x;
    this->y -= other.y;
    this->z -= other.z;
  }

  void mul_mut(T rhs) {
    this->x *= rhs;
    this->y *= rhs;
    this->z *= rhs;
  }

  void div_mut(T rhs) {
    this->x /= rhs;
    this->y /= rhs;
    this->z /= rhs;
  }

  Vec3<long> to_wide() const {
    Vec3<long> wide;
    wide.x = this->x;
    wide.y = this->y;
    wide.z = this->z;
    return wide;
  }

  Vec3<int> to_narrow() const {
    Vec3<int> narrow;
    narrow.x = this->x;
    narrow.y = this->y;
    narrow.z = this->z;
    return narrow;
  }
  
  T x;
  T y;
  T z;
};

void select_screen(int screen);

static byte reverse_byte(byte b){
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

#endif
