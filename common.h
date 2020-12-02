
#ifndef COMMON_H
#define COMMON_H

#include <Tiny4kOLED_common.h>
#ifndef TINY4KOLED_H
extern SSD1306Device oled;
#endif

typedef uint8_t byte;
typedef int8_t sbyte;

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

class Duration {
public:
  unsigned long as_millis() {
    return this->_micros / 1000;
  }

  unsigned long as_micros() {
    return this->_micros;
  }
  
  bool gt(unsigned long millis) {
    return this->_micros > millis * 1000;
  }

  bool lt(unsigned long millis) {
    return this->_micros < millis * 1000;
  }

  static Duration from_millis(unsigned long millis) {
    return Duration(millis*1000);
  }
  
private:
  unsigned long _micros;
  Duration(unsigned long _micros) {
    this->_micros = _micros;
  }
  friend class Instant;
};

class Instant {
public:
  Instant() {
    this->_micros = micros();
  }
  
  Duration elapsed() {
    return Instant().elapsed_since(*this);
  }

  Duration elapsed_since(const Instant &ref) {
    return Duration(this->_micros - ref._micros);
  }

  Instant delayed_by(const Duration &offset) {
    return Instant(this->_micros + offset._micros);
  }

  bool gt(Instant &other) {
    return this->_micros > other._micros;
  }

  bool lt(Instant &other) {
    return this->_micros < other._micros;
  }
private:
  unsigned long _micros;
  Instant(unsigned long _micros) {
    this->_micros = _micros;
  }
};

#endif
