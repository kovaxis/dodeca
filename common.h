
#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>

typedef uint8_t byte;

template <typename T>
class Vec3 {
   public:
    unsigned long magsq() const {
        return ((unsigned long)this->x) * this->x +
               ((unsigned long)this->y) * this->y +
               ((unsigned long)this->z) * this->z;
    }

    unsigned long distsq(const Vec3 &other) const {
        Vec3 tmp = *this;
        tmp.sub_mut(other);
        return tmp.magsq();
    }

    long dot(const Vec3 &other) const {
        return ((long)this->x) * other.x + ((long)this->y) * other.y +
               ((long)this->z) * other.z;
    }

    void add_mut(const Vec3 &other) {
        this->x += other.x;
        this->y += other.y;
        this->z += other.z;
    }

    void sub_mut(const Vec3 &other) {
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

static byte reverse_byte(byte b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

enum Orient {
    ORIENT_000,
    ORIENT_045,
    ORIENT_090,
    ORIENT_135,
    ORIENT_180,
    ORIENT_225,
    ORIENT_270,
    ORIENT_315,
};

enum BatStatus {
    BAT_CHARGING,
    BAT_CHARGED,
    BAT_LOW,
    BAT_NOT_CHARGING,
};

enum PowerStatus {
    POWER_ON,
    POWER_SHUTDOWN,
    POWER_OFF,
    POWER_STARTUP,
};

// A single frequency-duration pair.
//
// A frequency of 0 indicates no sound.
// A frequency of 0 with duration 0 indicates the end of a tone sequence.
// A non-zero frequency with duration 0 indicates that the tone sequence should
// loop.
struct Tone {
    unsigned int freq;
    unsigned int dur;
};

typedef struct {
    // Dimensions of the sprite.
    byte width;
    byte height;
    // Default position of the sprite.
    byte default_x;
    byte default_y;
    // Pointer to sprite data in PROGMEM
    const byte *data;
} Sprite;

#include "config.h"
#include "timekeep.h"

#endif
