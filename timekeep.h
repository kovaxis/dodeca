
#ifndef TIMEKEEP_H
#define TIMEKEEP_H

extern unsigned int _millis_counter;

void timekeep_init();

class Duration
{
public:
    signed long as_millis()
    {
        return this->_millis;
    }

    signed long as_micros()
    {
        return this->_millis * 1000l;
    }

    bool gt(signed int millis)
    {
        return this->_millis >= millis;
    }

    bool lt(signed int millis)
    {
        return this->_millis <= millis;
    }

    static Duration from_millis(signed int millis)
    {
        return Duration(millis);
    }

private:
    signed int _millis;
    Duration(signed int _millis)
    {
        this->_millis = _millis;
    }
    friend class Instant;
};

class Instant
{
public:
    Instant()
    {
        this->_millis = _millis_counter;
    }

    Duration elapsed()
    {
        return Instant().elapsed_since(*this);
    }

    Duration elapsed_since(const Instant &ref)
    {
        return Duration(this->_millis - ref._millis);
    }

    Instant delayed_by(const Duration &offset)
    {
        return Instant(this->_millis + offset._millis);
    }

    bool gt(Instant &other)
    {
        return ((signed int)(this->_millis - other._millis)) > 0;
    }

    bool lt(Instant &other)
    {
        return ((signed int)(this->_millis - other._millis)) < 0;
    }

private:
    unsigned int _millis;
    Instant(unsigned int _millis)
    {
        this->_millis = _millis;
    }
};

#endif