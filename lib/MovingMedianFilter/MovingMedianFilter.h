#ifndef MOVINGMEDIANFILTER_h
#define MOVINGMEDIANFILTER_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class MovingMedian
{
private:
    int windows_size = 3;
    float *values;
    float *ordered_values;

    int index_position = 0;

    float getMedian();
    void sort();

public:
    MovingMedian(int windows_size);
    virtual ~MovingMedian();

    void addValue(float value);
    float getRawValue();
    float getFiltered();
};

#endif /* MOVINGMEDIANFILTER_h */
