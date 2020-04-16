#ifndef HX711_h
#define HX711_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class MovingMedianFilter
{
private:
    int windows_size = 3;
    float *values;
    float *ordered_values;

    int index_position = 0;

    float getMedian();
    void sort();

public:
    MovingMedianFilter(int windows_size);
    virtual ~MovingMedianFilter();

    void addValue(float value);
    float getRawValue();
    float getFiltered();
};

#endif /* HX711_h */
