#ifndef MOVINGMEDIANFILTER_h
#define MOVINGMEDIANFILTER_h

class MovingMedian
{
private:
    int windows_size = 3;
    int middle;
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
