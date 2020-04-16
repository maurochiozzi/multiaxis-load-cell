#include <Arduino.h>
#include <MovingMedianFilter.h>

MovingMedianFilter::MovingMedianFilter(int _windows_size)
{
    windows_size = _windows_size;

    values = (float *)malloc(windows_size * sizeof(float));
    ordered_values = (float *)malloc(windows_size * sizeof(float));

    for (int i = 0; i < windows_size; i++)
    {
        values[i] = 0;
        ordered_values[i] = 0;
    }
}

MovingMedianFilter::~MovingMedianFilter()
{
    free(values);
    free(ordered_values);
}

void MovingMedianFilter::addValue(float value)
{
    values[index_position] = value;

    index_position = (index_position + 1) % windows_size;
}

float MovingMedianFilter::getRawValue()
{
    return values[index_position]; // Nope. Looks good, works terrible
}

void MovingMedianFilter::sort()
{
    float key;
    int i, j;
    for (i = 1; i < windows_size; i++)
    {
        key = ordered_values[i];
        j = i - 1;

        while (j >= 0 && ordered_values[j] > key)
        {
            ordered_values[j + 1] = ordered_values[j];
            j = j - 1;
        }
        ordered_values[j + 1] = key;
    }
}

float MovingMedianFilter::getMedian()
{
    memcpy(ordered_values, values, sizeof(float) * windows_size);
    sort();

    int middle = (windows_size / 2);

    if ((windows_size % 2))
    {
        return ordered_values[middle];
    }

    return ((ordered_values[middle - 1] + ordered_values[middle]) /
            2);
}

float MovingMedianFilter::getFiltered()
{
    return getMedian();
}