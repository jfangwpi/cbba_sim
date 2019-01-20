#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <cstdint>
#include <iostream>

namespace librav{
    enum class OccupancyType
    {
        FREE,
        OCCUPIED,
        UNKNOWN,
        INTERESTED
    };

    template<typename T>
    struct value2d
    {
        value2d():x(0),y(0){}
        value2d(T _x, T _y): x(_x), y(_y){}

        T x;
        T y;
    };

    using Position2D = value2d<int32_t>;

    template<typename T>
    struct Range2D
    {
        T min;
        T max;
    };

    template<typename T>
    struct BoundingBox
    {
        Range2D<T> x;
        Range2D<T> y;
    };
}


#endif /* COMMON_TYPES_HPP */