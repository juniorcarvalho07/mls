
/*****
*This file is part of hash3: https://github.com/sgh1/hash3
*Please see the license and readme!
*****/

#ifndef HASH3_INT3_H
#define HASH3_INT3_H

#include <iostream>

namespace hash3
{

/**
 * int3
 * minimal implementation of 3d index that
 * we need to help work with bins of the hash
 */

template <typename T>
class int3
{
public:

    int3( T i, T j, T k):
        x(i),y(j),z(k)
    {
    }

    int3(const int3& src):
        x(src.x),y(src.y),z(src.z)
    {
    }

    int3():
        x(0),y(0),z(0)
    {
    }

    bool operator==(const int3& other) const
    {
        return other.x == x &&
            other.y == y && other.z == z;
    }

    bool operator!=(const int3& other) const
    {
        return !(*this == other);
    }

    bool any_greater_than(const int3& other) const
    {
        return x > other.x ||
            y > other.y || z > other.z;
    }

    int3 operator-(const int3& other) const
    {
        return int3(    x-other.x,
                        y-other.y,
                        z-other.z   );
    }

    int3 max(const int3& other) const
    {
        return int3(    std::max(x,other.x),
                        std::max(y,other.y),
                        std::max(z,other.z) );
    }

    int3 min(const int3& other) const
    {
        return int3(    std::min(x,other.x),
                        std::min(y,other.y),
                        std::min(z,other.z) );
    }

    //for std::map, we need a sorting order
    //(or we can use unordered_map)
    bool operator<(const int3& other) const
    {
        if(x < other.x)
            return true;
        else if(x == other.x && y < other.y)
            return true;
        else if(x == other.x && y == other.y && z < other.z)
            return true;

        return false;
    }

    T x,y,z;

};

template <typename T>
std::ostream& operator<<(std::ostream& os, const int3<T>& i3)
{
    os << "(" << i3.x << "," << i3.y << "," << i3.z << ")";
    return os;
}

}


#endif /* INT3_H */