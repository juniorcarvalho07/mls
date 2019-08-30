#ifndef VECTOR_HH
#define VECTOR_HH

#include <iostream>
#include <math.h>

struct Vector
{
  double x, y, z;
  
  Vector(double _x = 0, double _y = 0, double _z = 0): x(_x), y(_y), z(_z)
  {}
  
  inline double const &operator[](unsigned int i) const
  {
    return *(&x + i);
  }
  
  inline double &operator[](unsigned int i)
  {
    return *(&x + i);
  }
  
  inline Vector operator-(Vector const &v) const
  {
    return Vector(x-v.x, y-v.y, z-v.z);
  }
  
  inline Vector operator+(Vector const &v) const
  {
    return Vector(x+v.x, y+v.y, z+v.z);
  }
  
  inline Vector operator*(double s) const
  {
    return Vector(s*x, s*y, s*z);
  }
  
  inline Vector operator/(double s) const
  {
    return operator*(1/s);
  }
  
  inline Vector &operator+=(Vector const &v)
  {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }
  
  inline Vector &operator-=(Vector const &v)
  {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }
  
  inline Vector &operator*=(double d)
  {
    x *= d;
    y *= d;
    z *= d;
    return *this;
  }
  
  inline Vector &operator/=(double d)
  {
    return *this *= 1.0/d;
  }

  inline Vector &operator=(Vector const &v) 
  {
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
  }
  
  // dot product
  inline double operator*(Vector const &v) const
  {
    return v.x*x + v.y*y + v.z*z;
  }
  
  // cross product
  inline Vector operator^(Vector const &v) const
  {
    return Vector(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
  }
  
  inline double sqrnorm() const 
  {
    return *this * *this;
  }
  
  inline double norm() const
  {
    return sqrt(sqrnorm());
  }
  
  inline Vector &normalize()
  {
    double n = this->norm();
    if (n != 0)
      *this *= (1.0/n);
    
    return *this;
  }
};

inline Vector operator*(double d, Vector const &v)
{
  return v * d;
}

inline std::ostream &operator<<(std::ostream &os, Vector const &v)
{
  return os << '[' << v.x << ", " << v.y << ", " << v.z << ']';
}

#endif

