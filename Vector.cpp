#include "defines.h"
                              // class header
#include "Vector.h"

Vector4 operator*(const double scalar, const Vector4 v)
{
  return v * static_cast<float>(scalar);
}
