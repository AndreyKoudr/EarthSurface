#pragma once

// LINT is always 64-bit
#define LINT long long int

// undefined
#define NOT_DEFINED -1
#define NOT_FOUND NOT_DEFINED

// useful macros
#define LIMIT(x,xmin,xmax) if (x < xmin) x = xmin; if (x > xmax) x = xmax
#define LIMIT_MIN(x,xmin) if (x < xmin) x = xmin
#define LIMIT_MAX(x,xmax) if (x > xmax) x = xmax

// tolerance
#define TOLERANCE(T) std::numeric_limits<T>::epsilon() * static_cast<T>(10.0)
#define TOLERANCE4 TOLERANCE(float)
#define TOLERANCE8 TOLERANCE(double)

// round
#define ROUND(x)  (int) (floor(x + .5))

//===== PI - associated ========================================================
#ifndef M_PI
  #define M_PI    3.14159265358979323846
#endif

#ifndef PI05
  #define PI05 (M_PI * 0.5)
#endif

#ifndef PI20
  #define PI20 (M_PI * 2.0)
#endif

#ifndef PI10
  #define PI10 M_PI
#endif

#ifndef PCI
  #define PCI (180.0 / M_PI)
#endif

#ifndef CPI
  #define CPI (M_PI / 180.0)
#endif

#ifndef MI_RAD
  #define MI_RAD  (M_PI / 10800.)
#endif

#ifndef RAD_MI
  #define RAD_MI  (10800.0 / M_PI)
#endif

#ifndef MI_45
  #define	MI_45	  2700.0
#endif

// swap
#define SWAP(T,x1,x2) { T temp = x1; x1 = x2; x2 = temp; }

