/**
BSD 2-Clause License

Copyright (c) 2020, Andrey Kudryavtsev (andrewkoudr@hotmail.com)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include "Math.h"
#include "Types.h"
#include <assert.h>

/**
  Conversion from/to lat/lon to/from linear coordinates in metres. Normally T MUST BE as
accurate as possible, i.e. double. Most function arguments are specified in MINUTES.
*/
template <class T> class LatLonCoord {
public:
  static_assert(sizeof(T) == sizeof(double),"Lat/Lon coordinates are accurate enough with 8-byte doubles only");

                              // constructor
  LatLonCoord(T latBase = 0.0, T lonBase = 0.0);
                              // set base
  void setBase(const T lat, const T lon);
                              // get base
  void getBase(T &lat, T &lon) const;
                              // convert lat/lon to x,y, coords in minutes
  void latLon2Linear(const T lat, const T lon, T& x, T& y) const;
  void latLon2Linear(const T lat, const T lon, float& x, float& y) const;
                              // convert x,y to lat/lon, coords in minutes
  void linear2LatLon(const T x, const T y, T& lat, T& lon) const;
                              // get approximate 1 degree width and height in metres
  void getDegreeLinearSizes(T &widthmetres, T &heightmetres);

private:
  T mLat(const T lat) const;    
  T getLatCos(const T lat) const; 
  T latit(const T mlt) const;     
  void setMileLength(const T l);   
  T getMileLength() const;
  static T rightMileLength();

  T latBase_ = 0;
  T lonBase_ = 0;
  T mileLength_ = 0;
  T centerCos_ = 0;
  T mLatBase_ = 0;
  T cosMile_ = 0;
};

template <class T> LatLonCoord<T>::LatLonCoord(T latBase, T lonBase) 
{
  setBase(latBase, lonBase);
  setMileLength(rightMileLength());
}

template <class T> void LatLonCoord<T>::getBase(T &lat, T &lon) const
{
  lat = latBase_;
  lon = lonBase_;
}

template <class T> void LatLonCoord<T>::setMileLength (const T l)   
{
  mileLength_ = l;
  cosMile_ = mileLength_ * centerCos_;
}

template <class T> T LatLonCoord<T>::getMileLength() const 
{ 
  return mileLength_;
}

template <class T> T LatLonCoord<T>::mLat(const T lat) const 
{
  return log(tan((lat / 2.0 + MI_45) * MI_RAD)) * RAD_MI;
}

template <class T> T LatLonCoord<T>::getLatCos(const T lat) const
{ 
  return cos((lat / 60.0) * CPI);
}

template <class T> T LatLonCoord<T>::latit(const T mlt) const
{
  return (atan(exp(mlt * MI_RAD)) * RAD_MI - MI_45) * T(2.0);
}

#define RIGHT_MILE_LENGTH 1851.8518519

template <class T> T LatLonCoord<T>::rightMileLength() 
{
  return RIGHT_MILE_LENGTH;
}

#undef RIGHT_MILE_LENGTH

template <class T> void LatLonCoord<T>::setBase(const T lat, const T lon) //! coords in  minutes
{
  latBase_ = lat;
  lonBase_ = lon;

  T  lat_angle = (latBase_ / T(60.0)) * CPI;
  centerCos_ = cos ( lat_angle );

  mLatBase_ = mLat(latBase_);

  cosMile_ = mileLength_ * centerCos_;
}

template <class T> void LatLonCoord<T>::latLon2Linear(const T lat, const T lon, T& x, T& y ) const
{
   T dlat = mLat(lat) - mLatBase_; 
   T dlon = lon - lonBase_;
           
   y = dlat * cosMile_;
   x = dlon * cosMile_;
}

template <class T> void LatLonCoord<T>::latLon2Linear(const T lat, const T lon, float &x, float &y) const 
{
   T dlat = mLat (lat) - mLatBase_, dlon = lon - lonBase_;
           
   y = static_cast<float>(dlat * cosMile_);
   x = static_cast<float>(dlon * cosMile_);
}

template <class T> void LatLonCoord<T>::linear2LatLon(const T x, const T y, T& lat, T& lon) const
{
  lat = latit((y / cosMile_) + mLatBase_);
  lon = (x / cosMile_) + lonBase_;
}

template <class T> void LatLonCoord<T>::getDegreeLinearSizes(T &widthmetres, T &heightmetres)
{
  T lat,lon;
  getBase(lat,lon);

  float x0,y0,x1,y1;

  latLon2Linear(lat,lon - 30.0,x0,y0);
  latLon2Linear(lat,lon + 30.0,x1,y1);
  widthmetres = x1 - x0;

  latLon2Linear(lat - 30.0,lon,x0,y0);
  latLon2Linear(lat + 30.0,lon,x1,y1);
  heightmetres = y1 - y0;
  
  assert(widthmetres > 0.0);
  assert(heightmetres > 0.0);
}

