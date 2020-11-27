#include <string>
#include <assert.h>
#include "Types.h"
#include "LatLon.h"
#include "Strings.h"

using namespace std;

void ParseLatitude(double lat, int *deg, int *min, int *sec, double *secfraction, char *ch)
{
                              // sign
  *ch = (lat >= 0) ? 'N' : 'S';

  lat = std::abs(lat);
                              // degrees
  *deg = (int) lat;
                              // minutes
  lat = (lat - (double) (*deg)) * 60.0;
  *min = (int) lat;
                              // seconds
  lat = (lat - (double) (*min)) * 60.0;
  *sec = (int) lat;
                              // fractional part of seconds
  *secfraction = lat - (double) (*sec);
}

void ParseLatitude(double lat, int *deg, int *min, int *sec, char *ch)
{
                              // sign
  *ch = (lat >= 0) ? 'N' : 'S';

  lat = std::abs(lat);
                              // degrees
  *deg = (int) lat;
                              // minutes
  lat = (lat - (double) (*deg)) * 60.0;
  *min = (int) lat;
                              // seconds
  lat = (lat - (double) (*min)) * 60.0;
                              // ROUND
  *sec = ROUND(lat);
  if (*sec == 60)
  {
    *sec = 0; (*min)++;
    if (*min == 60)
    {
      *min = 0; (*deg)++;
    }
  }

}

void ParseLatitude(double lat, int *deg, int *min, double *minfraction, char *ch)
{
                              // sign
  *ch = (lat >= 0) ? 'N' : 'S';

  lat = std::abs(lat);
                              // degrees
  *deg = (int) lat;
                              // minutes
  lat = (lat - (double) (*deg)) * 60.0;
  *min = (int) lat;
                              // fractional part of minutes
  *minfraction = lat - (double) (*min);
}

void ParseLatitude(double lat, int *deg, int *min, char *ch)
{
                              // sign
  *ch = (lat >= 0) ? 'N' : 'S';

  lat = std::abs(lat);
                              // degrees
  *deg = (int) lat;
                              // minutes
  lat = (lat - (double) (*deg)) * 60.0;
                              // ROUND
  *min = ROUND(lat);
  if (*min == 60)
  {
    *min = 0; (*deg)++;
  }
}

void ParseLongitude(double lon, int *deg, int *min, int *sec, double *secfraction, char *ch)
{
                              // sign
  *ch = (lon >= 0) ? 'E' : 'W';

  lon = std::abs(lon);
                              // degrees
  *deg = (int) lon;
                              // minutes
  lon = (lon - (double) (*deg)) * 60.0;
  *min = (int) lon;
                              // seconds
  lon = (lon - (double) (*min)) * 60.0;
  *sec = (int) lon;
                              // fractional part of seconds
  *secfraction = lon - (double) (*sec);
}

void ParseLongitude(double lon, int *deg, int *min, int *sec, char *ch)
{
                              // sign
  *ch = (lon >= 0) ? 'E' : 'W';

  lon = std::abs(lon);
                              // degrees
  *deg = (int) lon;
                              // minutes
  lon = (lon - (double) (*deg)) * 60.0;
  *min = (int) lon;
                              // seconds
  lon = (lon - (double) (*min)) * 60.0;
                              // ROUND
  *sec = ROUND(lon);
  if (*sec == 60)
  {
    *sec = 0; (*min)++;
    if (*min == 60)
    {
      *min = 0; (*deg)++;
    }
  }
}

void ParseLongitude(double lon, int *deg, int *min, double *minfraction, char *ch)
{
                              // sign
  *ch = (lon >= 0) ? 'E' : 'W';

  lon = std::abs(lon);
                              // degrees
  *deg = (int) lon;
                              // minutes
  lon = (lon - (double) (*deg)) * 60.0;
  *min = (int) lon;
                              // fractional part of minutes
  *minfraction = lon - (double) (*min);
}

void ParseLongitude(double lon, int *deg, int *min, char *ch)
{
                              // sign
  *ch = (lon >= 0) ? 'E' : 'W';

  lon = std::abs(lon);
                              // degrees
  *deg = (int) lon;
                              // minutes
  lon = (lon - (double) (*deg)) * 60.0;

  *min = ROUND(lon);
  if (*min == 60)
  {
    *min = 0; (*deg)++;
  }
}

std::string LatString(double lat, int mode, int numdigits, int numdigitsafterdot)
{
#define DEG_LEN1 2

	if (lat == LATLON_ILLEGAL)
	{
		return std::string("99") + DEGREE_CHAR_ASCII + "99";
	}
															// trim lon value
	while (lat < -90.0) lat += 180.0;
	while (lat >= 90.0) lat -= 180.0;

  std::string s,sd,sm,ss;
  int deg,min,sec; double fraction; char ch;

  switch (mode) {

  case LLSTRING_DEGFRACTION :
		s = to_string(std::abs(lat),numdigits,numdigitsafterdot) + ((lat >= 0) ? 'N' : 'S');
    break;

  case LLSTRING_DEGMIN :
    ParseLatitude(lat,&deg,&min,&ch);
    sd = to_string(deg,DEG_LEN1); 
    sm = to_string(min,2); 
    s = sd + DEGREE_CHAR_ASCII + sm + std::string("'") + ((lat >= 0) ? 'N' : 'S');
    break;

  case LLSTRING_DEGMINFRACTION :
    ParseLatitude(lat,&deg,&min,&fraction,&ch);
    fraction += min;
    sd = to_string(deg,DEG_LEN1); 
    sm = to_string(fraction,numdigits,numdigitsafterdot); 
    sm = replace(sm,' ','0');
    s = sd + DEGREE_CHAR_ASCII + sm + std::string("'") + ((lat >= 0) ? 'N' : 'S');
    break;

  case LLSTRING_DEGMINSEC :
    ParseLatitude(lat,&deg,&min,&sec,&ch);
    sd = to_string(deg,DEG_LEN1);
    sm = to_string(min,2);
    ss = to_string(sec,2);
    s = sd + DEGREE_CHAR_ASCII + sm + std::string("'") + ss + std::string("''") + ((lat >= 0) ? 'N' : 'S');
    break;

  case LLSTRING_DEGMINSECFRACTION :
    ParseLatitude(lat,&deg,&min,&sec,&fraction,&ch);
    sd = to_string(deg,DEG_LEN1); 
    sm = to_string(min,2); 
    fraction += sec;
    ss = to_string(fraction,numdigits,numdigitsafterdot); 
    ss = replace(ss,' ','0');
    s = sd + DEGREE_CHAR_ASCII + sm + std::string("'") + ss + std::string("''") + ((lat >= 0) ? 'N' : 'S');
    break;

  default :
    break;
  }

#undef DEG_LEN1

  return s;
}

std::string LonString(double lon, int mode, int numdigits, int numdigitsafterdot)
{
#define DEG_LEN2 3

	if (lon == LATLON_ILLEGAL)
	{
		return std::string("999") + DEGREE_CHAR_ASCII + "99";
	}
															// trim lon value
	while (lon < -180.0) lon += 360.0;
	while (lon >= 180.0) lon -= 360.0;

  std::string s,sd,sm,ss;
  int deg,min,sec; double fraction; char ch;

  switch (mode) {

  case LLSTRING_DEGFRACTION :
    s = to_string(std::abs(lon),numdigits,numdigitsafterdot) + ((lon >= 0) ? 'E' : 'W');
    break;

  case LLSTRING_DEGMIN :
    ParseLongitude(lon,&deg,&min,&ch);
    sd = to_string(deg,DEG_LEN2);
    sm = to_string(min,2);
    s = sd + DEGREE_CHAR_ASCII + sm + std::string("'") + ((lon >= 0) ? 'E' : 'W');
    break;

  case LLSTRING_DEGMINFRACTION :
    ParseLongitude(lon,&deg,&min,&fraction,&ch);
    fraction += min;
    sd = to_string(deg,3); 
    sm = to_string(fraction,numdigits,numdigitsafterdot); 
    sm = replace(sm,' ','0');
    s = sd + DEGREE_CHAR_ASCII + sm + std::string("'") + ((lon >= 0) ? 'E' : 'W');
    break;

  case LLSTRING_DEGMINSEC :
    ParseLongitude(lon,&deg,&min,&sec,&ch);
    sd = to_string(deg,DEG_LEN2);
    sm = to_string(min,2);
    ss = to_string(sec,2);
    s = sd + DEGREE_CHAR_ASCII + sm + std::string("'") + ss + std::string("''") + ((lon >= 0) ? 'E' : 'W');
    break;

  case LLSTRING_DEGMINSECFRACTION :
    ParseLongitude(lon,&deg,&min,&sec,&fraction,&ch);
    sd = to_string(deg,DEG_LEN2);
    sm = std::string(min,2);
    fraction += sec;
    ss = to_string(fraction,numdigits,numdigitsafterdot); 
    ss = replace(ss,' ','0');
    s = sd + DEGREE_CHAR_ASCII + sm + std::string("'") + ss + std::string("''") + ((lon >= 0) ? 'E' : 'W');
    break;

  default :
    break;
  }

#undef DEG_LEN2

  return s;
}

bool CorrectLat(double &value)
{
  bool corrected = false;

  if (value > 90.0)
  {
    value = 90.0;
    corrected = true;
  }

  if (value < -90.0)
  {
    value = -90.0;
    corrected = true;
  }

  assert(value >= -90.0 && value <= +90.0);

  return corrected;
}

bool CorrectLon(double &value)
{
  bool corrected = false;

  if (value > 180.0)
  {
    value = value - 360.0;
    corrected = true;
  }

  if (value < -180.0)
  {
    value = 360.0 + value;
    corrected = true;
  }

  assert(value >= -180.0 && value <= +180.0);

  return corrected;
}

LatLonRect::LatLonRect()
{
	latmin = lonmin = latmax = lonmax = 0;
  geocoord = nullptr;
}

LatLonRect::LatLonRect(double platmin, double plonmin, double platmax, double plonmax)
{
	latmin = platmin;
	lonmin = plonmin;
	latmax = platmax;
	lonmax = plonmax;
  trim();
  geocoord = nullptr;
}

LatLonRect::LatLonRect(double platmin, double plonmin, double platmax, double plonmax, 
  LatLonCoord<double> *pgeocoord)
{
	latmin = platmin;
	lonmin = plonmin;
	latmax = platmax;
	lonmax = plonmax;
  trim();
  geocoord = pgeocoord;
}

LatLonRect::~LatLonRect()
{
}

void LatLonRect::trim()
{
	if (latmin > latmax) 
    SWAP(double,latmin,latmax);
	if (lonmin > lonmax) 
    SWAP(double,lonmin,lonmax);
}

bool LatLonRect::XYToLatLon(double x, double y, double &lat, double &lon) const
{
  assert(geocoord != nullptr);
  if (geocoord == nullptr)
    return false;

  double la,lo;
  geocoord->linear2LatLon(x,y,la,lo);
  lat = la / 60.0;
  lon = lo / 60.0;
  return true;
}

bool LatLonRect::LatLonToXY(double lat, double lon, double &x, double &y) const
{
  assert(geocoord != nullptr);
  if (geocoord == nullptr)
    return false;

  geocoord->latLon2Linear(lat * 60.0,lon * 60.0,x,y);
  return true;
}

bool LatLonRect::LatLonToXY(double lat, double lon, float &x, float &y) const
{
  assert(geocoord != nullptr);
  if (geocoord == nullptr)
    return false;

  geocoord->latLon2Linear(lat * 60.0,lon * 60.0,x,y);
  return true;
}

//bool LatLonRect::XYToLatLon(float x, float y, double &lat, double &lon)
//{
//  assert(geocoord != nullptr);
//  if (geocoord == nullptr)
//    return false;
//
//  double la,lo;
//  geocoord->linear2LatLon(x,y,la,lo);
//  lat = la / 60.0;
//  lon = lo / 60.0;
//  return true;
//}

bool LatLonRect::overlap(LatLonRect &rect) const
{
	return !((rect.latmin > latmax) || (rect.latmax < latmin) || 
		(rect.lonmin > lonmax) || (rect.lonmax < lonmin));
}

bool LatLonRect::overlap(SLatLonRect &rect) const
{
	return !((rect.latmin > latmax) || (rect.latmax < latmin) || 
		(rect.lonmin > lonmax) || (rect.lonmax < lonmin));
}

bool LatLonRect::inside(LatLonRect &rect, double tolerance) const
{
	return ((rect.latmin >= latmin - tolerance) && (rect.latmax < latmax + tolerance) &&
		(rect.lonmin >= lonmin - tolerance) && (rect.lonmax < lonmax + tolerance));
}

bool LatLonRect::inside(double lat, double lon) const
{
	return ((lat >= latmin) && (lat <= latmax) && (lon >= lonmin) && (lon <= lonmax));
}

void LatLonRect::getCentre(double &lat, double &lon) const
{
  lat = (latmin + latmax) * 0.5;
  lon = (lonmin + lonmax) * 0.5;
}

std::string LatLonRect::fileName(const std::string &dir, const std::string &extension) const
{
  int ilatmin = ROUND(latmin);
  int ilonmin = ROUND(lonmin);
  int ilatmax = ROUND(latmax);
  int ilonmax = ROUND(lonmax);
  int idlat = ilatmax - ilatmin;
  int idlon = ilonmax - ilonmin;

  std::string name = ((ilatmin < 0) ? "S" : "N") + to_string(abs(ilatmin),2) + 
    ((ilonmin < 0) ? "W" : "E") + to_string(abs(ilonmin),3) + 
    to_string(abs(idlon),3) + to_string(abs(idlat),3);

  if (dir.length())
    name = addBackslash(dir) + name;

  if (extension.length())
    name = name + "." + extension;

  return name;
}

