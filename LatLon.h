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

#include "LatLonCoord.h"
#include <vector>
#include <string>

//===== Parsing values into deg/min/sec etc. ===================================

															// illegal value, always showing 99.999
#define LATLON_ILLEGAL 1000.0
                              // parse latitude
extern void ParseLatitude(double lat, int *deg, int *min, int *sec, double *secfraction, char *ch);
extern void ParseLatitude(double lat, int *deg, int *min, double *minfraction, char *ch);
extern void ParseLatitude(double lon, int *deg, int *min, char *ch);
extern void ParseLatitude(double lat, int *deg, int *min, int *sec, char *ch);
                              // parse longitude
extern void ParseLongitude(double lon, int *deg, int *min, int *sec, double *secfraction, char *ch);
extern void ParseLongitude(double lon, int *deg, int *min, int *sec, char *ch);
extern void ParseLongitude(double lon, int *deg, int *min, char *ch);
extern void ParseLongitude(double lon, int *deg, int *min, double *minfraction, char *ch);


//===== lat/lon strings ========================================================

                              // how to output lat/lon string 
enum {
  LLSTRING_DEGFRACTION,
  LLSTRING_DEGMIN,
  LLSTRING_DEGMINFRACTION,
  LLSTRING_DEGMINSEC,
  LLSTRING_DEGMINSECFRACTION   
};

                              // degree character
#define DEGREE_CHARPRINTER ((char) 248)
#define DEGREE_CHAR_ASCII ((char) 176)

                              // coordinate string, set numdigits as numdigitsafterdot + 3
extern std::string LatString(double lat, int mode, int numdigits, int numdigitsafterdot);
                              // coordinate string, set numdigits as numdigitsafterdot + 3
extern std::string LonString(double lon, int mode, int numdigits, int numdigitsafterdot);


//===== Correct periodic lat/lon values ========================================

                              // correct lat value to bring it to -90..+90;
                              // returns true is corrected
bool CorrectLat(double &value);
                              // correct lon value to bring it to -180..+180;
                              // returns true is corrected
bool CorrectLon(double &value);

//===== rectangle with lat,lon coordinates =====================================

typedef struct SLatLonRect {
	double latmin = 0;
	double lonmin = 0;
	double latmax = 0;
	double lonmax = 0;
} SLatLonRect;

/**
  This class represents conversion of lat/lon coordinates in degrees to/from metres.
  Latitude coordinates : 'S' is negative; for longitude coordinates 'W' is negative.
*/
class LatLonRect : public SLatLonRect {
private:
															// trim min/max
	void trim();

protected:
                              // to convert from/to linear coordinates;
                              // this is just a pointer owned by the whole area, do not
                              // deallocate
  LatLonCoord<double> *geocoord;

public:
                              // constructors
	LatLonRect();
	LatLonRect(double platmin, double plonmin, double platmax, double plonmax);
	LatLonRect(double platmin, double plonmin, double platmax, double plonmax, LatLonCoord<double> *pgeocoord);
                              // destructor
	~LatLonRect();
                              // suggested filename, like S23W043008004.etp,
                              // which means "square 8(lon) by 4(lat) degrees with 
                              // lower left corner at S23 W043"
                              // it is supposed here that LatLonRect boundaries lie
                              // on whole degree lines
                              // dir or extension can be ""
  std::string fileName(const std::string &dir, const std::string &extension) const;
                              // geocoord pointer should be not null for these
                              // transformations
                              // lat/lon to linear; geocoord should be inited
  bool LatLonToXY(double lat, double lon, double &x, double &y) const;
  bool LatLonToXY(double lat, double lon, float &x, float &y) const;
                              // linear to lat/lon; geocoord should be inited
  bool XYToLatLon(double x, double y, double &lat, double &lon) const;
//  bool XYToLatLon(float x, float y, double &lat, double &lon);

															// rectangles overlap?
	bool overlap(LatLonRect &rect) const;
															// rectangles overlap?
	bool overlap(SLatLonRect &rect) const;
															// rect inside this?
	bool inside(LatLonRect &rect, double tolerance = 0.0) const;
															// inside?
	bool inside(double lat, double lon) const;
                              // get central point
  void getCentre(double &lat, double &lon) const;
};

