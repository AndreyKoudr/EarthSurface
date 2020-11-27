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

#include "defines.h"
#include "Strings.h"
#include "HGTFiles.h"
#include "ESRIFiles.h"
#include "GeoMesh.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <Urlmon.h>

#include "zlib-win64\zlib.h"
#include "zlib-win64\contrib\minizip\unzip.h"
#include "zlib-win64\contrib\minizip\iowin32.h"
#include "zlib-win64\contrib\minizip\ioapi.h"

using namespace std;

/**
  Earth surface geometry (elevation)
  ----------------------------------
  This is an example how to load and correct any Earth surface geometry from SRTM data
and save it as an STL file.

  Compiler
  --------
  VS 2019, C++, without Windows specifics.

  Third-party
  -----------
  zip library only to unzip downloaded files.

  How it works
  ------------
  The program loads an SRTM height file with its lower left corner defined by
latitude/longitude into HGTFile class and water body data into ESRIFile class.
The first class contains Earth surface elevations with some 100x100 metres resolution,
the second contains water body contours which are  much more accurate.

  I wrote visualisation of Earth surface in marine simulators before and
ran into the problem : when a ship is in water on charts visuals may show that
it is aground. This was caused by inaccuracy of HGT data, so I had to improve it by 
correcting the data by a constrained deformation algorithmm described in the <I>Documents</I>
folder. It changes HGT file data to conform to real water shore lines.

  The HGT files are zipped and ESRI files are zipped as well (three files in a zip). A HGT
file structure is simple - it contains 1201 x 1201 elevation points in 2-byte signed integers.
An ESRI file is more compilicated - a zip contains three files. The ESRIFile class extracts
this data into groups of points organised as lat,lon and height.

  The current code <B>does not reproduce underwater geometry</B> as it would require another ETOPO1
data which is not accessible by loading files, but only via user interface. I downloaded it
before but a zipped archive is of 0.7Gb in size what makes it inappropriate to use in GitHub.

  Another thing the current sample code treats only one 1deg x 1deg tile.
  If the tile you specified (e.g. 25S 43W (-25 -43)) <B>does not contain any land</B>, the program
exits with exit code 1 and no STL file is generated.

  Anyway, I think it would be useful for anybody to freely use HGTFile and ESRIFile classes to
construct any Earth surface geometry from 60S to 60N (read SRTM docs) in case you need it.

  I tried to update the code to conform to modern C++ standards so changed it a little bit.

  Location of SRTM data
  ---------------------
  SRTM HGT files : 
  "https://dds.cr.usgs.gov/srtm/version2_1/SRTM3/"
   ... water body zip files :
  "https://dds.cr.usgs.gov/srtm/version2_1/SWBD/"

  Potential problems
  ------------------
  One possible potential problem is using URLDownloadToFile() to load data
from a secure https. I had a problem with this before, but now I have no problems
at all with downloading files to my Win10 laptop.

  Another thing : the code writes files into current directory (the source directory)
which must be writable.

  Tests
  -----
  >GeoArea -22 -43 -> OK, lots of mountains
  >GeoArea -23 -43 -> OK, Rio de Janeiro, mountains and water
  >GeoArea -24 -43 -> OK, mostly water
  >GeoArea -25 -43 -> OK, no land, exit by 1, no STL generated
*/

// Location of SRTM data
#define HGT_URL "https://dds.cr.usgs.gov/srtm/version2_1/SRTM3/"
// ... water body data_
#define SWBD_URL "https://dds.cr.usgs.gov/srtm/version2_1/SWBD/"

// issue an error message
void error(const string text)
{
  cout << endl << text.c_str() << endl << endl;
  cout << "  >GeoArea latmin lonmin" << endl << endl;
  cout << "e.g. >GeoArea -23 -43" << endl;
  cout << "loads Rio de Janeiro area" << endl << endl;
  cout << "(1) south latitude and west longitude are negative" << endl;
  cout << "(2) lat/lon values must be integer in [-60..59] and [-180..179] corresp." << endl << endl;
}

// extract file form zip
bool extractFileFromZIP(std::string zipfilename, std::string filename)
{
                              // open file
  zlib_filefunc64_def ffunc;
  fill_win32_filefunc64A(&ffunc);
  unzFile uf = unzOpen2_64(zipfilename.c_str(),&ffunc);
  void *buffer = nullptr;
  unsigned int size = 0;

  if (uf == nullptr)
    return false;
                              // load file contents
  int res = unzExtractCurrentData(uf,filename.c_str(),nullptr,&buffer,&size);
                              // failure
  if (res != 0)
  {
    if (buffer != nullptr) 
      free(buffer);
    unzClose(uf);
    return false;
  }

  if (buffer == nullptr)
  {
    unzClose(uf);
    return false;
  }
                              // success, close zip
  unzClose(uf);
                              // save buffer to file
  std::string fullfilename = justDirectory(zipfilename) + filename;

#if 1
  ofstream file(fullfilename,std::ofstream::binary | std::ofstream::out);
  if (file.is_open())
  {
    file.write((char *) buffer,size);
  } else
  {
    cout << "Unable to open file " << fullfilename << endl;
    return false;
  }
  file.close();

  free(buffer);

  return true;
#else
  FILE *file = fopen(fullfilename.c_str(),"wb");

  size_t saved = fwrite(buffer,1,size,file);
  fclose(file);
                              // this buffer done
  free(buffer);

  return (saved == size);
#endif
}

// main console
int main(int argc, char* argv[])
{
  if (argc != 3)
  {
    error("Wrong number of arguments");
    return 1;
  }
                              // parse lat/lon rectangle
  int latmin = atoi(argv[1]);
  int lonmin = atoi(argv[2]);
                              // to check starting point
  #define LAT_MIN -60
  #define LAT_MAX +59
  #define LON_MIN -180
  #define LON_MAX +179
                              // max area in squared degrees
  #define MAX_AREA 25

  if (latmin < LAT_MIN || latmin > LAT_MAX)
  {
    error("Lower corner latitude must be in range [-60..+59]");
    return 1;
  }

  if (lonmin < LON_MIN || lonmin > LON_MAX)
  {
    error("Lower corner longitude must be in range [-180..+179]");
    return 1;
  }

  int dlat = 1;
  int dlon = 1;

  if (dlat <= 0)
  {
    error("dlat (latitude size) must be positive");
    return 1;
  }
  if (dlon <= 0)
  {
    error("dlon (longitude size) must be positive");
    return 1;
  }

  int area = dlat * dlon;
  if (area > MAX_AREA)
  {
    error(string("Exercise area is too high, ") + to_string(area) + 
      " square degrees");
    return 1;
  }

  cout << "=========================================================================================" << endl;
  cout << "The program uses only data from SRTM HGT (height) and SWBD (water body) files," << endl;
  cout << "not using ETOPO1 data which contains seabed depths because it is accessible only via GUI." << endl;
  cout << "In addition, this simplified version outputs STL for 1 square degree only." << endl;
  cout << "=========================================================================================" << endl << endl;

  int latmax = latmin + dlat;
  int lonmax = lonmin + dlon;
  cout << "Lat min " << latmin << " lon min " << lonmin << " lat max " << latmax << 
    " lon max " << lonmax << endl;
                              // defaultdepth is set where depth is unknown
  float defaultdepth = -19.0;
                              // Zcoastline is Z of coast contour, set it to -4 to make rivers deeper
  float Zcoastline = -4.0;

  cout << "defaultdepth " << defaultdepth << " Z of coast line " << Zcoastline << endl << endl;
                              // HGT sub-directories
  const char *HGTdirs[6] =
  {
    "Africa",
    "Australia",
    "Eurasia",
    "Islands",
    "North_America",
    "South_America"
  };
                              // SWBD sub-directories
  const char *SWBDdirs[2] =
  {
    "SWBDeast",
    "SWBDwest"
  };

  for (int i = latmin; i < latmax; i++)
  {
    if (i > LAT_MAX)
      break;
    int ilat = i;

    for (int j = lonmin; j < lonmax; j++)
    {
      int ilon = j;
      if (ilon > LON_MAX)
        ilon -= 360;
          
      cout << "Processing square with lat " << ilat << " and lon " << ilon << endl;
                              // load SWBD (single zip file with three files inside)
      string swbdname;
      string fullswbdname;
      bool swbdfound = false;
      string swbdurl;

      for (int i = 0; i < CD_TOTAL; i++)
      {
        swbdname = LatLonToESRIFilename(static_cast<double>(ilat),static_cast<double>(ilon),i) + ".zip";
        fullswbdname = swbdname;
        for (int swbdsubdir = 0; swbdsubdir < 2; swbdsubdir++)
        {
          swbdurl = string(SWBD_URL) + SWBDdirs[swbdsubdir] + "/" + swbdname;
          HRESULT res = URLDownloadToFileA(nullptr,swbdurl.c_str(),fullswbdname.c_str(),0,nullptr);
          if (res == 0)
          {
            swbdfound = true;
            break;
          }
        }
        if (swbdfound)
          break;
      }

      if (swbdfound)
      {
        cout << "File from " << swbdurl << " downloaded into file " << fullswbdname << endl;
      } else
      {
        cout << "Failed to download file from " << swbdurl << endl;
      }
                              // HGT
      string hgtname = LatLonToHGTFilename(static_cast<double>(ilat),static_cast<double>(ilon)) + ".hgt.zip";
      string shorthgtname = LatLonToHGTFilename(static_cast<double>(ilat),static_cast<double>(ilon)) + ".hgt";
                              // write to current directory; it must be writable
      string fullhgtname = hgtname;
      bool hgtfound = false;
      string hgturl;
      for (int hgtsubdir = 0; hgtsubdir < 6; hgtsubdir++)
      {
        hgturl = string(HGT_URL) + HGTdirs[hgtsubdir] + "/" + hgtname;
        HRESULT res = URLDownloadToFileA(nullptr,hgturl.c_str(),fullhgtname.c_str(),0,nullptr);
        if (res == 0)
        {
          hgtfound = true;
          break;
        }
      }
  
      if (hgtfound)
      {
        cout << "File from " << hgturl << " downloaded into file " << fullhgtname << endl;
                                // now we need to unpack file
        if (!extractFileFromZIP(fullhgtname,shorthgtname))
        {
          error(string("Unable to extract file ") + shorthgtname + " from " + fullhgtname);
          return 1;
        }
        cout << "File " << shorthgtname << " extracted from " << fullhgtname << endl;
      } else
      {
        cout << "Failed to download file from " << hgturl << endl;
        return 1;
      }
                              // add new file
      unique_ptr<ESRIFile> waterbody;
      if (swbdfound)
      {
        waterbody = unique_ptr<ESRIFile>(new ESRIFile(fullswbdname.c_str()));
      }

      cout  << endl << "Correcting height data by water body contours..." << endl;

      HGTFile hgt(shorthgtname,waterbody.get(),defaultdepth,Zcoastline);

      GeoMesh mesh(&hgt.data[0],hgt.latmin,hgt.lonmin,hgt.latmax,
        hgt.lonmax,HGTFile::SIZE - 1,HGTFile::SIZE - 1);

      TTriangles<float> tris;
      mesh.toTriangles(tris);

      string STLfilename = shorthgtname + ".STL";

      cout << endl <<  "Saving " << STLfilename << " file..." << endl;

      tris.saveSTL(STLfilename,shorthgtname + " elevation (no depths)",true);

      break;
    }
  }

  cout << endl <<  "Press [ENTER]" << endl;
  getchar();

	return 0;
}
