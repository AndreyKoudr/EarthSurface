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

#include <vector>
#include <string>
#include "Vector.h"
#include "LatLon.h"

using namespace std;

#pragma pack(push,1)

//===== ESRI shapefiles ========================================================
// https://dds.cr.usgs.gov/srtm/version2_1/SWBD/
// used to represent water body data. Each SWBD zip file contains three files :
// .shp, .shx and .dbf. This file decribes a cell as HxxxVyyc, where Hxxx is 
// longtude (H is E or W), Vyy - latitude (V is N or S) and c (continental delivery)
// can be   
// a - Australia
// e - Eurasia
// f - Africa
// i - Islands
// n - North America          
// s - South America    
//
// All integers in ESRI file are 32-bit, all floats are doubles (8 byte)                    

                              // continental delivery
enum {
  CD_AUSTRALIA,
  CD_EURASIA,
  CD_AFRICA,
  CD_ISLANDS,
  CD_NAMERICA,
  CD_SAMERICA,

  CD_TOTAL
};
                              // characters in the end of ESRI file name
extern const char ESRIContDeliveryChars[CD_TOTAL];

                              // parse SWBD file name, like e054n39e.zip
bool ESRIFilenameToLatLon(const string &filename, double *lat, double *lon, 
  int *contdelivery, string &errorString);
                              // convert ESRI filename like e054n39e.zip to lat/lon min/max
bool ESRIFilenameToRect(const string &filename, SLatLonRect *rect, string &errorString);
                              // convert lat/lon (lower left corner) to ESRI file name
                              // without extension, like e054n39e.zip
string LatLonToESRIFilename(double lat, double lon, int contdelivery);

                             // change endianness for an array of size 32-bit integers
void BSwap32(size_t size, void *addr);
                              // data types in ESRI file
typedef int ESRIint;
typedef double ESRIfloat;

                              // ESRI file
class ESRIFile : public LatLonRect {

protected:
                              // extract SHP data
  bool ExtractSHP(char *data, int size);
                              // extract SHX data
  bool ExtractSHX(char *data, int size);
                              //NB extract DBF data; DBF header contains only one
                              // field descriptor, which is NOT usually the case
  bool ExtractDBF(char *data, int size);

public:
                                // record type, taken from DBF
  enum {
    SWBD_OCEAN,
    SWBD_LAKE,
    SWBD_RIVER,

    SWBD_TOTAL
  };
                                // ESRI shape types
  enum {
    ESRI_NULLSHAPE = 0,
    ESRI_POINT = 1,
    ESRI_POLYLINE = 3,
                                // filled part is to the right of moving
                                // along points
    ESRI_POLYGON = 5,
    ESRI_MULTIPOINT = 8,
    ESRI_POINTZ = 11,
    ESRI_POLYLINEZ = 13,
    ESRI_POLYGONZ = 15,
    ESRI_MULTIPOINTZ = 18,
    ESRI_POINTM = 21,
    ESRI_POLYLINEM = 23,
    ESRI_POLYGONM = 25,
    ESRI_MULTIPOINTM = 28,
    ESRI_MULTIPATCH = 31,

    ESRI_SHAPETOTAL
  };
                                // 100-byte header of SHP file
  typedef struct SSHPHeader {
                                // must be 9994 (bigendian)
    ESRIint filecode;
                                // 5 unused ints
    ESRIint unused[5];
                                // data length in 16-bit words, including 50 words of header, bigendian
    ESRIint filelength;
                                // must be 1000 little endian
    ESRIint version;
                                // all the rest are little endian
    ESRIint shapetype;
                                // bounding box (little endian)
    ESRIfloat xmin;
    ESRIfloat ymin;
    ESRIfloat xmax;
    ESRIfloat ymax;
    ESRIfloat zmin;
    ESRIfloat zmax;
    ESRIfloat mmin;
    ESRIfloat mmax;
                                // cast big endian into little endian
    void CorrectEndianness()
    {
      BSwap32(7,this); 
    };
                                // shape type OK?
    bool ShapeTypeOK()
    {
      return (shapetype == 0 || shapetype == 1 || shapetype == 3 || shapetype == 5 || 
        shapetype == 8 || shapetype == 11 || shapetype == 13 || shapetype == 15 ||
        shapetype == 18 || shapetype == 21 || shapetype == 23 || shapetype == 25 ||
        shapetype == 28 || shapetype == 31);
    };

  } SSHPHeader;

                                // SHP file record header (8 bytes)
  typedef struct SSHPRecHeader {
                                // record number, first record is 1 (big endian)
    ESRIint recnumber;
                                // content length in 16-bit words (big)
    ESRIint contlength;
                                // cast big endian into little endian
    void CorrectEndianness()
    {
      BSwap32(2,this);
    };

  } SSHPRecHeader;
                                // SHX file record header (8 bytes)
  typedef struct SSHXRecHeader {
                                // record number, first record is 1 (big endian)
    ESRIint offset;
                                // content length in 16-bit words (big)
    ESRIint contlength;
                                // cast big endian into little endian
    void CorrectEndianness()
    {
      BSwap32(2,this);
    };

  } SSHXRecHeader;

                                // DBF field descriptor
  typedef struct {
                                // field name
    char fieldname[11];
                                // field type
    char fieldtype;
                                // field data address
    unsigned int fielddataaddress;
                                // field length
    unsigned char fieldlength;
                                // field decimal count
    unsigned char fielddeccount;
                                // reserved
    char reserved0[2];
                                // work area ID
    unsigned char workareaID;
                                // reserved
    char reserved1[2];
                                // SETFIELDS flag
    unsigned char setfieldsflag;
                                // reserved
    char reserved2[8];

  } SDBFFieldDescriptor;

                                // DBF file header
  typedef struct {
                                // first 2 bits - version number
    unsigned char version;
                                // update time
    char updatetime[3];
                                // # records in the table
    int numrecords;
                                // header size
    unsigned short headersize;
                                // record size
    unsigned short recordsize;
                                // reserved
    char reserved0[3];
                                // reserved
    char reserved1[13];
                                // reserved
    char reserved2[4];
                                // field descriptor array
    SDBFFieldDescriptor fielddescriptor;
                                // field terminator
    char fieldterminator;

  } SDBFHeader;

                              // record contents to handle and draw
  class CESRIRecord {
  public:
                              // shape type
    ESRIint shapetype;
                              // SWBD_... type (ocean, lake or river)
    ESRIint SWBDtype;
                              // offset in bytes from the start of main SHP file
                              // (this value is loaded from index .SHX file)
    ESRIint offset;
                              // record length in bytes 
                              // (this value is loaded from index .SHX file)
    ESRIint length;
                              // min/max
    TVector<ESRIfloat> min;
    TVector<ESRIfloat> max;
                              // 0-based indices of parts, meaningful for
                              // ESRI_POLYLINE, ESRI_POLYGON
    std::vector<ESRIint> parts;
                              // points of shape type
    std::vector<TVector<ESRIfloat> > points;
                              // constructors
    CESRIRecord();
    CESRIRecord(ESRIint pshapetype);
                              // copy constructor and assignment operator
    CESRIRecord(const CESRIRecord &other);
    CESRIRecord &operator=(const CESRIRecord &other);
                              // destructor
    ~CESRIRecord();
                              // read XY point, fill Z with "no data"
    TVector<ESRIfloat> ReadXYPoint(char *data, int &offset);
                              // read single float
    ESRIfloat ReadFloat(char *data, int &offset);
                              // read integer
    ESRIint ReadNumPoints(char *data, int &offset);
                              // returns signed area in lat/lon units for a part;
                              // returns 0.0 in failure
    ESRIfloat Area(int partno, ESRIfloat accuracymetres);
                              // returns -1 if this is NOT water point or
                              // PART number if this IS water point;
                              // accuracy in metres is very approximate
    ESRIint PointInside(ESRIfloat lat, ESRIfloat lon, ESRIfloat accuracymetres);
  };
                              // anything less than -1e-38 means "no data"
  static const double ESRInodata;
                              // file is OK, it contains only POLYGONZ records
                              // and all data have been read successfully
                              // from SHP,SHX and DBF files and placed into records;
                              // this is int, as there is something unexplainable
                              // happens with struct memory alignment here
  int OK;
                              // data can be bad, e.g. first part of record has negative
                              // area (land), like N05E100)
  int dataOK;
                              // continental delivery (see CD_... enumeration)
  int contdelivery;
                              // error
  string errorString;
                              // SHP header
  SSHPHeader SHPheader;
                              // records
  std::vector<CESRIRecord> records;
                              // default constructor
  ESRIFile();
                              // from file
  ESRIFile(const char *filename);
                              // destructor
  ~ESRIFile();
                              // returns -1 if this is NOT water point or
                              // RECORD (not record part) number if this IS water point;
                              // accuracy in metres is very approximate
  ESRIint PointInside(ESRIfloat lat, ESRIfloat lon, ESRIfloat accuracymetres);

                              // get all coast line points with Z = 0 as lon and lat (!)
                              // in X,Y vector coordinate by parts of max partsize 
                              // points in each part; parts[i] marks beginning of new part;
                              // if distance between prev and next points exceeds maxdist
                              // (deg), a new part is started, Zlevel is normally 0 (waterlevel)
  template <class T>
  void GetZ0Points(int partsize, T maxdist, T Zlevel, std::vector<TVector<T> > &points, 
    std::vector<int> &parts, T tolerance) const
  {
    points.clear();
    parts.clear();

    for (int j = 0; j < static_cast<int>(records.size()); j++)
    {
      for (int l = 0; l < records[j].parts.size(); l++)
      {
        int i0 = records[j].parts[l];
        int i1 = (l < records[j].parts.size() - 1) ? 
          records[j].parts[l + 1] - 1 :
          static_cast<int>(records[j].points.size()) - 1;
                       
        int numpoints = i1 - i0 + 1;

        parts.push_back(static_cast<int>(points.size()));

        int numparts = numpoints / partsize;
        if (records[j].points.size() % partsize) numparts++;
        int maxcount = (numparts > 0) ? (numpoints / numparts) : numpoints;
        int count = 0;

        for (int m = i0; m <= i1; m++)
        {

          TVector<T> point(static_cast<T>(records[j].points[m].X),
            static_cast<T>(records[j].points[m].Y),
            static_cast<T>(records[j].points[m].Z));
//            static_cast<T>(records[j].points[m].Z - Zlevel)); //? maybe better to make shores steeper
//          TVector<T> point(records[j].points[m].X,records[j].points[m].Y,Zlevel);
                              // throw outside points
          if (point.X <= lonmin)
            continue;
          if (point.X >= lonmax)
            continue;
          if (point.Y <= latmin)
            continue;
          if (point.Y >= latmax)
            continue;
                              // check if point is not on boundary
          if (std::abs(point.X - lonmin) < tolerance)
            continue;
          if (std::abs(point.X - lonmax) < tolerance)
            continue;
          if (std::abs(point.Y - latmin) < tolerance)
            continue;
          if (std::abs(point.Y - latmax) < tolerance)
            continue;
                                // otherwise, add this point
          points.push_back(point);
          count++;
                              // distance between successive points is too big
          //TVector<T> testpoint(-1.0013888,50.784027,0.00000000);
          //T d = !(point - testpoint);
          //if (d < 0.00001)
          //{
          //  int gsgsgsg = 0;
          //}
                              // remove last point and start new count
          if (count > 1 && points.size() > 1)
          {
            T dist = !(points[points.size() - 1] - points[points.size() - 2]);
            if (dist > maxdist)
            {
              points.erase(points.end() - 1);
              parts.push_back(static_cast<int>(points.size()));
              count = 0;
            }
          }
                              // max part size reached
          if (count > maxcount && count > 0)
          {
            parts.push_back(static_cast<int>(points.size()));
            count = 0;
          }
        }
      }
    }
  }

};

#pragma pack(pop)