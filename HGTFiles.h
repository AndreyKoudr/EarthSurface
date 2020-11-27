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
#include "LatLon.h"
#include "Vector.h"
#include "ESRIFiles.h"

/**
  NASA .hgt file ("https://dds.cr.usgs.gov/srtm/version2_1/SRTM3/")
consists of big-endian signed 2-byte integers, SIZE (1201) by SIZE points. 
One tile covers 1 square degree of lat/lon, lon-points being rows. 
File naming corresponds to lower left corner of the tile, e.g. S23W043.hgt
*/

using namespace std;
                              // undefined depth
#define DEPTH_UNDEFINED -32768
                              // convert HGT filename like N50W003.hgt to lat/lon
bool HGTFilenameToLatLon(const string &filename, double *lat, double *lon,
  string &errorString);
                              // convert lat/lon (lower left corner) to HGT file name
                              // without extension, like N50W003
string LatLonToHGTFilename(double lat, double lon);

                              // square mesh size x size points; points are
                              // numbered from top and left to right; each row
                              // contains size points; i is row number, j is
                              // column number
inline int getPointNumber(int i, int j, int size)
{
  assert(i >= 0 && i < size);
  assert(j >= 0 && j < size);

  return (i * size + j);
}
                              // square mesh size x size points
                              // get neighbour points (data point numbers) to 
                              // compute common normal,
                              // returns number of points, 4 for inner, 2 for
                              // corners, 3 for sides;
                              // neighbour points are given counter-clockwise to
                              // facilitate cross-product calculations
extern int getNeighbourPoints(int i, int j, int nei[4], int size);

                              // HGT file
class HGTFile : public LatLonRect {
public:
                              // 1201 points in each row and column
  static const int SIZE = 1201;
                              // four neighbours
  enum {
    LEFT,
    RIGHT,
    BOTTOM,
    TOP,

    NTOTAL
  };
                              // pointer to array of 1201x1201 points;
                              // if empty, object is not ready
  std::vector<signed short int> data;
                              // four neighbours
  HGTFile *neighbours[NTOTAL];
                              // default constructor
  HGTFile();
                              // from file
  HGTFile(const string &filename);
                              // from file and correct from more accurate waterbody data
  HGTFile(const string &filename, const ESRIFile *waterbody, float defaultdepth, float Zcoastline);
                              // of constant height, like ocean without islands
  HGTFile(short int height, double platmin, double plonmin);
                              // build max four vectors as neighbours around a
                              // point; vector X,Y coordinates are
                              // (1) approximate
                              // (2) with origin at lower left file corner
                              // i being point row (counted from top), j - column
  int getNeighbourPoints(int i, int j, Vector4 nei[4], int &boundmask);
                              // calculate normalised normal averaged around point
                              // i(row),j(column); on boundaries there are less 
                              // points, 2 at corners and 2 on verticals/horizontals
  Vector4 getAverageNormal(std::vector<Vector4> &vectors, int i, int j);
                              // get normal for this file from current data
                              // ignoring neighbours
  Vector4 getAverageNormalPrim(int i, int j, int &boundmask);
                              // get normal for this file from current data
                              // taking neighbours into account
  Vector4 getAverageNormal(int i, int j);
                              // return error string
  const string& errorString();

private:
                              // error string
  string errorString_;
                              // big endian to little endian, correct data errors;
                              // this is called from constructor
  void correctData();
                              // big endian to little endian, correct data errors;
                              // make corrections from more accurate waterbody data
  void correctData(const ESRIFile *waterbody, float defaultdepth, float Zcoastline);
                              // compute mesh coordinates as float vectors, numbered
                              // by rows from top, all in all 1201 x 1201 vectors
  void computeVectors(std::vector<Vector4> &vectors, Vector4 &min, Vector4 &max);
                              // zero out pointers to neighbours
  void initNeighbours();
                              // return number of neightbours
  int getNeighbours(int i, int j, int neighbours[8]);
                              // get neightbour height
  int findNeighbourHeight(int i, int j);

                              // get approximate vector for a point
                              // vector X,Y coordinates are
                              // (1) approximate
                              // (2) with origin at lower left file corner
                              // i being point row (counted from top), j - column
  inline Vector4 getPointVector(int i, int j)
  {
    assert(geocoord != nullptr);
    assert(i >= 0 && i < SIZE);
    assert(j >= 0 && j < SIZE);

    double cellwidth,cellheight;
    geocoord->getDegreeLinearSizes(cellwidth,cellheight);

    Vector4 v(static_cast<float>(cellwidth * static_cast<double>(j) / static_cast<double>(SIZE - 1)),
      static_cast<float>(cellheight * static_cast<double>(SIZE - 1 - i) / static_cast<double>(SIZE - 1)),
      static_cast<float>(data[getPointNumber(i,j,SIZE)]));

    return v;
  }
                              // point number in data array
  inline int pointNumber(int i, int j)
  {
    int ret = i * SIZE + j;
    assert(ret >= 0 && ret < SIZE * SIZE);
    return ret;
  }

};



