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
#include "LatLonCoord.h"
#include "Vector.h"
#include "Triangles.h"
#include "Systems.h"

/**
  Mesh of nx by ny small equal cells, (nx + 1) * (ny + 1) points

nx is not necessarily equal to ny. Is used to quickly compute height and 
normal. Is constructed by adding new parts (new meshes) which makes
recalculation of data array (heights of short ints for all mesh). Also
this is the only place which makes constrained deformation correction of 
geometry near more accurate coast line.
*/

using namespace std;

class GeoMesh : public LatLonRect {
public:
                              // default constructor
  GeoMesh();
                              // destructor
  ~GeoMesh() = default;
                              // of constant height, like ocean without islands; pnx,pny are
                              // number of cells, number of data_ points will be (pnx + 1) * (pny + 1)
  GeoMesh(short int height, double platmin, double plonmin, double platmax, double plonmax, int pnx, int pny);
                              // constructor from CHeighFile
  GeoMesh(signed short int *pdata, double platmin, double plonmin, double platmax, 
    double plonmax, int pnx = 1200, int pny = 1200);
                              // create sub mesh (inside current mesh)
  bool createSubMesh(double platmin, double plonmin, double platmax, double plonmax, int rowcolmargin,
    GeoMesh &submesh, int &rowmin, int &colmin);
                              // load etopo ASCII file like that :
                              // 43.2833333333333385 47.8666666666666671 117
                              // 43.3000000000000043 47.8666666666666671 117
                              // 43.31666666666667 47.8666666666666671 117
                              // 43.3333333333333428 47.8666666666666671 115
                              // 43.3500000000000085 47.8666666666666671 114
                              // 43.3666666666666742 47.8666666666666671 114
                              // 43.38333333333334 47.8666666666666671 110
                              // ...
  //bool LoadETOPOASCII(string &filename);
                              // get cell row and column
  bool getRowCol(double lat, double lon, int *row, int *col, double *U = nullptr, double *V = nullptr);
                              // get row/col parameters; rows are counted from above, cols from left
  void getPointParms(int row, int col, double *lat, double *lon, float *height);
                              // get address of data_, row is counted downward
  signed short int *dataAddr(int row, int col);
                              // get approximate (from closest mesh node) height/depth
  signed short int getZ(double lat, double lon);
                              // get exact height/depth with normal if not nullptr
  template <class T> 
  T getZAccurate(double lat, double lon, TVector<T> *normal);
                              // returns true if point is inside area and in case 
                              // of negative Z
  bool getDepth(double lat, double lon, float *depth);

                              //===== Application of constraints (to move Z points 
                              // close to coast lines)

                              // get position
  template <class T>
  TVector<T> getPosition(T U, T V);
                              // get U,V parameters, [0..1]
  template <class T>
  TVector<T> getParameters(TVector<T> point);
                              // distort mesh by applying a constrained deformation
                              // near coast line (points of lon/lat))
  template <class T> 
  bool applyCoastLine(std::vector<TVector<T> > &points, T &maxdist);
                              // add tris to TTriangles; they can be stored in files 
  bool toTriangles(TTriangles<float> &tris);

private:
                              // to get access to nx_,ny_ in GeoMesh
  friend class HGTFile;
                              // to mark undefined depth
  static signed short int DEPTH_UNDEF;
                              // file version
  static int fileversion_;
                              // file header
  static int fileheader_;
                              // geo coordinates are owned by this mesh;
                              // is recalculated
                              // on adding new parts
  LatLonCoord<double> geocoord_;
                              // number of cells along x and y
  int nx_;
  int ny_;
                              // cell size in degrees of lat/lon
  double dlat_;
  double dlon_;
                              // cell size in metres
  double celldx_;
  double celldy_;
                              // pointer to array of nx_ + 1 by ny_ + 1 points;
                              // if empty, object is not ready
  std::vector<signed short int> data_;

                              // square mesh nx_ x ny_ cells; points are
                              // numbered from top and left to right; each row
                              // contains (nx_ + 1) points; i is row number, j is
                              // column number
  inline int getPointNumber(int i, int j);
                              // get 4 (inner point), 3 or 2 (border) points;
                              // i is row number, j is column number
  int getNeighbourPoints(int i, int j, int nei[4]);
  int getNeighbourPoints(int i, int j, int inei[4], int jnei[4]);
                              // get X,Y,Z vector for a point
                              // i being point row (counted from top), j - column
  template <class T> TVector<T> getPointVector(int i, int j);
                              // calculate dlat_,dlon_,celldx_,celldy_
  void updateCellSizes();
                              // compute basis function, U is [0..1], two
                              // values are ready : parm at point i and 1 - parm
                              // at point i + 1
  template <class T>
  void computeBasisU(T U, int &i, T &parm);
                              // compute basis function, V is [0..1], two
                              // values are ready : parm at point i and 1 - parm
                              // at point i + 1
  template <class T>
  void computeBasisV(T V, int &i, T &parm);
                              // compute basis function, U is [0..1]
  template <class T>
  void computeBasisU(T U, std::vector<T> &func);
                              // compute basis function, V is [0..1]
  template <class T>
  void computeBasisV(T V, std::vector<T> &func);
};

template <class T> T GeoMesh::getZAccurate(double lat, double lon, TVector<T> *normal)
{
  assert(data_.size() > 0);

  int row,col;
  double U,V;
  if (getRowCol(lat,lon,&row,&col,&U,&V))
  {
                              // interpolate
    if (row > 0 && col <= nx_)
    {
                              // linear shape functons
      T shapefunc[4];

      T E = T(U * 2.0 - 1.0);
      T N = T(V * 2.0 - 1.0);

      T Ep = T(1.0) + E;
      T Em = T(1.0) - E;
      T Np = T(1.0) + N;
      T Nm = T(1.0) - N;

      shapefunc[0] = T(0.25) * Em * Nm;
      shapefunc[1] = T(0.25) * Ep * Nm;
      shapefunc[2] = T(0.25) * Ep * Np;
      shapefunc[3] = T(0.25) * Em * Np;

      int r0 = row * (nx_ + 1);
      int r1 = (row - 1) * (nx_ + 1);
      T v0 = (T) data_[r0 + col];
      T v1 = (T) data_[r0 + col + 1];
      T v2 = (T) data_[r1 + col + 1];
      T v3 = (T) data_[r1 + col];

      if (normal != nullptr)
      {
        TVector<T> V0 = TVector<T>(0,0,v0);
        TVector<T> V1 = TVector<T>(T(celldx_),0,v1);
        TVector<T> V2 = TVector<T>(T(celldx_),T(celldy_),v2);
        TVector<T> V3 = TVector<T>(0,T(celldy_),v3);

        *normal = +((V2 - V0) ^ (V3 - V1));
      }

      T result = v0 * shapefunc[0] + v1 * shapefunc[1] + v2 * shapefunc[2] + v3 * shapefunc[3];
      return result;
    } else
    {
                            // abnormal
      assert(false);
      if (normal != nullptr) *normal = TVector<T>(0,0,1);
      return T(data_[row * (nx_ + 1) + col]);   
    }               
  } else
  {
    return T(0.0);
  }
}

template <class T>
void GeoMesh::computeBasisU(T U, int &i, T &parm)
{
  assert(U >= 0.0 && U <= 1.0);
                            // get interval number
  i = (int) (U * nx_);
                            // get parameter value within interval
  T DU = T(1.0) / T(nx_);
  parm = (U - DU * T(i)) / DU;

  //assert(parm >= T(0.0) && parm <= T(1.0));

  if (parm < T(0.0)) parm = T(0.0);
  if (parm > T(1.0)) parm = T(1.0);
}
                            // compute basis function, V is [0..1], two
                            // values are ready : parm at point i and 1 - parm
                            // at point i + 1
template <class T>
void GeoMesh::computeBasisV(T V, int &i, T &parm)
{
  assert(V >= T(0.0) && V <= T(1.0));
                            // get interval number
  i = (int) (V * ny_);
                            // get parameter value within interval
  T DV = T(1.0) / T(ny_);
  parm = (V - DV * T(i)) / DV;

  //assert(parm >= 0.0 && parm <= T(1.0));

  if (parm < T(0.0)) parm = T(0.0);
  if (parm > T(1.0)) parm = T(1.0);
}

                            // compute basis function, U is [0..1]
template <class T>
void GeoMesh::computeBasisU(T U, std::vector<T> &func)
{
  func.resize(nx_ + 1);
  memset(&func[0],0,func.size() * sizeof(T));

  int i; T parm;
  computeBasisU(U,i,parm);

  func[i] = parm;
  func[i + 1] = T(1.0) - parm;
}


template <class T>
void GeoMesh::computeBasisV(T V, std::vector<T> &func)
{
  func.resize(ny_ + 1);
  memset(&func[0],0,func.size() * sizeof(T));

  int i; T parm;
  computeBasisV(V,i,parm);

  func[i] = parm;
  func[i + 1] = T(1.0) - parm;
}

template <class T>
TVector<T> GeoMesh::getPosition(T U, T V)
{
                            // these normally should be both 1 degree
  T Dlon = static_cast<T>(lonmax - lonmin);
  T Dlat = static_cast<T>(latmax - latmin);

  int ix; T parmx;
  computeBasisU(U,ix,parmx);

  int iy; T parmy;
  computeBasisV(V,iy,parmy);

  int rowl = ny_ - iy;
  int rowu = rowl - 1;

  T z0 = static_cast<T>(data_[rowl * (nx_ + 1) + ix]);
  T z1 = static_cast<T>(data_[rowl * (nx_ + 1) + ix + 1]);
  T z2 = static_cast<T>(data_[rowu * (nx_ + 1) + ix + 1]);
  T z3 = static_cast<T>(data_[rowu * (nx_ + 1) + ix]);

  T z = z0 * parmx * parmy + z1 * (T(1.0) - parmx) * parmy + 
    z2 * (T(1.0) - parmx) * (T(1.0) - parmy) + z3 * parmx * (T(1.0) - parmy);

  TVector<T> p(static_cast<T>(lonmin) + Dlon * U,static_cast<T>(latmin) + Dlat * V,z);

  return p;
}
                            // get U,V parameters, [0..1]
template <class T>
TVector<T> GeoMesh::getParameters(TVector<T> point)
{
  TVector<T> uv;
                            // these normally should be both 1 degree
  T Dlon = static_cast<T>(lonmax - lonmin);
  T Dlat = static_cast<T>(latmax - latmin);

  uv.X = (point.X - static_cast<T>(lonmin)) / Dlon;
  uv.Y = (point.Y - static_cast<T>(latmin)) / Dlat;

  assert(uv.X >= T(0.0) && uv.X <= T(1.0));
  assert(uv.Y >= T(0.0) && uv.Y <= T(1.0));

  return uv;
}
                            // distort mesh by applying a constrained deformation
                            // near coast line (points of lon/lat))
template <class T>
bool
GeoMesh::applyCoastLine(std::vector<TVector<T> > &points, T &maxdist)
{
                            // compute U,V for all these points
  std::vector<TVector<T> > uv(points.size());
  for (int i = 0; i < static_cast<int>(uv.size()); i++)
  {
    uv[i] = getParameters(points[i]);
  }
                            // compute displacements
  std::vector<TVector<T> > disp(points.size());
  for (int i = 0; i < static_cast<int>(uv.size()); i++)
  {
    disp[i] = points[i] - getPosition(uv[i].X,uv[i].Y);
  }
                            // system matrix R
  std::vector<T> R((nx_ + 1) * (ny_ + 1) * uv.size());
                            // right-hand side
  std::vector<TVector<T> > D = disp;

  int count = 0;
  int numV = ny_ + 1;
  int numU = nx_ + 1;

  std::vector<T> funcsU;
  std::vector<T> funcsV;
  for (int j = 0; j < numV; j++)
  {
    for (int k = 0; k < numU; k++)
    {
      for (int q = 0; q < static_cast<int>(uv.size()); q++)
      {
                            // compute basis functions at source point
        computeBasisU(uv[q].X,funcsU);
        computeBasisV(uv[q].Y,funcsV);

        T f = funcsV[j] * funcsU[k];
        R[count++] = f;
      }
    }
  }
                            // solve system
  bool res = SolveATASystem<T,int>(numV * numU,static_cast<int>(uv.size()),&R[0],&D[0],
//    static_cast<T>(1.0e-14));
    TOLERANCE(T));

  if (!res)
    return false;

  count = 0;
  for (int j = 0; j < numV; j++)
  {
    for (int k = 0; k < numU; k++)
    {
      TVector<T> sum(0,0,0);

      for (int q = 0; q < static_cast<int>(uv.size()); q++)
      {
                          // compute basis functions at source point
        computeBasisU(uv[q].X,funcsU);
        computeBasisV(uv[q].Y,funcsV);

        T f = funcsV[j] * funcsU[k];

        sum += D[q] * f;
      }

      int index = (ny_ - j) * numU + k;
      data_[index] += ROUND(sum.Z);
    }
  }
                            // calculate max distance from target points
  maxdist = 0;
  for (int i = 0; i < static_cast<int>(uv.size()); i++)
  {
    TVector<T> oldPoint = points[i];
    TVector<T> newPoint = getPosition(uv[i].X,uv[i].Y);
    T dist = !(newPoint - oldPoint);
    if (dist > maxdist)
    {
      maxdist = dist;
    }
  }

  return true;
}

inline int GeoMesh::getPointNumber(int i, int j)
{
  assert(i >= 0 && i <= ny_);
  assert(j >= 0 && j <= nx_);

  return (i * (nx_ + 1) + j);
}

template <class T> TVector<T> GeoMesh::getPointVector(int i, int j)
{
  assert(i >= 0 && i <= ny_);
  assert(j >= 0 && j <= nx_);
                            // get lat/lon
  double lon = lonmin + (lonmax - lonmin) * static_cast<double>(j) / static_cast<double>(nx_);
  double lat = latmax + (latmin - latmax) * static_cast<double>(i) / static_cast<double>(ny_);
                            // to linear
  double x,y;
  LatLonToXY(lat,lon,x,y);
                            // vector
  TVector<T> v(static_cast<T>(x),static_cast<T>(y),static_cast<T>(data_[getPointNumber(i,j)]));

  return v;
}


//                              // 30N000E030030.geo
//bool GEOFilenameToLatLon(string &filename, double *lat, double *lon,
//  double *dlat_, double *dlon_, bool messages);
//                              // convert to rect
//bool GEOFilenameToRect(string &filename, SLatLonRect *rect);
//                              // 30N000E030030
//string LatLonToGEOFilename(double lat, double lon, double dlat_, double dlon_);

