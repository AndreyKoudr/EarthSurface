#include "defines.h"
#include "Types.h"
#include "Strings.h"
#include "HGTFiles.h"
#include "GeoMesh.h"
#include "Triangles.h"
#include <limits>
#include <fstream>

                              // swap bytes of 16-bit integer
inline void Swap16(signed short int *v)
{
  char *b = (char *) v;
  char temp = b[0];
  b[0] = b[1];
  b[1] = temp;
}
                              // swap short int, avoid negative
void static BSwap16NoNegative(size_t size, void *addr)
{
  signed short int *v = (signed short int *) addr;

  for (size_t i = 0; i < size; i++)
  {
    Swap16(v);

    if (*v < 0)
    {
      *v = 0;
    }
    v++;
  }
}
                               // file exists?
bool static fileExists(const string &path)
{
  ifstream f(path.c_str());
  return f.good();
}

bool HGTFilenameToLatLon(const string &filename, double *lat, double *lon,
  string &errorString)
{
                              // parse file name to get lat/lon min/max
  string s = upCase(forceExtension(justFileName(filename),""));
                              // cut it into two parts
  bool easterly = false;
  int n = find(s,'E');
  if (n != NOT_DEFINED)
  {
    easterly = true;
  } else
  {
    n = find(s,'W');
    if (n != NOT_DEFINED)
    {
    } else
    {
      errorString = string("NASA .hgt file ") + filename + 
        " has a wrong name (should be like N50W003.hgt)";
      return false;
    }
  }
                              // go on
                              // divide into two strings
  string slat = getSubString(s,0,n - 1);
  bool northern = false;
  if (slat[0] == 'N')
  {
    northern = true;
  } else if (slat[0] == 'S')
  {
  } else
  {
    errorString = string("NASA .hgt file ") + filename + 
      " has a wrong name (should be like N50W003.hgt)";
    return false;
  }

  deleteChars(slat,0,1);
  *lat = atof(slat.c_str())  * (northern ? 1.0 : -1.0);
                              // now longitude
  string slon = getSubString(s,n + 1,static_cast<int>(s.length()) - 1);
  *lon = atof(slon.c_str()) * (easterly ? 1.0 : -1.0);
                              // full success
  return true;
}

string LatLonToHGTFilename(double lat, double lon)
{
  CorrectLat(lat);
  CorrectLon(lon);

  int ilat = ROUND(lat);
  int ilon = ROUND(lon);

  string filename = 
    ((ilat >= 0) ? string("N") : string("S")) +
    to_string(abs(ilat),2) +
    ((ilon >= 0) ? string("E") : string("W")) +
    to_string(abs(ilon),3);

  return filename;
}

static int getNeighbourPoints(int i, int j, int nei[4], int size)
{
  int size1 = size - 1;
  int size2 = size - 2;
  if (i == 0 && j == 0)
  {
    nei[0] = getPointNumber(1,0,size);
    nei[1] = getPointNumber(0,1,size);
    return 2;
  } else if (i == 0 && j == size1)
  {
    nei[0] = getPointNumber(0,size2,size);
    nei[1] = getPointNumber(1,size1,size);
    return 2;
  } else if (i == size1 && j == 0)
  {
    nei[0] = getPointNumber(size1,1,size);
    nei[1] = getPointNumber(size2,0,size);
    return 2;
  } else if (i == size1 && j == size1)
  {
    nei[0] = getPointNumber(size2,size1,size);
    nei[1] = getPointNumber(size1,size2,size);
    return 2;
  } else if (i == 0)
  {
    nei[0] = getPointNumber(i,j - 1,size);
    nei[1] = getPointNumber(i + 1,j,size);
    nei[2] = getPointNumber(i,j + 1,size);
    return 3;
  } else if (i == size1)
  {
    nei[0] = getPointNumber(i,j + 1,size);
    nei[1] = getPointNumber(i - 1,j,size);
    nei[2] = getPointNumber(i,j - 1,size);
    return 3;
  } else if (j == 0)
  {
    nei[0] = getPointNumber(i + 1,j,size);
    nei[1] = getPointNumber(i,j + 1,size);
    nei[2] = getPointNumber(i - 1,j,size);
    return 3;
  } else if (j == size1)
  {
    nei[0] = getPointNumber(i - 1,j,size);
    nei[1] = getPointNumber(i,j - 1,size);
    nei[2] = getPointNumber(i + 1,j,size);
    return 3;
  } else
                              // inner point
  {
    nei[0] = getPointNumber(i + 1,j,size);
    nei[1] = getPointNumber(i,j + 1,size);
    nei[2] = getPointNumber(i - 1,j,size);
    nei[3] = getPointNumber(i,j - 1,size);
    return 4;
  }
}

HGTFile::HGTFile() : LatLonRect()
{
  initNeighbours();
}

HGTFile::HGTFile(const string &filename) : LatLonRect()
{
  initNeighbours();
                            // file exists?
  if (!fileExists(filename))
    return;
                            // parse file name
  string justfilename = justFileName(filename);
  if (!HGTFilenameToLatLon(justfilename,&latmin,&lonmin,errorString_))
    return;
                            // set rectangle limits
                            //NB periodic longitude is not treated here
  latmax = latmin + 1.0;
  lonmax = lonmin + 1.0;
                            // open file
  ifstream fp(filename.c_str(),std::ios::in | std::ios::binary);
  if (!fp.is_open())
  {
    errorString_ = "Could not open file " + filename;
    return;
  }
                            // read file contents into data array
  data.resize(SIZE * SIZE);
  fp.read((char *) &data[0],data.size() * sizeof(short int));
                            // failure
  if (fp)
  {
  } else
  {
    errorString_ = "Could not read from file " + filename;
    fp.close();
                            // to mark failure
    data.clear();
    return;
  }
                            // success
  fp.close();
                            // correct data
  correctData();
}


HGTFile::HGTFile(const string &filename, const ESRIFile *waterbody, float defaultdepth, 
  float Zcoastline) : LatLonRect()
{
  initNeighbours();
                            // file exists?
  if (!fileExists(filename))
    return;
                            // parse file name
  string justfilename = justFileName(filename);
  if (!HGTFilenameToLatLon(justfilename,&latmin,&lonmin,errorString_))
    return;
                            // set rectangle limits
                            //NB periodic longitude is not treated here
  latmax = latmin + 1.0;
  lonmax = lonmin + 1.0;
                            // open file
  ifstream fp(filename.c_str(),std::ios::in | std::ios::binary);
  if (!fp.is_open())
  {
    errorString_ = "Could not open file " + filename;
    return;
  }
                            // read file contents into data array
  data.resize(SIZE * SIZE);
  fp.read((char *) &data[0],data.size() * sizeof(short int));
                            // failure
  if (fp)
  {
  } else
  {
    errorString_ = "Could not read from file " + filename;
    fp.close();
                            // to mark failure
    data.clear();
    return;
  }
                            // success
  fp.close();
                            // correct data
  correctData(waterbody,defaultdepth,Zcoastline);
}


HGTFile::HGTFile(short int height, double platmin, double plonmin) :
  LatLonRect(platmin,plonmin,platmin + 1.0,plonmin + 1.0)
{
  initNeighbours();
  data.resize(SIZE * SIZE,height);
}

void HGTFile::correctData()
{
  assert(data.size() == SIZE * SIZE);
                            // swap bytes and set all negative numbers to zero
  BSwap16NoNegative(data.size(),&data[0]);
}

                              // get min & max values of coordinates
template <class T> static bool getMinMax(std::vector<TVector<T> > vectors, int from, int to, 
  TVector<T> &min, TVector<T> &max)
{
  int n = static_cast<int>(vectors.size());

  bool ok = (from >= 0 && from < n && to >= 0 && to < n && to >= from);
  assert(ok);
  if (!ok)
    return false;

  min = max = vectors[from];

  for (int i = from + 1; i <= to; i++) 
  {
    TVector<T> v = vectors[i];
    if (v.X > max.X) max.X = v.X;
    if (v.Y > max.Y) max.Y = v.Y;
    if (v.Z > max.Z) max.Z = v.Z;
    if (v.X < min.X) min.X = v.X;
    if (v.Y < min.Y) min.Y = v.Y;
    if (v.Z < min.Z) min.Z = v.Z;
  }

  return true;
}

void HGTFile::correctData(const ESRIFile *waterbody, float defaultdepth, float Zcoastline)
{
  assert(defaultdepth < 0);

  signed short int v1,v2;
  signed short int *v = &data[0];
  double lat = latmax;
  double dlatlon = 1.0 / (double) SIZE;
  int undefinedcount = 0;
  for (int i = 0; i < SIZE; i++)
  {
    double lon = lonmin;
    for (int j = 0; j < SIZE; j++)
    {
                              // swap bytes
      v1 = *v << 8;
      v2 = (*v >> 8) & 0x00FF;
      *v = v1 | v2;

      if (*v == DEPTH_UNDEFINED)
      {
        undefinedcount++;
      } else
      {
                              // set default, no geo info here
        if (*v <= 0)
        {
          *v = static_cast<signed short int>(defaultdepth);
        }
      }

      v++;
      lon += dlatlon;
    };
    lat -= dlatlon;
  };
                              // remove undefined points - find average
                              // across neighbours
  v = &data[0];
  for (int i = 0; i < SIZE; i++)
  {
    for (int j = 0; j < SIZE; j++)
    {
      if (*v == DEPTH_UNDEFINED)
      {
        int h = findNeighbourHeight(i,j);
        *v = static_cast<signed short int>(h);
      }

      v++;
    }
  }
                              // second pass : apply restriction on coast lines
  if (waterbody != nullptr)
  {
    std::vector<TVector<float> > points;
    std::vector<int> parts;
    waterbody->GetZ0Points<float>(100,static_cast<float>(0.1),Zcoastline,points,parts,static_cast<float>(0.001));
                              // make copy of whole mesh to call its functions
    GeoMesh mesh(&data[0],latmin,lonmin,latmax,lonmax,SIZE - 1,SIZE - 1);

    for (int ip = 0; ip < static_cast<int>(parts.size()); ip++)
    {
      //printf("  part %d of %d ",ip + 1,static_cast<int>(parts.size()));

      int i0 = parts[ip];
      int i1 = (ip < static_cast<int>(parts.size()) - 1) ? (parts[ip + 1] - 1) : (static_cast<int>(points.size()) - 1);

      if (i1 <= i0)
        continue;

      TVector<float> min,max;
      if (getMinMax(points,i0,i1,min,max))
      {
                              // make sub points
        std::vector<TVector<float> > subpoints;
        subpoints.reserve(i1 - i0 + 1);
        for (int i = i0; i <= i1; i++)
        {
          subpoints.push_back(points[i]);
        }
                              // make submesh
        GeoMesh submesh;
        int rowmin,colmin;
        mesh.createSubMesh(min.Y,min.X,max.Y,max.X,1,submesh,rowmin,colmin);
                              // check if all points are inside new boundaries
        for (int i = 0; i < static_cast<int>(subpoints.size()); i++)
        {
          assert(subpoints[i].X >= submesh.lonmin && subpoints[i].X <= submesh.lonmax); 
          assert(subpoints[i].Y >= submesh.latmin && subpoints[i].Y <= submesh.latmax); 
        }
                              // exclude duplicates of constrained points        
        vector<LINT> replacement;
        removeDupNodes<float>(subpoints,replacement,static_cast<float>(0.000001)); // tolerance for floats in metres

        float maxdist = 0;
        bool res = submesh.applyCoastLine(subpoints,maxdist);

        if (res)
        {
                              // copy data back
          int row = rowmin;
          for (int i = 0; i <= submesh.ny_; i++)
          {
            int datarow = mesh.ny_ - row;
            signed short int *addrfrom = mesh.dataAddr(datarow,colmin);
            signed short int *addrto = submesh.dataAddr(submesh.ny_ - i,0);

            memmove(addrfrom,addrto,(submesh.nx_ + 1) * sizeof(signed short int));

            row++;
          }
          //printf(" done\n");
        } else
        {
          //printf(" failed\n");
        }
      } else
      {
        //printf(" failed\n");
      }
    }

#ifdef _DEBUG
    signed short int maxdiff = 0;
    int diffcount = 0;
    for (size_t i = 0; i < SIZE * SIZE; i++)
    {
      signed short int d = std::abs(mesh.data_[i] - data[i]);
      if (d > maxdiff)
        maxdiff = d;
      if (d > 0)
        diffcount++;
    }
#endif
                              // copy data back from temporary geo mesh
    memmove(&data[0],&mesh.data_[0],SIZE * SIZE * sizeof(signed short int));
  }
}

void HGTFile::computeVectors(std::vector<Vector4> &vectors, Vector4 &min, Vector4 &max)
{
  assert(geocoord != nullptr);
                              // allocate space
  vectors.resize(SIZE * SIZE);
                              // fill coordinates and normals
  double lat = latmax;
  double dlatlon = 1.0 / (double) (SIZE - 1);

  int count = 0;
  for (int i = 0; i < SIZE; i++)
  {
    double lon = lonmin;
    for (int j = 0; j < SIZE; j++)
    {
                              // fill x,y
      LatLonToXY(lat - dlatlon,lon,vectors[count].X,vectors[count].Y);
                              // fill z
      vectors[count].Z = static_cast<float>(data[count]);

      if (count == 0)
      {
        min = max = vectors[count];
      } else
      {
        if (vectors[count].X < min.X) min.X = vectors[count].X;
        if (vectors[count].X > max.X) max.X = vectors[count].X;
        if (vectors[count].Y < min.Y) min.Y = vectors[count].Y;
        if (vectors[count].Y > max.Y) max.Y = vectors[count].Y;
        if (vectors[count].Z < min.Z) min.Z = vectors[count].Z;
        if (vectors[count].Z > max.Z) max.Z = vectors[count].Z;
      }

      lon += dlatlon;
      count++;
    }
    lat -= dlatlon;
  }
}

Vector4 HGTFile::getAverageNormal(std::vector<Vector4> &vectors, int i, int j)
{
  Vector4 centre = vectors[getPointNumber(i,j,SIZE)];
  int nei[4];
  int numnei = ::getNeighbourPoints(i,j,nei,SIZE);

  Vector4 normal;
  for (int k = 0; k < numnei - 1; k++)
  {
    normal = normal + ((vectors[nei[k + 1]] - centre) ^ (vectors[nei[k]] - centre));
  }
  normal /= static_cast<float>(numnei - 1);
  normal = +normal;

  return normal;
}
                              // boundary mask
#define BOUND_TOP     0x00000001
#define BOUND_BOTTOM  0x00000002
#define BOUND_LEFT    0x00000004
#define BOUND_RIGHT   0x00000008

int HGTFile::getNeighbourPoints(int i, int j, Vector4 nei[4], int &boundmask)
{
  assert(geocoord != nullptr);

  int size1 = SIZE - 1;
  int size2 = SIZE - 2;
  if (i == 0 && j == 0)
  {
    nei[0] = getPointVector(1,0);
    nei[1] = getPointVector(0,1);
    boundmask = BOUND_TOP | BOUND_LEFT;
    return 2;
  } else if (i == 0 && j == size1)
  {
    nei[0] = getPointVector(0,size2);
    nei[1] = getPointVector(1,size1);
    boundmask = BOUND_TOP | BOUND_RIGHT;
    return 2;
  } else if (i == size1 && j == 0)
  {
    nei[0] = getPointVector(size1,1);
    nei[1] = getPointVector(size2,0);
    boundmask = BOUND_BOTTOM | BOUND_LEFT;
    return 2;
  } else if (i == size1 && j == size1)
  {
    nei[0] = getPointVector(size2,size1);
    nei[1] = getPointVector(size1,size2);
    boundmask = BOUND_BOTTOM | BOUND_RIGHT;
    return 2;
  } else if (i == 0)
  {
    nei[0] = getPointVector(i,j - 1);
    nei[1] = getPointVector(i + 1,j);
    nei[2] = getPointVector(i,j + 1);
    boundmask = BOUND_TOP;
    return 3;
  } else if (i == size1)
  {
    nei[0] = getPointVector(i,j + 1);
    nei[1] = getPointVector(i - 1,j);
    nei[2] = getPointVector(i,j - 1);
    boundmask = BOUND_BOTTOM;
    return 3;
  } else if (j == 0)
  {
    nei[0] = getPointVector(i + 1,j);
    nei[1] = getPointVector(i,j + 1);
    nei[2] = getPointVector(i - 1,j);
    boundmask = BOUND_LEFT;
    return 3;
  } else if (j == size1)
  {
    nei[0] = getPointVector(i - 1,j);
    nei[1] = getPointVector(i,j - 1);
    nei[2] = getPointVector(i + 1,j);
    boundmask = BOUND_RIGHT;
    return 3;
  } else
                              // inner point
  {
    nei[0] = getPointVector(i + 1,j);
    nei[1] = getPointVector(i,j + 1);
    nei[2] = getPointVector(i - 1,j);
    nei[3] = getPointVector(i,j - 1);
    boundmask = 0;
    return 4;
  }
}

Vector4 HGTFile::getAverageNormalPrim(int i, int j, int &boundmask)
{
                              // point where normal is calculated
  Vector4 centre = getPointVector(i,j);
                              // get neighbour points
  Vector4 nei[4];
  int numnei = getNeighbourPoints(i,j,nei,boundmask);
                              // make 3D vectors
  Vector4 normal;
  for (int k = 0; k < numnei - 1; k++)
  {
    normal += ((nei[k + 1] - centre) ^ (nei[k] - centre));
  }
  normal /= static_cast<float>(numnei - 1);
  normal = +normal;

  return normal;
}

Vector4 HGTFile::getAverageNormal(int i, int j)
{
                              // from this file
  int boundmask = 0;
  Vector4 normal = getAverageNormalPrim(i,j,boundmask);
  int count = 1;
                              // if boundary point, take neighbour normals into account
  if (boundmask > 0)
  {
    int nboundmask = 0;
    for (int k = 0; k < NTOTAL; k++)
    {
                              // check existing neighbours
      if (neighbours[k] == nullptr)
        continue;

      if (k == TOP && (boundmask & BOUND_TOP))
      {
        Vector4 nnormal = neighbours[k]->getAverageNormalPrim(SIZE - 1,j,nboundmask);
        normal += nnormal;
        count++;
      }

      if (k == BOTTOM && (boundmask & BOUND_BOTTOM))
      {
        Vector4 nnormal = neighbours[k]->getAverageNormalPrim(0,j,nboundmask);
        normal += nnormal;
        count++;
      }

      if (k == LEFT && (boundmask & BOUND_LEFT))
      {
        Vector4 nnormal = neighbours[k]->getAverageNormalPrim(i,SIZE - 1,nboundmask);
        normal += nnormal;
        count++;
      }

      if (k == RIGHT && (boundmask & BOUND_RIGHT))
      {
        Vector4 nnormal = neighbours[k]->getAverageNormalPrim(i,0,nboundmask);
        normal += nnormal;
        count++;
      }
    }
  }
                              // average across own file and neighbours
  normal /= float(count);
  return normal;
}

void HGTFile::initNeighbours()
{
#if 1
  for (auto n : neighbours)
  {
    n = nullptr;
  }
#else
  for (int i = 0; i < NTOTAL; i++)
  {
    neighbours[i] = nullptr;
  }
#endif
}

int HGTFile::getNeighbours(int i, int j, int neighbours[8])
{
  if (i < 0 || j < 0)
    return 0;
  if (i >= SIZE || j >= SIZE)
    return 0;

  int count = 0;
  if (i == 0 && j == 0)
  {
    neighbours[count++] = pointNumber(i + 1,j);
    neighbours[count++] = pointNumber(i + 1,j + 1);
    neighbours[count++] = pointNumber(i,j + 1);
  } else if (i == 0 && j == SIZE - 1)
  {
    neighbours[count++] = pointNumber(i,j - 1);
    neighbours[count++] = pointNumber(i + 1,j - 1);
    neighbours[count++] = pointNumber(i + 1,j);
  } else if (i == SIZE - 1 && j == 0)
  {
    neighbours[count++] = pointNumber(i,j + 1);
    neighbours[count++] = pointNumber(i - 1,j + 1);
    neighbours[count++] = pointNumber(i - 1,j);
  } else if (i == SIZE - 1 && j == SIZE - 1)
  {
    neighbours[count++] = pointNumber(i,j - 1);
    neighbours[count++] = pointNumber(i - 1,j - 1);
    neighbours[count++] = pointNumber(i - 1,j);
  } else if (i == 0)
  {
    neighbours[count++] = pointNumber(i,j - 1);
    neighbours[count++] = pointNumber(i,j + 1);
    neighbours[count++] = pointNumber(i + 1,j - 1);
    neighbours[count++] = pointNumber(i + 1,j);
    neighbours[count++] = pointNumber(i + 1,j + 1);
  } else if (i == SIZE - 1)
  {
    neighbours[count++] = pointNumber(i,j - 1);
    neighbours[count++] = pointNumber(i,j + 1);
    neighbours[count++] = pointNumber(i - 1,j - 1);
    neighbours[count++] = pointNumber(i - 1,j);
    neighbours[count++] = pointNumber(i - 1,j + 1);
  } else if (j == 0)
  {
    neighbours[count++] = pointNumber(i - 1,j);
    neighbours[count++] = pointNumber(i + 1,j);
    neighbours[count++] = pointNumber(i - 1,j + 1);
    neighbours[count++] = pointNumber(i,j + 1);
    neighbours[count++] = pointNumber(i + 1,j + 1);
  } else if (j == SIZE - 1)
  {
    neighbours[count++] = pointNumber(i - 1,j);
    neighbours[count++] = pointNumber(i + 1,j);
    neighbours[count++] = pointNumber(i - 1,j - 1);
    neighbours[count++] = pointNumber(i,j - 1);
    neighbours[count++] = pointNumber(i + 1,j - 1);
  } else
  {
    neighbours[count++] = pointNumber(i - 1,j - 1);
    neighbours[count++] = pointNumber(i,j - 1);
    neighbours[count++] = pointNumber(i + 1,j - 1);
    neighbours[count++] = pointNumber(i + 1,j);
    neighbours[count++] = pointNumber(i + 1,j + 1);
    neighbours[count++] = pointNumber(i,j + 1);
    neighbours[count++] = pointNumber(i - 1,j + 1);
    neighbours[count++] = pointNumber(i - 1,j);
  }

  return count;
}

int HGTFile::findNeighbourHeight(int i, int j)
{
                              // find neighbour points
  int neighbours[8];
  int n = getNeighbours(i,j,neighbours);
                              // find average defined height
  int count = 0;
  float sum = 0.0;
  for (int i = 0; i < n; i++)
  {
    if (data[neighbours[i]] != DEPTH_UNDEFINED)
    {
      sum += static_cast<float>(data[neighbours[i]]);
      count++;
    }
  }

  if (count > 0)
  {
                              // success
    sum /= static_cast<float>(count);
    return ROUND(sum);
  } else
  {
                              // failure
    return 0;
  }
}

const string& HGTFile::errorString()
{
  return errorString_;
}

                              // boundary mask
#undef BOUND_TOP
#undef BOUND_BOTTOM
#undef BOUND_LEFT
#undef BOUND_RIGHT
