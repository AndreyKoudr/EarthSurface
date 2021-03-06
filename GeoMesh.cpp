#include "defines.h"
#include "Strings.h"
#include "GeoMesh.h"

#include <fstream>
#include <limits>


int GeoMesh::fileversion_ = 0;
int GeoMesh::fileheader_ = 0xED0F5677;
signed short int GeoMesh::DEPTH_UNDEF = -32768;

GeoMesh::GeoMesh() : LatLonRect(), nx_(0), ny_(0), dlat_(0), dlon_(0), celldx_(0), celldy_(0)
{
}

GeoMesh::GeoMesh(short int height, double platmin, double plonmin, double platmax, double plonmax, int pnx, int pny) :
  LatLonRect(platmin,plonmin,platmax,plonmax,&geocoord_), nx_(pnx), ny_(pny)
{
  data_.resize((nx_ + 1) * (ny_ + 1),height);

  geocoord_.setBase((latmin + latmax) * 0.5 * 60.0,(lonmin + lonmax) * 0.5 * 60.0);

  updateCellSizes();
}

GeoMesh::GeoMesh(signed short int *pdata, double platmin, double plonmin, double platmax, 
  double plonmax, int pnx, int pny) :
  LatLonRect(platmin,plonmin,platmax,plonmax,&geocoord_), nx_(pnx), ny_(pny)
{
  data_.resize((nx_ + 1) * (ny_ + 1));

  memmove(&data_[0],pdata,(nx_ + 1) * (ny_ + 1) * sizeof(signed short int));

  geocoord_.setBase((latmin + latmax) * 0.5 * 60.0,(lonmin + lonmax) * 0.5 * 60.0);

  updateCellSizes();
}

bool GeoMesh::createSubMesh(double platmin, double plonmin, double platmax, double plonmax, int margin,
  GeoMesh &submesh, int &rowmin, int &colmin)
{
                        // new row/col
  int rowmax,colmax;
                        // parameters within cell counted right (U) and upward (V)
  double Umin,Vmin,Umax,Vmax;
                        // lower left corner, row is counted downward
  getRowCol(platmin,plonmin,&rowmin,&colmin,&Umin,&Vmin);
  rowmin = ny_ - rowmin;
                        // upper right corner, row is counted downward
  getRowCol(platmax,plonmax,&rowmax,&colmax,&Umax,&Vmax);
  rowmax = ny_ - rowmax;
                        // apply margin
  colmin -= margin;
  if (colmin < 0) 
    colmin = 0;
  rowmin -= margin;
  if (rowmin < 0) 
    rowmin = 0;

  colmax += margin;
  if (colmax > nx_ - 1) 
    colmax = nx_ - 1;
  rowmax += margin;
  if (rowmax > ny_ - 1) 
    rowmax = ny_ - 1;

  int subnx = colmax - colmin + 1;
  int subny = rowmax - rowmin + 1;

  double sublatmin = latmin + static_cast<double>(rowmin) * (latmax - latmin) / static_cast<double>(ny_);
  double sublatmax = latmin + static_cast<double>(rowmax + 1) * (latmax - latmin) / static_cast<double>(ny_);
  double sublonmin = lonmin + static_cast<double>(colmin) * (lonmax - lonmin) / static_cast<double>(nx_);
  double sublonmax = lonmin + static_cast<double>(colmax + 1) * (lonmax - lonmin) / static_cast<double>(nx_);
                              // check if new rectangle is inside old
  assert(sublatmin >= latmin && sublatmin <= latmax);
  assert(sublatmax >= latmin && sublatmax <= latmax);
  assert(sublonmin >= lonmin && sublonmin <= lonmax);
  assert(sublonmax >= lonmin && sublonmax <= lonmax);
  assert(sublatmax > sublatmin);
  assert(sublonmax > sublonmin);
                              // create flat sub mesh
  submesh.LatLonRect::LatLonRect(sublatmin,sublonmin,sublatmax,sublonmax);
  submesh.nx_ = subnx;
  submesh.ny_ = subny;

  submesh.data_.resize((submesh.nx_ + 1) * (submesh.ny_ + 1));

  submesh.geocoord_.setBase((submesh.latmin + submesh.latmax) * 0.5 * 60.0,
    (submesh.lonmin + submesh.lonmax) * 0.5 * 60.0);

  submesh.updateCellSizes();
                              // copy data_ for subrectangle
  int row = rowmin;
  for (int i = 0; i <= subny; i++)
  {
    int datarow = ny_ - row;
    signed short int *addrfrom = dataAddr(datarow,colmin);
    signed short int *addrto = submesh.dataAddr(subny - i,0);

    memmove(addrto,addrfrom,(subnx + 1) * sizeof(signed short int));

    row++;
  }

  return true;
}

void GeoMesh::updateCellSizes()
{
  dlat_ = (latmax - latmin) / static_cast<double>(ny_);
  dlon_ = (lonmax - lonmin) / static_cast<double>(nx_);

  double xmin,ymin,xmax,ymax;
  geocoord_.latLon2Linear(latmin * 60.0,lonmin * 60.0,xmin,ymin);
  geocoord_.latLon2Linear(latmax * 60.0,lonmax * 60.0,xmax,ymax);
  double dx = fabs(xmax - xmin);
  double dy = fabs(ymax - ymin);

  celldx_ = dx / static_cast<double>(nx_);
  celldy_ = dy / static_cast<double>(ny_);
}

int GeoMesh::getNeighbourPoints(int i, int j, int nei[4])
{
                              // upper left
  if (i == 0 && j == 0)
  {
    nei[0] = getPointNumber(1,0);
    nei[1] = getPointNumber(0,1);
    return 2;
                              // upper right
  } else if (i == 0 && j == nx_)
  {
    nei[0] = getPointNumber(0,nx_ - 1);
    nei[1] = getPointNumber(1,nx_);
    return 2;
                              // lower left
  } else if (i == ny_ && j == 0)
  {
    nei[0] = getPointNumber(ny_,1);
    nei[1] = getPointNumber(ny_ - 1,0);
    return 2;
                              // lower right
  } else if (i == ny_ && j == nx_)
  {
    nei[0] = getPointNumber(ny_ - 1,nx_);
    nei[1] = getPointNumber(ny_,nx_ - 1);
    return 2;
                              // upper
  } else if (i == 0)
  {
    nei[0] = getPointNumber(i,j - 1);
    nei[1] = getPointNumber(i + 1,j);
    nei[2] = getPointNumber(i,j + 1);
    return 3;
                              // lower
  } else if (i == ny_)
  {
    nei[0] = getPointNumber(i,j + 1);
    nei[1] = getPointNumber(i - 1,j);
    nei[2] = getPointNumber(i,j - 1);
    return 3;
                              // left
  } else if (j == 0)
  {
    nei[0] = getPointNumber(i + 1,j);
    nei[1] = getPointNumber(i,j + 1);
    nei[2] = getPointNumber(i - 1,j);
    return 3;
                              // right
  } else if (j == nx_)
  {
    nei[0] = getPointNumber(i - 1,j);
    nei[1] = getPointNumber(i,j - 1);
    nei[2] = getPointNumber(i + 1,j);
    return 3;
  } else
                              // inner point
  {
    nei[0] = getPointNumber(i + 1,j);
    nei[1] = getPointNumber(i,j + 1);
    nei[2] = getPointNumber(i - 1,j);
    nei[3] = getPointNumber(i,j - 1);
    return 4;
  }
}

int GeoMesh::getNeighbourPoints(int i, int j, int inei[4], int jnei[4])
{
  int nei[4];
  int num = getNeighbourPoints(i,j,nei);

  for (int k = 0; k < num; k++)
  {
    inei[k] = nei[k] / (nx_ + 1);
    jnei[k] = nei[k] % (nx_ + 1);

    assert(nei[k] == getPointNumber(inei[k],jnei[k]));
  }

  return num;
}

bool GeoMesh::getRowCol(double lat, double lon, int *row, int *col, double *U, double *V)
{
  assert(data_.size() > 0);
  assert(dlat_ > 0);
  assert(dlon_ > 0);

#ifdef _DEBUG
  // AreaCreator DEBUG for N59S110
  if (fabs(lat - 60.0) < 0.00000001 && fabs(lon + 110.0) < 0.00000001)
  {
    int gsggsgsg = 0;
  }
#endif

  if (inside(lat,lon))
  {
    *row = (int) ((lat - latmin) / dlat_);
                              // 15-07-05
    LIMIT(*row,0,ny_ - 1);
//    LIMIT(*row,0,ny_);
    *col = (int) ((lon - lonmin) / dlon_);
                              // 15-07-05
    LIMIT(*col,0,nx_ - 1);
//    LIMIT(*col,0,nx_);

    if (U != nullptr)
    {
      *U = ((lon - lonmin) - dlon_ * (double) (*col)) / dlon_;
    }

    if (V != nullptr)
    {
      *V = ((lat - latmin) - dlat_ * (double) (*row)) / dlat_;
    }

    *row = ny_ - *row;
                              
    return true;                  
  } else
  {
    return false;
  }
}

signed short int *GeoMesh::dataAddr(int row, int col)
{
  return &data_[row * (nx_ + 1) + col];
}

signed short int GeoMesh::getZ(double lat, double lon)
{
  assert(data_.size() > 0);

  int row,col;
  if (getRowCol(lat,lon,&row,&col))
  {
                              // get sample
    return data_[row * (nx_ + 1) + col];                  
  } else
  {
    return DEPTH_UNDEF;
  }
}

bool GeoMesh::getDepth(double lat, double lon, float *depth)
{
  *depth = getZAccurate<float>(lat,lon,nullptr);

  return (*depth < 0.0);
}

void GeoMesh::getPointParms(int row, int col, double *lat, double *lon, float *height)
{      
  int index = row * (nx_ + 1) + col;
  double dlat_ = latmax - latmin;
  double dlon_ = lonmax - lonmin;
  *lon = lonmin + dlon_ * (double) col / (double) nx_;
  *lat = latmin + dlat_ * (double) (ny_ - row) / (double) nx_;
  assert(*lat >= latmin && *lat <= latmax);
  assert(*lon >= lonmin && *lon <= lonmax);
  LIMIT(*lat,latmin,latmax);
  LIMIT(*lon,lonmin,lonmax);
  *height = (float) data_[index];
}


bool GeoMesh::toTriangles(TTriangles<float> &tris)
{
                            // loop over cells (each cell is 2 triangles), going
                            // DOWNWARD
  for (int i = 0; i < ny_; i++)
  {
    for (int j = 0; j < nx_; j++)
    {
                          // get cell corners
      TVector<float> corners[4];
      corners[0] = getPointVector<float>(i + 1,j);
      corners[1] = getPointVector<float>(i + 1,j + 1);
      corners[2] = getPointVector<float>(i,j + 1);
      corners[3] = getPointVector<float>(i,j);
                          // get 4 normals (this will define subdivision into 2
                          // triangles)
      TVector<float> normals[4];
                          // diagonal 0-2
      normals[0] = +((corners[2] - corners[1]) ^ (corners[0] - corners[1]));
      normals[1] = +((corners[0] - corners[3]) ^ (corners[2] - corners[3]));
                          // diagonal 1-3
      normals[2] = +((corners[1] - corners[0]) ^ (corners[3] - corners[0]));
      normals[3] = +((corners[3] - corners[2]) ^ (corners[1] - corners[2]));
                          // define division
      float dot01 = fabs(normals[0] * normals[1]);
      float dot23 = fabs(normals[2] * normals[3]);

      if (dot01 > dot23)
      {
        tris.addTri(corners[0],corners[1],corners[2],0.0);
        tris.addTri(corners[2],corners[3],corners[0],0.0);
      } else
      {
        tris.addTri(corners[3],corners[0],corners[1],0.0);
        tris.addTri(corners[1],corners[2],corners[3],0.0);
      }
    }
  }

  return true;
}

/*
                              // 30N000E030030.geo
bool GEOFilenameToLatLon(string &filename, double *lat, double *lon,
  double *dlat_, double *dlon_, bool messages)
{
                              // parse file name to get lat/lon min/max
  string s = UpCase(ForceExtension(JustFileName(filename),""));
                              // cut it into two parts
  bool easterly = false;
  int n = Find(s,'E');
  if (n != NOT_DEFINED)
  {
    easterly = true;
  } else
  {
    n = Find(s,'W');
    if (n != NOT_DEFINED)
    {
    } else
    {
      if (messages) ErrorMessageBox(string("GEO file ") + filename + 
        " has a wrong name (should be like 30N000E030030.geo)");
      return false;
    }
  }
                              // go on
                              // divide into two strings
  string slat = GetSubString(s,0,n - 1);
  bool northern = false;
  if (slat[0] == 'N')
  {
    northern = true;
  } else if (slat[0] == 'S')
  {
  } else
  {
    if (messages)
    { 
      ErrorMessageBox(string("GEO file ") + filename + 
        " has a wrong name (should be like 30N000E030030.geo)");
    }
    return false;
  }
  Delete(slat,0,1);
  *lat = atof(slat.c_str())  * (northern ? 1.0 : -1.0);
                              // now longitude
  string slon = GetSubString(s,n + 1,n + 3);
  *lon = atof(slon.c_str()) * (easterly ? 1.0 : -1.0);

  string sdlat = GetSubString(s,n + 4,n + 6);
  *dlat_ = atof(sdlat.c_str());

  string sdlon = GetSubString(s,n + 7,n + 9);
  *dlon_ = atof(sdlon.c_str());
                              // full success
  return true;
}

bool GEOFilenameToRect(string &filename, SLatLonRect *rect)
{
  double dlat_ = 0;
  double dlon_ = 0;
  bool OK = GEOFilenameToLatLon(filename,&rect->latmin,&rect->lonmin,&dlat_,&dlon_,false);
  if (OK)
  {
    rect->latmax = rect->latmin + dlat_;
    rect->lonmax = rect->lonmin + dlon_;
    CorrectLat(rect->latmax);
    CorrectLon(rect->lonmax);
  }
  return OK;
}

string LatLonToGEOFilename(double lat, double lon, double dlat_, double dlon_)
{
  CorrectLat(lat);
  CorrectLon(lon);

  int ilat = ROUND(lat);
  int ilon = ROUND(lon);
  int idlat = ROUND(dlat_);
  int idlon = ROUND(dlon_);

  string filename = 
    ((ilat >= 0) ? string("N") : string("S")) +
    to_string(abs(ilat),2) +
    ((ilon >= 0) ? string("E") : string("W")) +
    to_string(abs(ilon),3) +
    to_string(idlat,3) + to_string(idlon,3);

  return filename;
}

bool GeoMesh::LoadETOPOASCII(string &filename)
{
                            // file exists?
  if (!FileExists(filename))
    return false;
                            // open and read line by line
  std::ifstream file;
  std::ios_base::openmode oMode(std::ios::in|std::ios::binary);
  file.open(filename.c_str(), oMode);

  if (!file.is_open())
  {
    return false;
  }
                          // min/max should be taken from lines
  nx_ = 0;
  ny_ = 0;
                          // rough to compare lat/lon
  double tolerance = 1.0e-6;
                          // inited
  bool inited = false;
  double lonprev = 0.0;
  double latprev = 0.0;
  dlon_ = 0.0;
  dlat_ = 0.0;
                          // empty data_
  data_.clear();
                          // line by line...
  while (!file.eof())
  {
    string line;
    std::getline(file,line);
                          // last line might well be without ending (CR)/LF
    if (file.fail()) {
      break;
    }
                          // trim line
    line = MyProjects1::Common::Trim(line," \n\r\t");
                          // skip empty
    if (line.length() == 0)
      continue;
                          // lon, lat, height
                          // 43.3333333333333428 47.8666666666666671 115
                          // 43.3500000000000085 47.8666666666666671 114
                          // 43.3666666666666742 47.8666666666666671 114
                          // 43.38333333333334 47.8666666666666671 110
    int pos1[100], pos2[100];
    int n = ParseWords(line,' ',pos1,pos2,100);

    if (n == 3)
    {
      double lon = atof(line.substr(pos1[0],pos2[0] - pos1[0] + 1).c_str());
      double lat = atof(line.substr(pos1[1],pos2[1] - pos1[1] + 1).c_str());
      signed short int height = static_cast<signed short int>(atoi(line.substr(pos1[2],pos2[2] - pos1[2] + 1).c_str()));
                              // register height
      data_.push_back(height);
                              // catch dlat_/lon
      if (!inited)
      {
        lonprev = lon;
        latprev = lat;
        lonmin = lonmax = lon;
        latmin = latmax = lat;

        inited = true;
      } else
      {
                              // min/max
        if (lon < lonmin) lonmin = lon;
        if (lon > lonmax) lonmax = lon;
        if (lat < latmin) latmin = lat;
        if (lat > latmax) latmax = lat;
                              // dlon_/dlat_
        if (fabs(lon - lonprev) > tolerance)
        {
          dlon_ = fabs(lon - lonprev);
        }
        if (fabs(lat - latprev) > tolerance)
        {
                              // this is the new row
          if (nx_ == 0)
                              // num cols is actually one less than num points
            nx_ = static_cast<int>(data_.size()) - 2;
                              // count rows
          ny_++;

          dlat_ = fabs(lat - latprev);
        }
                              // min/max

        lonprev = lon;
        latprev = lat;
      }
    }
  }

  file.close();
                              // now check all parameters
  int nxcur = ROUND((lonmax - lonmin) / dlon_);
  int nycur = ROUND((latmax - latmin) / dlat_);

  bool OK1 = (nx_ == nxcur);
  bool OK2 = (ny_ == nycur);

  bool OK3 = ((nx_ + 1) * (ny_ + 1) == data_.size());

  assert(OK1 && OK2 && OK3);

  bool OK = (OK1 && OK2 && OK3);
                              // delete old geo coordinates
  DELETE_CLASS_SETNULL(geocoord_);
                              // reset geo coordinates
  if (OK)
  {
    geocoord_ = new CGeoCoord();
    geocoord_.setBase((latmin + latmax) * 0.5 * 60.0,(lonmin + lonmax) * 0.5 * 60.0);

    updateCellSizes();
  }

  return OK;
}
*/
