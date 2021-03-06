#include "HGTFiles.h"
#include "ESRIFiles.h"
#include "Strings.h"
#include "Vector.h"
#include <limits>
#include <fstream>

#include "zlib-win64\zlib.h"
#include "zlib-win64\contrib\minizip\unzip.h"
#include "zlib-win64\contrib\minizip\iowin32.h"
#include "zlib-win64\contrib\minizip\ioapi.h"

using namespace std;

/** Poly closed? */
template <class T>
bool PolyClosed(vector<TVector<T> > &poly, T tolerance)
{
  if (poly.size() < 3)
    return false;

  T dist = !(poly[0] - poly[poly.size() - 1]);
  bool closed = (dist < tolerance);

  return closed; 
}

/** Get signed poly area,
  poly area is two-dimensional : only X,Y components are meaningful */
template <class T>
T PolyAreaSigned(vector<TVector<T> > &poly, T tolerance)
{
  T area = T(0.0);

  int n = static_cast<int>(poly.size());

  if (n < 3) 
    return T(0.0);
                              // closed?
  bool closed = PolyClosed(poly,tolerance);

  for (int i = 0; i < n - 1; i++)
  {
    T x1 = poly[i].X;
    T y1 = poly[i].Y;
    T x2 = poly[i + 1].X;
    T y2 = poly[i + 1].Y;
    area += (x1 * y2 - y1 * x2);
  };

  if (!closed)
    area += (poly[0].Y * poly[n - 1].X - poly[0].X * poly[n - 1].Y);

  area /= T(2.0);

  return area;
}

/** Is point inside polygon? Simple ray-tracing (two-dimensional) */
template <class T>
bool InsidePoly(vector<TVector<T> > &poly, const TVector<T> &point, T tolerance)
{
  int i, j, nvert = static_cast<int>(poly.size());
  bool c = false;

  for (i = 0, j = nvert - 1; i < nvert; j = i++) 
  {
    if (((poly[i].Y >= point.Y) != (poly[j].Y >= point.Y)) &&
      (point.X <= (poly[j].X - poly[i].X) * (point.Y - poly[i].Y) / (poly[j].Y - poly[i].Y) + poly[i].X))
    c = !c;
  }

  return c;
}

/** Swap bytes of 32-bit integer */
inline void Swap32(int *v)
{
  char *b = (char *) v;
  char temp = b[0];
  b[0] = b[3];
  b[3] = temp;
  temp = b[1];
  b[1] = b[2];
  b[2] = temp;
}

/** Change endianness for an array of size 32-bit integers */
void BSwap32(size_t size, void *addr)
{
  int *v = (int *) addr;
  for (size_t i = 0; i < size; i++)
  {
    Swap32(v);
    v++;
  }
}

/** Part pf ESRI file name */
const char ESRIContDeliveryChars[CD_TOTAL] = {'a','e','f','i','n','s'};
  
                              // example : e012s05f.zip, upcase : E012S05F.ZIP
bool ESRIFilenameToLatLon(const string &filename, double *lat, double *lon, int *contdelivery,
  string &errorString)
{
                              // parse file name to get lat/lon min/max
  string s = upCase(forceExtension(justFileName(filename),""));

  if (s.length() != 8)
  {
    errorString = string("SWBD file ") + filename + 
      " has a wrong length (should be like W003N50E.*)";
    return false;
  }
                              // parse continent
  if (s[7] == 'A')
  {
    *contdelivery = CD_AUSTRALIA;
  } else if (s[7] == 'E')
  {
    *contdelivery = CD_EURASIA;
  } else if (s[7] == 'F')
  {
    *contdelivery = CD_AFRICA;
  } else if (s[7] == 'I')
  {
    *contdelivery = CD_ISLANDS;
  } else if (s[7] == 'N')
  {
    *contdelivery = CD_NAMERICA;
  } else if (s[7] == 'S')
  {
    *contdelivery = CD_SAMERICA;
  } else 
  {
    errorString = string("SWBD file ") + filename + 
      " has a wrong last character (continental delivery); it should be like W003N50E.*";
    return false;
  }

  if (s[0] == 'E')
  {
    string sl = getSubString(s,1,3);
    *lon = atof(sl.c_str());
  } else if (s[0] == 'W')
  {
    string sl = getSubString(s,1,3);
    *lon = -atof(sl.c_str());
  } else
  {
    errorString = string("SWBD file ") + filename + 
      " has a wrong first character (can be E or W); it should be like W003N50E.*";
    return false;
  }

  if (s[4] == 'N')
  {
    string sl = getSubString(s,5,6);
    *lat = atof(sl.c_str());
  } else if (s[4] == 'S')
  {
    string sl = getSubString(s,5,6);
    *lat = -atof(sl.c_str());
  } else
  {
    errorString = string("SWBD file ") + filename + 
      " has a wrong fifth character (can be N or S); it should be like W003N50E.*";
    return false;
  }
                              // full success
  return true;
}

bool ESRIFilenameToRect(const string &filename, SLatLonRect *rect, string &errorString)
{
  int contdelivery = 0;

  bool OK = ESRIFilenameToLatLon(filename,&rect->latmin,&rect->lonmin,&contdelivery,errorString);
  if (OK)
  {
    rect->latmax = rect->latmin + 1.0;
    rect->lonmax = rect->lonmin + 1.0;
    CorrectLat(rect->latmax);
    CorrectLon(rect->lonmax);
  }
  return OK;
}
                              // e012s05f
string LatLonToESRIFilename(double lat, double lon, int contdelivery)
{
  assert(contdelivery >= 0 && contdelivery < CD_TOTAL);

  CorrectLat(lat);
  CorrectLon(lon);

  int ilat = ROUND(lat);
  int ilon = ROUND(lon);

  string filename = 
    ((ilon >= 0) ? string("e") : string("w")) +
    to_string(abs(ilon),3) +
    ((ilat >= 0) ? string("n") : string("s")) +
    to_string(abs(ilat),2) + ESRIContDeliveryChars[contdelivery];

  return filename;
}

const double ESRIFile::ESRInodata = -1.0e-38;

ESRIFile::ESRIFile() : LatLonRect(), contdelivery(NOT_DEFINED)
{
  OK = false;
  dataOK = true;
}
                               // file exists?
bool static fileExists(const string &path)
{
  ifstream f(path.c_str());
  return f.good();
}

ESRIFile::ESRIFile(const char *filename) : LatLonRect(), contdelivery(NOT_DEFINED)
{
  OK = false;
  dataOK = true;

  string sfilename = string(filename);
                            // file exists?
  if (!fileExists(sfilename))
    return;
                            // parse file name
  string justfilename = justFileName(sfilename);
  if (!ESRIFilenameToLatLon(justfilename,&latmin,&lonmin,&contdelivery,errorString))
    return;
                            // set rectangle limits
                            //NB periodic longitude is not treated here
  latmax = latmin + 1.0;
  lonmax = lonmin + 1.0;
                              // open file
  zlib_filefunc64_def ffunc;
  fill_win32_filefunc64A(&ffunc);
  unzFile uf = unzOpen2_64(filename,&ffunc);
  void *buffer = nullptr;
  unsigned int size = 0;

  if (uf == nullptr)
    return;
                              // load main shape file
  string shpfile = forceExtension(justFileName(sfilename),"shp");
  int res = unzExtractCurrentData(uf,shpfile.c_str(),nullptr,&buffer,&size);
                              // failure
  if (res != 0)
  {
    if (buffer != nullptr) 
      free(buffer);
    unzClose(uf);
    return;
  }

  if (buffer == nullptr)
  {
    unzClose(uf);
    return;
  }
                              // extract records
  OK = ExtractSHP((char *) buffer,size);
                              // this buffer done
  free(buffer);

  if (!OK) 
  {
    unzClose(uf);
    return;
  }
                              // load index file (not needed actually) -
                              // it contains records file offsets and lengths,
                              // but we keep all records already in memory
  string shxfile = forceExtension(justFileName(sfilename),"shx");
  res = unzExtractCurrentData(uf,shxfile.c_str(),nullptr,&buffer,&size);
                              // failure
  if (buffer == nullptr)
  {
    unzClose(uf);
    return;
  }
                              // extract records
  OK = ExtractSHX((char *) buffer,size);
                              // SHX info in not important - it contains
                              // record offsets and lengths with a file;
                              // we keep everything in memory, therefore
                              // not very worried about file offsets
  OK = true;
                              // this buffer done
  free(buffer);

  if (!OK) 
  {
    unzClose(uf);
    return;
  }
                              // load DBF
  string dbffile = forceExtension(justFileName(sfilename),"dbf");
  res = unzExtractCurrentData(uf,dbffile.c_str(),nullptr,&buffer,&size);
                              // failure
  if (buffer == nullptr)
  {
    unzClose(uf);
    return;
  }
                              // extract records
  OK = ExtractDBF((char *) buffer,size);
                              // this buffer done
  free(buffer);
                              // success
  unzClose(uf);
                              // every first part of a record must have positive area
                              // (e.g. N05E100 is contains erroneous data)
  for (size_t i = 0; i < records.size(); i++)
  {
    ESRIfloat area = records[i].Area(0,1.0);
    if (area > 0.0)
      dataOK = false;
  }
                              // success
  OK = dataOK;
}

ESRIFile::~ESRIFile()
{
}

bool ESRIFile::ExtractSHP(char *data, int size)
{
                            // check header sizes
  assert(sizeof(SSHPHeader) == 100);
  if (sizeof(SSHPHeader) != 100)
    return false;

  assert(sizeof(SSHPRecHeader) == 8);
  if (sizeof(SSHPRecHeader) != 8)
    return false;
                            // read header
  int offset = 0;
  memmove(&SHPheader,data,sizeof(SHPheader));
  offset += sizeof(SHPheader);
                            // convert big to little
  SHPheader.CorrectEndianness();
                            // wrong shape type
  if (!SHPheader.ShapeTypeOK())
    return false;
                            // read records
  while (offset < size)
  {
    SSHPRecHeader recheader = {0};
    memmove(&recheader,data + offset,sizeof(recheader));
    offset += sizeof(recheader);
                            // convert big to little
    recheader.CorrectEndianness();
                            // data length in bytes?
    ESRIint recsize = recheader.contlength * 2;
                            // read shape type
    ESRIint shapetype = 0;
    memmove(&shapetype,data + offset,sizeof(shapetype));
    offset += sizeof(shapetype);

    switch (shapetype) {
      case ESRI_NULLSHAPE :
      {
        CESRIRecord record(shapetype);

        records.push_back(record);

        break;
      }
      case ESRI_POINT :
      {
        CESRIRecord record(shapetype);

        TVector<ESRIfloat> point = record.ReadXYPoint(data,offset);

        record.min = record.max = point;
        record.points.push_back(point);

        records.push_back(record);

        break;
      }
      case ESRI_MULTIPOINT :
      {
        CESRIRecord record(shapetype);

        record.min = record.ReadXYPoint(data,offset);
        record.max = record.ReadXYPoint(data,offset);

        ESRIint numpoints = record.ReadNumPoints(data,offset);

        for (int i = 0; i < numpoints; i++)
        {
          TVector<ESRIfloat> point = record.ReadXYPoint(data,offset);
          record.points.push_back(point);
        }

        records.push_back(record);

        break;
      }
      case ESRI_POLYLINE : case ESRI_POLYGON :
      {
        CESRIRecord record(shapetype);

        record.min = record.ReadXYPoint(data,offset);
        record.max = record.ReadXYPoint(data,offset);

        ESRIint numparts = record.ReadNumPoints(data,offset);
        ESRIint numpoints = record.ReadNumPoints(data,offset);
                              // read boundaries of parts, 0-based
        record.parts.resize(numparts,-1);
        memmove(&record.parts[0],data + offset,sizeof(ESRIint) * numparts);
        offset += sizeof(ESRIint) * numparts;
                              // read points
        for (int i = 0; i < numpoints; i++)
        {
          TVector<ESRIfloat> point = record.ReadXYPoint(data,offset);
          record.points.push_back(point);
        }

        records.push_back(record);

        break;
      }
      case ESRI_POLYGONZ :
      {
        CESRIRecord record(shapetype);

        record.min = record.ReadXYPoint(data,offset);
        record.max = record.ReadXYPoint(data,offset);

        ESRIint numparts = record.ReadNumPoints(data,offset);
        ESRIint numpoints = record.ReadNumPoints(data,offset);
                              // read boundaries of parts, 0-based
        record.parts.resize(numparts,-1);
        memmove(&record.parts[0],data + offset,sizeof(ESRIint) * numparts);
        offset += sizeof(ESRIint) * numparts;
                              // read points
        for (int i = 0; i < numpoints; i++)
        {
          TVector<ESRIfloat> point = record.ReadXYPoint(data,offset);
          record.points.push_back(point);
        }
        TVector<ESRIfloat> zrange = record.ReadXYPoint(data,offset);
        record.min.Z = zrange.X;
        record.max.Z = zrange.Y;
                              // read Z
        for (int i = 0; i < numpoints; i++)
        {
          record.points[i].Z = record.ReadFloat(data,offset);
        }

        if (offset < size)
        {
                                // check if we have next M array or not :
                                // read next shape type. if this is not as
                                // previous, read M array
          offset += sizeof(recheader);
                                  // read shape type
          ESRIint shapetype = 0;
          memmove(&shapetype,data + offset,sizeof(shapetype));
          offset += sizeof(shapetype);
                                  // roll back
          offset -= (sizeof(shapetype) + sizeof(recheader));
                                  // read M array
          if (shapetype != ESRI_POLYGONZ)
          {
            TVector<ESRIfloat> mrange = record.ReadXYPoint(data,offset);
            record.min.W = mrange.X;
            record.max.W = mrange.Y;
                                  // read M into W
            for (int i = 0; i < numpoints; i++)
            {
              record.points[i].W = record.ReadFloat(data,offset);
            }
          }
        }

        records.push_back(record);

        break;
      }
      default :
      {
        return false;
      }
    }
  }

  return true;
}

bool ESRIFile::ExtractSHX(char *data, int size)
{
                            // check header sizes
  assert(sizeof(SSHPHeader) == 100);
  if (sizeof(SSHPHeader) != 100)
    return false;

  assert(sizeof(SSHPRecHeader) == 8);
  if (sizeof(SSHPRecHeader) != 8)
    return false;
                            // read header
  int offset = 0;
  memmove(&SHPheader,data,sizeof(SHPheader));
  offset += sizeof(SHPheader);
                            // convert big to little
  SHPheader.CorrectEndianness();
                            // wrong shape type
  if (!SHPheader.ShapeTypeOK())
    return false;
                            // read records
  ESRIint reccount = 0;
  while (offset < size)
  {
    SSHXRecHeader recheader = {0};
    memmove(&recheader,data + offset,sizeof(recheader));
    offset += sizeof(recheader);
                            // convert big to little
    recheader.CorrectEndianness();

    records[reccount].offset = recheader.offset * 2;
    records[reccount].length = recheader.contlength * 2;

    reccount++;
  }

  //assert(reccount == records.size());

  return (reccount == records.size());
}

bool ESRIFile::ExtractDBF(char *data, int size)
{
  assert(sizeof(SDBFFieldDescriptor) == 32);

  ESRIint offset = 0;
  SDBFHeader header = {0};
                            // read
  memmove(&header,data + offset,sizeof(header));
  offset += sizeof(header);
                            // read records (one field only)
  ESRIint count = 0;
  ESRIint reccount = 0;
  vector<char> buffer(header.recordsize - 1);
  offset = header.headersize;
  while (offset < size && reccount < header.numrecords)
  {
                            // read "record OK" byte; asterisk * means record
                            // is deleted
    unsigned char OK = ' ';
    memmove(&OK,data + offset,sizeof(OK));
    offset += sizeof(OK);
                            // read field
    memmove(&buffer[0],data + offset,buffer.size());
    offset += static_cast<ESRIint>(buffer.size());

    ESRIint type = NOT_DEFINED;
    buffer[5] = 0;
    string str = &buffer[0];
    if (str == "BA040")
    {
      type = SWBD_OCEAN;
    } else if (str == "BH080")
    {
      type = SWBD_LAKE;
    } else if (str == "BH140")
    {
      type = SWBD_RIVER;
    } else
    {
      assert(false);
      return false;
    }

    if (OK == ' ')
    {
      records[count].SWBDtype = type;
      count++;
    }

    reccount++;
  }

  return (count == records.size());
}

ESRIFile::CESRIRecord::CESRIRecord()
{
  shapetype = ESRI_NULLSHAPE;
  SWBDtype = NOT_DEFINED;
  offset = length = 0;
}

ESRIFile::CESRIRecord::CESRIRecord(ESRIint pshapetype)
{
  shapetype = pshapetype;
  SWBDtype = NOT_DEFINED;
  offset = length = 0;
}


ESRIFile::CESRIRecord::~CESRIRecord()
{
}

#define COPY(object) object = other.object

ESRIFile::CESRIRecord::CESRIRecord(const ESRIFile::CESRIRecord &other)
{
  COPY(shapetype);
  COPY(SWBDtype);
  COPY(offset);
  COPY(length);
  COPY(min);
  COPY(max);
  COPY(parts);
  COPY(points);
}

ESRIFile::CESRIRecord &ESRIFile::CESRIRecord::operator=(const ESRIFile::CESRIRecord &other)
{
  COPY(shapetype);
  COPY(SWBDtype);
  COPY(offset);
  COPY(length);
  COPY(min);
  COPY(max);
  COPY(parts);
  COPY(points);

  return *this;
}

TVector<ESRIfloat> ESRIFile::CESRIRecord::ReadXYPoint(char *data, int &offset)
{
  TVector<ESRIfloat> point;

  memmove(&point,data + offset,sizeof(ESRIfloat) * 2);
  offset += sizeof(ESRIfloat) * 2;
  point.Z = ESRInodata;

  return point;
}

ESRIfloat ESRIFile::CESRIRecord::ReadFloat(char *data, int &offset)
{
  ESRIfloat value = ESRInodata;

  memmove(&value,data + offset,sizeof(ESRIfloat));
  offset += sizeof(ESRIfloat);

  return value;
}

ESRIint ESRIFile::CESRIRecord::ReadNumPoints(char *data, int &offset)
{
  ESRIint numpoints;

  memmove(&numpoints,data + offset,sizeof(numpoints));
  offset += sizeof(numpoints);

  return numpoints;
}

ESRIfloat ESRIFile::CESRIRecord::Area(int partno, ESRIfloat accuracymetres)
{
  assert(partno < static_cast<int>(parts.size()));

                              // 1 degree is very approximately 100km
  ESRIfloat tolerance = accuracymetres / 100000.0;

  if (partno < parts.size())
  {
    int i0 = parts[partno];
    int i1 = (partno < static_cast<int>(parts.size()) - 1) ? parts[partno + 1] - 1 : 
      static_cast<int>(points.size()) - 1;
                              // take points from this part
    int numpoints = i1 - i0 + 1;
    vector<TVector<ESRIfloat> > points2;
    points2.reserve(numpoints);
    for (int m = i0; m <= i1; m++)
    {
      points2.push_back(points[m]);
    }
                              // calculate
    ESRIfloat area = PolyAreaSigned<ESRIfloat>(points2,tolerance);

    return area;
  } else
  {
    return 0.0;
  }
}

ESRIint ESRIFile::CESRIRecord::PointInside(ESRIfloat lat, ESRIfloat lon, ESRIfloat accuracymetres)
{
                              // 1 degree is very approximately 100km
  ESRIfloat tolerance = accuracymetres / 100000.0;

  for (int l = 0; l < static_cast<int>(parts.size()); l++)
  {
    int i0 = parts[l];
    int i1 = (l < static_cast<int>(parts.size()) - 1) ? parts[l + 1] - 1 : 
      static_cast<int>(points.size()) - 1;
                              // take points from this part
    int numpoints = i1 - i0 + 1;
    vector<TVector<ESRIfloat> > points2;
    points2.reserve(numpoints);
    for (int m = i0; m <= i1; m++)
    {
      points2.push_back(points[m]);
    }
                              // take water only
    ESRIfloat area = PolyAreaSigned<ESRIfloat>(points2,tolerance);
    if (area > 0)
    {
                              // returns part number if success
      if (InsidePoly<ESRIfloat>(points2,TVector<ESRIfloat>(lon,lat),tolerance))
        return l;
    }
  }
 
  return -1;
}


ESRIint ESRIFile::PointInside(ESRIfloat lat, ESRIfloat lon, ESRIfloat accuracymetres)
{
  for (ESRIint i = 0; i < static_cast<ESRIint>(records.size()); i++)
  {
    if (records[i].PointInside(lat,lon,accuracymetres))
      return i;
  }

  return -1;
}

