# EarthSurface

  Earth surface geometry (elevation)
  ----------------------------------
  This is an example how to load and correct any Earth surface geometry from SRTM data
and save it as an STL file.

  Compiler
  --------
  VS 2019, C++, without Windows specifics except <I>URLDownloadToFile()</I>.

  How it works
  ------------
  The program downloads an SRTM height file into HGTFile class and water body data into ESRIFile class.
The first class contains Earth surface elevations with ~100x100 metres resolution,
the second contains water body contours which are  much more accurate.

  I wrote visualisation of Earth surface in marine simulators before and
ran into a problem : when a ship is in water on charts visuals may show that
it is aground. This was caused by inaccuracy of HGT data, so I had to improve it by 
correcting the data by a constrained free form deformation algorithm (FFD) described in the <I>Documents</I>
folder. It changes HGT file data to conform to real water shore lines.

  The HGT files are zipped and ESRI files are zipped as well (three files in a zip). A HGT
file structure is simple - it contains 1201 x 1201 elevation points in 2-byte signed integers.
An ESRI file is more compilicated - a zip contains three files. The ESRIFile class extracts
this data into groups of points organised as lat,lon and height. Each file covers area of
1 x 1 degree and is named after lat/lon of its lower left corner.

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
    
  Third-party
  -----------
  zip library only to unzip downloaded files. Readme and licence are inside zlib-win64 directory.
  
