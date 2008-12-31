
#include "fileio.hpp"
#include "uquadtree.hpp"
#include <math.h>
using namespace std;
/**
 * Loads from a netCDF file.
 * Elevation values are assumed to be integer meters.  Projection is
 * assumed to be geographic.
 *
 * You should call SetupConversion() after loading if you will be doing
 * heightfield operations on this grid.
 *
 * \returns \c true if the file was successfully opened and read.
 */
void load_hm_file(HeightMapInfo *hm,const char *filename){
  FILE *fp= fopen(filename,"rb");
  if(!fp){
    fprintf(stderr,"Cannot open %s\n",filename);
    return;
  }
  float data[2];
  int idata[2];
	
  fread((char *)data,sizeof(float),2,fp);
  hm->x_origin=data[0];
  hm->y_origin=data[1];

  fread((char *)data,sizeof(float),2,fp);
  float min=data[0];
  float max=data[1];
  float zrange = max-min;
  fread((char *)idata,sizeof(int),2,fp);
  hm->YSize=idata[0];
  hm->XSize=idata[1];

  hm->RowWidth=hm->XSize;
  hm->Scale=5;
  hm->Data = new uint16[hm->XSize * hm->YSize];
  float tmp;
  int range = (int) pow(2,8) - 1;
  for(int i=0; i < hm->XSize * hm->YSize; i++){
    fread((char *)&tmp,sizeof(float),1,fp);
    //if(isinf(tmp))
    //	    hm->Data[i]=100;//	    
    tmp=min;
    //	    hm->Data[i]=0;
    // else
    //	  else
    // hm->Data[i]=0;
    hm->Data[i]=((uint16)(((tmp-min)/zrange)*(range)));
    //printf("%f %f %f %d %d\n",tmp, (tmp-min)/zrange,zrange,range,hm->Data[i]);
  }
  hm->x_origin=24576;
  hm->y_origin=24576;

}
bool LoadFromCDF(const char *szFileName,short &nx, short &ny,float *&data)
{
        int id;


        /* open existing netCDF dataset */
        int status = nc_open(szFileName, NC_NOWRITE, &id);
        if (status != NC_NOERR)
                return false;

        // get dimension IDs
        int id_side = 0, id_xysize = 0;
        nc_inq_dimid(id, "side", &id_side);
        status = nc_inq_dimid(id, "xysize", &id_xysize);
        if (status != NC_NOERR)
        {
	  std::string msg;
	  // Error messages can be turned into strings with nc_strerror
	  msg = "Could not determine size of CDF file. Error: ";
	  msg += nc_strerror(status);
	  nc_close(id);                           // close netCDF dataset
	  std::cerr << msg<<endl;
	  return false;
        }

        size_t xysize_length = 0;
        nc_inq_dimlen(id, id_xysize, &xysize_length);

        // get variable IDs
        int id_xrange = 0, id_yrange = 0, id_zrange = 0;
        int id_spacing = 0, id_dimension = 0, id_z = 0;
        nc_inq_varid(id, "x_range", &id_xrange);
        nc_inq_varid(id, "y_range", &id_yrange);
        nc_inq_varid(id, "z_range", &id_zrange);
        nc_inq_varid(id, "spacing", &id_spacing);
        nc_inq_varid(id, "dimension", &id_dimension);
        nc_inq_varid(id, "z", &id_z);

        // get values of variables
        double xrange[2], yrange[2], zrange[2], spacing[2];
        int dimension[2] = { 0, 0 };
        nc_get_var_double(id, id_xrange, xrange);
        nc_get_var_double(id, id_yrange, yrange);
        nc_get_var_double(id, id_zrange, zrange);
        nc_get_var_double(id, id_spacing, spacing);
        nc_get_var_int(id, id_dimension, dimension);

        double *z;
        try
        {
                z = new double[xysize_length];
        }
        catch (bad_alloc&)
        {

	  size_t bytes = sizeof(double)*xysize_length;
	  fprintf(stderr,"Could not allocate %d bytes (%.1f MB, %.2f GB)\n",
		  bytes, (float)bytes/1024/1024, (float)bytes/1024/1024/1024);
	  nc_close(id);                           // close netCDF dataset
	  return false;
        }

      
	data=new float[xysize_length];
        nc_get_var_double(id, id_z, z);
      

        nc_close(id);                           // close netCDF dataset

        // Now copy the values into the vtElevationGrid object
        nx = dimension[0];
        ny = dimension[1];
	
        int i, j;
        for (i = 0; i <nx; i++)
        {
                for (j = 0; j < ny; j++)
                {
		  //SetValue(i, m_iRows-1-j, (short)z[j*m_iColumns+i]);
		  data[i*ny + j]=z[j*ny+i];
                }
        }
        /*if (progress_callback != NULL) progress_callback(90);

        m_proj.SetProjectionSimple(false, 0, EPSG_DATUM_WGS84);

        m_EarthExtents.left = xrange[0];
        m_EarthExtents.right = xrange[1];
        m_EarthExtents.top = yrange[1];
        m_EarthExtents.bottom = yrange[0];

        ComputeCornersFromExtents();
	*/
        // delete temporary storage
        delete z;
	
        return true;


}



