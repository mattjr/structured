/*--------------------------------------------------------------------
 *	$Id: grd2ply.cpp,v 1.2 2008-12-09 23:26:03 m.roberson Exp $
 *
 *	Copyright (c) 1991-2007 by P. Wessel and W. H. F. Smith
 *	See COPYING file for copying and redistribution conditions.
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; version 2 of the License.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	Contact info: gmt.soest.hawaii.edu
 *--------------------------------------------------------------------*/
/*
 * grd2xyz.c reads a grid file and prints out the x,y,z values to
 * standard output.
 *
 * Author:	Paul Wessel
 * Date:	3-JAN-1991
 * Version:	4
 */
bool localframe=false;
#include "gmt.h"
#include <GeographicConversions/ufRedfearn.h>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <sstream>

#include <auv_config_file.hpp>

#include <GeographicConversions/ufRedfearn.h>

#include <cv.h>
#include <highgui.h>
#include <TriMesh.h>
using namespace std;
using namespace libplankton;
	int v_count=0;
int f_count=0;
FILE *fp=NULL;
string out_name;
//
// Global variables for command-line arguments
//
static string localiser_config_file_name;
static string mesh_file_name;
static string xy_name;
#include "gmt_parse_z_io.h"	/* To define the Z structure used for parsing */

struct GRD2XYZ_CTRL {
	struct E {	/* -E[<nodata>] */
		BOOLEAN active;
		double nodata;
	} E;
	struct S {	/* -S[r] */
		BOOLEAN active;
		BOOLEAN reverse;
	} S;
	struct W {	/* -W[<weight>] */
		BOOLEAN active;
		double weight;
	} W;
	struct Z Z;
};

int main (int argc, char **argv)
{
	BOOLEAN error = FALSE, global = FALSE, b_only = FALSE;
	BOOLEAN first = TRUE;

	int i, j, k, ij, nm, nx, ny, gmt_ij, n_suppressed = 0, n_out, n_files = 0;
	int n_total = 0;
		
	float *z;
	int vvalid=0;
	int fvalid=0;
	int *sup_remap;
	double w, e, s, n, *x, *y, out[4], d_value;
   double lat_origin, long_origin;
 double gridConvergence, pointScale;
   string zone;
	struct GRD_HEADER grd;
	struct GMT_Z_IO io;
	struct GRD2XYZ_CTRL *Ctrl;

	void *New_Grd2xyz_Ctrl (), Free_Grd2xyz_Ctrl (struct GRD2XYZ_CTRL *C);
	
	argc = GMT_begin (argc, argv);

	Ctrl = (struct GRD2XYZ_CTRL *)New_Grd2xyz_Ctrl ();	/* Allocate and initialize a new control structure */
	
	w = e = s = n = 0.0;

	for (i = 1; i < argc; i++) {
		if (argv[i][0] == '-') {
			switch (argv[i][1]) {
				/* Common parameters */

				case 'b':
					b_only = TRUE;
				case 'H':
				case 'R':
				case 'V':
				case ':':
				case 'f':
				case '\0':
					error += GMT_parse_common_options (argv[i], &w, &e, &s, &n);
					break;

				/* Supplemental options */

				case 'E':
					Ctrl->E.active = TRUE;
					if (argv[i][2]) Ctrl->E.nodata = atof (&argv[i][2]);
					break;
				case 'L':	/* For backwards compatibility only; use -f instead */
					GMT_io.out_col_type[0] = GMT_IS_LON;
					GMT_io.out_col_type[1] = GMT_IS_LAT;
					break;
				case 'Z':
					Ctrl->Z.active = TRUE;
					error += GMT_parse_z_io (&argv[i][2], &Ctrl->Z);
					break;
				case 'S':
					Ctrl->S.active = TRUE;
					if (argv[i][2] == 'r') Ctrl->S.reverse = TRUE;
					break;
				case 'W':
					Ctrl->W.active = TRUE;
					Ctrl->W.weight = (argv[i][2]) ? atof (&argv[i][2]) : 1.0;
			        case 'F':
				  
				  if(argv[i+1] && argv[i+2]) {
				    localframe=true;
				    localiser_config_file_name=argv[i+1];
				    out_name=argv[i+2];
				    fprintf(stderr,"config %s out %s\n",localiser_config_file_name.c_str(),out_name.c_str());
				    i+=2;
				    argc-=2;
				  }
					break;

				default:
					error = TRUE;
					GMT_default_error (argv[i][1]);
					break;
			}
		}
		else
			n_files++;
	}

	if (argc == 1 || GMT_give_synopsis_and_exit) {
		fprintf (stderr, "grd2xyz %s - Converting netCDF grdfile(s) to ASCII xyz data\n\n", GMT_VERSION);
		fprintf( stderr, "usage: grd2xyz <grdfiles> [-E[<nodata>]] [%s] [%s] [-S[r]] [-V]\n", GMT_Ho_OPT, GMT_Rgeo_OPT);
		fprintf( stderr, "\t[-W[<weight>]] [-Z[<flags>]] [%s] [%s] [%s] > xyzfile\n", GMT_t_OPT, GMT_bo_OPT, GMT_f_OPT);

		if (GMT_give_synopsis_and_exit) exit (EXIT_FAILURE);

		fprintf (stderr, "\n\t<grdfiles> is one or more grid files to convert\n");
		fprintf (stderr, "\n\tOPTIONS:\n");
		fprintf (stderr, "\t-E Write ESRI ArcInfo ASCII interchange format.  Only one grid file can be specified\n");
		fprintf (stderr, "\t   Optionally append nodata value to represent NaNs [-9999]\n");
		fprintf (stderr, "\t-H Write 1 ASCII header record [Default is no header]\n");
		GMT_explain_option ('R');
		fprintf (stderr, "\t-S Suppress output for nodes whose z equals NaN [Default prints all nodes]\n");
		fprintf (stderr, "\t   Append r to reverse the suppression (only output NaN nodes)\n");
		GMT_explain_option ('V');
		fprintf (stderr, "\t-W Write xyzw using supplied weight (or 1 if not given) [Default is xyz]\n");
		fprintf (stderr, "\t-Z sets exact specification of resulting 1-column output z-table\n");
		fprintf (stderr, "\t   If data is in row format, state if first row is at T(op) or B(ottom)\n");
		fprintf (stderr, "\t     Then, append L or R to indicate starting point in row\n");
		fprintf (stderr, "\t   If data is in column format, state if first columns is L(left) or R(ight)\n");
		fprintf (stderr, "\t     Then, append T or B to indicate starting point in column\n");
		fprintf (stderr, "\t   Append x if gridline-registered, periodic data in x without repeating column at xmax\n");
		fprintf (stderr, "\t   Append y if gridline-registered, periodic data in y without repeating row at ymax\n");
		fprintf (stderr, "\t   Specify one of the following data types (all binary except a):\n");
		fprintf (stderr, "\t     a  Ascii\n");
		fprintf (stderr, "\t     c  signed 1-byte character\n");
		fprintf (stderr, "\t     u  unsigned 1-byte character\n");
		fprintf (stderr, "\t     h  signed short 2-byte integer\n");
		fprintf (stderr, "\t     H  unsigned short 2-byte integer\n");
		fprintf (stderr, "\t     i  signed 4-byte integer\n");
		fprintf (stderr, "\t     I  unsigned 4-byte integer\n");
		fprintf (stderr, "\t     l  long (4- or 8-byte) integer\n");
		fprintf (stderr, "\t     f  4-byte floating point single precision\n");
		fprintf (stderr, "\t     d  8-byte floating point double precision\n");
		fprintf (stderr, "\t   [Default format is scanline orientation in ascii representation: -ZTLa]\n");
		GMT_explain_option (':');
		GMT_explain_option ('o');
		GMT_explain_option ('n');
		GMT_explain_option ('f');
		GMT_explain_option ('.');
		exit (EXIT_FAILURE);
	}

	if (n_files == 0) {
		fprintf (stderr, "%s: GMT SYNTAX ERROR:  Must specify at least one input file\n", GMT_program);
		error++;
	}

	if (n_files > 1 && Ctrl->E.active) {
		fprintf (stderr, "%s: GMT SYNTAX ERROR:  -E can only handle one input file\n", GMT_program);
		error++;
	}

	if (Ctrl->Z.active && Ctrl->E.active) {
		fprintf (stderr, "%s: GMT SYNTAX ERROR:  -E is not compatible with -Z\n", GMT_program);
		error++;
	}
	if (b_only && Ctrl->Z.active) GMT_io.binary[GMT_OUT] = FALSE;

	GMT_init_z_io (Ctrl->Z.format, Ctrl->Z.repeat, Ctrl->Z.swab, Ctrl->Z.skip, Ctrl->Z.type, &io);

	if ((GMT_io.binary[GMT_OUT] || io.binary) && GMT_io.io_header[GMT_OUT]) {
		fprintf (stderr, "%s: GMT SYNTAX ERROR.  Binary output data cannot have header -H\n", GMT_program);
		error++;
	}

	if (error) exit (EXIT_FAILURE);

	if (Ctrl->Z.active && io.binary) GMT_io.binary[GMT_OUT] = TRUE;
	
	if (b_only && Ctrl->Z.active) fprintf (stderr, "%s: GMT Warning.  -Z overrides -bo\n", GMT_program);
	if (b_only && Ctrl->E.active) fprintf (stderr, "%s: GMT Warning.  -E overrides -bo\n", GMT_program);

  // IMPORTANT:- specify the ellipsoid and projection for GeographicConversions
   UF::GeographicConversions::Redfearn gpsConversion("WGS84","UTM");
   double local_easting, local_northing, easting, northing;

   Config_File *config_file;
   if(localframe){
   try
   {
      config_file = new Config_File( localiser_config_file_name );
   }
   catch( string error )
   {
      cerr << "ERROR - " << error << endl;
      exit( 1 );
   }

  
   if( !config_file->get_value( "LATITUDE", lat_origin ) )
   {
      cerr << "ERROR - the localiser config file doesn't define option"
           << " LATITUDE. This was previously LATITUDE"
           << " in the stereo config file." << endl;
      exit(1);
   }
   if( !config_file->get_value( "LONGITUDE", long_origin ) )
   {
      cerr << "ERROR - the localiser config file doesn't define option"
           << " LONGITUDE. This was previously LONGITUDE"
           << " in the stereo config file." << endl;
      exit(1);
   }

 gpsConversion.GetGridCoordinates( lat_origin, long_origin,
          zone, local_easting, local_northing, gridConvergence, pointScale);

   }
#ifdef SET_IO_MODE
		GMT_setmode (GMT_OUT);
#endif

	n_out = (Ctrl->W.active) ? 4 : 3;
	out[3] = Ctrl->W.weight;
	for (k = 1; k < argc; k++) {
		if (argv[k][0] == '-') continue;	/* Skip the options */

		GMT_err_fail (GMT_read_grd_info (argv[k], &grd), argv[k]);

		if (gmtdefs.verbose) fprintf (stderr, "%s: Working on file %s\n", GMT_program, argv[k]);

		nm = grd.nx * grd.ny;
		n_total += nm;

		if (e > w && n > s) {	/* Subset */
			global = (fabs (grd.x_max - grd.x_min) == 360.0);
			if (!global && (w < grd.x_min || e > grd.x_max)) error = TRUE;
			if (s < grd.y_min || n > grd.y_max) error = TRUE;
			if (error) {
				fprintf (stderr, "%s: GMT ERROR: Subset exceeds data domain!\n", GMT_program);
				exit (EXIT_FAILURE);
			}
			GMT_err_fail (GMT_adjust_loose_wesn (&w, &e, &s, &n, &grd), "");	/* Make sure w,e,s,n matches header spacing */
			nx = GMT_get_n (w, e, grd.x_inc, grd.node_offset);
			ny = GMT_get_n (s, n, grd.y_inc, grd.node_offset);

			z = (float *) GMT_memory (VNULL, (size_t) (nx * ny), sizeof (float), GMT_program);
	sup_remap = (int *) GMT_memory (VNULL, (size_t) (nx * ny), sizeof (int), GMT_program);

			GMT_err_fail (GMT_read_grd (argv[k], &grd, z, w, e, s, n, GMT_pad, FALSE), argv[k]);
		}
		else {
			z = (float *) GMT_memory (VNULL, (size_t) nm, sizeof (float), GMT_program);
			sup_remap = (int *) GMT_memory (VNULL, (size_t) nm, sizeof (int), GMT_program);

			GMT_err_fail (GMT_read_grd (argv[k], &grd, z, 0.0, 0.0, 0.0, 0.0, GMT_pad, FALSE), argv[k]);
		}

		GMT_err_fail (GMT_set_z_io (&io, &grd), argv[k]);

		if (Ctrl->Z.active) {
		  if (GMT_io.io_header[GMT_OUT] && !io.binary) fprintf (GMT_stdout, "%s\n", grd.z_units);

			for (ij = 0; ij < io.n_expected; ij++) {
			  ((void (*)(GMT_Z_IO*,int,int*))io.get_gmt_ij)(&io, ij, &gmt_ij);
				d_value = z[gmt_ij];
				if (Ctrl->S.active && (GMT_is_dnan (d_value) + Ctrl->S.reverse) == 1) {
					n_suppressed++;
					sup_remap[gmt_ij]=-1;
					continue;
				}
				if ((io.x_missing && io.gmt_i == io.x_period) || (io.y_missing && io.gmt_j == 0)) continue;
				((void (*)(FILE*,float))io.write_item) (GMT_stdout, d_value);
				sup_remap[gmt_ij]=vvalid++;
			}
		}
		else if (Ctrl->E.active) {
			fprintf (GMT_stdout, "ncols %d\nnrows %d\n", grd.nx, grd.ny);
			if (!grd.node_offset) {	/* Gridline format */
				fprintf (GMT_stdout, "xllcenter ");
				fprintf (GMT_stdout, gmtdefs.d_format, grd.x_min + 0.5 * grd.x_inc);
				fprintf (GMT_stdout, "\nyllcenter ");
				fprintf (GMT_stdout, gmtdefs.d_format, grd.y_min + 0.5 * grd.y_inc);
			}
			else {	/* Pixel format */
				fprintf (GMT_stdout, "xllcorner ");
				fprintf (GMT_stdout, gmtdefs.d_format, grd.x_min);
				fprintf (GMT_stdout, "\nyllcorner ");
				fprintf (GMT_stdout, gmtdefs.d_format, grd.y_min);
			}
			fprintf (GMT_stdout, "\ncellsize ");
			fprintf (GMT_stdout, gmtdefs.d_format, grd.x_inc);
			fprintf (GMT_stdout, "\nnodata_value %d\n", irint (Ctrl->E.nodata));
			for (j = 0; j < grd.ny; j++) {	/* Scanlines, starting in the north (ymax) */
				ij = j * grd.nx;
				for (i = 0; i < grd.nx; i++, ij++) {
					if (GMT_is_fnan (z[ij]))
						fprintf (GMT_stdout, "%d", irint (Ctrl->E.nodata));
					else
						fprintf (GMT_stdout, "%d", irint ((double)z[ij]));
					if (i < (grd.nx-1)) fprintf (GMT_stdout, " ");
				}
				fprintf (GMT_stdout, "\n");
			}
		}
		else {

			x = (double *) GMT_memory (VNULL, (size_t) grd.nx, sizeof (double), GMT_program);
			y = (double *) GMT_memory (VNULL, (size_t) grd.ny, sizeof (double), GMT_program);

			/* Compute grid node positions once only */

			for (j = 0; j < grd.ny; j++) y[j] = GMT_j_to_y (j, grd.y_min, grd.y_max, grd.y_inc, grd.xy_off, grd.ny);
			for (i = 0; i < grd.nx; i++) x[i] = GMT_i_to_x (i, grd.x_min, grd.x_max, grd.x_inc, grd.xy_off, grd.nx);
		
			for (j = ij = 0; j < grd.ny; j++) for (i = 0; i < grd.nx; i++, ij++){ 
			    if (!(Ctrl->S.active && (GMT_is_dnan (z[ij]) + Ctrl->S.reverse) == 1) )
      			      sup_remap[ij]=v_count++;
			    else{
			      sup_remap[ij]=-1;
			    }
			  }

	for (j = ij = 0; j < grd.ny; j++) for (i = 0; i < grd.nx; i++, ij++) {
		    d_value = z[ij];
		
		if(GMT_is_dnan (d_value)  )
		  continue;
		int iout[3];
		unsigned char c=3;
		if(sup_remap[ij] != -1 && sup_remap[ij+1] != -1 && sup_remap[ij+grd.nx+1] != -1 && ij+1 < (grd.ny * grd.nx) && ij+grd.nx+1 < (grd.ny * grd.nx))
		  f_count++;
	  if(sup_remap[ij] != -1 && sup_remap[ij+grd.nx] != -1 && sup_remap[ij+grd.nx+1] != -1 && ij+grd.nx < (grd.ny * grd.nx) && ij+grd.nx+1 < (grd.ny * grd.nx))
	    f_count++;
	 
	  }
	if(f_count == 0){
	  fprintf(stderr,"no faces valid bailing\n");
	  exit(-1);
	}
	
	fp=fopen(out_name.c_str(),"wb");
	if(!fp)
{
	  fprintf(stderr,"unable to open file\n");
	  exit(-1);
	}
			fprintf(fp,"ply\n");
			if (GMT_io.binary[GMT_OUT]) fprintf(fp,"format binary_little_endian 1.0\n");
		else fprintf(fp,"format ascii 1.0\n");
			//fprintf(fp,"comment %s\n",part_description);
			//fprintf(fp,"comment PLY exporter written by Paul Adams\n");
			fprintf(fp,"element vertex %d\n",v_count);
			fprintf(fp,"property float x\n");
			fprintf(fp,"property float y\n");
			
			fprintf(fp,"property float z\n");
			
			fprintf(fp,"element face %d\n",f_count);
			
			fprintf(fp,"property list uchar int vertex_indices\n");
			
			fprintf(fp,"end_header\n");
		
		

			for (j = ij = 0; j < grd.ny; j++) for (i = 0; i < grd.nx; i++, ij++) {
				out[2] = z[ij];
				if (Ctrl->S.active && (GMT_is_dnan (out[2]) + Ctrl->S.reverse) == 1) {
					n_suppressed++;
				
					continue;
				}
				if(localframe){
				  gpsConversion.GetGridCoordinates(y[j],x[i],
								   zone, easting, northing, 
								   gridConvergence, pointScale);
			
				  easting-=local_easting;
				  northing-=local_northing;

				  out[0] = easting;	out[1] = northing;
				}else{
				  out[0] = x[i];	out[1] = y[j];
				}
				if(GMT_io.binary[GMT_OUT])
				  fwrite(  out,n_out,sizeof(float),fp);
				else
				  fprintf(fp,"%f %f %f\n",out[0],out[1],out[2]);

				vvalid++;
			  }
			
			GMT_free ((void *)x);
			GMT_free ((void *)y);
		}

		for (j = ij = 0; j < grd.ny; j++) for (i = 0; i < grd.nx; i++, ij++) {
		    d_value = z[ij];
		
		if(GMT_is_dnan (d_value)  )
		  continue;
		int iout[3];
		unsigned char c=3;
		if(sup_remap[ij] != -1 && sup_remap[ij+1] != -1 && sup_remap[ij+grd.nx+1] != -1 && ij+1 < (grd.ny * grd.nx) && ij+grd.nx+1 < (grd.ny * grd.nx))
		  {


		    iout[2]=sup_remap[ij];
		    iout[1]=sup_remap[ij+1];
		    iout[0]=sup_remap[ij+grd.nx+1];
		    if(GMT_io.binary[GMT_OUT]){
		      fwrite(&c,sizeof(unsigned char),1,fp);
		      fwrite(iout,sizeof(int),3,fp);
		    }
		    else
		      fprintf(fp,"%d %d %d %d\n",c,iout[0],iout[1],iout[2]);
		    
		    fvalid++;
		  }
		if(sup_remap[ij] != -1 && sup_remap[ij+grd.nx] != -1 && sup_remap[ij+grd.nx+1] != -1 && ij+grd.nx < (grd.ny * grd.nx) && ij+grd.nx+1 < (grd.ny * grd.nx)){
		c=3;
		iout[0]=sup_remap[ij];
		iout[1]=sup_remap[ij+grd.nx];
		iout[2]=sup_remap[ij+grd.nx+1];
	
		if(GMT_io.binary[GMT_OUT]){
		  fwrite(&c,sizeof(unsigned char),1,fp);
		  fwrite(iout,sizeof(int),3,fp);
		}
		else
		  fprintf(fp,"%d %d %d %d\n",c,iout[0],iout[1],iout[2]);

		fvalid++;
		}
		
	}
	GMT_free ((void *)z);
	}

	fprintf(stderr,"%d vvalid %d fvalid\n",vvalid,fvalid);
	fclose(fp);
	if (gmtdefs.verbose) fprintf (stderr, "%s: %d values extracted\n", GMT_program, n_total - n_suppressed);
	if (n_suppressed && gmtdefs.verbose) {
		if (Ctrl->S.reverse)
			fprintf (stderr, "%s: %d finite values suppressed\n", GMT_program, n_suppressed);
		else
			fprintf (stderr, "%s: %d NaN values suppressed\n", GMT_program, n_suppressed);
	}

	Free_Grd2xyz_Ctrl (Ctrl);	/* Deallocate control structure */

	GMT_end (argc, argv);

	exit (EXIT_SUCCESS);
}

void *New_Grd2xyz_Ctrl () {	/* Allocate and initialize a new control structure */
	struct GRD2XYZ_CTRL *C;
	
	C = (struct GRD2XYZ_CTRL *) GMT_memory (VNULL, 1, sizeof (struct GRD2XYZ_CTRL), "New_Grd2xyz_Ctrl");
	
	/* Initialize values whose defaults are not 0/FALSE/NULL */
	
	C->E.nodata = -9999.0;
	C->W.weight = 1.0;
	C->Z.type = 'a';
	C->Z.format[0] = 'T';	C->Z.format[1] = 'L';
		
	return ((void *)C);
}

void Free_Grd2xyz_Ctrl (struct GRD2XYZ_CTRL *C) {	/* Deallocate control structure */
	GMT_free ((void *)C);	
}
