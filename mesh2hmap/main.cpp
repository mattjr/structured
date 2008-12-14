// -*- linux-c -*-

// Copyright 2006, Tim Hessint

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <stdio.h> // fscanf(), sscanf(),fprintf(), stdout, FILE, ...
#include <string.h> // strncmp(), strlen(), strchr()
#include <stdlib.h> // malloc(), free(), exit()
#include <assert.h> // assert()
#include <ctype.h> // isalnum()
#include <math.h> // pow()

#include "matrix.h"
#include "parser.h"
#include "mesh2hmap.h"
#include "output.h"
     #include <netcdf.h>




// Parse command line options and make mesh from VRML file
//   --normal=+x       default = +z
//   --m-pix=float     default = 1
//   --x-m-pix=float   default = m-pix
//   --y-m-pix=float   default = m-pix
//   --bpp=int         default = 8
//   --min=int         default = 0
//   --max=int         default = 2^bpp - 1
//   --print           print mesh and exit, do not make heightmap
//   --vrml            parse a vrml file (default)
//   --native          parse file in native format
//
//   vrml2hmap [options] meshfile [pgmfilename]
//
char config_name[512];
int main(int argc, char **argv) 
{
	char *usage = "Usage: mesh2hmap [ --normal=[+-][xyz] \n"
		"\t[--m-pix=# | --size=WxH] ] meshfile [pgmfile]\n";
	char *version = "0.3";

	char *norm_str = "--normal=";
	char *m_str = "--m-pix=";
	char *x_str = "--x-pix=";
	char *y_str = "--y-pix=";
	char *bpp_str = "--bpp=";
	char *min_str = "--min=";
	char *max_str = "--max=";
	char *size_str = "--size=";
	char *wdth_str = "--width=";
	char *hght_str = "--height=";
	char *bg_str = "--bg=";

	// boolean options
	char *prnt_str = "--print";
	char *raw_str = "--raw";
	char *gmt_str = "--gmt=";
	char *vrml_str = "--vrml";
	char *ntv_str = "--native";
	char *help_str = "--help";
	char *h_str = "-h";
	char *ver_str = "--version";
	char *v_str = "-v";
	
	// booleans set by command-line options
	char print = 0;
	char raw = 0;
	char gmt = 0;
	char vrml = 1;
	char native = 0;

	// filenames to be set
	char *meshfile = 0;
	char *outfile = 0;
	
	// hmap options to be set
	char normal[10] = "+z";
	float m_pix, x_m_pix, y_m_pix;
	m_pix = x_m_pix = y_m_pix = 0.0;
	int bpp = 8;
	int min = 0;
	int max = 0;
	int width, height;
	width = height = 0;

	// colour options to be set
	// Initialised to a triplet of 11 zeros so ti is large eneough
	// to contain 3 32 bit numbers. 2^32 is 10 didits plus 1 for 
	// minus sign
	char max_clr[3*11 + 3];
	char min_clr[3*11 + 3];
	char bg_clr[3*11 + 3];
	max_clr[0] = min_clr[0] = bg_clr[0] = '\0';
	int bg = 0;

	if (argc == 1) {
		puts(usage); 
		exit(0);
	}

	int i;
	for(i = 1; i < argc; i++) {
		
		if (strncmp(argv[i], norm_str, strlen(norm_str)) == 0 ) {
			sscanf(argv[i],"--normal=%s",normal);
		} else if (strncmp(argv[i], m_str, strlen(m_str)) == 0 ) {
			sscanf(argv[i],"--m-pix=%f",&m_pix);
		} else if (strncmp(argv[i], x_str, strlen(x_str)) == 0) {
			sscanf(argv[i],"--x-pix=%f",&x_m_pix);
		} else if (strncmp(argv[i], y_str, strlen(y_str)) == 0) {
			sscanf(argv[i],"--y-pix=%f",&y_m_pix);
		} else if (strncmp(argv[i], bpp_str, strlen(bpp_str)) == 0) {
			sscanf(argv[i],"--bpp=%d",&bpp);
		} else if (strncmp(argv[i], min_str, strlen(min_str)) == 0) {
			strcpy(min_clr, &argv[i][strlen(min_str)]);
		} else if (strncmp(argv[i], max_str, strlen(max_str)) == 0) {
			strcpy(max_clr, &argv[i][strlen(max_str)]);
		} else if (strncmp(argv[i], bg_str, strlen(bg_str))==0) {
			strcpy(bg_clr, &argv[i][strlen(bg_str)]);
		} else if (strncmp(argv[i], size_str, strlen(size_str)) == 0) {
			sscanf(argv[i],"--size=%dx%d",&width,&height);
		} else if (strncmp(argv[i],wdth_str,strlen(wdth_str)) == 0) {
			sscanf(argv[i],"--width=%d",&width);
		} else if (strncmp(argv[i], hght_str, strlen(hght_str)) == 0) {
			sscanf(argv[i],"--height=%d",&height);
		} else if (strncmp(argv[i], prnt_str, strlen(prnt_str)) == 0) {
			print = 1;
		} else if (strncmp(argv[i], raw_str, strlen(raw_str)) == 0) {
			raw = 1;
		} else if (strncmp(argv[i], gmt_str, strlen(gmt_str)) == 0) {
			gmt = 1;
			sscanf(argv[i],"--gmt=%s",config_name);
		} else if (strncmp(argv[i], vrml_str, strlen(vrml_str)) == 0) {
			vrml = 1; native = 0;
		} else if (strncmp(argv[i], ntv_str, strlen(ntv_str)) == 0) {
			vrml = 0; native = 1;
		} else if ( (strcmp(argv[i], help_str) == 0)
				|| (strcmp(argv[i], h_str) == 0) ) {
			puts(usage); exit(EXIT_SUCCESS);
		} else if ( (strcmp(argv[i], ver_str) == 0) 
				|| (strcmp(argv[i], v_str) == 0) ) {
			puts(version); exit(EXIT_SUCCESS);
		} else if ( isalnum(argv[i][0]) || strchr("./",argv[i][0]) ) {
			if (meshfile == 0) {
				meshfile = argv[i];
			} else if (outfile == 0) {
				outfile = argv[i];
			} else {
				puts("Too many arguments!");
				puts(usage);
			}
		} else { // must be incorrect command-line option
			printf("Error: Unknown command-line option '%s'\n", 
			       argv[i]);
			puts(usage);
			exit(1);
		}
	}
	
			   

	if (meshfile == 0) {
		puts("Error: no mesh file specified...exiting");
		exit(1);
	}

	mesh_t mesh;
	//	if (native) native2mesh(&mesh, meshfile);
	//	else if (vrml) vrml2mesh(&mesh, meshfile);
	TriMesh *tmesh = TriMesh::read(meshfile);

	if(tmesh == NULL) {
		printf("Failed to open '%s', aborting\n", meshfile);
		abort();
	}
	trimesh2mesh(&mesh, tmesh);

	if (print) {
		print_mesh(&mesh);
		return 0;
	}
	if ( (mesh.num_vert < 1) || (mesh.num_poly < 1) ) {
		puts("Error: mesh was not parsed...exiting");
		return 1;
	}
	


	int x, y, z, invert;
	char *norm_ptr = &normal[0];
	if (norm_ptr[0] == '-') { invert = 1; norm_ptr++; } 
	else if (norm_ptr[0] == '+') { invert = 0; norm_ptr++; } 
	else invert = 0;

	if (norm_ptr[0] == 'x') { z = 0; x = 1; y = 2; } 
	else if (norm_ptr[0] == 'y') { z = 1; x = 2; y = 0; } 
	else if (norm_ptr[0] == 'z') { z = 2; x = 0; y = 1; }


	if ( (m_pix == 0.0) && (x_m_pix == 0.0) && (y_m_pix == 0.0) ) {
		x_m_pix = y_m_pix = 1.0;
	} else if ( m_pix != 0.0 ) { x_m_pix = y_m_pix = m_pix;	}


	hmap_t hmap;
	hmap.rows = height;
	hmap.cols = width;
	mesh2hmap(&hmap, &mesh, x, y, z, invert, x_m_pix, y_m_pix);
	//print_hmap(&hmap); return 0;
	fmatrix_free(mesh.vert);
	irowarray_free(mesh.poly, mesh.num_poly);



	if ( raw ) {
		print_raw_hmap(outfile, &hmap);
		fmatrix_free(hmap.map);
		return 0;
	}

	if(gmt){
	  print_gmt_hmap(outfile, &hmap,x_m_pix,y_m_pix,config_name);
	  fmatrix_free(hmap.map);
	  return 0;
	}
	int range = (int) pow(2,bpp) - 1;

	char *p[] = {max_clr, min_clr, bg_clr};
	int r,g,b;
	char colour = 0; // are we colourising
	char colour_error = 0;
	int conversions;

	for( i = 0; i < 3; i++) {
		if ( strlen( p[i] ) == 0 ) continue;
		conversions = sscanf(p[i], "%d %d %d", &r, &g, &b);
		if ( conversions == 1 ) {
			switch ( i ) {
				case 0: max = r; break;
				case 1: min = r; break;
				case 2: bg = r; break;
			}
			// make colour string zero length
			p[i][0] = '\0';
		} else if ( conversions == 3 ) {
			if ( r > range ) r = range;
			else if ( r < 0 ) r = 0;
			if ( g > range ) g = range;
			else if ( g < 0 ) g = 0;
			if ( b > range ) b = range;
			else if ( b < 0 ) b = 0;
			sprintf(p[i], "%d %d %d", r, g, b);
			colour = 1;
		} else if ( conversions == 0 ) {
			char tmp = 1; // assume we'll find a colour
			if (strcmp(p[i],"min")==0 && i==2) {
				bg = -2; tmp = 0;
				strcpy(p[i],min_clr); 
			} else if (strcmp(p[i],"max")==0 && i==2) {
				bg = -1; tmp = 0;
				strcpy(p[i],max_clr); 
			} else if (strcmp(p[i], "black") == 0 ) {
				sprintf(p[i],"%d %d %d",0,0,0);
			} else if (strcmp(p[i], "white") == 0 ) {
				sprintf(p[i],"%d %d %d",255,255,255);
			} else if ( strcmp(p[i],"red") == 0 ) {
				sprintf(p[i],"%d %d %d",range,0,0);
			} else if ( strcmp(p[i],"green") == 0 ) {
				sprintf(p[i],"%d %d %d",0,range,0);
			} else if ( strcmp(p[i],"blue") == 0 ) {
				sprintf(p[i],"%d %d %d",0,0,range);
			} else if ( strcmp(p[i],"cyan")  == 0 ) {
				sprintf(p[i],"%d %d %d",0,range,range);
			} else if ( strcmp(p[i],"magenta") == 0 ) {
				sprintf(p[i],"%d %d %d",range,0,range);
			} else if ( strcmp(p[i],"yellow")  == 0 ) {
				sprintf(p[i],"%d %d %d",range,range,0);
			} else {
				colour_error = i + 1;
			}

			colour = tmp || colour;
		} else {
			colour_error = i + 1;
		}

		if ( colour_error ) {
			char option[6];
			switch ( colour_error - 1 ) {
				case 0: strcpy(option, max_str); break;
				case 1: strcpy(option, min_str); break;
				case 2: strcpy(option, bg_str); break;
			}
			printf("Error: Unrecognised colour/height "
				"value: %s%s\n", option, p[i]);
			exit(EXIT_FAILURE);
		}
		

	}
	

	if ( (max == 0) || (max > range) ) max = range;
	if ( min < 0 ) min = 0.0;

	if ( colour ) {
		if ( strlen(max_clr) == 0 ) 
			sprintf(max_clr, "%d %d %d", range, range, range);
		if ( strlen(min_clr) == 0 )
			sprintf(min_clr, "%d %d %d", min, min, min); 
		if ( strlen(bg_clr) == 0 ) 
			sprintf(bg_clr, "%d %d %d", bg, bg, bg); 

		hmap2ppm(outfile, &hmap, range, min, max, 
				min_clr, max_clr, bg_clr);
	} else {
		hmap2pgm(outfile, &hmap, range, min, max, bg);
	}
	


	fmatrix_free(hmap.map);
	
	return 0;

}

