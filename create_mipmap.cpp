//
// structured - Tools for the Generation and Visualization of Large-scale
// Three-dimensional Reconstructions from Image Data. This software includes
// source code from other projects, which is subject to different licensing,
// see COPYING for details. If this project is used for research see COPYING
// for making the appropriate citations.
// Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
//
// This file is part of structured.
//
// structured is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// structured is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with structured.  If not, see <http://www.gnu.org/licenses/>.
//


#include <OGF/cells/types/cells_library.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/map_builder.h>

#include <OGF/cells/map/map_editor.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/cells/map_algos/atlas_generator.h>
#include <OGF/cells/map_algos/pm_manager.h>
#include <OGF/image/types/image.h>
#include <OGF/cells/map_algos/variational_map_splitter.h>
#include <OGF/image/types/image_library.h>
#include <OGF/image/algos/rasterizer.h>
#include <OGF/image/algos/morpho_math.h>
#include <OGF/image/io/image_serializer_ppm.h>
#include <OGF/basic/os/file_system.h>
#include <OGF/cells/io/map_serializer_obj.h>
#include "mipmap.h"
int main(int argc,char **argv){

    OGF::ImageSerializer_ppm *p = new OGF::ImageSerializer_ppm;
    std::ifstream infile(argv[1]);

    OGF::Image* img = p->serialize_read(infile,false) ;
    std::ofstream outf(argv[2]);
    int origX=atoi(argv[3]);
    int origY=atoi(argv[4]);
    int level=atoi(argv[5]);

    const int src_width = img->width();
    const int src_height =img->height();
    const int dst_width = (int)std::max(ceil(origX/pow(2,level)),1.0);
    const int dst_height =(int)std::max(ceil(origY/pow(2,level)),1.0);
    printf("%d %d\n",dst_width,dst_height);
    OGF::Image* ds_img = new OGF::Image(OGF::Image::RGB,dst_width,dst_height);

    OGF::Image* maskImage = new OGF::Image(OGF::Image::GRAY,src_width,src_height);

    unsigned char *maskData = maskImage->base_mem_byte_ptr();
    OGF::Image* maskImageDS = new OGF::Image(OGF::Image::GRAY,dst_width,dst_height);

    unsigned char *maskDataDS = maskImageDS->base_mem_byte_ptr();

    unsigned char *s =  img->base_mem_byte_ptr();
    unsigned char *s_target = s+(src_width*src_height*img->bytes_per_pixel());

    unsigned char *ad = maskData;
    while (s!=s_target) {
      if(s[0]  != 0 || s[1] != 0 || s[2] != 0)
        *ad = 255;
      ad++;
      s+=img->bytes_per_pixel();
    };

    scale_image_boxmax(maskDataDS,dst_width,dst_height,maskData,src_width,src_height);


    OGF::MorphoMath morpho(img) ;
    int count =10;
    for(int i=0; i<count; i++) {
        morpho.dilate(1) ;
    }

    // unsigned char *colorData = new unsigned char[ src_width * src_height * 3 ];


    generate_mipmaps(ds_img->base_mem_byte_ptr(),img->base_mem_byte_ptr(),
                     src_width,src_height,img->bytes_per_pixel(),dst_width,dst_height,
                     DDS_MIPMAP_FILTER_LANCZOS,0,
                     0,0.0);

    //memset(ds_img->base_mem_byte_ptr(),255,dst_width*dst_height*img->bytes_per_pixel());
    //count =1;
  /*  OGF::MorphoMath morpho(ds_img) ;

    for(int i=0; i<count; i++) {
        morpho.dilate(1) ;
    }*/

 /*   s =  ds_img->base_mem_byte_ptr();
    s_target = s+(dst_width*dst_height*ds_img->bytes_per_pixel());

    ad = maskDataDS;
    while (s!=s_target) {
      if(*ad == 0){
          *s=0;
          *(s+1)=0;
          *(s+2)=0;
      }
      ad++;
      s+=ds_img->bytes_per_pixel();
    };*/
    p->serialize_write(outf,ds_img);
    delete p;
    delete maskImage;
    delete maskImageDS;


}
