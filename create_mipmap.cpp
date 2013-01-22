
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
    const int dst_width = ceil(origX/pow(2,level));
    const int dst_height =ceil(origY/pow(2,level));
    printf("%d %d\n",dst_width,dst_height);
    OGF::Image* ds_img = new OGF::Image(OGF::Image::RGB,dst_width,dst_height);


    OGF::MorphoMath morpho(img) ;
    int count =10;
    for(int i=0; i<count; i++) {
        morpho.dilate(1) ;
    }

  /*  // unsigned char *colorData = new unsigned char[ src_width * src_height * 3 ];
    unsigned char *maskData = new unsigned char[ src_width * src_height ];

    unsigned char *s =  img->base_mem_byte_ptr();
    unsigned char *s_target = s+(src_width*src_height*img->bytes_per_pixel());

    unsigned char *ad = maskData;
    while (s!=s_target) {
      if(s[0]  == s[1] && s[1] == s[2] && s[2] == 0)
        *ad = 255;
      ad++;
      s+=img->bytes_per_pixel();
    };
*/

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
    p->serialize_write(outf,ds_img);
    delete p;


}
