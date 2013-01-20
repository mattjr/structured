
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
    const int src_width = img->width();
    const int src_height =img->height();
    const int dst_width = img->width()/atoi(argv[3]);
    const int dst_height =img->height()/atoi(argv[4]);
    OGF::Image* ds_img = new OGF::Image(OGF::Image::RGB,dst_width,dst_height);



    // unsigned char *colorData = new unsigned char[ src_width * src_height * 3 ];
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


    generate_mipmaps(ds_img->base_mem_byte_ptr(),img->base_mem_byte_ptr(),
                     maskData,src_width,src_height,img->bytes_per_pixel(),0,2,
                     DDS_MIPMAP_FILTER_LANCZOS,0,
                     0,0.0);

    int count =1;
    OGF::MorphoMath morpho(ds_img) ;

    for(int i=0; i<count; i++) {
        morpho.dilate(1) ;
    }
    p->serialize_write(outf,ds_img);
    delete p;


}
