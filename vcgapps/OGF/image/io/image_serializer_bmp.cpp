/*
 *  GXML/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000 Bruno Levy
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  If you modify this software, you should include a notice giving the
 *  name of the person performing the modification, the date of modification,
 *  and the reason for such modification.
 *
 *  Contact: Bruno Levy
 *
 *     levy@loria.fr
 *
 *     ISA Project
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 */


#include <OGF/image/io/image_serializer_bmp.h>
#include <OGF/image/types/image.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/io/b_stream.h>

namespace OGF {

//_________________________________________________________

    // TODO: 1) Use BinaryStream (to be endian-independent)
    //       2) Use Memory:: functions
    //       3) Why do we need to allocate a temp buffer 
    //           before copying it to the image ? Read directly
    //           to the image.
    //       4) There was a call to free() (for a memory bloc
    //           allocated with new[]!!!)

    
    bool read_bmp_header(BinaryInputStream& bin_in, 
        Numeric::int32* w, Numeric::int32* h, Numeric::int32* bpp)
    {
        Numeric::int32 temp_int32=0;
        Numeric::int16 temp_int16=0;
        Numeric::int32 width=0,height=0;
        Numeric::int16 bpp_temp=0;
        

        // decode the header to get the width  height and bpp
        bin_in>>temp_int16;         //  0 -  1  sType
        bin_in>>temp_int32;         //  2 -  5  iSizeOfFile
        bin_in>>temp_int16;         //  6 -  7  sReserved1
        bin_in>>temp_int16;         //  8 -  9  sReserved2
        bin_in>>temp_int32;         // 10 - 13  iOffBits
        bin_in>>temp_int32;         // 14 - 17  iSize
        bin_in>>width;              // 18 - 21  iWidth
        bin_in>>height;             // 22 - 25  iHeight
        bin_in>>temp_int16;         // 26 - 27  sPlanes
        bin_in>>bpp_temp;           // 28 - 29  sBitCount
        bpp_temp/=8;                // bits => bytes
        bin_in>>temp_int32;         // 30 - 33  iCompression
        bin_in>>temp_int32;         // 34 - 37  iSizeImage
        bin_in>>temp_int32;         // 38 - 41  iXpelsPerMeter
        bin_in>>temp_int32;         // 42 - 45  iYpelsPerMeter
        bin_in>>temp_int32;         // 46 - 49  iClrUsed
        bin_in>>temp_int32;         // 50 - 53  iClrImportant

        *w=width;
        *h=height;
        *bpp=bpp_temp;

        return true;
    }

    bool write_bmp_header(BinaryOutputStream& bin_out, 
        Numeric::int32* w, Numeric::int32* h, Numeric::int32* bpp)
    {
        Numeric::int32 temp_int32=0;
        Numeric::int16 temp_int16=0;
        Numeric::int32 width=*w,height=*h;
        Numeric::int16 bpp_temp=*bpp;

        // encode the header to get the width  height and bpp
        // lots of fields not yet implemented
        temp_int16=19778;       // bmp id
        bin_out<<temp_int16;         //  0 -  1  sType
        // header is 54 bytes long
        temp_int32=54+(width+(width*bpp_temp)%4)*height*bpp_temp;
        bin_out<<temp_int32;         //  2 -  5  iSizeOfFile
        temp_int16=0;
        bin_out<<temp_int16;         //  6 -  7  sReserved1
        bin_out<<temp_int16;         //  8 -  9  sReserved2
        temp_int32=54;          // bmp header
        bin_out<<temp_int32;         // 10 - 13  iOffBits
        temp_int32=40;          // bytes to first pixel
        bin_out<<temp_int32;         // 14 - 17  iSize
        bin_out<<width;              // 18 - 21  iWidth
        bin_out<<height;             // 22 - 25  iHeight
        temp_int16=1;           // only 1 plane
        bin_out<<temp_int16;         // 26 - 27  sPlanes
        bpp_temp*=8;                 // bytes => bits
        bin_out<<bpp_temp;           // 28 - 29  sBitCount
        temp_int32=0;           // no compression
        bin_out<<temp_int32;         // 30 - 33  iCompression
        temp_int32=(width+(width*bpp_temp)%4)*height*bpp_temp;
        bin_out<<temp_int32;         // 34 - 37  iSizeImage
        temp_int32=70;          // almost screen ppm
        bin_out<<temp_int32;         // 38 - 41  iXpelsPerMeter
        bin_out<<temp_int32;         // 42 - 45  iYpelsPerMeter
        temp_int32=0;           // unused
        bin_out<<temp_int32;         // 46 - 49  iClrUsed
        bin_out<<temp_int32;         // 50 - 53  iClrImportant

        return true;
    }

    Image* ImageSerializer_bmp::serialize_read(std::istream& in) {
        Numeric::int32 width = 0 ;
        Numeric::int32 height = 0 ;
        Numeric::int32 bpp = 0 ;
        BinaryInputStream bin_in(in,OGF_LITTLE_ENDIAN);
        Image* result = new Image ;

        ogf_assert(read_bmp_header(bin_in, &width, &height, &bpp)) ;
        Image::ColorEncoding color_encoding ;
        switch(bpp) {
        case 1:
            color_encoding = Image::GRAY ;
            break ;
        case 3:
            color_encoding = Image::RGB ;
            break ;
        case 4:
            color_encoding = Image::RGBA ;
            break ;
        default:
            Logger::err("ImageSerializer_bmp")<<"unknown color encoding"
                <<std::endl;
            delete result;
            return nil;
        }
        // initialize image with color encoding, width and height
        result->initialize(color_encoding, width, height);
        // load the content of the file
        Numeric::uint8 *pixel = (Numeric::uint8 *)result->base_mem();
        {for (int i = 0; i < (height); i++) {
            {for (int j = 0; j < (width); j++) {
                Numeric::uint8 temp;
                {for (int k=0;k<bpp;k++) {
                    bin_in>>temp;
                    pixel[(i*width+j)*bpp+k]=temp;
                }}
            }}
            // a BMP row must be multiple of 4 so we have to check 
            // useless entries
            {for (int k = 0; k < (width*bpp)%4 ; k++) {
                Numeric::uint8 temp;
                bin_in>>temp;
            }}
        }}
        // bmp files are in bgr
        rgb_to_bgr(*result);
        return result ;
    }

    bool ImageSerializer_bmp::serialize_write(
        std::ostream& out, const Image *image) 
    {
        // check the pixel format of the image
        if (image->color_encoding()!=Image::GRAY &&
            image->color_encoding()!=Image::RGB  &&
            image->color_encoding()!=Image::RGBA )
        {
            Logger::err("ImageSerializer_bmp")<<"Color encoding not supported"
                <<std::endl;
            return false;
        }

        // local vars
        Numeric::int32 width = image->width() ;
        Numeric::int32 height = image->height() ;
        Numeric::int32 bpp = image->bytes_per_pixel() ;
        BinaryOutputStream bin_out(out,OGF_LITTLE_ENDIAN);

        // as bmp are flipped vertically and in bgr
        // we have to change the image but,
        // we must copy the picture first cause it's const
        Image image_copy(image);
        flip_image(image_copy);
        rgb_to_bgr(image_copy);

        // copy the header to the file
        ogf_assert(write_bmp_header(bin_out, &width, &height, &bpp)) ;
        // copy pixels to the file
        Numeric::uint8 *pixel = (Numeric::uint8 *)image_copy.base_mem();
        {for (int i = 0; i < (height); i++) {
            {for (int j = 0; j < (width); j++) {
                Numeric::uint8 temp;
                {for (int k=0;k<bpp;k++) {
                    temp=pixel[(i*width+j)*bpp+k];
                    bin_out<<temp;
                }}
            }}
            // a BMP row must be multiple of 4 so we have to check 
            // useless entries
            {for (int k = 0; k < (width*bpp)%4 ; k++) {
                Numeric::uint8 temp=0;
                bin_out<<temp;
            }}
        }}

        // replace the image in its normal state
        flip_image(image_copy);
        rgb_to_bgr(image_copy);

        return true;
    }

//_________________________________________________________

}

