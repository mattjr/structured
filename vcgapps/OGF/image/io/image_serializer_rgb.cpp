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


#include <OGF/image/io/image_serializer_rgb.h>
#include <OGF/image/types/image.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/io/stdio_compat.h>

namespace OGF {

//_________________________________________________________

    // This code makes use of the classical RGB loading functions,
    // tweaked using stdio_compat, to make them handle C++ streams
    // instead of FILE*

    static void bwtorgba(unsigned char *b,unsigned char *l,int n) {
        while(n--) {
            l[0] = *b;
            l[1] = *b;
            l[2] = *b;
            l[3] = 0xff;
            l += 4; b++;
        }
    }

    static void rgbtorgba(
        unsigned char *r,unsigned char *g,unsigned char *b,
        unsigned char *l,int n
    ) {
        while(n--) {
            l[0] = r[0];
            l[1] = g[0];
            l[2] = b[0];
            l[3] = 0xff;
            l += 4; r++; g++; b++;
        }
    }

    static void rgbatorgba(
        unsigned char *r,unsigned char *g,unsigned char *b,unsigned char *a,
        unsigned char *l,int n
    ) {
        while(n--) {
            l[0] = r[0];
            l[1] = g[0];
            l[2] = b[0];
            l[3] = a[0];
            l += 4; r++; g++; b++; a++;
        }
    }

    typedef struct _ImageRec {
        unsigned short imagic;
        unsigned short type;
        unsigned short dim;
        unsigned short xsize, ysize, zsize;
        unsigned int min, max;
        unsigned int wasteBytes;
        char name[80];
        unsigned long colorMap;
        std::istream* file;
//        FILE* file;
        unsigned char *tmp, *tmpR, *tmpG, *tmpB;
        unsigned long rleEnd;
        unsigned int *rowStart;
        int *rowSize;
    } ImageRec ;

    static void ConvertShort(unsigned short *array, long length) {
        unsigned long b1, b2;
        unsigned char *ptr;
        
        ptr = (unsigned char *)array;
        while (length--) {
            b1 = *ptr++;
            b2 = *ptr++;
            *array++ = static_cast<unsigned short>((b1 << 8) | (b2));
        }
    }

    static void ConvertLong(unsigned *array, long length) {
        unsigned long b1, b2, b3, b4;
        unsigned char *ptr;
        
        ptr = (unsigned char *)array;
        while (length--) {
            b1 = *ptr++;
            b2 = *ptr++;
            b3 = *ptr++;
            b4 = *ptr++;
            *array++ = (b1 << 24) | (b2 << 16) | (b3 << 8) | (b4);
        }
    }

    static ImageRec* ImageOpen(std::istream& input) {
        union {
            int testWord;
            char testByte[4];
        } endianTest;
        ImageRec *image;
        int swapFlag;
        int x;
        
        endianTest.testWord = 1;
        if (endianTest.testByte[0] == 1) {
            swapFlag = 1;
        } else {
            swapFlag = 0;
        }

        image = (ImageRec *)malloc(sizeof(ImageRec));
        if (image == NULL) {
            fprintf(stderr, "Out of memory!\n");
            exit(1);
        }

        image->file = &input ;
//      image->file = fopen("aluminium.rgb", "rb") ;

        fread(image, 1, 12, image->file);
        
        if (swapFlag) {
            ConvertShort(&image->imagic, 6);
        }

        image->tmp = (unsigned char *)malloc(image->xsize*256);
        image->tmpR = (unsigned char *)malloc(image->xsize*256);
        image->tmpG = (unsigned char *)malloc(image->xsize*256);
        image->tmpB = (unsigned char *)malloc(image->xsize*256);
        if (
            image->tmp == NULL || image->tmpR == NULL || image->tmpG == NULL ||
            image->tmpB == NULL
        ) {
            fprintf(stderr, "Out of memory!\n");
            exit(1);
        }

        if ((image->type & 0xFF00) == 0x0100) {
            x = image->ysize * image->zsize * sizeof(unsigned);
            image->rowStart = (unsigned *)malloc(x);
            image->rowSize = (int *)malloc(x);
            if (image->rowStart == NULL || image->rowSize == NULL) {
                fprintf(stderr, "Out of memory!\n");
                exit(1);
            }
            image->rleEnd = 512 + (2 * x);
            fseek(image->file, 512, SEEK_SET);
            fread(image->rowStart, 1, x, image->file);
            fread(image->rowSize, 1, x, image->file);
            if (swapFlag) {
                ConvertLong(image->rowStart, x/sizeof(unsigned));
                ConvertLong((unsigned *)image->rowSize, x/sizeof(int));
            }
        }
        return image;
    }

    static void ImageClose(ImageRec *image) {
        fclose(image->file);
        free(image->tmp);
        free(image->tmpR);
        free(image->tmpG);
        free(image->tmpB);
        free(image);
    }

    static void ImageGetRow(
        ImageRec *image, unsigned char *buf, int y, int z
    ) {
        unsigned char *iPtr, *oPtr, pixel;
        int count;
        
        if ((image->type & 0xFF00) == 0x0100) {
            fseek(image->file, image->rowStart[y+z*image->ysize], SEEK_SET);
            fread(
                image->tmp, 1, (unsigned int)image->rowSize[y+z*image->ysize],
                image->file
            );

            iPtr = image->tmp;
            oPtr = buf;
            while (1) {
                pixel = *iPtr++;
                count = (int)(pixel & 0x7F);
                if (!count) {
                    return;
                }
                if (pixel & 0x80) {
                    while (count--) {
                        *oPtr++ = *iPtr++;
                    }
                } else {
                    pixel = *iPtr++;
                    while (count--) {
                        *oPtr++ = pixel;
                    }
                }
            }
        } else {
            fseek(
                image->file, 
                512+(y*image->xsize)+(z*image->xsize*image->ysize),
                SEEK_SET
            );
            fread(buf, 1, image->xsize, image->file);
        }
    }

    unsigned* read_texture(
        std::istream& input, int *width, int *height, int *components
    ) {
        unsigned *base, *lptr;
        unsigned char *rbuf, *gbuf, *bbuf, *abuf;
        ImageRec *image;
        int y;
        
        image = ImageOpen(input);
        
        if(!image)
            return NULL;
        (*width)=image->xsize;
        (*height)=image->ysize;
        (*components)=image->zsize;
        base = (unsigned *)malloc(image->xsize*image->ysize*sizeof(unsigned));
        rbuf = (unsigned char *)malloc(image->xsize*sizeof(unsigned char));
        gbuf = (unsigned char *)malloc(image->xsize*sizeof(unsigned char));
        bbuf = (unsigned char *)malloc(image->xsize*sizeof(unsigned char));
        abuf = (unsigned char *)malloc(image->xsize*sizeof(unsigned char));
        if(!base || !rbuf || !gbuf || !bbuf)
            return NULL;
        lptr = base;

        for(y=0; y<image->ysize; y++) {
            if(image->zsize>=4) {
                ImageGetRow(image,rbuf,y,0);
                ImageGetRow(image,gbuf,y,1);
                ImageGetRow(image,bbuf,y,2);
                ImageGetRow(image,abuf,y,3);
                rgbatorgba(
                    rbuf,gbuf,bbuf,abuf,(unsigned char *)lptr,image->xsize
                );
                lptr += image->xsize;
            } else if(image->zsize==3) {
                ImageGetRow(image,rbuf,y,0);
                ImageGetRow(image,gbuf,y,1);
                ImageGetRow(image,bbuf,y,2);
                rgbtorgba(rbuf,gbuf,bbuf,(unsigned char *)lptr,image->xsize);
                lptr += image->xsize;
            } else {
                ImageGetRow(image,rbuf,y,0);
                bwtorgba(rbuf,(unsigned char *)lptr,image->xsize);
                lptr += image->xsize;
            }
        }
        ImageClose(image);
        free(rbuf);
        free(gbuf);
        free(bbuf);
        free(abuf);
        
        return (unsigned *) base;
    }

//_________________________________________________________

    Image* ImageSerializer_rgb::serialize_read(std::istream& in) {
        int width = 0 ;
        int height = 0 ;
        int bpp = 0 ;
        unsigned* data = read_texture(in, &width, &height, &bpp) ;

        // Note: it seems that data is always encoded as a RGBA image,
        // bpp seems to only return the original color encoding...
        bpp = 4 ;

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
            ogf_assert(false) ;
            break ;
        }

        Image* result = new Image(color_encoding, width, height) ;
        memcpy(result->base_mem(), data, width * height * bpp) ;
        free(data) ;
        
        return result ;
    }

    bool ImageSerializer_rgb::read_supported() const {
        return true ;
    }

//_________________________________________________________

    

}

