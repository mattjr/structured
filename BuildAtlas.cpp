#include "BuildAtlas.h"
#include "Extents.h"
#include <vips/vips.h>
#include "vcgapps/GenParam.h"
#include <osgDB/WriteFile>
#include "MemUtils.h"
using namespace std;
using namespace SpatialIndex;
using namespace vpb;
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <vips/vips.h>

#ifdef WITH_DMALLOC
#include <dmalloc.h>
#endif /*WITH_DMALLOC*/
#include <vips/intl.h>

/* Our main parameter struct.
 */
typedef struct {
        double xshrink;		/* Shrink factors */
        double yshrink;
        int mw;			/* Size of area we average */
        int mh;
        int np;			/* Number of pels we average */
} ShrinkInfo;

/* Our per-sequence parameter struct. We hold an offset for each pel we
 * average.
 */
typedef struct {
        REGION *ir;
        int *off;
} SeqInfo;

/* Free a sequence value.
 */
static int
shrink_stop( void *vseq, void *a, void *b )
{
        SeqInfo *seq = (SeqInfo *) vseq;

        IM_FREEF( im_region_free, seq->ir );

        return( 0 );
}

/* Make a sequence value.
 */
static void *
shrink_start( IMAGE *out, void *a, void *b )
{
        IMAGE *in = (IMAGE *) a;
        ShrinkInfo *st = (ShrinkInfo *) b;
        SeqInfo *seq;

        if( !(seq = IM_NEW( out, SeqInfo )) )
                return( NULL );

        /* Init!
         */
        seq->ir = NULL;
        seq->off = NULL;
        seq->ir = im_region_create( in );
        seq->off = IM_ARRAY( out, st->np, int );
        if( !seq->off || !seq->ir ) {
                shrink_stop( seq, in, st );
                return( NULL );
        }

        return( (void *) seq );
}
/* Integer shrink.
 */
#define ishrink(TYPE) \
        for( y = to; y < bo; y++ ) { \
                TYPE *q = (TYPE *) IM_REGION_ADDR( ror, le, y ); \
                \
                for( x = le; x < ri; x++ ) { \
                        int ix = x * st->xshrink; \
                        int iy = y * st->yshrink; \
                        TYPE *p = (TYPE *) IM_REGION_ADDR( ir, ix, iy ); \
                        \
                        for( k = 0; k < ir->im->Bands; k++ ) { \
                                int sum = 0; \
                                int *t = seq->off; \
                                \
                                for( z = 0; z < st->np; z++ ) \
                                        sum += p[*t++]; \
                                 \
                                *q++ = sum / st->np; \
                                p++; \
                        } \
                } \
        }
/* Integer shrink.
 */
#define ishrink_noblack(TYPE) \
        for( y = to; y < bo; y++ ) { \
                TYPE *q = (TYPE *) IM_REGION_ADDR( ror, le, y ); \
                \
                for( x = le; x < ri; x++ ) { \
                        int ix = x * st->xshrink; \
                        int iy = y * st->yshrink; \
                        TYPE *p = (TYPE *) IM_REGION_ADDR( ir, ix, iy ); \
                         bzero(chk,st->np*sizeof(unsigned char));\
                        for( k = 0; k < ir->im->Bands; k++ ) { \
                                int *t = seq->off; \
                                \
                                for( z = 0; z < st->np; z++ ) \
                                        chk[z] += (p[*t++] == 0) ? 1 : 0; \
                                 \
                                p++; \
                        } \
                       p = (TYPE *) IM_REGION_ADDR( ir, ix, iy ); \
                                                                         \
                                  for( k = 0; k < ir->im->Bands; k++ ) { \
                                     int sum = 0; \
                                     int valid=0;\
                                    int *t = seq->off; \
                                                          \
                                       for( z = 0; z < st->np; z++ ) \
                                           if(chk[z] < 3 ){ sum += p[*t++]; valid++;} \
                                                              \
                                               *q++ = (valid >0) ? sum / valid : 0; \
                                                p++; \
                                   } \
                } \
        }



/* Shrink a REGION.
 */
 int
shrink_gen( REGION *ror, void *vseq, void *a, void *b )
{
        SeqInfo *seq = (SeqInfo *) vseq;
        ShrinkInfo *st = (ShrinkInfo *) b;
        REGION *ir = seq->ir;
        Rect *r = &ror->valid;
        Rect s;
        int le = r->left;
        int ri = IM_RECT_RIGHT( r );
        int to = r->top;
        int bo = IM_RECT_BOTTOM(r);

        int x, y, z, k;

        /* What part of the input image do we need? Very careful: round left
         * down, round right up.
         */
        s.left = r->left * st->xshrink;
        s.top = r->top * st->yshrink;
        s.width = ceil( IM_RECT_RIGHT( r ) * st->xshrink ) - s.left;
        s.height = ceil( IM_RECT_BOTTOM( r ) * st->yshrink ) - s.top;
        if( im_prepare( ir, &s ) )
                return( -1 );

        /* Init offsets for pel addressing. Note that offsets must be for the
         * type we will address the memory array with.
         */
        unsigned char *chk=(unsigned char *)malloc(st->np*sizeof(unsigned char)); bzero(chk,st->np*sizeof(unsigned char));

        for( z = 0, y = 0; y < st->mh; y++ )
                for( x = 0; x < st->mw; x++ )
                        seq->off[z++] = (IM_REGION_ADDR( ir, x, y ) -
                                IM_REGION_ADDR( ir, 0, 0 )) /
                                IM_IMAGE_SIZEOF_ELEMENT( ir->im );

        switch( ir->im->BandFmt ) {
       case IM_BANDFMT_UCHAR: 		ishrink_noblack(unsigned char); break;
       // case IM_BANDFMT_UCHAR: 		ishrink(unsigned char); break;

       /* case IM_BANDFMT_CHAR: 		ishrink(char); break;
        case IM_BANDFMT_USHORT: 	ishrink(unsigned short); break;
        case IM_BANDFMT_SHORT: 		ishrink(short); break;
        case IM_BANDFMT_UINT: 		ishrink(unsigned int); break;
        case IM_BANDFMT_INT: 		ishrink(int);  break;
        case IM_BANDFMT_FLOAT: 		fshrink(float); break;
        case IM_BANDFMT_DOUBLE:		fshrink(double); break;
*/
        default:
                im_error( "im_shrink", "%s", _( "unsupported input format" ) );
                free(chk);

                return( -1 );

        }
        free(chk);

        return( 0 );
}

static int
shrink_noblack( IMAGE *in, IMAGE *out, double xshrink, double yshrink )
{
        ShrinkInfo *st;

        /* Check parameters.
         */
        if( !in || vips_bandfmt_iscomplex( in->BandFmt ) ) {
                im_error( "im_shrink", "%s", _( "non-complex input only" ) );
                return( -1 );
        }
        if( xshrink < 1.0 || yshrink < 1.0 ) {
                im_error( "im_shrink",
                        "%s", _( "shrink factors should both be >1" ) );
                return( -1 );
        }
        if( im_piocheck( in, out ) )
                return( -1 );

        /* Prepare output. Note: we round the output width down!
         */
        if( im_cp_desc( out, in ) )
                return( -1 );
        out->Xsize = in->Xsize / xshrink;
        out->Ysize = in->Ysize / yshrink;
        out->Xres = in->Xres / xshrink;
        out->Yres = in->Yres / yshrink;
        if( out->Xsize <= 0 || out->Ysize <= 0 ) {
                im_error( "im_shrink",
                        "%s", _( "image has shrunk to nothing" ) );
                return( -1 );
        }

        /* Build and attach state struct.
         */
        if( !(st = IM_NEW( out, ShrinkInfo )) )
                return( -1 );
        st->xshrink = xshrink;
        st->yshrink = yshrink;
        st->mw = ceil( xshrink );
        st->mh = ceil( yshrink );
        st->np = st->mw * st->mh;

        /* Set demand hints. We want THINSTRIP, as we will be demanding a
         * large area of input for each output line.
         */
        if( im_demand_hint( out, IM_SMALLTILE, in, NULL ) )
                return( -1 );

        /* Generate!
         */
        if( im_generate( out,
                shrink_start, shrink_gen, shrink_stop, in, st ) )
                return( -1 );

        return( 0 );
}

/* Wrap up the above: do IM_CODING_LABQ as well.
 */
int
im_shrink_noblack( IMAGE *in, IMAGE *out, double xshrink, double yshrink )
{
        if( xshrink == 1 && yshrink == 1 ) {
                return( im_copy( in, out ) );
        }
        else if( in->Coding == IM_CODING_LABQ ) {
                IMAGE *t[2];

                if( im_open_local_array( out, t, 2, "im_shrink:1", "p" ) ||
                        im_LabQ2LabS( in, t[0] ) ||
                        shrink_noblack( t[0], t[1], xshrink, yshrink ) ||
                        im_LabS2LabQ( t[1], out ) )
                        return( -1 );
        }
        else if( in->Coding == IM_CODING_NONE ) {
                if( shrink_noblack( in, out, xshrink, yshrink ) )
                        return( -1 );
        }
        else {
                im_error( "im_shrink", "%s", _( "unknown coding type" ) );
                return( -1 );
        }

        return( 0 );
}
// im_shrink: shrink image by xfac, yfac times
vips::VImage shrink_noblack(vips::VImage &in, double xfac, double yfac )
{
    vips::VImage out;

    /* vips::Vargv _vec( "im_shrink" );

        _vec.data(0) = in.image();
        _vec.data(1) = out.image();
        *((double*) _vec.data(2)) = xfac;
        *((double*) _vec.data(3)) = yfac;
        _vec.call();*/
    im_shrink_noblack(in.image(),out.image(),xfac,yfac);
    out._ref->addref( in._ref );

    return( out );
}
bool readMatrixToScreen(std::string fname,osg::Matrixd &viewProj){
    std::fstream file(fname.c_str(), std::ios::binary|std::ios::in);
    if(!file.good()){
        fprintf(stderr,"Can't open %s\n",fname.c_str());
        return false;
    }

    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            file.read(reinterpret_cast<char*>(&(viewProj(i,j))), sizeof(double));

    return true;
}

bool readMatrix(std::string fname,osg::Matrix &viewProj){
    std::fstream file(fname.c_str(), std::ios::binary|std::ios::in);
    if(!file.good()){
        fprintf(stderr,"Can't open %s\n",fname.c_str());
        return false;
    }

    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            file.read(reinterpret_cast<char*>(&(viewProj(i,j))), sizeof(double));

    return true;
}
void generateAtlasAndTexCoordMappingFromExtents(const std::vector<mosaic_cell> &mosaic_cells,
                                                       const osg::Vec2 minT,
                                                       const osg::Vec2 maxT,int origX,int origY,
                                                       const osg::Matrix &toTex,
                                                       TightFitAtlasBuilder* atlas,
                                                       osg::Vec4 &ratio,
                                                       int level){
    double downsampleFactor=pow(2.0,level);
    int downsampledImageIndex=level-1;

    //  printf("Downsample Factor %f\n",downsampleFactor);
    //double downsampleRatio=1.0/downsampleFactor;
    // std::cout<< minT << " " << maxT<<std::endl;
    // printf("%f %f\n",downsampleRatio,downsampleRatio);
    int x=/*0;*/(int)std::max((int)floor(minT.x()),0);
    int y=/*0;*/(int)std::max((int)floor(minT.y()),0);
    int xMax=/*origX;*/(int)std::min((int)ceil(maxT.x()),origX);
    int yMax=/*origY;*/(int)std::min((int)ceil(maxT.y()),origY);

    // xMax=origX;   yMax=origY; x=0;y=0;


    //int xRange=(xMax-x);
    // int yRange=(yMax-y);
    // cout << "FFFF" <<minT<<" "<<maxT<<endl;
    // printf("X:%d -- %d Y:%d -- %d ",x,xMax,y,yMax);
    osg::BoundingBoxd areaBB(x,y,0,xMax,yMax,1);
    //osg::Vec2 subSize=osg::Vec2(xRange,yRange);
    //Need bias of 1.0 or will round down
    /* double maxSide=std::max(osg::Image::computeNearestPowerOfTwo(xRange,1.0),osg::Image::computeNearestPowerOfTwo(yRange,1.0));
    osg::Vec2 subSize=osg::Vec2(maxSide,maxSide);
    if(downsampleRatio*maxSide < 1.0){
        printf("Clipping %f %f to",downsampleRatio,downsampleFactor);
        downsampleRatio=1.0/maxSide;
        downsampleFactor=maxSide;
        printf("%f %f\n",downsampleRatio,downsampleFactor);

    }*/

    // osg::Vec2 downsampleSize(subSize.x(),subSize.y());
    // downsampleSize.x()*=downsampleRatio;
    //  downsampleSize.y()*=downsampleRatio;
    // printf("Range %d %d\n",xRange,yRange);
    // printf("%f %f %f %f\n",subSize.x(),subSize.y(),downsampleSize.x(),downsampleSize.y());

    {

        /*  unsigned char *tmpI=new unsigned char[totalX*totalY*3];     vips::VImage tmpT(tmpI,totalX,totalY,3,vips::VImage::FMTUCHAR);

        for(int i=0; i < mosaic_cells.size(); i++){
            if(mosaic_cells[i].mutex && mosaic_cells[i].img){
                OpenThreads::ScopedLock<OpenThreads::Mutex> lock(*mosaic_cells[i].mutex);
                tmpT.insertplace(*mosaic_cells[i].img,(int)mosaic_cells[i].bbox.xMin(),(int)mosaic_cells[i].bbox.yMin());
            }
        }
        tmpT.write("dang.v");*/
        // exit(-1);
        //   vips::VImage osgImage(image->data(),downsampleSize.x(),downsampleSize.y(),3,vips::VImage::FMTUCHAR);
        //  bzero(image->data(),downsampleSize.x()*downsampleSize.y()*3);
        bool checkFit=false;
        int mosaics_added=0;
        int maxMosaics=0;
        for(int i=0; i < (int)mosaic_cells.size(); i++){
            // cout << "AREA " <<areaBB._min << " " << areaBB._max<<"\n";
            // cout << "MOSIAC " <<i << " "<<mosaic_cells[i].bbox._min << " " << mosaic_cells[i].bbox._max<<"\n";
            if(mosaic_cells[i].name == "null")
                continue;
            if(areaBB.intersects(mosaic_cells[i].bbox)){
                if(!mosaic_cells[i].mutex || !mosaic_cells[i].img){
                    fprintf(stderr,"Fail to get mutex or img %s\n",mosaic_cells[i].name.c_str());
                    exit(-1);
                }
                vips::VImage *used_img=NULL;
                /*int ratio=1;

                int closestDownsample;
                for(closestDownsample=0; closestDownsample< (int)mosaic_cells[i].levels_ds.size()-1; closestDownsample++){
                    if(mosaic_cells[i].levels_ds[closestDownsample] >= downsampleFactor)
                        break;
                }*/
                if(downsampleFactor == 1){
                    used_img=mosaic_cells[i].img;
                    //ratio=1;

                }else{
                    used_img=mosaic_cells[i].img_ds[downsampledImageIndex];
                    /*
                    ratio=downsampleFactor/mosaic_cells[i].levels_ds[closestDownsample];
                    if(mosaic_cells[i].levels_ds[closestDownsample] == 1)
                        used_img=mosaic_cells[i].img;
                    else
                        used_img=new vips::VImage(mosaic_cells[i].name_ds[closestDownsample].c_str(),"r");*/


                }

                osg::BoundingBoxd imgBB=areaBB.intersect(mosaic_cells[i].bbox);
                int tileStartX= (int)(imgBB.xMin()-mosaic_cells[i].bbox.xMin());
                int tileStartY= (int)(imgBB.yMin()-mosaic_cells[i].bbox.yMin());
                int tileEndX=(int)(imgBB.xMax()-mosaic_cells[i].bbox.xMin());
                int tileEndY= (int)(imgBB.yMax()-mosaic_cells[i].bbox.yMin());
                int tileRangeX=tileEndX-tileStartX;
                int tileRangeY=tileEndY-tileStartY;
                //int outOffsetX=(int)(imgBB.xMin()-areaBB.xMin());
                //int outOffsetY=(int)(imgBB.yMin()-areaBB.yMin());
                //downsampledXoff=0;
                //downsampledYoff=0;
                //int downsampledXoff=outOffsetX/downsampleFactor;
                //int downsampledYoff=outOffsetY/downsampleFactor;
                int downsampledtileStartX=(int)floor(tileStartX/downsampleFactor);
                int downsampledtileStartY=(int)floor(tileStartY/downsampleFactor);


                int downsampledtileRangeX=(int)floor(tileRangeX/downsampleFactor);
                int downsampledtileRangeY=(int)floor(tileRangeY/downsampleFactor);
                if(!checkFit){
                    maxMosaics=atlas->getMaxNumImagesPerAtlas(downsampledtileRangeX,downsampledtileRangeY);
                    checkFit=true;
                }

                /*  int fullTileSize=mosaic_cells[i].bbox.xMax()-mosaic_cells[i].bbox.xMin();
                while(tmpCntX<outOffsetX){
                    tmpCntX+=fullTileSize;
                    downsampledXoff++;
                }
                double remX=fmod((double)outOffsetX,downsampleFactor);
                double remY=fmod((double)outOffsetY,downsampleFactor);



                printf("Getting %d %d %d %d ---\n Moasic [%d] %d %d %d %d \n From %d %d %d %d [ %d %d] \n Out offset %d %d [%f %f] [%f %f] %f [%d %d] ",x,y,xMax,yMax,i,(int)mosaic_cells[i].bbox.xMin(),
                       (int)mosaic_cells[i].bbox.yMin(),
                       (int)mosaic_cells[i].bbox.xMax(),
                       (int)mosaic_cells[i].bbox.yMax(),
                       tileStartX,
                       tileStartY,
                       tileEndX,
                       tileEndY,
                       tileRangeX,
                       tileRangeY,
                       outOffsetX,
                       outOffsetY,
                       (outOffsetX*downsampleRatio),
                       (outOffsetY*downsampleRatio),
                       tileRangeX*downsampleRatio,
                       tileRangeY*downsampleRatio,
downsampledXoff,
downsampledYoff,
                       downsampleSize.x()
                       );
*/
                // if(mosaic_cells[i].img->Bands() == 3)
                //if(tileRangeX < ratio || tileRangeY < ratio)
                // continue;
                if(downsampledtileRangeX < 1 || downsampledtileRangeY < 1)
                    continue;

                //  vips::VImage tmpI=mosaic_cells[i].img->extract_area(tileStartX,
                //                                                    tileStartY,
                //                                                  tileRangeX,
                //                                                tileRangeY).shrink(downsampleFactor,downsampleFactor);
                //printf(" Size [%d %d]\n",used_img->Xsize(),used_img->Ysize());
                /*  osgImage.insertplace(used_img->extract_area(downsampledtileStartX,
                                                            downsampledtileStartY,
                                                            downsampledtileRangeX,
                                                            downsampledtileRangeY).shrink(ratio,ratio),*/
                {

                    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(*mosaic_cells[i].mutex);
                    osg::ref_ptr<osg::Image> image = new osg::Image;
                    image->allocateImage(downsampledtileRangeX,downsampledtileRangeY, 1, GL_RGB,GL_UNSIGNED_BYTE);
                    vips::VImage osgImage(image->data(),image->s(),image->t(),3,vips::VImage::FMTUCHAR);
                    used_img->extract_area(downsampledtileStartX,
                                           downsampledtileStartY,
                                           downsampledtileRangeX,
                                           downsampledtileRangeY)/*.shrink(downsampleFactor,downsampleFactor)*/.write(osgImage);
                   /* osgDB::writeImageFile(*image,"1.png");
                    osgImage.write("2.png");
                    exit(0);*/
                    typedef osg::Matrix::value_type Float;
                    //Float tileOrigSizeX=mosaic_cells[i].bbox.xMax()-mosaic_cells[i].bbox.xMin();
                    //   Float tileOrigSizeY=mosaic_cells[i].bbox.yMax()-mosaic_cells[i].bbox.yMin();

                    double xscale=Float(origX)/Float(tileRangeX);
                    double yscale=Float(origY)/Float(tileRangeY);
                    // printf("%f %f\n",xscale*(-tileRangeX/origX),yscale*(-tileRangeY/origY));

                    atlas->offsetMats[i]=osg::Matrix::scale(xscale,yscale, 1.0)*
                            osg::Matrix::translate(xscale*(-imgBB.xMin()/origX),yscale*(-imgBB.yMin()/origY),0.0);
                    //osg::Matrix::translate(xscale*(-mosaic_cells[i].bbox.xMin()/origX),yscale*(-mosaic_cells[i].bbox.yMin()/origY),0.0);
                    // printf("%d \n",i);
                    // cout <<   atlas->offsetMats[i]<<endl;
                    // cout <<imgBB._min<<" "<<imgBB._max<<endl;
                    //  printf("\r %04d/%04d  maxMosaics: %04d ",i,(int)mosaic_cells.size(),maxMosaics);
                    fflush(stdout);
                    if(dynamic_cast<VipsAtlasBuilder*>(atlas)){
                        vips::VImage *v=new vips::VImage (used_img->extract_area(downsampledtileStartX,
                                                                                 downsampledtileStartY,
                                                                                 downsampledtileRangeX,
                                                                                 downsampledtileRangeY).flipver()) ;
                        atlas->atlasSourceMatrix[i]=v;
                        dynamic_cast<VipsAtlasBuilder*>(atlas)->addSource( v);
                    }else{
                        atlas->addSource(image);
                        atlas->atlasSourceMatrix[i]=image;

                    }
                    mosaics_added++;

                }
                if(checkFit && mosaics_added > maxMosaics){
                    fprintf(stderr,"Gonna fail probably more mosaics then will fit in atlas\n");
                }
                //if(downsampleFactor != 1 || mosaic_cells[i].levels_ds[closestDownsample] != 1){
                //   delete used_img;
                //}

            }
        }

        // osgImage.write("tmp.v");
        //exit(-1);
        //osg::setNotifyLevel(osg::INFO);
        if(dynamic_cast<VipsAtlasBuilder*>(atlas)){
                dynamic_cast<VipsAtlasBuilder*>(atlas)->buildAtlas();
            if( dynamic_cast<VipsAtlasBuilder*>(atlas)->getNumAtlases() != 1){
                fprintf(stderr,"Atlas fail!!! Downsample didn't allow for one atlas WIP! SEE MATT\nHopefully this will one day be replaced with working code :-)\n%d atlas\n",atlas->getNumAtlases());
                exit(-1);
            }
        }else{
            atlas->buildAtlas();
        //   printf("%d %d\n",atlas->getAtlasByNumber(0)->t(),atlas->getAtlasByNumber(0)->s());

            if( atlas->getNumAtlases() != 1){
                fprintf(stderr,"Atlas fail!!! Downsample didn't allow for one atlas WIP! SEE MATT\nHopefully this will one day be replaced with working code :-)\n%d atlas\n",atlas->getNumAtlases());
                exit(-1);
            }
        }
        //   exit(-1);
    }
    ratio=osg::Vec4(x,y,origX,origY);//subSize.x(),subSize.y());
    //osg::Vec2 f(xRange-subSize.x(),yRange-subSize.y());
    //std::cout << f<<std::endl;
}


void generateAtlasAndTexCoordMappingFromExtentsVips(const std::vector<mosaic_cell> &mosaic_cells,

                                                       VipsAtlasBuilder* atlas,bool flat,int level
                                                       ){


        for(int i=0; i < (int)mosaic_cells.size(); i++){
            if(mosaic_cells[i].name == "null")
                continue;
                if(!mosaic_cells[i].mutex || !mosaic_cells[i].img){
                    fprintf(stderr,"Fail to get mutex or img %s\n",mosaic_cells[i].name.c_str());
                   // exit(-1);
                    continue;
                }

                   vips::VImage* used_img=mosaic_cells[i].img;



                        vips::VImage *v=new vips::VImage (*used_img) ;
                        if(!v)
                            continue;
                        atlas->atlasSourceMatrix[i]=v;
                        if(level >0 ){
                            char tmp[1024];
                            sprintf(tmp,"%s-%02d.ppm",osgDB::getNameLessExtension(mosaic_cells[i].name).c_str(),level);
                            vips::VImage* level_img=new vips::VImage (tmp);
                            if(!level_img){
                                fprintf(stderr,"Failed to load ds image %s\n",tmp);
                                exit(-1);
                            }
                            dynamic_cast<VipsAtlasBuilder*>(atlas)->addSource( v,level_img,level);
                        }
                        else
                            dynamic_cast<VipsAtlasBuilder*>(atlas)->addSource( v);

                        //cout << used_img->filename()<<endl;
                    //mosaics_added++;


        }


     atlas->buildAtlas();

        if(  dynamic_cast<VipsAtlasBuilder*>(atlas)->getNumAtlases() != 1){
            fprintf(stderr,"VipsAtlasBuilder: Downsample didn't allow for one atlas FitCheck failing for some reason look there.\n%d atlas\n",atlas->getNumAtlases());
            exit(-1);
        }

        if(atlas->_atlasList.front()->_sourceList.size() < mosaic_cells.size()){
            fprintf(stderr,"Atlas sizes different %d %d Downsample didn't allow for one atlas FitCheck failing for some reason look there.\n",(int)atlas->_atlasList.front()->_sourceList.size() ,(int)mosaic_cells.size());
            exit(-1);
        }

}

void loadMosaicCells(std::string fname,int &totalX,int &totalY,std::vector<mosaic_cell> &mosaic_cells){
    ifstream inf( fname.c_str() );


    if(!inf.good()){
        fprintf(stderr,"Can't open image_areas.txt\n");
        exit(-1);
    }
    int cnt=0;
    while(!inf.eof()){
        char fname[1024];
        char fname_tif[1024];
        int minx,maxx,miny,maxy;
        std::vector<int> levels;
        char tmp_l[8192];
        int num_levels;
        //bool add=true;
        if( inf>> minx >>maxx >>miny>>maxy>>fname>>fname_tif >>num_levels){


            /*   for(int i=0;i< num_levels; i++){
                int tmp_level;
                inf>>tmp_level;
                levels.push_back(tmp_level);
            }
*/
            if(cnt ==0){
                totalX=maxx;
                totalY=maxy;

            }else{
                mosaic_cell cell;
                cell.bbox=osg::BoundingBoxd(miny,minx/*totalX-minx-(maxx-minx)*/,-FLT_MAX,maxy,/*totalX-maxx-(maxx-minx)*/ maxx,FLT_MAX);
                cell.name=std::string(fname);
                cell.levels=num_levels;
                cell.img=NULL;
                if(cell.name != "null"){
                    if(osgDB::fileExists(cell.name)){
                        cell.img = new vips::VImage(cell.name.c_str());
                        cell.img_ds.resize(cell.levels,NULL);
                        cell.name_ds.resize(cell.levels);

                        for(int i=0;i <(int)cell.levels; i++){
                            sprintf(tmp_l,"%s-%d.ppm",osgDB::getNameLessExtension(fname_tif).c_str(),i+1);
                            if(!osgDB::fileExists(tmp_l)){
                               // add=false;
                                break;
                            }
                            /*  vips::VImage *img=new vips::VImage(tmp_l);
                        if(!img){
                            std::cerr << "Can't open downsampled "<<tmp_l<<  " on reimaging run\n";
                        }
                        cell.img_ds[i]=img;*/
                            cell.img_ds[i]=NULL;

                            cell.name_ds[i]=string(tmp_l);
                        }
                        cell.mutex=new OpenThreads::Mutex;


                        // cout << "id: "<< id <<" \n" << plow[0] << " "<< plow[1]<< "\n"<< phigh[0]<<" "<< phigh[1]<<endl;
                        // cell_coordinate_map[rangeTC(minp,
                        //                                               maxp)]=mosaic_cells.size();
                        //   if(add)
                        //   mosaic_cells.push_back(cell);
                        /*std::map< MyDataSet::rangeTC ,int>::iterator itr;
                        for(itr=cell_coordinate_map.begin(); itr!=cell_coordinate_map.end(); itr++)
                            cout << "["<<itr->first.min() <<" - "<< itr->first.max()<< "] : " << itr->second<<"\n";*/

                    }else{
                        std::cerr << "Can't open "<<cell.name<<  " on reimaging run\n";
                        cell.img=NULL;
                    }
                }
                mosaic_cells.push_back(cell);


            }
            cnt++;


        }
    }
}

VipsAtlasBuilder* createVTAtlas(const osg::Matrix &viewProj,int totalX,int totalY,int POTAtlasSize,
                                const std::vector<mosaic_cell> &mosaic_cells,
                                bool writeAtlas,double scaleFactor,string basedir,string imgOutput,bool flat,double maxRes,int levelRun,int rowRun){
    int tileSize=256;
    int border=1;
    VipsAtlasBuilder* _atlas =new VipsAtlasBuilder(POTAtlasSize,mosaic_cells.size(),tileSize,border,false );
   // _atlas->setMargin(0);
  /*  int maxSize=pow(2.0,17);
    _atlas->setMaximumAtlasSize(maxSize,maxSize);
*/
    osg::Vec4 ratio(0.0,0.0,0,0);

    osg::Vec2 minT(0,0),maxT(totalX,totalY);
    osg::Matrix bottomLeftToTopLeft= (osg::Matrix::scale(1,-1,1)*osg::Matrix::translate(0,totalY,0));

    osg::Matrix toTex=viewProj*( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*totalX,0.5*totalY,0.5f))*bottomLeftToTopLeft;
    vips::VImage *atlas_image;
    cout <<"Loading images...\n";
    //im__print_all();

  //  if(!flat){
        generateAtlasAndTexCoordMappingFromExtentsVips(mosaic_cells,_atlas,flat,levelRun);
        if(!_atlas || !_atlas->_atlasList.size() || !_atlas->_atlasList.front() ){
            fprintf(stderr,"Can't create atlas bailing\n");
            return NULL;
        }
        printf("Loaded images\n");
      //  im__print_all();

        VipsAtlasBuilder::VAtlas *vatlas=_atlas->_atlasList.front();
        atlas_image=vatlas->_image;
   /* }else{
         unsigned char *tmpI=new unsigned char[totalX*totalY*3];
          atlas_image = new vips::VImage(tmpI,totalX,totalY,3,vips::VImage::FMTUCHAR);
            for(int i=0; i < mosaic_cells.size(); i++){
                if(mosaic_cells[i].img){
                    atlas_image->insertplace(*mosaic_cells[i].img,(int)mosaic_cells[i].bbox.xMin(),(int)mosaic_cells[i].bbox.yMin());
                }
            }
    }*/
#if 0
    // ("subtile.tif");
    osg::BoundingBoxd bbox;
    int origX=totalX,origY=totalY;
    osg::Matrix bottomLeftToTopLeft= (osg::Matrix::scale(1,-1,1)*osg::Matrix::translate(0,origY,0));
    int downsampleFactor=1;
    // osg::Matrix toTex=viewProj*( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*origX,0.5*origY,0.5f))*bottomLeftToTopLeft;
    for(int i=0; i <mosaic_cells.size(); i++){
        double xsize=mosaic_cells[i].bbox.xMax()-mosaic_cells[i].bbox.xMin();
        double ysize=mosaic_cells[i].bbox.yMax()-mosaic_cells[i].bbox.yMin();
        int tileStartX= (int)(mosaic_cells[i].bbox.xMin());
        int tileStartY= (int)(mosaic_cells[i].bbox.yMin());
        int tileEndX=(int)(mosaic_cells[i].bbox.xMax());
        int tileEndY= (int)(mosaic_cells[i].bbox.yMax());
        int tileRangeX=tileEndX-tileStartX;
        int tileRangeY=tileEndY-tileStartY;
        bool loadNew=false;
        vips::VImage *used_img=NULL;
        int downsampledImageIndex=0;
        bool debug=false;
        if(debug){
            char tmps[1024];
            //for(int k=0; k<500; k++)
            sprintf(tmps,"%d",i);
            //vips::VImage tmp[3];
            IMAGE *tmp[3];
            for(int k=0; k <3; k++){
                //tmp[k].initdesc(tileRangeX,tileRangeY,1,vips::VImage::FMTUCHAR,vips::VImage::NOCODING,vips::VImage::LUMINACE,1.0,1.0,0,0);
                tmp[k]=im_open("tmp","p");
                im_text(tmp[k],tmps,"times 142",0,0,300);
                //  tmp[k].text("times italic 14","times italic 14",100,0,100);
                // tmp[k].write("tmp.png");
                //            im_copy
            }
            im_bandjoin(tmp[0],tmp[1],tmp[0]);//.bandjoin(tmp[1]);
            //tmp[0].bandjoin(tmp[2]);
            im_bandjoin(tmp[0],tmp[2],tmp[0]);
            used_img=new vips::VImage;
            used_img->initdesc(tileRangeX,tileRangeY,3,vips::VImage::FMTUCHAR,vips::VImage::NOCODING,vips::VImage::sRGB,1.0,1.0,0,0);
            double xfac=tileRangeX/(double)tmp[0]->Xsize,yfac=tileRangeY/(double)tmp[0]->Ysize;
            IMAGE *tmplarge=im_open("mp","p");
            im_affine(tmp[0],tmplarge,
                      xfac ,0 ,0 ,yfac ,
                      0, 0,
                      0, 0, (tmp[0]->Xsize * xfac), (tmp[0]->Ysize * yfac));
            used_img->insertplace( tmplarge,0,0);
            //  used_img->write("tmp.png");

        }else{
            if(downsampleFactor == 1){
                used_img=mosaic_cells[i].img;

            }else{


                //  ratio=downsampleFactor/mosaic_cells[i].levels_ds[closestDownsample];
                // if(mosaic_cells[i].levels_ds[closestDownsample] == 1)

                //used_img=mosaic_cells[i].img_ds[downsampledImageIndex];
                //else
                used_img=new vips::VImage(mosaic_cells[i].name_ds[downsampledImageIndex].c_str(),"r");
                if(used_img)
                    loadNew=true;
            }
        }
        _atlas->atlasSourceMatrix[i]=used_img;
        typedef osg::Matrix::value_type Float;
        double xscale=Float(origX)/Float(tileRangeX);
        double yscale=Float(origY)/Float(tileRangeY);
        // printf("%f %f\n",xscale*(-tileRangeX/origX),yscale*(-tileRangeY/origY));
        cout << tileStartX << " "<< tileStartY<<endl;
        _atlas->offsetMats[i]=osg::Matrix::scale(xscale,yscale, 1.0)*
                osg::Matrix::translate(xscale*(-tileStartX/(double)origX),yscale*(-tileStartY/(double)origY),0.0);
        cout << i << " "<<_atlas->offsetMats[i]<<endl;
        //osg::Matrix::translate(xscale*(-mosaic_cells[i].bbox.xMin()/origX),yscale*(-mosaic_cells[i].bbox.yMin()/origY),0.0);
        // printf("%d \n",i);
        // cout <<   atlas->offsetMats[i]<<endl;
        // cout <<imgBB._min<<" "<<imgBB._max<<endl;
        printf("\r %04d/%04d ",i,(int)mosaic_cells.size());
        fflush(stdout);
        _atlas->addSource(used_img);
        if(loadNew)
            delete used_img;
    }
    _atlas->buildAtlas();
    if(_atlas->getNumAtlases() > 1){
        fprintf(stderr,"Problem can't fit all in one atlas bailing!!!!!\n");
        exit(-1);
    }

    if(!_atlas->_atlasList.size() || !_atlas->_atlasList.front()){
        fprintf(stderr,"No atlas bailing!!!!!\n");
        exit(-1);
    }

    for(int i=0; i <mosaic_cells.size(); i++){
        int atlasHeight=_atlas->getAtlasHeight();
        int atlasWidth=_atlas->getAtlasWidth();

        double xsize=mosaic_cells[i].bbox.xMax()-mosaic_cells[i].bbox.xMin();
        double ysize=mosaic_cells[i].bbox.yMax()-mosaic_cells[i].bbox.yMin();
        int tileStartX= (int)(mosaic_cells[i].bbox.xMin());
        int tileStartY= (int)(mosaic_cells[i].bbox.yMin());
        int tileEndX=(int)(mosaic_cells[i].bbox.xMax());
        int tileEndY= (int)(mosaic_cells[i].bbox.yMax());
        int tileRangeX=tileEndX-tileStartX;
        int tileRangeY=tileEndY-tileStartY;
        typedef osg::Matrix::value_type Float;

        double xscale=Float(atlasWidth)/Float(tileRangeX);
        double yscale=Float(atlasHeight)/Float(tileRangeY);
        cout << "offset "<< _atlas->offsetMats[i];
        cout << "tex " << _atlas->getTextureMatrix((unsigned int ) i);//=osg::Matrix::scale(xscale,yscale, 1.0)*
        //       osg::Matrix::translate(xscale*(-tileStartX/(double)atlasWidth),yscale*(-tileStartY/(double)atlasHeight),0.0);
    }

#endif
    if(maxRes > 0){
        if(atlas_image->Xsize()*scaleFactor > maxRes){
            double tmpScale=maxRes/(double)osg::Image::computeNearestPowerOfTwo(atlas_image->Xsize());
            if(scaleFactor>tmpScale){
                printf("New scale factor for max res %f was %f\n",tmpScale,scaleFactor);
                scaleFactor=tmpScale;
            }
        }
    }
    double downsampleFactorMult=1.0/scaleFactor;
    printf("--------------Atlas Size %dx%d------------------\n",(int)(atlas_image->Xsize()*scaleFactor),(int)(atlas_image->Ysize()*scaleFactor));
    if(imgOutput.size()){
     // shrink_noblack(*atlas_image,downsampleFactorMult,downsampleFactorMult).write(imgOutput.c_str());
        printf("Writing out img %s\n",imgOutput.c_str());
        atlas_image->shrink(downsampleFactorMult,downsampleFactorMult).write(imgOutput.c_str());
        printf("Done\n");
    }
   // im__print_all();


    int sizeLevel=atlas_image->Xsize()*scaleFactor;
    char dirname[1024];
    sprintf(dirname,"%s/vtex",basedir.c_str());
    osgDB::makeDirectory(dirname);
    chmod(dirname,0775);
    int maxLevels=    (int)ceil(std::log((double)sizeLevel/tileSize) / std::log(2.0));

    if(writeAtlas){
        int adjustedTileSize=tileSize-(border *2);
        for(int level=0; sizeLevel>=adjustedTileSize; level++,sizeLevel/=2 ){
            if(levelRun>=0){
                if(level < levelRun || level > levelRun)
                    continue;
            }
            sprintf(dirname,"%s/vtex/tiles_b%d_level%d",basedir.c_str(),border,level);
            int numXtiles=(sizeLevel/adjustedTileSize);
            int numYtiles=(sizeLevel/adjustedTileSize);
            osgDB::makeDirectory(dirname);
            chmod(dirname,0775);
            int cnt=0;


            for(int x=0; x<numXtiles; x++){
                if(rowRun >=0){
                    if(x < rowRun || x > rowRun)
                        continue;
                }
                for(int y=0; y <numYtiles;y++){
                    printf("\33[2K\r");
                    printf("\rDoing VT Level %02d/%02d - %dx%d - %.2f%%",level,maxLevels,numXtiles,numYtiles,100.0*(cnt++/(double)(numYtiles*numXtiles)));
                    fflush(stdout);

                    vips::VImage part ;
                    double downsampleFactor=pow(2.0,level)*downsampleFactorMult;

                    if (x != 0 && y !=0 && x !=(numXtiles-1) && y != (numYtiles-1) ){
                        if(vatlas->_level == level){
                            part=  vatlas->_ds_image->extract_area(x * adjustedTileSize - border, y * adjustedTileSize - border, tileSize,tileSize);
                        }else{

                            part=  shrink_noblack(*atlas_image,downsampleFactor,downsampleFactor).extract_area(x * adjustedTileSize - border, y * adjustedTileSize - border, tileSize,tileSize);
                        }
                    }else{
                        int offx=(x * adjustedTileSize - border);
                        int offy=(y * adjustedTileSize - border);
                        int w= ( x * adjustedTileSize + adjustedTileSize + border)-offx;
                        int h=( y * adjustedTileSize + adjustedTileSize + border)-offy;

                        int sizeX=atlas_image->Xsize()/downsampleFactor;
                        int sizeY=atlas_image->Ysize()/downsampleFactor;

                        if(w + offx >= sizeX)
                            w=sizeX-offx-1;
                        if(h + offy >= sizeY)
                            h=sizeY-offy-1;
                        int x1=0,y1=0;

                        if(offx < 0){
                            x1=-offx;
                            offx=0;
                        }
                        if(offy <0){
                            y1=-offy;
                            offy=0;
                        }
                      //  printf("Extracing %d,%d -- %d -- %d width %d height %d\n",offx,offy,offx+w,offy+h,sizeX,sizeY);
                        vips::VImage tmpI;
                        if(vatlas->_level == level){
                            tmpI =  vatlas->_ds_image->extract_area(offx , offy , w,h);
                        }
                        else{

                            tmpI =  shrink_noblack(*atlas_image,downsampleFactor,downsampleFactor).extract_area(offx , offy , w,h);

                        }
                        vips::VImage blackI= vips::VImage::black(tileSize,tileSize,3);

                        part =blackI.insert_noexpand(tmpI,x1,y1);
                        if (x == 0)		part.insertplace(part.extract_area(border, 0, border, adjustedTileSize + border*2), 0, 0);
                        if (y == 0)		part.insertplace(part.extract_area(0, border, adjustedTileSize + border*2, border), 0, 0);
                       if (x == numXtiles)	part.insertplace(part.extract_area(adjustedTileSize, 0, border, adjustedTileSize + border*2), adjustedTileSize + border, 0);
                       if (y == numYtiles)	part.insertplace(part.extract_area(0, adjustedTileSize, adjustedTileSize + border*2, border), 0, adjustedTileSize + border);

                    }
                    char tmp[1024];
                    sprintf(tmp,"%s/tile_%d_%d_%d.jpg",dirname,level,x,y);
                    if(level >maxLevels-3-1){
                        dilateEdgeNew(part,tmp,1    );
                    }else{
                        part.write(tmp);
                    }
                }
            }
        }
        printf("\rDoing Level %02d/%02d\n",maxLevels,maxLevels);

    }
    return _atlas;
}


bool shouldUseSparseMode(const std::vector<mosaic_cell> &mosaic_cells,double sparseRatio, const osg::Vec2 minT,
                         const osg::Vec2 maxT, osg::ref_ptr<TightFitAtlasBuilder> &atlas,int leveloffset,        const std::set<int> &seenids){
    double ratioFill=0;

    set<int>::iterator it;
    do{
        double downsampleFactor=pow(2.0,leveloffset);
        double downsampleRatio=1.0/downsampleFactor;
        double atlasArea=atlas->getMaximumAtlasHeight()*atlas->getMaximumAtlasWidth();
        double virtArea=(maxT.x()-minT.x())*(maxT.y()-minT.y())*downsampleRatio*downsampleRatio;
        double totalArea=0;
        for(it=seenids.begin(); it!=seenids.end(); it++){
            if(*it>=0 && *it <  (int)mosaic_cells.size()){
                double xsize=mosaic_cells[*it].bbox.xMax()-mosaic_cells[*it].bbox.xMin();
                double ysize=mosaic_cells[*it].bbox.yMax()-mosaic_cells[*it].bbox.yMin();
                totalArea+=xsize*ysize*downsampleRatio*downsampleRatio;
                if(downsampleFactor > xsize){
                    fprintf(stderr,"Failed to downsample images to fit in one atlas %f %f \n",downsampleFactor,xsize);
                    exit(-1);
                }
            }
        }
        //   printf("%f %f\n",atlasArea ,totalArea);
        if(atlasArea >= totalArea){
            if(totalArea>0){
                ratioFill=totalArea/virtArea;
                if(ratioFill < sparseRatio)
                    return true;
            }
            break;
        }
        leveloffset++;
    }while(leveloffset < 100);
    return false;
}

SpatialIndex::ISpatialIndex* createTree(const std::vector<mosaic_cell> &mosaic_cells){

    SpatialIndex::ISpatialIndex* tree;
    SpatialIndex::IStorageManager* memstore;
    SpatialIndex::IStorageManager* manager;

    //double utilization;
    int capacity;

    // utilization=0.7;
    capacity=4;

    memstore = StorageManager::createNewMemoryStorageManager();
    // Create a new storage manager with the provided base name and a 4K page size.
    id_type indexIdentifier;

    manager = StorageManager::createNewRandomEvictionsBuffer(*memstore, 10, false);
    tree = RTree::createNewRTree(*manager, 0.7, capacity,capacity,2, SpatialIndex::RTree::RV_RSTAR, indexIdentifier);
    double plow[2], phigh[2];
    for(int i=0; i<(int)mosaic_cells.size(); i++){
        plow[0]= mosaic_cells[i].bbox.xMin();
        plow[1]= mosaic_cells[i].bbox.yMin();

        phigh[0]=mosaic_cells[i].bbox.xMax();
        phigh[1]=mosaic_cells[i].bbox.yMax();

        Region r = Region(plow, phigh, 2);
        id_type id=i;
        tree->insertData(0, 0, r,id);
    }
    return tree;
}

namespace vpb{

id_type getImageIndexForPoint(const SpatialIndex::Point &p,CompositeDestination::TileList &_tiles){
    bool notfound=true;
    id_type index=-1;
    for(CompositeDestination::TileList::iterator titr = _tiles.begin();
        titr != _tiles.end() && notfound;
        ++titr)
    {
        MyDestinationTile* tile = dynamic_cast<MyDestinationTile*>(titr->get());
        for(DestinationTile::Sources::iterator itr = tile->_sources.begin();
            itr != tile->_sources.end() && notfound;
            ++itr)
        {
            TexturedSource *source=dynamic_cast<TexturedSource*>((*itr).get());
            if(!source || !source->tree || source->_cameras.size() == 0)
                continue;


            ObjVisitor pt_vis(p,8);
            source->intersectsWithQuery(p,pt_vis);
            if(pt_vis.GetResultCount()){
                notfound=false;
                index=pt_vis.GetResults()[0];
                return index;
            }

        }
    }
    return index;
}
};


void generateImageFromExtents(const std::vector<mosaic_cell> &mosaic_cells,
                                                       const osg::Vec2 minT,
                                                       const osg::Vec2 maxT,int origX,int origY,
                                                       osg::Vec4 &texsize,
                                                       const osg::Matrix &toTex,
                                                       osg::ref_ptr<osg::Image> &image,
                                                       osg::Vec4 &ratio,
                                                       int level){

    double downsampleFactor=pow(2.0,level);
    int downsampledImageIndex=level-1;
    //  printf("Downsample Factor %f\n",downsampleFactor);
    double downsampleRatio=1.0/downsampleFactor;
    //std::cout<< minT << " " << maxT<<std::endl;
    // printf("%f %f\n",downsampleRatio,downsampleRatio);
    int x=(int)std::max((int)floor(minT.x()),0);
    int y=(int)std::max((int)floor(minT.y()),0);
    int xMax=(int)std::min((int)ceil(maxT.x()),origX);
    int yMax=(int)std::min((int)ceil(maxT.y()),origY);
    int xRange=(xMax-x);
    int yRange=(yMax-y);
    // cout << "FFFF" <<minT<<" "<<maxT<<endl;
    // printf("X:%d -- %d Y:%d -- %d ",x,xMax,y,yMax);
    osg::BoundingBoxd areaBB(x,y,0,xMax,yMax,1);

    //Need bias of 1.0 or will round down
    double maxSide=std::max(osg::Image::computeNearestPowerOfTwo(xRange,1.0),osg::Image::computeNearestPowerOfTwo(yRange,1.0));
    osg::Vec2 subSize=osg::Vec2(maxSide,maxSide);
    if(downsampleRatio*maxSide < 1.0){
        printf("Clipping %f %f to",downsampleRatio,downsampleFactor);
        downsampleRatio=1.0/maxSide;
        downsampleFactor=maxSide;
        printf("%f %f\n",downsampleRatio,downsampleFactor);
        downsampledImageIndex=(int)ceil(std::log(maxSide)/std::log(2.0) )-2;

    }

    texsize[0]=origX;
    texsize[1]=origY;
    texsize[2]=subSize.x();
    texsize[3]=subSize.y();
    osg::Vec2 downsampleSize(subSize.x(),subSize.y());
    downsampleSize.x()*=downsampleRatio;
    downsampleSize.y()*=downsampleRatio;
    // printf("%f %f\n",downsampleSize.x(),downsampleSize.y());
    // printf("Range %d %d\n",xRange,yRange);
    // printf("%f %f %f %f\n",subSize.x(),subSize.y(),downsampleSize.x(),downsampleSize.y());
    image = new osg::Image;
    image->allocateImage(downsampleSize.x(),downsampleSize.y(), 1, GL_RGB,GL_UNSIGNED_BYTE);
    if(image->data() == 0 ){
        fprintf(stderr,"Failed to allocate\n");
        exit(-1);
    }
    {

        /*  unsigned char *tmpI=new unsigned char[totalX*totalY*3];     vips::VImage tmpT(tmpI,totalX,totalY,3,vips::VImage::FMTUCHAR);

        for(int i=0; i < mosaic_cells.size(); i++){
            if(mosaic_cells[i].mutex && mosaic_cells[i].img){
                OpenThreads::ScopedLock<OpenThreads::Mutex> lock(*mosaic_cells[i].mutex);
                tmpT.insertplace(*mosaic_cells[i].img,(int)mosaic_cells[i].bbox.xMin(),(int)mosaic_cells[i].bbox.yMin());
            }
        }
        tmpT.write("dang.v");*/
        // exit(-1);
        vips::VImage osgImage(image->data(),downsampleSize.x(),downsampleSize.y(),3,vips::VImage::FMTUCHAR);
        bzero(image->data(),downsampleSize.x()*downsampleSize.y()*3);
        for(int i=0; i < (int)mosaic_cells.size(); i++){
            // cout << "AREA " <<areaBB._min << " " << areaBB._max<<"\n";
            // cout << "MOSIAC " <<i << " "<<mosaic_cells[i].bbox._min << " " << mosaic_cells[i].bbox._max<<"\n";
            bool loadNew=false;
            if(mosaic_cells[i].name == "null")
                continue;
            if(areaBB.intersects(mosaic_cells[i].bbox)){
                if(!mosaic_cells[i].mutex || !mosaic_cells[i].img){
                    fprintf(stderr,"Fail to get mutex or img %s\n",mosaic_cells[i].name.c_str());
                    exit(-1);
                }
                vips::VImage *used_img=NULL;

                /*int closestDownsample;
                for(closestDownsample=0; closestDownsample< (int)mosaic_cells[i].levels_ds.size()-1; closestDownsample++){
                    if(mosaic_cells[i].levels_ds[closestDownsample] >= downsampleFactor)
                        break;
                }*/
                if(downsampleFactor == 1){
                    used_img=mosaic_cells[i].img;

                }else{


                    //  ratio=downsampleFactor/mosaic_cells[i].levels_ds[closestDownsample];
                    // if(mosaic_cells[i].levels_ds[closestDownsample] == 1)

                    //used_img=mosaic_cells[i].img_ds[downsampledImageIndex];
                    //else
                    //printf("%d %d \n",downsampledImageIndex,i);
                   // printf("%s\n",mosaic_cells[i].name_ds[downsampledImageIndex].c_str());
                    used_img=new vips::VImage(mosaic_cells[i].name_ds[downsampledImageIndex].c_str(),"r");
                    if(!used_img)
                        continue;
                    loadNew=true;

                }

                osg::BoundingBoxd imgBB=areaBB.intersect(mosaic_cells[i].bbox);
                int tileStartX= (int)(imgBB.xMin()-mosaic_cells[i].bbox.xMin());
                int tileStartY= (int)(imgBB.yMin()-mosaic_cells[i].bbox.yMin());
                int tileEndX=(int)(imgBB.xMax()-mosaic_cells[i].bbox.xMin());
                int tileEndY= (int)(imgBB.yMax()-mosaic_cells[i].bbox.yMin());
                int tileRangeX=tileEndX-tileStartX;
                int tileRangeY=tileEndY-tileStartY;
                int outOffsetX=(int)(imgBB.xMin()-areaBB.xMin());
                int outOffsetY=(int)(imgBB.yMin()-areaBB.yMin());
                //downsampledXoff=0;
                //downsampledYoff=0;
                int downsampledXoff=outOffsetX/downsampleFactor;
                int downsampledYoff=outOffsetY/downsampleFactor;
                int downsampledtileStartX=(int)floor(tileStartX/downsampleFactor);
                int downsampledtileStartY=(int)floor(tileStartY/downsampleFactor);



                int downsampledtileRangeX=(int)floor(tileRangeX/downsampleFactor);
                int downsampledtileRangeY=(int)floor(tileRangeY/downsampleFactor);
                // printf("%d %d COMPARE \n",used_img->Xsize(),(int)((mosaic_cells[i].bbox.yMax()-mosaic_cells[i].bbox.yMin())/downsampleFactor));
                /*  int fullTileSize=mosaic_cells[i].bbox.xMax()-mosaic_cells[i].bbox.xMin();
                while(tmpCntX<outOffsetX){
                    tmpCntX+=fullTileSize;
                    downsampledXoff++;
                }
                double remX=fmod((double)outOffsetX,downsampleFactor);
                double remY=fmod((double)outOffsetY,downsampleFactor);



                printf("Getting %d %d %d %d ---\n Moasic [%d] %d %d %d %d \n From %d %d %d %d [ %d %d] \n Out offset %d %d [%f %f] [%f %f] %f [%d %d] ",x,y,xMax,yMax,i,(int)mosaic_cells[i].bbox.xMin(),
                       (int)mosaic_cells[i].bbox.yMin(),
                       (int)mosaic_cells[i].bbox.xMax(),
                       (int)mosaic_cells[i].bbox.yMax(),
                       tileStartX,
                       tileStartY,
                       tileEndX,
                       tileEndY,
                       tileRangeX,
                       tileRangeY,
                       outOffsetX,
                       outOffsetY,
                       (outOffsetX*downsampleRatio),
                       (outOffsetY*downsampleRatio),
                       tileRangeX*downsampleRatio,
                       tileRangeY*downsampleRatio,
downsampledXoff,
downsampledYoff,
                       downsampleSize.x()
                       );
*/
                // if(mosaic_cells[i].img->Bands() == 3)
                //if(tileRangeX < ratio || tileRangeY < ratio)
                //   continue;
                //  vips::VImage tmpI=mosaic_cells[i].img->extract_area(tileStartX,
                //                                                    tileStartY,
                //                                                  tileRangeX,
                //                                                tileRangeY).shrink(downsampleFactor,downsampleFactor);
                //printf(" Size [%d %d]\n",used_img->Xsize(),used_img->Ysize());
                /*  osgImage.insertplace(used_img->extract_area(downsampledtileStartX,
                                                            downsampledtileStartY,
                                                            downsampledtileRangeX,
                                                            downsampledtileRangeY).shrink(ratio,ratio),*/
                if(downsampledtileRangeX < 1 || downsampledtileRangeY < 1)
                    continue;

                {

                    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(*mosaic_cells[i].mutex);

                    osgImage.insertplace(used_img->extract_area(downsampledtileStartX,
                                                                downsampledtileStartY,
                                                                downsampledtileRangeX,
                                                                downsampledtileRangeY)/*.shrink(downsampleFactor,downsampleFactor)*/,
                                         downsampledXoff,downsampledYoff );

                }
                if(loadNew)
                    delete used_img;
                /*if(downsampleFactor != 1 || mosaic_cells[i].levels_ds[closestDownsample] != 1){
                    //   delete used_img;
                }*/

            }
        }
        // osgImage.write("tmp.v");
        //exit(-1);
    }
    ratio=osg::Vec4(x,y,subSize.x(),subSize.y());
    //osg::Vec2 f(xRange-subSize.x(),yRange-subSize.y());
    //std::cout << f<<std::endl;
}
