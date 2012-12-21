#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <strings.h>

#include <vips/vips.h>
#include <vips/vips>
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
int main( int argc, char **argv ){
  IMAGE *in,*out;
  int sizex,sizey;

  /* Start up VIPS. This will load translations, init the
   * threading system, init any libraries that VIPS uses and load any
   * plugins.
   */
  if (im_init_world (argv[0]))
    error_exit ("unable to start VIPS");

  if (argc != 5)
    error_exit ("usage: %s <filename> <filename> sizex sizey", g_get_prgname ());

  if (!(in = im_open (argv[1], "r")))
    error_exit ("unable to open");
  if (!(out = im_open (argv[2], "w")))
    error_exit ("unable to open");

  sizex=atoi(argv[3]);
  sizey=atoi(argv[4]);

  if (im_shrink_noblack(in,out, sizex,sizey))
    error_exit ("unable to find shrink no black");

  im_close (in);
  im_close (out);


  return (0);
}


