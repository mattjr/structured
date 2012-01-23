#include "render_utils.h"
using namespace std;


std::string format_elapsed(double d)
{
    char buf[256] = {0};

    if( d < 0.00000001 )
    {
        // show in ps with 4 digits
        sprintf(buf, "%0.4f ps", d * 1000000000000.0);
    }
    else if( d < 0.00001 )
    {
        // show in ns
        sprintf(buf, "%0.0f ns", d * 1000000000.0);
    }
    else if( d < 0.001 )
    {
        // show in us
        sprintf(buf, "%0.0f us", d * 1000000.0);
    }
    else if( d < 0.1 )
    {
        // show in ms
        sprintf(buf, "%0.0f ms", d * 1000.0);
    }
    else if( d <= 60.0 )
    {
        // show in seconds
        sprintf(buf, "%0.2f s", d);
    }
    else if( d < 3600.0 )
    {
        // show in min:sec
        sprintf(buf, "%01.0f:%02.2f", floor(d/60.0), fmod(d,60.0));
    }
    // show in h:min:sec
    else
        sprintf(buf, "%01.0f:%02.0f:%02.2f", floor(d/3600.0), floor(fmod(d,3600.0)/60.0), fmod(d,60.0));

    return buf;
}
// Fast downsize of both width and height by 50%
// Only supports 8-bit RGB images
void fast_mipmap_downsize( const IplImage *source, IplImage *dest )
{

    assert( dest->width ==source->width/2 );// && source->width%2 ==0 );
    assert( dest->height==source->height/2 );//&& source->height%2==0 );
    assert( dest->depth==IPL_DEPTH_8U && source->depth==IPL_DEPTH_8U );
    assert( dest->nChannels==3 && source->nChannels==3 );
    if( source->width%2 !=0 || source->height%2!=0){
        cvResize(source,dest);
        return;
    }
    unsigned char *s1, *s2, *d;
    int sum;
    for( int y=0; y<dest->height; y++ )
    {
        s1 = (unsigned char*)&source->imageData[source->widthStep*(y*2)];
        s2 = (unsigned char*)&source->imageData[source->widthStep*(y*2+1)];
        d  = (unsigned char*)&dest->imageData[dest->widthStep*y];
        for( int j=dest->width; j-- ; s1+=3, s2+=3 )
        {
            // When we do integer division by 4 we can add two to get rounding up
            // and down happening correctly
            //*d++ = (*s1 + *((s1++)+3) + *s2 + *((s2++)+3) + 2)/4;
            //*d++ = (*s1 + *((s1++)+3) + *s2 + *((s2++)+3) + 2)/4;
            //*d++ = (*s1 + *((s1++)+3) + *s2 + *((s2++)+3) + 2)/4;

            // For some reason this is faster. Rounding up is done by checking if
            // the remainder after dividing by four is 2 or 3.
            sum = *s1 + *((s1++)+3) + *s2 + *((s2++)+3);
            *d++ = sum/4 + (sum%4)/2;
            sum = *s1 + *((s1++)+3) + *s2 + *((s2++)+3);
            *d++ = sum/4 + (sum%4)/2;
            sum = *s1 + *((s1++)+3) + *s2 + *((s2++)+3);
            *d++ = sum/4 + (sum%4)/2;
        }
    }
}

osg::Vec3 jetColorMap(const float& val) {
       assert(val <= 1.0f && val >= 0.0f );

       //truncated triangles where sides have slope 4
       osg::Vec3 jet;

       jet[0] = min(4.0f * val - 1.5f,-4.0f * val + 4.5f) ;
       jet[1] = min(4.0f * val - 0.5f,-4.0f * val + 3.5f) ;
       jet[2] = min(4.0f * val + 0.5f,-4.0f * val + 2.5f) ;


       jet[0] = clamp(jet[0], 0.0f, 1.0f);
       jet[1] = clamp(jet[1], 0.0f, 1.0f);
       jet[2] = clamp(jet[2], 0.0f, 1.0f);

       return jet;
}
static void rgb2hsv(const osg::Vec3 &rgb, float *h, float *s, float *v)
{
   float r, g, b;
   float max, min;

   r = rgb[0];
   g = rgb[1];
   b = rgb[2];

   max = std::max(r, std::max(g, b));
   min = std::min(r, std::min(g, b));

   if (eq(max, min)){
       *h = 0;
   }else if (eq(max, r) && g >= b){
       *h = 60.f / 360.f * ((g - b) / (max - min));
   }else if (eq(max, r)){ // g < b is implicit
       *h = (60.f / 360.f * ((g - b) / (max - min))) + 1;
   }else if (eq(max, g)){
       *h = (60.f / 360.f * ((b - r) / (max - min))) + (120.f / 360.f);
   }else{ // eq(max, b)
       *h = (60.f / 360.f * ((r - g) / (max - min))) + (240.f / 360.f);
   }

   if (isZero(max)){
       *s = 0.f;
   }else{
       *s = 1 - (min / max);
   }

   *v = max;
}
osg::Vec3d rgb2xyz( osg::Vec3 c ) {
   /*osg::Vec3d tmp;
   tmp.x() = ( c.x() > 0.04045 ) ? pow( ( c.x() + 0.055 ) / 1.055, 2.4 ) : c.x() / 12.92;
   tmp.y() = ( c.y() > 0.04045 ) ? pow( ( c.y() + 0.055 ) / 1.055, 2.4 ) : c.y() / 12.92,
   tmp.z() = ( c.z() > 0.04045 ) ? pow( ( c.z() + 0.055 ) / 1.055, 2.4 ) : c.z() / 12.92;*/

   // The following performs an inverse "gamma correction" specified by the sRGB
   // color space.  sRGB is defined by a cononical definition of a display
   // monitor and has been standardized by the International Electrotechnical
   // Commission (IEC 61966-2-1).  However, it is a non-linear color space, which
   // means that it will invalidate the color computations done by OpenGL.
   // Furthermore, most OS or display drivers will do some gamma correction on
   // there own, so displaying these colors directly usually results in overly
   // bright images.  Thus, the non-linear RGB values are inappropriate for VTK,
   // so the code is commented out.  If someone needs sRGB values in the future,
   // there should be a separate set of color conversion functions for that.
   //   if ( r > 0.04045 ) r = pow(( r + 0.055 ) / 1.055, 2.4);
   //   else               r = r / 12.92;
   //   if ( g > 0.04045 ) g = pow(( g + 0.055 ) / 1.055, 2.4);
   //   else               g = g / 12.92;
   //   if ( b > 0.04045 ) b = pow(( b + 0.055 ) / 1.055, 2.4);
   //   else               b = b / 12.92;
   osg::Vec3 xyz;
   //Observer. = 2 deg, Illuminant = D65
   xyz[0] = c[0] * 0.4124 + c[1] * 0.3576 + c[2] * 0.1805;
   xyz[1] = c[0] * 0.2126 + c[1] * 0.7152 + c[2] * 0.0722;
   xyz[2] = c[0] * 0.0193 + c[1] * 0.1192 + c[2] * 0.9505;
   return xyz;
}

osg::Vec3 xyz2lab( osg::Vec3 c ) {
   const double ref_X = 0.9505;
   const double ref_Y = 1.000;
   const double ref_Z = 1.089;
   double var_X = c[0] / ref_X;  //ref_X = 0.9505  Observer= 2 deg, Illuminant= D65
   double var_Y = c[1] / ref_Y;  //ref_Y = 1.000
   double var_Z = c[2] / ref_Z;  //ref_Z = 1.089

   if ( var_X > 0.008856 ) var_X = pow(var_X, 1.0/3.0);
   else                    var_X = ( 7.787 * var_X ) + ( 16.0 / 116.0 );
   if ( var_Y > 0.008856 ) var_Y = pow(var_Y, 1.0/3.0);
   else                    var_Y = ( 7.787 * var_Y ) + ( 16.0 / 116.0 );
   if ( var_Z > 0.008856 ) var_Z = pow(var_Z, 1.0/3.0);
   else                    var_Z = ( 7.787 * var_Z ) + ( 16.0 / 116.0 );
   osg::Vec3 Lab;
   Lab[0] = ( 116 * var_Y ) - 16;
   Lab[1] = 500 * ( var_X - var_Y );
   Lab[2] = 200 * ( var_Y - var_Z );
   return Lab;
}

osg::Vec3 rgb2lab(osg::Vec3 c) {
   osg::Vec3 lab = xyz2lab( rgb2xyz( c ) );
   return osg::Vec3( lab.x() / 100.0, 0.5 + 0.5 * ( lab.y() / 127.0 ), 0.5 + 0.5 * ( lab.z() / 127.0 ));
}

double standard_dev( std::vector<double> &v ) {
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size() - mean * mean);
    return stdev;
}
