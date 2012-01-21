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


