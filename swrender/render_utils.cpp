#include "render_utils.h"
using namespace std;
#define KILOBYTE_FACTOR 1024.0
#define MEGABYTE_FACTOR ( 1024.0 * 1024.0 )
#define GIGABYTE_FACTOR ( 1024.0 * 1024.0 * 1024.0 )
std::string get_size_string(double kb){
    int buflen=8192;
    char buf[buflen];
    double size = kb*1024;
    double displayed_size;
    if( size < MEGABYTE_FACTOR )
    {
        displayed_size =  size / KILOBYTE_FACTOR;
        snprintf( buf, buflen,  "%'.1f KB" , displayed_size );
    }
    else if( size < GIGABYTE_FACTOR )
    {
        displayed_size = size / MEGABYTE_FACTOR;
        snprintf( buf, buflen,  "%'.1f MB" , displayed_size );
    }
    else
    {
        displayed_size =  size / GIGABYTE_FACTOR;
        snprintf( buf, buflen,  "%'.1f GB" , displayed_size );
    }
    return buf;
}

//////////////////////////////////////////////////////////////////////////////
//
// process_mem_usage(double &, double &) - takes two doubles by reference,
// attempts to read the system-dependent data for a process' virtual memory
// size and resident set size, and return the results in KB.
//
// On failure, returns 0.0, 0.0

void process_mem_usage(double& vm_usage, double& resident_set)
{
    using std::ios_base;
    using std::ifstream;
    using std::string;

    vm_usage     = 0.0;
    resident_set = 0.0;

    // 'file' stat seems to give the most reliable results
    //
    ifstream stat_stream("/proc/self/stat",ios_base::in);

    // dummy vars for leading entries in stat that we don't care about
    //
    string pid, comm, state, ppid, pgrp, session, tty_nr;
    string tpgid, flags, minflt, cminflt, majflt, cmajflt;
    string utime, stime, cutime, cstime, priority, nice;
    string O, itrealvalue, starttime;

    // the two fields we want
    //
    unsigned long vsize;
    long rss;

    stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
            >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
            >> utime >> stime >> cutime >> cstime >> priority >> nice
            >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

    stat_stream.close();

    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
    vm_usage     = vsize / 1024.0;
    resident_set = rss * page_size_kb;
}

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

int log2i(uint32_t value) {
    int r = 0;
    while ((value >> r) != 0)
        r++;
    return r-1;
}

int log2i(uint64_t value) {
    int r = 0;
    while ((value >> r) != 0)
        r++;
    return r-1;
}

