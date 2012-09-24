#ifndef CHECKTHREADPOOL_H
#define CHECKTHREADPOOL_H
#include "vpb/ThreadPool"
#include <vips/vips.h>
class CheckThreadPool :public vpb::ThreadPool{
public:
    CheckThreadPool(unsigned int numThreads, bool requiresGraphicsContext):haveWorkingContext(_requiresGraphicsContext)
    {
        _numThreads=numThreads;
        _requiresGraphicsContext=requiresGraphicsContext;
        this->init();
#if VIPS_MINOR_VERSION > 24
    vips_init(NULL);
#endif

    }
    void init(void);
    bool haveWorkingContext;
};

#endif // CHECKTHREADPOOL_H
