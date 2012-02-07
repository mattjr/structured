#ifndef CHECKTHREADPOOL_H
#define CHECKTHREADPOOL_H
#include "vpb/ThreadPool"
class CheckThreadPool :public vpb::ThreadPool{
public:
    CheckThreadPool(unsigned int numThreads, bool requiresGraphicsContext):haveWorkingContext(_requiresGraphicsContext)
    {
        _numThreads=numThreads;
        _requiresGraphicsContext=requiresGraphicsContext;
        this->init();

    }
    void init(void);
    bool haveWorkingContext;
};

#endif // CHECKTHREADPOOL_H
