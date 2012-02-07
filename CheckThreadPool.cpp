#include "CheckThreadPool.h"
void CheckThreadPool::init(void)
{
    _numRunningOperations = 0;
    _done = false;

    _operationQueue = new osg::OperationQueue;
    _blockOp = new vpb::BlockOperation;

    osg::GraphicsContext* sharedContext = 0;

    _maxNumberOfOperationsInQueue = 64;

    for(unsigned int i=0; i<_numThreads; ++i)
    {
        osg::ref_ptr<osg::OperationThread> thread;
        osg::ref_ptr<osg::GraphicsContext> gc;

        if (_requiresGraphicsContext)
        {
            osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
            traits->x = 0;
            traits->y = 0;
            traits->width = 1;
            traits->height = 1;
            traits->windowDecoration = false;
            traits->doubleBuffer = false;
            traits->sharedContext = sharedContext;
            traits->pbuffer = true;


            gc = osg::GraphicsContext::createGraphicsContext(traits.get());

            if (!gc)
            {
                traits->pbuffer = false;
                gc = osg::GraphicsContext::createGraphicsContext(traits.get());
            }

            if (gc.valid())
            {
                gc->realize();

                if (!sharedContext) sharedContext = gc.get();

                gc->createGraphicsThread();

                thread = gc->getGraphicsThread();
            }else{
                printf("CHECK TP override: Failed to create graphics thread\n");

                haveWorkingContext=false;
                thread = new osg::OperationThread;
                thread->setParent(this);
            }

        }
        else
        {

            thread = new osg::OperationThread;
            thread->setParent(this);

        }

        if (thread.valid())
        {
            thread->setOperationQueue(_operationQueue.get());

            _threads.push_back(ThreadContextPair(thread, gc));
        }
    }
}
