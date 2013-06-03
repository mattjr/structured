//
// structured - Tools for the Generation and Visualization of Large-scale
// Three-dimensional Reconstructions from Image Data. This software includes
// source code from other projects, which is subject to different licensing,
// see COPYING for details. If this project is used for research see COPYING
// for making the appropriate citations.
// Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
//
// This file is part of structured.
//
// structured is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// structured is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with structured.  If not, see <http://www.gnu.org/licenses/>.
//

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
