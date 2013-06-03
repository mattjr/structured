/* * structured - Tools for the Generation and Visualization of Large-scale
 * Three-dimensional Reconstructions from Image Data. This software includes
 * source code from other projects, which is subject to different licensing,
 * see COPYING for details. If this project is used for research see COPYING
 * for making the appropriate citations.
 * Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
 *
 * This file is part of structured.
 *
 * structured is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * structured is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with structured.  If not, see <http://www.gnu.org/licenses/>.
 */

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
