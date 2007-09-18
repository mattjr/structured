/////////////////////////////////////////////////////////////////////////////////
//
//  This file is part of the TLIB image processing for computer vision library.
//  Copyright (C) 2003-2006 Sebastien Grange
//  EPFL - Swiss Federal Institute of Technology
//  All rights reserved.
// 
//  This library is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License("GPL") version 2
//  as published by the Free Software Foundation.
// 
//  \author     Sebastien Grange
//  \version    1.0
//  \date       01/2004
//
//  <http://imtsg14.epfl.ch/tlib>
//
/////////////////////////////////////////////////////////////////////////////////


#ifndef __TLTHREAD_H__
#define __TLTHREAD_H__

#define LINUX

#ifdef WIN32
#include <windows.h>
#define tlThreadHandle HANDLE
#endif

#ifdef LINUX
#include <pthread.h>
#include <stdio.h>
#define  TL_ERROR_THREAD -1
#define tlThreadHandle pthread_t
#define TLTHREAD_MAX	100	/* max # of threads */
#endif
#define    TL_ERRNO_MSG(x,y) printf(x)
typedef void (*TLTHREAD_FUNC_PTR)(void);
// error codes
enum tl_errors { 
  TL_NO_ERROR, 
  TL_ERROR, 
  TL_ERROR_INVALID_FORMAT,
  TL_ERROR_GEOMETRY, 
  TL_ERROR_NULL_POINTER,
  TL_ERROR_INDEX_OUT_OF_BOUNDS,
  TL_ERROR_MAX_REACHED,
  TL_ERROR_ALLOCATION_FAILED,
  TL_ERROR_NOT_IMPLEMENTED, 
  TL_ERROR_FILE_IO, 
  TL_ERROR_FILE_NOT_FOUND_OR_CORRUPT, 
  TL_ERROR_NOT_REQUIRED,
  TL_ERROR_NOT_FOUND, 
  TL_ERROR_NOT_AVAILABLE, 
  TL_ERROR_NOT_IN_IMAGE,
  TL_ERROR_STACK_OVERFLOW, 
  TL_ERROR_TIMEOUT
};

int  tlThreadCreate         (TLTHREAD_FUNC_PTR func, tlThreadHandle *handle);
void tlThreadExit           (int returnCode);
int  tlThreadKill           (tlThreadHandle *handle);
int  tlThreadChangePriority (tlThreadHandle *handle, int priority);
int  tlThreadJoin           (tlThreadHandle *handle, int *returnCode, int timeout);

// sleep (suspend current thread for a period)
void tlThreadSleep         (unsigned int msec);

#endif
