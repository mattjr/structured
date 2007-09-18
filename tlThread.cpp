/////////////////////////////////////////////////////////////////////////////////
//
//  This file is part of the TLIB image processing for computer vision library.
//  Copyright (C) 2003-2006 Sebastien Grange, Terry Fong
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



#include "tlThread.h"

#ifdef LINUX
#ifdef __cplusplus
extern "C" {
#endif
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#ifdef __cplusplus
}
#endif
#endif

#ifdef WIN32
#include <windows.h>
#include <winbase.h>
#endif



///  \file tlThread.h 
///  \brief Thread management wrappers for TLIB.
///
///  OS independant thread management routines for TLIB. 


#ifdef LINUX

///  \warning
///  The following is probably not completely reentrant/thread-safe...  
///  Would be best to guard _tlThreadFunc with a mutex and/or turn 
///  tlThread into a full-fledged class. 
static TLTHREAD_FUNC_PTR _tlThreadFunc;

static void *
_tlThreadFuncWrapper (void *pArg)
{
#ifdef DEBUG
  printf ("-->threadFuncWrapper, calling func at 0x%x\n", _tlThreadFunc);
#endif
  _tlThreadFunc ();
#ifdef DEBUG
  printf ("-->threadFuncWrapper, returning\n");
#endif
  return (NULL);
}

static int              _tlThreadInit                   = 0;
static pthread_cond_t   _tlThreadCond[TLTHREAD_MAX];
static pthread_mutex_t  _tlThreadMutex[TLTHREAD_MAX];
static pthread_t        _tlThreadHandle[TLTHREAD_MAX];

static int
_tlThreadFindHandle (tlThreadHandle handle)
{
  int i;

  for (i = 0; i < TLTHREAD_MAX; i++) {
    if (_tlThreadHandle[i] == handle) {
      return (i);
    }
  }

  return (-1);
}

static void
_tlThreadInitSlot (int i)
{
  pthread_cond_init (&_tlThreadCond[i], NULL);
  pthread_mutex_init (&_tlThreadMutex[i], NULL);
  _tlThreadHandle[i] = 0;
}

#endif



///  Create a thread. On success, the identifier of the newly created 
///  thread is stored in handle and 0 is returned.  
///  On error, a non-zero error code is returned.
///
///  \param func pointer to the function to run in thread
///  \param handle \b [out] thread identifier
///
///  \return
///  0 on success, -1 otherwise.\n
///  See \ref errors "error management" for details.

int
tlThreadCreate (TLTHREAD_FUNC_PTR func, 
                tlThreadHandle    *handle)
{
#if !defined(WIN32) && !defined(LINUX)
  // not implemented for this OS
  return TL_ERROR_NOT_IMPLEMENTED;
#endif

#ifdef WIN32
  DWORD threadID;

  // create thread
  *handle = CreateThread (NULL, 0, (LPTHREAD_START_ROUTINE) func, NULL, NULL, &threadID);
  if (*handle == NULL) {
    TL_ERRNO_MSG ("tlThreadCreate()", TL_ERROR_THREAD);
    return -1;
  }
#endif

#ifdef LINUX
  int i;

  if (!_tlThreadInit) {
    for (i = 0; i < TLTHREAD_MAX; i++) {
      _tlThreadInitSlot (i);
    }
    _tlThreadInit = 1;
  }
  _tlThreadFunc = (TLTHREAD_FUNC_PTR) func;
  if (pthread_create (handle, NULL, _tlThreadFuncWrapper, NULL)) {
    perror ("tlThreadCreate");
    TL_ERRNO_MSG ("tlThreadCreate()", TL_ERROR_THREAD);
    return -1;
  }

  // save the handle
  if ((i = _tlThreadFindHandle (0)) == -1) {
    TL_ERRNO_MSG ("tlThreadCreate()", TL_ERROR_THREAD);
    return -1;
  }
#ifdef DEBUG
  printf ("tlThreadCreate: thread %d : slot %d\n", (int) *handle, i);
#endif
  _tlThreadHandle[i] = *handle;
#endif

  // success
  return 0;
}



///  Exit the thread (from within the thread). The return value of
///  the thread can be consulted from another thread using tlThreadJoin().
///
///  \param returnCode return value of the exiting thread on success, -1 otherwise.
///  See \ref errors "error management" for details.

void
tlThreadExit (int returnCode)
{

#if !defined(WIN32) && !defined(LINUX)
  // not implemented for this OS
  return TL_ERROR_NOT_IMPLEMENTED;
#endif

#ifdef WIN32
  ExitThread (returnCode);
#endif

#ifdef LINUX
  int i;
  pthread_t handle = pthread_self ();

#ifdef DEBUG
  printf ("tlThreadExit: started\n");
#endif

  // find the handle
  if ((i = _tlThreadFindHandle (handle)) == -1) {
    TL_ERRNO_MSG ("tlThreadExit()", TL_ERROR_THREAD);
    return;
  }
  pthread_mutex_lock (&_tlThreadMutex[i]);

  // cond_signal restarts thread waiting on the condition
#ifdef DEBUG
  printf ("tlThreadExit: cond_signal (slot %d)\n", i);
#endif

  if (pthread_cond_signal (&_tlThreadCond[i])) {
    TL_ERRNO_MSG ("tlThreadExit()", TL_ERROR_THREAD);
    return ;
  }
  pthread_mutex_unlock (&_tlThreadMutex[i]);
  pthread_exit ((void *) returnCode);

#endif
}



///  Kill the thread.
///
///  \param handle pointer to the thread identifier
///
///  \return
///  0 on success, -1 otherwise.\n
///  See \ref errors "error management" for details.
///
///  \remark
///  Programmers should call tlThreadJoin() to be sure that the thread
///  has exited cleanly (and that it has had a chance to free any resources
///  that it was using).

int
tlThreadKill (tlThreadHandle *handle)
{

#if !defined(WIN32) && !defined(LINUX)
  // not implemented for this OS
  return TL_ERROR_NOT_IMPLEMENTED;
#endif

#ifdef WIN32
  TerminateThread (*handle, TL_ERROR_THREAD);
#endif

#ifdef LINUX
  // \warning
  // the following relies on the fact that with LinuxThreads, each
  // thread is actually a kernel process with its own PID, so external
  // signals are always directed to one particular thread. 
  // THIS DIFFERS FROM THE POSIX STANDARD... 
  if (pthread_kill (*handle, SIGKILL)) {
    TL_ERRNO_MSG ("tlThreadKill()", TL_ERROR_THREAD);
    return -1;
  }
#endif
  return 0;
}



///  Change thread priority.
///
///  \param handle pointer to the thread identifier
///  \param priority priority to give to the thread
///
///  \return
///  0 on success, -1 otherwise.\n
///  See \ref errors "error management" for details.
///
///  \remark
///  This call has no effect under Linux because we cannot change 
///  thread (process) priorities if we are not the superuser.

int
tlThreadChangePriority (tlThreadHandle *handle, 
                        int            priority)
{

#if !defined(WIN32) && !defined(LINUX)
  // not implemented for this OS
  return TL_ERROR_NOT_IMPLEMENTED;

#endif

#ifdef WIN32
  if (!SetThreadPriority (*handle, priority)) {
    TL_ERRNO_MSG ("tlThreadChangePriority()", TL_ERROR_THREAD);
  }
#endif

#ifdef LINUX
  TL_ERRNO_MSG ("tlThreadChangePriority()", TL_NO_ERROR);
  return -1;
#endif
  return 0;
}



///  Block until the thread exits.
///
///  \param handle pointer to the thread identifier
///  \param returnCode priority to give to the thread
///  \param timeout time limit before the function returns (msec)
///
///  \return
///  On success, the return value of the thread is stored in returnCode 
///  and the function returns 0 on success, -1 otherwise.\n
///  See \ref errors "error management" for details..

int
tlThreadJoin (tlThreadHandle *handle, 
              int            *returnCode, 
              int            timeout)
{

#if !defined(WIN32) && !defined(LINUX)
  // not implemented for this OS
  return TL_ERROR_NOT_IMPLEMENTED;
#endif

#ifdef WIN32
  switch (WaitForSingleObject (*handle, timeout)) {
  case WAIT_ABANDONED:
    return TL_ERROR_THREAD;
  case WAIT_OBJECT_0:
    {
      DWORD exitCode;
      GetExitCodeThread (*handle, &exitCode);
      *returnCode = (int) exitCode;
      return 0;
    }
  case WAIT_TIMEOUT:
    return TL_ERROR_TIMEOUT;
  default:
    return TL_ERROR_THREAD;
  }
#endif

#ifdef LINUX
  void *retval;
  int i;
  struct timespec timeoutSpec;
  struct timeval curTime;
  long int sec, nsec;

  sec = timeout / 1000;
  nsec = (timeout % 1000) * 1000000;
  gettimeofday (&curTime, NULL);
  timeoutSpec.tv_sec = curTime.tv_sec + sec;
  timeoutSpec.tv_nsec = curTime.tv_usec * 1000 + nsec;
  if (timeoutSpec.tv_nsec >= 1000000000) {
    timeoutSpec.tv_sec++;
    timeoutSpec.tv_nsec -= 1000000000;
  }
#ifdef DEBUG
  printf ("tlThreadJoin: started\n");
#endif

  // find the handle
  if ((i = _tlThreadFindHandle (*handle)) == -1) {
    TL_ERRNO_MSG ("tlThreadJoin()", TL_ERROR_THREAD);
    return -1;
  }
#ifdef DEBUG
  printf ("tlThreadJoin: thread %d : slot %d\n", (int) *handle, i);
#endif
  pthread_mutex_lock (&_tlThreadMutex[i]);

  // cond_wait unlocks the mutex and waits for the condition to be signaled
#ifdef DEBUG
  printf ("tlThreadJoin: cond_wait for %d msec\n", timeout);
#endif
  if (pthread_cond_timedwait
    (&_tlThreadCond[i], &_tlThreadMutex[i], &timeoutSpec) == ETIMEDOUT) {
      TL_ERRNO_MSG ("tlThreadJoin()", TL_ERROR_TIMEOUT);
      return -1;
    }
#ifdef DEBUG
    printf ("tlThreadJoin: calling pthread_join\n");
#endif
    if (pthread_join (*handle, (void **) &retval)) {
      perror ("tlThreadJoin");
      return TL_ERROR_THREAD;
    }
    // thread exited cleanly, so free the slot...
    _tlThreadInitSlot (i);
    *returnCode = (int) retval;
    return 0;
#endif
}



///  Suspend execution for a period.
///
///  \param msec sleep period (msec)

void
tlThreadSleep (unsigned int msec)
{
#if !defined(WIN32) && !defined(LINUX)
  // not implemented for this OS
  return TL_ERROR_NOT_IMPLEMENTED;
#endif

#ifdef WIN32
  Sleep (msec);
#endif

#ifdef LINUX
  unsigned long usecs = (unsigned long) (msec * 1e3);
#if 0
  // usleep is limited by the system timer resolution (currently 10ms). 
  // Thus, sleeps near 20 msec are not particularly accurate.
  usleep (usecs);
#else 
  // select seems to be more accurate!
  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = usecs;
  select (0, NULL, NULL, NULL, &timeout);
#endif
#endif
}
