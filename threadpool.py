#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:ts=4:sw=4:et
# (c) 2007 under terms of LGPL v 2.1
# by Vsevolod S. Balashov <vsevolod@balashov.name> 
import commands
from threading import Thread

def threaded(func):
    def proxy(*args, **kwargs):
        thread = Thread(target=func, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return proxy

from Queue import Queue

class Pool(Queue):
    def __init__(self, maxsize):
        assert maxsize > 0, 'maxsize > 0 required for Pool class'
        Queue.__init__(self, maxsize)
        for i in range(maxsize):
            thread = Thread(target = self._worker)
            thread.setDaemon(True)
            thread.start()

    def _worker(self):
        while True:
            try:
                func, args, kwargs = self.get()
                func(*args, **kwargs)
            except:
                self.task_done()
                self.join()
                raise
            self.task_done()

    def addJob(self, func, *args, **kwargs):
        self.put((func, args, kwargs))

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_value, traceback):
        self.join()

def threadpool(pool):
    assert pool.__class__ == Pool, 'threadpool decorator require a Pool object'
    def decorator(func):
        def proxy(*args, **kwargs):
            pool.put((func, args, kwargs))
            return pool
        return proxy
    return decorator
