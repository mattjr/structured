#!/usr/bin/env python
# -*- coding: utf-8 -*-
from threadpool import *
from progressbar import *
from time import sleep
from random import random
import sys
if len(sys.argv) > 2:
    pool = Pool(int(sys.argv[2]))
else:
    pool = Pool(3)
    
@threadpool(pool)
def runcmd_threadpool(cm,i,total):
    print 'Processing %d/%d' % (i +1 , total)
    cmd= 'csh -c \'%s\'' % cm[:-1]
    commands.getstatusoutput(cmd)
    #sleep(random())
    #pbar.update(i)    
    

print 'threadpool shell command run'
fname = sys.argv[1];
try:
    cmdfile = open(fname, 'r')
except IOError:
    print 'Cannot open file %s for reading. Check file exists and try again.' % fname
    sys.exit(0)
total=0
i=0
total=sum(1 for line in cmdfile)
cmdfile.close()
cmdfile = open(fname, 'r')
#pbar = ProgressBar().start()
for line in cmdfile:
    runcmd_threadpool(line,i,total)
    i=i+1;
pool.join()
#pbar.finish()
print 'done'
print ''

#@threaded
#def test_threaded(i):
#    print 'threaded %i enter' % i
 #   sleep(random())
 #   print 'threaded %i exit' % i

#print 'threaded example'
#for i in range(5):
#    test_threaded(i)
#print 'done'
