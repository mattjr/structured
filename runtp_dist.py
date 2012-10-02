#!/usr/bin/env python
# -*- coding: utf-8 -*-
from threadpool_dist import *
from progressbar import *
from time import sleep
from random import random
import sys
import os
import thread
import commands
import string
import random
import re
import getpass
def detectCPUs():
    """
    Detects the number of CPUs on a system. Cribbed from pp.
    """
    # Linux, Unix and MacOS:
    if hasattr(os, "sysconf"):
        if os.sysconf_names.has_key("SC_NPROCESSORS_ONLN"):
            # Linux & Unix:
            ncpus = os.sysconf("SC_NPROCESSORS_ONLN")
        if isinstance(ncpus, int) and ncpus > 0:
            return ncpus
        else: # OSX:
            return int(os.popen2("sysctl -n hw.ncpu")[1].read())
    # Windows:
    if os.environ.has_key("NUMBER_OF_PROCESSORS"):
       ncpus = int(os.environ["NUMBER_OF_PROCESSORS"]);
    if ncpus > 0:
        return ncpus
    return 1 # Default

main = ThreadPool(0)

if len(sys.argv) > 2:
    try:
        cfgfile = open(sys.argv[2], 'r')
    except IOError:
        print 'Cannot open file %s for reading. Check file exists and try again.' % sys.argv[2]
        sys.exit(0)
    for line in cfgfile:
        words = string.split(line, ' ')
        if len(words) >= 2:
            if(words[0] == 'LOCAL'):
                main.createWorkersLocal(int(words[1]))
                print 'ThreadPool spawning %d local threads' % len(main.workers)
            elif(words[0] == 'REMOTE'):
                main.createWorkersRemote(int(words[1]),words[2])
                print 'ThreadPool spawning %d remote threads on %s' % (int(words[1]),words[2])
            elif(words[0] == 'SLURM'):
                if len(words) == 4:
                    main.createWorkersSLURM(int(words[1]),words[2],words[3],title=title)
                    print 'ThreadPool spawning %d slurm threads using host %s' % (int(words[1]),words[3])
                else:
                    main.createWorkersSLURM(int(words[1]),words[2],title=title)
                    print 'ThreadPool spawning %d slurm threads all hosts' % (int(words[1]))


else:
    print 'Must pass cfg file'
    sys.exit(0)
print 'ThreadPool spawning %d total threads' % len(main.workers)



def replace_all(text, dic):
    for i, j in dic.iteritems():
        text = text.replace(i, j)
    return text


def runcmd_threadpool(cm,runwith):
    shell = 'bash'
    #runwith ='ssh auv@archipelago srun /bin/bash '
    if len(runwith) > 0:
        match = re.search('([\w.-]+)@([\w.-]+)', runwith)
        if match:
            username = match.group(1)
            curruser = getpass.getuser()
            if curruser != username:
                reps = {curruser:username}
                cm = replace_all(cm, reps)
        cmd= '%s "%s -c \'%s\' "' % (runwith[:-1],shell,cm[:-1])
    else:
        cmd= '%s -c \'%s\' ' % (shell,cm[:-1])

    s,o = commands.getstatusoutput(cmd)
    if s != 0:
        print 'problem running: ', cmd
        print 'output : ',o
        errfn='./%s/%d-cmd-%03d.txt'% ('.',os.getpid(),random.random())
        print 'wrote log to %s ' % errfn
        f = open(errfn, 'w')
        f.write( 'problem running: %s' % cmd)
        f.write('output : %s' % o)
        f.close()
    return 0
   
# this will be called each time a result is available
def print_result(request, result):
    pbar.inc()
# this will be called when an exception occurs within a thread
# this example exception handler does little more than the default handler
def handle_exception(request, exc_info):
        if not isinstance(exc_info, tuple):
            # Something is seriously wrong...
            print request
            print exc_info
            raise SystemExit
        print "**** Exception occured in request #%s: %s" % \
          (request.requestID, exc_info)
    
class Curr(ProgressBarWidget):
    "Just the percentage done."
    def update(self, pbar):
        return '%03d' % pbar.currval
    
class Max(ProgressBarWidget):
    "Just the percentage done."
    def update(self, pbar):
        return '%03d' % pbar.maxval


fname = sys.argv[1];
cmd_counter = 0


if len(sys.argv) > 3:
    title = sys.argv[3]
else:
    title = 'Cmds: '


widgets = [title, ' ',Percentage(), ' ',Curr(),'/',Max(),' ', Bar(),
                   ' ', ETA()]

dir,end = os.path.split(fname)
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
pbar = ProgressBar(widgets=widgets, maxval=total)
pbar.start()
data = []
for line in cmdfile:
    data.append(line)
#    runcmd_threadpool(line,i,total,dir)
    i=i+1;
requests = makeRequests(runcmd_threadpool, data, print_result, handle_exception)
for req in requests:
    main.putRequest(req)
   # print "Work request #%s added." % req.requestID
pbar.start()
main.wait()

pbar.finish()

print ''

f = open('timing.txt', 'a')
f.write('%s %f\n' % (title,pbar.seconds_elapsed))
f.close()
#@threaded
#def test_threaded(i):
#    print 'threaded %i enter' % i
 #   sleep(random())
 #   print 'threaded %i exit' % i

#print 'threaded example'
#for i in range(5):
#    test_threaded(i)
#print 'done'
