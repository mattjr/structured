#!/usr/bin/env python
"""Run tasks in a thread pool.

  Invoke as runtasks.py task_list config_file [ job title ]
"""

import sys # for argv

import random # for port/key generation

import multiprocessing # for the cpu_count functionality

# for the nice display
from progressbar import ProgressBar, ProgressBarWidget, Percentage, Bar, ETA

import taskworkers # the related library with many useful functions

import string # for the keygen
import os #for checking env

import logging
import time
# Widgets for the progress bar
class Curr(ProgressBarWidget):
    "Just the percentage done."
    def update(self, pbar):
        return '%03d' % pbar.currval
    
class Max(ProgressBarWidget):
    "Just the percentage done."
    def update(self, pbar):
        return '%03d' % pbar.maxval

def key_generator(size=12, chars=string.letters + string.digits):
    return ''.join(random.choice(chars) for x in range(size))

# parse the arguments first

# check we have sufficient arguments
if len(sys.argv) < 3:
    raise Exception("Must give task_file and config_file")

# now open the task and config file
try:
    task_file = open(sys.argv[1], 'r')
except IOError:
    print "Could not open task file '{0}' for reading.".format(sys.argv[1])
    raise

try:
    cfg_file = open(sys.argv[2], 'r')
except IOError:
    print "Could not open config file '{0}' for reading.".format(sys.argv[2])
    raise

# if a third is given it is the title, else default
if len(sys.argv) > 3:
    title = sys.argv[3]
else:
    title = "threadpool"

# create the central manager
port = random.randint(50000, 65000)
authkey = key_generator()
manager = taskworkers.initialise_server(port, authkey)
job_queue = manager.job_queue()
job_results = manager.job_results()
job_count = 0
job_finished = 0
#hold = manager.hold()
params = (port, authkey, manager)
numworkers = 0

threads = []

# load all the commands in to a list and get the count
commands = task_file.readlines()
total = len(commands)

# load the commands and now run them on the pool
for command in commands:
    job_queue.put(command)
    job_count += 1

# parse the configuration file
for line in cfg_file:
    # this creates all the workers
    elements = line.split()

    if len(elements) == 0:
        continue  # blank line, get next

    client_type = elements[0]

    if client_type == "AUTO":
        threads += taskworkers.create_auto(params)
        numworkers += multiprocessing.cpu_count()

    elif client_type == "LOCAL":
        if len(elements) < 2:
            raise ValueError("Not enough parameters for LOCAL, need n workers.")
        threads += taskworkers.create_local(params, int(elements[1]))
        numworkers += int(elements[1])

    elif client_type == "REMOTE":
        if len(elements) < 2:
            raise ValueError("Not enough parameters for REMOTE, need n workers and remote name.")
        threads += taskworkers.create_remote(params, int(elements[1]), remote=elements[2])
        numworkers += int(elements[1])

    elif client_type == "SLURM":
        if len(elements) < 2:
            raise ValueError("Not enough parameters for SLURM, need n workers.")
        threads += taskworkers.create_slurm(params, int(elements[1]), gateway='archipelago', nodes=elements[2:])
        numworkers += int(elements[1])

    elif client_type == "SLURM_LOCAL":
        if len(elements) < 2:
            raise ValueError("Not enough parameters for SLURM_LOCAL, need n workers and optional node names.")
        threads += taskworkers.create_slurm_local(params, int(elements[1]), nodes=elements[2:])
        numworkers += int(elements[1])

    elif client_type == "SLURM_REMOTE":
        if len(elements) < 2:
            raise ValueError("Not enough parameters for SLURM_REMOTE, need n workers and optional node names.")
	argnum = min(int(elements[1]),total)
        threads += taskworkers.create_slurm_remote(params, argnum, gateway='archipelago', nodes=elements[2:])
        numworkers += argnum


# put termination/empty/none commands on queue, one for each worker
worker_terminations = numworkers
while worker_terminations:
    job_queue.put(None)
    worker_terminations -= 1

print "Spawned {0} workers.".format(numworkers)


# create the progress bar
if os.environ.get('FILEOUTPUT_STATUS') == '1':
    print '%s: %d tasks' % (title ,total)
    widgets = [Percentage()]
    progress_bar = ProgressBar(widgets=widgets, maxval=total,term_width=5,fd=sys.stdout,using_file=True)
else:
    widgets = [title, ' ', Percentage(), ' ', Curr(), '/', Max(), ' ', Bar(), ' ', ETA()]
    progress_bar = ProgressBar(widgets=widgets, maxval=total,fd=sys.stderr)


# release the hold that kept the clients active
#hold = False

# display the progress bar
progress_bar.start()
#hack to keep jobs from finishing too fast
time.sleep(10)
ranOK=0
# wait for jobs...
while not job_finished == job_count:
    # when there are results pending
    result = job_results.get()

    if result[0] != 0:
        # this was an error
        sys.stderr.write("runtasks.py: problem running: {0}\noutput: {1}".format(result[1], result[2]))
    else:
        ranOK += 1
    # increment the count
    progress_bar.inc()
    job_finished += 1
    job_results.task_done()
#hack for now because otherwise dones exit
if ranOK == 0:
    sys.stderr.write('runtasks.py: No tasks ran sucessfully! Bailing.\n')
    sys.exit(-1)

logging.debug("jobs all finished")
#job_results.close()
job_queue.close()

# make sure they are all done...
logging.debug("job queue joining")
job_queue.join()

logging.debug("prog bar finishing")
progress_bar.finish()

logging.debug("manager shutting down")
manager.shutdown()

#print "waiting for controller threads"
#for t in threads:
#    t.join()
#    print "joined controller thread"

logging.debug("writing timing")
timing = open('timing.txt', 'a')
timing.write("{0} {1}\n".format(title, progress_bar.seconds_elapsed))
timing.close()

