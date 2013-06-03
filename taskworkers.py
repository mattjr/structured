#!/usr/bin/env python

# structured - Tools for the Generation and Visualization of Large-scale
# Three-dimensional Reconstructions from Image Data. This software includes
# source code from other projects, which is subject to different licensing,
# see COPYING for details. If this project is used for research see COPYING
# for making the appropriate citations.
# Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
#
# This file is part of structured.
#
# structured is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# structured is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with structured.  If not, see <http://www.gnu.org/licenses/>.


import multiprocessing # for the workers
import subprocess # for calling the shell
from multiprocessing.managers import SyncManager # for the syncing

import os.path

import Queue

import logging


def get_remote_script():
    """Get the full path of the script to execute."""
    return os.path.join(os.path.dirname(os.path.realpath(__file__)), 'taskworkers.py')

# the functions to create the worker processes
# gateway is the computer to run srun on
# gateway/remote is the computer to ssh to
# node are the nodes in the cluster, either list or comma sep. string
def create_auto(params):
    """Create a set of local workers with as many workers as cores."""
    n = multiprocessing.cpu_count()

    return client_start(params[2], n)

def create_local(params, n):
    """Create a set of n local workers."""
    return client_start(params[2], n)

def create_remote(params, n, remote, username=None):
    """Create a set of n remote workers."""
    return [spawn_thread(spawn_remote, (remote, params[0], params[1], n, username))]

def create_slurm(params, n, gateway=None, username=None, nodes=None):
    """Create a set of n slurm workers (detect if in cluster)."""
    # detect the hostname and check if in valid list...
    # not really a good choice to use

    # then call the correct function with the args

    if local:
        create_slurm_local(params, n, nodes)
    else:
        create_slurm_remote(params, n, gateway, username, nodes)

def create_slurm_local(params, n, nodes=None):
    """Create a set of n slurm workers (we are in the cluster)."""

    kwargs = {}
    kwargs['port'] = params[0]
    kwargs['authkey'] = params[1]
    kwargs['n'] = 1 # the final 1 is the number of threads

    # srun args
    if not nodes:
        pass
    elif isinstance(nodes, basestring): # isinstance(nodes, str) in py3k
        kwargs['srun_args'] = "-q -w {0}".format(nodes)
    elif isinstance(nodes, list):
        # create comma separated list through magic
        kwargs['srun_args'] = "-q -w {0}".format(",".join(map(str, nodes)))

    # at this question there is a choice - create all threads in one
    # instance, or blocks of threads in many instances or single threads
    # in n instances

    # for now, n instances with single threads
    procs = []
    for i in xrange(n):
        p = spawn_thread(spawn_slurm_local, kwargs=kwargs)
        procs.append(p)

    return procs

def create_slurm_remote(params, n, gateway, username=None, nodes=None):
    """Create a set of n slurm workers (we are not in the cluster)."""

    kwargs = {}
    kwargs['port'] = params[0]
    kwargs['authkey'] = params[1]
    kwargs['n'] = 1 # the final 1 is the number of threads
    kwargs['gateway'] = gateway
    kwargs['remote_user'] = username

    # srun args
    if not nodes:
        pass
    elif isinstance(nodes, basestring): # isinstance(nodes, str) in py3k
        kwargs['srun_args'] = "-q -w {0}".format(nodes)
    elif isinstance(nodes, list):
        # create comma separated list through magic
        kwargs['srun_args'] = "-q -w {0}".format(",".join(map(str, nodes)))

    # at this question there is a choice - create all threads in one
    # instance, or blocks of threads in many instances or single threads
    # in n instances

    # for now, n instances with single threads
    procs = []
    for i in xrange(n):
        p = spawn_thread(spawn_slurm_remote, kwargs=kwargs)
        procs.append(p)

    return procs

# the multiprocessing functions
def execute_command(command_line):
    # execute the command
    pipe = subprocess.Popen(command_line, stdout=subprocess.PIPE, shell=True, universal_newlines=True, stderr=subprocess.STDOUT)

    output = ''.join(pipe.stdout.readlines())
    status = pipe.wait()

    # not 100% sure this is needed
    if status != 0:
        # write this out!
        # somewhere
        # probably print back to base... and write out
        message = (status, command_line, output)
    else:
        message = (status, command_line, "output")

    return message


def initialise_server(port, authkey):
    
    class JobQueueManager(SyncManager):
        pass

    # create queues for commands and results
    job_queue = multiprocessing.JoinableQueue()
    job_results = multiprocessing.JoinableQueue()

    # register with manager
    JobQueueManager.register('job_queue', callable=lambda: job_queue)
    JobQueueManager.register('job_results', callable=lambda: job_results)

    # create our instance, listen on all IPs
    manager = JobQueueManager(address=('', port), authkey=authkey)
    manager.start()

    return manager

def initialise_client(ip, port, authkey):
    
    class ServerQueueManager(SyncManager):
        pass

    ServerQueueManager.register('job_queue')
    ServerQueueManager.register('job_results')

    manager = ServerQueueManager(address=(ip, port), authkey=authkey)
    manager.connect()

    return manager

# the individual threads in each client
def client_thread(manager):
    """Run a single thread doing the processing."""

    # the intent here is to stay in the thread whilst hold is True
    # or there are items in the queue
    job_queue = manager.job_queue()
    job_results = manager.job_results()
    #hold = manager.hold()

    # loop until None command
    while True:
	try:
            command = job_queue.get()
        except IOError as e:
            job_results.put((-2, "job_queue.get()", "IOError: {0}".format(e)))
            break

        # poison pill command
        if command is None:
            # note that we have done the task
            # before quitting out
            job_queue.task_done()
            break

        # now do the command
	try:
            result = execute_command(command)
        except Exception as e:
            # capture the exception text and return it
            job_results.put((-1, command, "Exception: {0}".format(e)), True, 1)
            logging.debug("Exception from execute_command.")
        else:
            # worked, no exception capture result and return as well
            job_results.put(result, True, 1)
        finally:
            # always make sure the task is flagged as done
            job_queue.task_done()

        # there may be an exception if we can't push to the queue
        # as it is full... I think unlikely. Give it 5 seconds however

        # mark the task complete
        logging.debug("task complete")

    logging.debug("exiting Client thread, no pending jobs.")

def client_start(manager, threads):
    procs = []
    for i in range(threads):
        p = multiprocessing.Process(target=client_thread,
            args=(manager,))
        procs.append(p)
        p.start()

    return procs

def spawn_thread(command, args=(), kwargs={}):
    proc = multiprocessing.Process(target=command, args=args, kwargs=kwargs)
    proc.start()

    return proc
    

def spawn_remote(remote_host, port, authkey, n, remote_user=None):
    """This is blocking, should be called from a thread/process."""

    elements = {}
    elements['remote_host'] = remote_host
    elements['remote_user'] = remote_user
    elements['args'] = "REMOTE {0} {1} {2}".format(port, authkey, n)
    elements['script'] = get_remote_script()

    if remote_user:
        cmd = "/usr/bin/ssh {remote_user}@{remote_host} /usr/bin/python {script} {args}".format(**elements)
    else:
        cmd = "/usr/bin/ssh {remote_host} /usr/bin/python {script} {args}".format(**elements)

    logging.debug("spawn_remote: Calling '{0}'".format(cmd))
    subprocess.call(cmd, shell=True)
    logging.debug("spawn_remote: finished")
    #sys.exit(0)

def spawn_slurm_local(port, authkey, n=1, srun_args=None):
    """This is blocking, should be called from a thread/process."""
    elements = {}
    elements['srun_args'] = srun_args or ""
    elements['script_args'] = "SLURM_LOCAL {0} {1} {2}".format(port, authkey, n)
    elements['script'] = get_remote_script()

    cmd = "/usr/bin/srun {srun_args} python {script} {script_args}".format(**elements)
    logging.debug("spawn_slurm_local: Calling '{0}'".format(cmd))
    subprocess.call(cmd, shell=True)
    logging.debug("spawn_slurm_local: finished")
    #sys.exit(0)

def spawn_slurm_remote(port, authkey, gateway, n=1, remote_user=None, srun_args=None):
    """This is blocking, should be called from a thread/process."""
    elements = {}
    elements['srun_args'] = srun_args or ""
    elements['script_args'] = "SLURM_REMOTE {0} {1} {2}".format(port, authkey, n)
    elements['remote_user'] = remote_user
    elements['script'] = get_remote_script()
    elements['remote_host'] = gateway

    if remote_user:
        cmd = "ssh {remote_user}@{remote_host} srun {srun_args} python {script} {script_args}".format(**elements)
    else:
        cmd = "ssh {remote_host} srun {srun_args} python {script} {script_args}".format(**elements)

    logging.debug("spawn_slurm_remote: Calling '{0}'".format(cmd))
    subprocess.call(cmd, shell=True)
    logging.debug("spawn_slurm_remote: finished")
    #sys.exit(0)


if __name__ == '__main__':
    # this is where the remote/slurm clients enter
    import sys # for argv
    import os # for environ

    if len(sys.argv) != 5:
        raise Exception("Four arguments required CLIENT_TYPE PORT AUTHKEY N_WORKERS. {0}".format(sys.argv))

    client_type = sys.argv[1]
    port = int(sys.argv[2])
    authkey = sys.argv[3]
    n = int(sys.argv[4])

    # create and connect the manager to the remote instance
    if client_type == "REMOTE":
        # get the environment variable for SSH_CLIENT
        server = os.environ['SSH_CLIENT'].split()[0]
        client_manager = initialise_client(server, port, authkey)

    elif client_type == "SLURM_LOCAL":
        # get the environment variavle SLURM_LAUNCH_NODE_IPADDR
        server = os.environ['SLURM_LAUNCH_NODE_IPADDR']
        client_manager = initialise_client(server, port, authkey)

    elif client_type == "SLURM_REMOTE":
        # go back to the SSH_CLIENT and trace it back
        server = os.environ['SSH_CLIENT'].split()[0]
        client_manager = initialise_client(server, port, authkey)
    else:
        raise Exception("Unknown type.")

    # start the workers
    procs = client_start(client_manager, n)

    # and wait for them to finish
    for p in procs:
        p.join()
        logging.debug("{0}: Joined a worker thread.".format(client_type))
    logging.debug("{0}: Exiting from worker process.".format(client_type))