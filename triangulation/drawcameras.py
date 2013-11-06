#!/usr/bin/env python
##################################################
# Copyright (c) INRIA (France) 2011, 2012, 2013
# 
# This file is part of inria-mvs. You can redistribute it and/or
# modify it under the terms of the GNU General Public License.
# 
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
# 
# Author: Jean-Paul CHIEZE <jean-paul.chieze@inria.fr>
# 
##################################################

import sys, os, re, string
from PIL import Image
import myoptions
from myutils import *
import numpy as np

options = [ ('-o',1,True,'output-ply','ply file for cameras positions'),
            ('-i',1,True,'bundler-file','bundler file (bundle/bundle.out or pmvs/bundle.rd.out)'),
            ]
class MyException(Exception):
    """Bad parameter or file.
    The execution cannot continue
    """
def write_ply(file,cams):
    nbc = 0
    for v in cams:
        if(v != None):
            nbc += 1
    f = open(file,'w')
    print >>f, """ply
format ascii 1.0
element face 0
property list uchar int vertex_indices
element vertex %d
property float x
property float y
property float z
property uchar diffuse_red
property uchar diffuse_green
property uchar diffuse_blue
end_header""" %nbc
    for v in cams:
        if v != None:
            (x,y,z) = v
            print >>f, "%f %f %f 0 255 255" %(x,y,z)
    f.close()

def main(argv):
    usage = myoptions.usage
    outfile = None
    infile = None
    opts, args = myoptions.parse_args(options,argv,usage)
    for opt, arg in opts:
        if (opt == '-i'):
            infile = arg
        elif (opt == '-o'):
            outfile = arg
        else:
            usage('Unknown option %s' %opt)
    if (len(args) != 0):
        usage()

    (f,nbpts,focals,cams) = get_bundle_header(infile)
    cams = calc_cam_positions(cams,focals)
    write_ply(outfile,cams)

def mk_doc():
    str1 = """
     Make a ply file of cameras positions.

"""
    str2 = """
      Example::

          $bindir/tool drawcameras -i pmvs/bundle.rd.out -o pmvs/cameras.ply

"""
    return myoptions.prepare_doc(options,str1,str2,name = __name__)

__doc__ = mk_doc()

if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except MyException as inst:
        print inst 
        sys.exit(1)
    sys.exit(0)
