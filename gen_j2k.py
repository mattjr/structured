#!/usr/bin/env python
from progressbar import *
import sys
import os
import multiprocessing
from PIL import Image
black = (0,0,0)

class Curr(ProgressBarWidget):
    "Just the percentage done."
    def update(self, pbar):
        return '%03d' % pbar.currval
    
class Max(ProgressBarWidget):
    "Just the percentage done."
    def update(self, pbar):
        return '%03d' % pbar.maxval


widgets = ['J2K Gen', ' ',Percentage(), ' ',Curr(),'/',Max(),' ', Bar(),
                   ' ', ETA()]
if len(sys.argv) < 6:
    threads=multiprocessing.cpu_count()
else:
    threads = int(sys.argv[5])

outfile=sys.argv[1]
rows=int(sys.argv[2])
cols=int(sys.argv[3])
imgsize=int(sys.argv[4])
img = Image.new("RGB", [imgsize,imgsize],black)
tmpimg="/tmp/test1.ppm"
img.save(tmpimg)
totalx=rows*imgsize
print 'Generating a j2k image %dx%d using %d threads' %(totalx,totalx,threads)
args=' Sdims={%d,%d} Stiles={%d,%d}  Corder=RPCL ORGgen_plt=yes ORGtparts=R  Cblk={32,32} -rate 0.25 Clayers=30 Clevels=12 ' %(totalx,totalx,imgsize,imgsize)
total=rows*cols
pbar = ProgressBar(widgets=widgets, maxval=total)
pbar.start()
valid=0
invalid=0
for i in range(0,rows):
    for j in range(0,cols):
        filename=os.path.realpath(os.getcwd()+'/mosaic/image_r%04d_c%04d_rs%04d_cs%04d.tif' % (i,j,rows,cols))
        if os.path.exists(filename) == False:
            #print filename
            filename=tmpimg
            invalid=invalid+1
        else:
            valid=valid+1
            img = Image.open(filename)
        cmd='kdu_compress -i %s -o %s  -quiet %s -frag %d,%d,1,1 -num_threads %d' % (filename,outfile,args,i,j,threads)
        #print cmd
        os.system(cmd)
        pbar.inc()
        
     #   sys.exit(0)
        
pbar.finish()
print '%04d/%04d of %d' %(valid,invalid,total)
