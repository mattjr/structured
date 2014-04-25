#!/usr/bin/env python
from progressbar import *
import sys
import os
import multiprocessing
import tarfile
import subprocess,shlex
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
if len(sys.argv) < 4:
    threads=multiprocessing.cpu_count()
else:
    threads = int(sys.argv[3])

outfile=sys.argv[2]
if os.path.isfile(outfile):
    os.remove(outfile)
mosaic_txt=sys.argv[1]
pathname =  os.path.dirname(os.path.realpath(__file__)) 
if os.path.exists(pathname+'/kd/') == False:
      tarfilepath=pathname+'/kd.tar.bz2'
      tfile = tarfile.open(tarfilepath, 'r:bz2')
      print 'Decompressing %s' % tarfilepath
      if os.access(pathname, os.W_OK) == True: 
        tfile.extractall(pathname)
      else:
          tfile.extractall("/tmp/")
          pathname="/tmp/"
myenv=os.environ.copy()
myenv['LD_LIBRARY_PATH'] = pathname+'/kd/'
pairs=set()
fname_dict=dict([])
with open(mosaic_txt) as f:
    line=f.readline()
    #print line
    totalx,totaly,imgsize,imgsizey,rows,cols =[int(i) for i in line.rstrip().split(' ')[:-1]]
    #print  totalx,totaly,imgsize,imgsizey,rows,cols
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
    for line in f:
        x1,y1,z1,x2,y2,z2,j,i,filename =line.rstrip().split(' ')
        if filename == 'null':
            continue
        i=int(i)
        j=int(j)
        pairs.add((i,j))
        fname_dict[(i,j)]=filename
#filename=os.path.realpath(os.getcwd()+'/mosaic/image_r%04d_c%04d_rs%04d_cs%04d.tif' % (i,j,rows,cols))
for i in range(0,rows):
    for j in range(0,cols):
        if (i,j) not in pairs:
            #print filename
            filename=tmpimg
            invalid=invalid+1
        else:
            valid=valid+1
            filename=fname_dict[(i,j)]
        if os.path.isfile(filename) == False:
            print 'Error file missing "%s"' % filename
        

        cmd='%s/kd/kdu_compress -i %s -o %s  -quiet %s -frag %d,%d,1,1 -num_threads %d' % (pathname,filename,outfile,args,i,j,threads)
        #print cmd
        parsedcmd=shlex.split(cmd)
        child = subprocess.Popen(parsedcmd, env=myenv,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        streamdata = child.communicate()[0]
        rc = child.returncode
        if rc < 0:
            print 'Failed to execute command'
            print streamdata
        pbar.inc()
       # print '%04d/%04d of %d' %(valid,invalid,total)
        
     #   sys.exit(0)
        
pbar.finish()
print '%04d/%04d of %d' %(valid,invalid,total)
