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
from scipy import weave
import numpy as np

class MyException(Exception):
    """Bad parameter or file.
    The execution cannot continue
    """

def draw_point(buf,ix,iy,size,color,cross = False):
    """
    draw a colored square around point (ix,iy)
    """
    code = """
 struct pts {
  int y;
  int nbx;
  int x[5];
};
struct pts rpoints[] = {
  {-1,3,-1,0,1},
  {0,2,-1,1,0},
  {1,3,-1,0,1},
};
struct pts cpoints[] = {
  {-2,1,0},
  {-1,1,0},
  {0,5,-2,-1,0,1,2},
  {1,1,0},
  {2,1,0},
};
#define NBLR sizeof(rpoints) / sizeof(struct pts)
#define NBLC sizeof(cpoints) / sizeof(struct pts)
#include <stdio.h>
int nbl;
struct pts *points;
if(cross == 0) {
  nbl = NBLR;
  points = rpoints;
} else {
  nbl = NBLC;
  points = cpoints;
}
int i, j, k, ky, kx, k0;
int rdimx = 3 * dimx;
for(j=0;j<nbl;j++) {
      ky = points[j].y + iy;
      if(ky < 0 || ky >= dimy) continue;
      k0 = ky * rdimx;
      for(i=0;i<points[j].nbx;i++) {
	kx = points[j].x[i] + ix;
//printf("i,j = %d,%d, kx= %d, k0 = %d\\n",i,j,kx,k0);
	if(kx < 0 || kx >= dimx) continue;
	k = k0 + 3 * kx;
	*(buf+k++) = color[0];
	*(buf+k++) = color[1];
	*(buf+k) = color[2];
    }
}
"""
    (dimx,dimy) = (size[0],size[1])
    weave.inline(code,['buf' ,'ix','iy','dimx','dimy','color','cross'])

def draw_color_rect(buf,ix,iy,size,wrect,color):
    """
    draw a square centerd on x,y filled with color
    """
    code = """
int nd = %d;
int x, y, i, j;
int ny = 1 + 2 * nd;
int nx = ny;
y = iy - nd;
if (y < 0) {
  ny += y;
  y = 0;
} else 
   if ((y + ny) > dimy)  ny -= y + ny - dimy;
x = ix - nd;
if (x < 0) {
  nx += x;
  x = 0;
} else 
   if ((x + nx) > dimx)  nx -= x + nx - dimx;
int k = y * dimx * 3 + 3 * x;
int deltak = 3 * (dimx - nx);
for (i = 0;i < ny;i++) {
  for (j = 0;j < nx;j++) {
#if 1
     *(buf+k++) = color[0];
     *(buf+k++) = color[1];
     *(buf+k++) = color[2];
#else
    *(buf+k) = (*(buf+k) / 2) + (color[0] / 2);
    k++;
    *(buf+k) = (*(buf+k) / 2) + (color[1] / 2);
    k++;
    *(buf+k) = (*(buf+k) / 2) + (color[2] / 2);
    k++;
#endif
  }
  k += deltak;
}

""" %wrect
    (dimx,dimy) = (size[0],size[1])
    #ll lqprint "XX %d %d" %(ix,iy)
    if(ix < 0 or iy < 0 or ix >= dimx or iy >= dimy): return()
    weave.inline(code,['buf' ,'ix','iy','dimx','dimy','color'])

def c_projection(matr,pts3d,pts2d):
    """
    fills pts2d (2xn array) with projection of pts3d (3xn array)
    """
    code = """
int jj = 0;
int i;
float X[3];
float x;
int ki, kj;
for( i =0;i < npts;i++) {
   double *mat = matr;
   for(ki = 0;ki < 3; ki++) {
     x = 0;
     for(kj = 0;kj < 3;kj++) 
         x += *(mat+kj) * *(pts3d + jj + kj);
     x += *(mat + 3);
     X[ki] = x;
     mat += 4;
  }
  x = X[2];
  *(pts2d + jj) = X[0] / x;
  *(pts2d + jj + 1) = X[1] / x;
  *(pts2d + jj + 2) = x;
   jj += 3;
}

"""
    npts = pts3d.shape[1]
    weave.inline(code,['matr','pts3d','pts2d','npts'])

def c_find_common_points(pts1,pts2,npts,nx,ny):

    code_mark = """
    int j, i;
    int npt = 0, ndbl = 0;
    float *zpts = (float *)malloc(nx * ny *sizeof(float));
    for (i = 0;i < npts;i++) {
      int x = (int) (pts[3 * i] + 0.5);
      int y = (int) (pts[3 * i + 1] + 0.5);
      float z = pts[3 * i + 2];
      if(x < 0 || y < 0 || x >= nx || y >= ny)
          continue;
      j = x * ny + y;
      npt++;
      if (buf[j] == 0) {
          buf[j] = i + 1;
          zpts[j] = z;
#if 1
      } else if(zpts[j] > z) {
          buf[j] = i + 1;
          zpts[j] = z;
#else
    } else {
         buf[j] = -1;
#endif
          ndbl++;
     }
  }
  printf("XX (%d) %d pts, %d dbl\\n",npts,npt,ndbl);
"""
    code_cmn = """
    int i, nb;
    int k = 0;
    for (i = 0;i < nx * ny;i++) {
      if(buf1[i] > 0 && buf2 [i] > 0) {
         cmnpts[k++] = buf1[i] - 1;
         cmnpts[k++] = buf2[i] - 1;
         nb++;
    }
  }
    return_val = nb;
"""
    buf1 = np.zeros((ny*nx),dtype = np.int)
    buf2 = np.zeros((ny*nx),dtype = np.int)
    pts = pts1
    buf = buf1
    weave.inline(code_mark,['buf','pts','npts','nx','ny'])
    pts = pts2
    buf = buf2
    weave.inline(code_mark,['buf','pts','npts','nx','ny'])
    cmnpts = np.empty((2,npts),dtype=np.int,order="F")
    nb = weave.inline(code_cmn,['buf1','buf2','cmnpts','nx','ny'])
    print "XX CMN %s %s" %(str(cmnpts[:,0]),str(cmnpts[:,1]))
    return (nb,cmnpts)

def decode_line(s,l,func = float):
    """
    split a string and convert each token with <func> 
    and append the result to list <l>
    """
    x = s.split()
    for v in x:
        l.append(func(v))
    return(l)

def get_bundle_cam(f):
    l = []
    for i in range(0,5):
        s = (f.readline()).strip()
        decode_line(s,l)

    matP = np.array([[l[3],l[4],l[5],l[12]],
                     [l[6],l[7],l[8],l[13]],
                     [l[9],l[10],l[11],l[14]]])
    focal = l[0]
    return (f,focal,matP)

def get_bundle_header(bundler_file,num = None):
    """
    reads header of <dir>/pmvs/bundle.rd.out
    output : file handle (pointing to the 1st point), nb of points
            + if num is given >= 0 : focal length and 3x4 Rotation-translation matrix of camera
            if num < 0 : array of focals and array of matrices for all cameras
    """
    matPs = []
    focals = []
    f = open(bundler_file,'r')
    s = f.readline()
    s = f.readline()
    s = s.strip()
    l = s.split()
    if len(l) != 2:
        raise Exception, "Bad 1st line <%s> in bundle file" %s
    nbcam = int(l[0])
    nbpts = int(l[1])
    if num != None and num >= nbcam:
        raise Exception, "Only %d cameras" %nbcam
    for i in xrange(0,nbcam):
        (f,focal,m) = get_bundle_cam(f)
        if num < 0 or i == num:
            focals.append(focal)
            matPs.append(m)
    if num < 0:
        return(f,nbpts,focals,matPs)
    else:
        return(f,nbpts,focals[0],matPs[0])
        
def read_projmatrx(file,return_coef = False,return_pairs = False):
    """
    return the 3x4 projection matrix read from the given pmvs .txt file
    """
    mat = []
    with open(file,'r') as f:
        lines = f.readlines()
        s = lines[0].strip()
        if (s != 'CONTOUR'):
            raise Exception, "Bad format <%s> for projection file" %s
        for i in range(1,4):
            s = lines[i].strip()
            l = s.split()
            for j in range(0,4):
                mat.append(float(l[j]))
    if return_coef or return_pairs:
        coef = None
        pairs = []
        if(len(lines) >= 6):
            if(lines[4].strip() == 'COEF'):
                coef = float(lines[5].strip())
        if return_pairs:
            if (len(lines) >= 10) and (lines[8].strip() == 'PAIRS'):
                nbp = int(lines[9].strip())
                if (len(lines) >= (10 + nbp)):
                    for i in xrange(0,nbp):
                        xl = map(lambda x : int(x) ,lines[10 + i].strip().split())
                        pairs.append(xl)
                    return(np.array(mat).reshape(3,4),coef,pairs)
        return(np.array(mat).reshape(3,4),coef)
    else:
        return(np.array(mat).reshape(3,4))

def get_bundle_points(f,nbpts,numcam):
    """
    input : filehandle to bundle file (ready to read 1st point)
            nb of points, and num of picture
    output (lists): - 3d coordinates of all points (numcam == None)
                       of points matched in camera numcam
                    - corresponding 2d coordinates if numcam is specified
                    (origine au centre de l'image)
                    - colors of returned points
    """
    pts3d = []
    pts2d = []
    ptscol = []
    for k in xrange(0,nbpts):
        l = []
        s = (f.readline()).strip()
        decode_line(s,l)
        l.append(1.)
        X = np.array(l)
        s = (f.readline()).strip()
        cols = []
        decode_line(s,cols,int)
        if (numcam == None):
            pts3d.append(X)
            ptscol.append(np.array(cols))
            f.readline()
        else:
            keys = []
            s = (f.readline()).strip()
            decode_line(s,keys)
            n = int(keys[0])
            i = 1
            for j in xrange(0,n):
                if(int(keys[i]) != numcam):
                    i += 4
                    continue
                rx = keys[i+2]
                ry = keys[i+3]
                i += 4
                pts2d.append((rx,ry))
                pts3d.append(X)
                ptscol.append(np.array(cols))
                break
    return(pts3d,pts2d,ptscol)

def get_ply_array(ptsfile):
    """
    read the ply file and return the points coord as a 3xn array
    """
    nbpts = 0
    with open(ptsfile,'r') as f:
        s = f.readline()
        if s.strip() != 'ply':
            raise Exception, "Not a ply file!"
        while True:
            s = f.readline().strip()
            l = s.split()
            if (len(l) == 3 and l[0] == 'element' and l[1] == 'vertex'):
                nbpts = int(l[2])
                continue
            if(s.strip() == 'end_header'):
                break
            if (s == ''):
                break
    # start with points
        kk = 0
        pts3d = np.empty((3,nbpts),dtype=np.float,order="F")
        while True:
            s = (f.readline()).strip()
            if (s == ''):
                break
            l = s.split()
            n = len(l)
            if(n < 3):
                continue
            ln = [float(l[i]) for i in range(0,n)]
            pts3d[0,kk] = ln[0]
            pts3d[1,kk] = ln[1]
            pts3d[2,kk] = ln[2]
            kk += 1
    return pts3d

def get_ply_points(ptsfile,return_normals = False):
    """
    read the ply file and return the lists of points and colors
    """
    pts3d = []
    ptscol = []
    ptsnorm = []
    with open(ptsfile,'r') as f:
        s = f.readline()
        if s.strip() != 'ply':
            raise Exception, "Not a ply file!"
        while True:
            s = f.readline()
            if(s.strip() == 'end_header'):
                break
            if (s == ''):
                break
    # start with points
            
        while True:
            s = (f.readline()).strip()
            if (s == ''):
                break
            l = s.split()
            n = len(l)
            if(n < 3):
                continue
            ln = [float(l[i]) for i in range(0,n)]
            pts3d.append(np.array([ln[0],ln[1],ln[2],1.]))
            if (n > 3):
                ptscol.append(np.array([ln[n-3],ln[n-2],ln[n-1]]))
            if (n > 6 and return_normals):
                ptsnorm.append(np.array([ln[3],ln[4],ln[5]]))
    if return_normals:
        return(pts3d,ptscol,ptsnorm)
    else:
        return(pts3d,ptscol)

def projection(matrx,pts3d):
    """
    returns the list of  projections of <pts3d> points (origin at the image center).
    <matrx> is the projection matrix of camera and <size> the image width and height
    """
    ptsxy = []
    n = len(pts3d)
##    print "MATR %s" %str(matrx)
    for i in xrange(0,n):
        X = pts3d[i]
        Y = np.dot(matrx,X)
        x = int(0.5 + Y[0] / Y[2])
        y = int(0.5 + Y[1] / Y[2])
        ptsxy.append((x,y))
    return(ptsxy)

def norm_coord(pts,size):
    (nx,ny) = size
    nx2 = 0.5 + (float(nx) / 2)
    ny2 = 0.5 + (float(ny) / 2)
    n = len(pts);
    npts = []
    for i in xrange(0,len(pts)):
        (rx,ry) = pts[i]
        rx = int(rx + nx2)
        ry = ny - int(ny2 + ry)
        npts.append((rx,ry))
    return npts


color = np.array([0,255,0])
color2 = np.array([255,0,0])
color3 = np.array([255,255,0])

def draw_projection(ptsxy,refpts,ptscol,obuf,size,wrect,cross = False):
    """
    input : 2d projections list (ptsxy), original 2d points (refpts)
           colors of points (ptscol), image buffer (obuf), image size
           drawing options (wrect,cross)
           Points are in image coordinates (0,0 = top left corner)
    writes 2d points (ptsxy) in the image buffer <obuf>; 
    if wrect is None :  a litle green square (cross = False) or
                       cross (cross = True) is drawn centered on each point
                       and if refpts is non empty, a red square or croos
                       is drawn on these points
                       If refpts[i] == ptsxy[i] : draw in yellow
    if wrect is given : at each point, fill a square of (2*wrect +1) size with the point color
    """
    (nx,ny) = size
##    nx2 = 0.5 + (float(nx) / 2)
##    ny2 = .5 + (float(ny) / 2)
    n2d = len(refpts)
    ncol = len(ptscol)
    n = len(ptsxy)
    for i in xrange(0,n):
        (x,y) = ptsxy[i]
        if(wrect != None):
            draw_color_rect(obuf,x,y,size,wrect,ptscol[i])
        else:
            draw_point(obuf,x,y,size,color)
            if n2d > 0:
                (rx,ry) = refpts[i]
##               rx = int(rx + nx2)
##                ry = ny - int(ny2 + ry)
            #print "XXX %d %d (%d %d)" %(x,y,rx,ry)
                if (rx == x and ry == y):
                    col2 = color3
                else:
                    col2 = color2
                draw_point(obuf,rx,ry,size,col2,cross)

def calc_camparams(m,tzsign,verb=False):
    """
    calculate intrinsics and extrinsics parameter of camera from 3x4 projection matrix m
    NB : tzsign is ignored and will be removed in a next release
    """
    a1 = m[0,0:3]
    a2 = m[1,0:3]
    a3 = m[2,0:3]
    b = np.array([[m[0,3]],[m[1,3]],[m[2,3]]])
    eps = 1
    rho=eps / np.linalg.norm(a3)
    r3 = rho * a3
    x0 = rho * rho * np.dot(a1.T, a3)
    y0 = rho * rho * np.dot(a2.T,a3)
    b1 = np.cross(a1,a3)
    b2 = np.cross(a2,a3);
    theta = np.arccos(- np.dot(b1.T,b2) / (np.linalg.norm(b1)*np.linalg.norm(b2)))
# The magnifications are assumed to be positive..
    alpha = rho * rho * np.linalg.norm(b1) * np.sin(theta)
    beta = rho * rho * np.linalg.norm(b2) * np.sin(theta)
    b23 = np.cross(a2,a3)
    r1 = (1. / np.linalg.norm(b23)) * b23
    r2 = np.cross(r3,r1)
    K = np.array([[alpha, -alpha * np.cos(theta) / np.sin(theta), x0],
                  [ 0, beta/np.sin(theta), y0], [0, 0, 1]])
    t = rho * np.linalg.solve(K,b)
# The sign of the solution is unknow : the last element in
# original and recalculated projection matrices must be the same
    R = np.array([r1.T, r2.T, r3.T])
    M = m
    M2 = np.dot(K,np.concatenate((R,t),1))
    tzsign = M[0,0] * M2[0,0] * M[2,3] * M2[2,3]
    if tzsign < 0:
        r3 = -r3
        r2 = -r2
        t = -t
        R = np.array([r1.T, r2.T, r3.T])
        M2 = np.dot(K,np.concatenate((R,t),1))

    rho =M[0,0] / M2[0,0]
    M2 = rho * M2
    intrins = np.array([alpha,beta,180 * theta / np.pi])
    if verb:
        print "\nrho %f\nM= %s\nM2= %s" %(rho,str(M),str(M2))
        print "R= %s\nt= %s\nintrinsic= %s" %(str(R),str(t),str(intrins))
        print "x0,y0 %s %s" %(x0,y0)
        X = np.array([[0.,0.,0.,1.],[t[0,0],t[1,0],-1.,1.]])
        pts = projection(m,X)
        pts2 = projection(M2,X)
        (x,y) = pts[0];(x1,y1) = pts2[0]
        print "Proj de 0 : %.3f %.3f, %.3f %.3f" %(x,y,x1,y1)
        (x,y) = pts[1];(x1,y1) = pts2[1]
        print "Proj de C : %.3f %.3f, %.3f %.3f\n" %(x,y,x1,y1)

    return (K,R,t,M2)

def calc_transform(pts3d1,pts3d2,cmn_pts):
    """
    renvoie la matrice de transformation pour passer du repere 1 au repere 2
    input : pts3d1 pts3d2, liste d'arrays de dim 4
            cmn_pts : liste de couples d'indices dans pts3d1 pts3d2
    """
    nb = len(cmn_pts)
    if (nb < 6):
        print "Not enough (%d) common points!" %nb
        return None
#   if(nb > 25):
#        nb = 25
    print "-- %d common projections" %nb
    A = np.zeros((3*nb,12))
    zer = np.zeros((4,))
    b = np.zeros((3*nb,))
    for i in xrange(0,nb):
        (i1,i2) = cmn_pts[i]
        A[3 * i,:] = np.concatenate((pts3d1[i1],zer,zer))
        A[3 * i + 1,:] = np.concatenate((zer,pts3d1[i1],zer))
        A[3 * i + 2,:] = np.concatenate((zer,zer,pts3d1[i1]))
        b[3*i:3*(i+1)] = pts3d2[i2][0:3]
    x = np.linalg.lstsq(A,b)[0]
    x = np.concatenate((x,np.array([0.,0.,0.,1.])))
    matr = x.reshape((4,4))
##    print "M= %s" %str(matr)
    emoy = 0.
    emax = 0.
    for i in xrange(0,nb):
        (i1,i2) = cmn_pts[i]
        p1 = pts3d1[i1]
        p2 = pts3d2[i2]
        y = np.dot(matr,p1)
        e = np.linalg.norm(y - p2)
        if e > emax:
            emax = e
        emoy += e
    emoy /= nb
    print "ERR max %f, moy %f" %(emax,emoy)
    return matr

def print_matrix(M,file= None):
    """
    print matrix m*n  as m lines of n floats
    """
    (m,n) = M.shape
    for i in xrange(0,m):
        l = map(lambda x : str(x),M[i,:])
        s = ' '.join(l)
        if file == None:
            print s
        else:
            print >>file, s


def mk_KRt(mat,focal,w,h):
    """
    makes matrix in pmvs2 format from bundler camera data
    """
    K = np.array([[-focal,0, 0.5 * w - 0.5],
                       [0,focal,0.5 * h - 0.5],
                       [0,0,1]])
    M = np.dot(K,mat)
    return ( - np.concatenate((M,np.array([0,0,0,1],ndmin=2)),0))

def read_metis_graph(file):
    """
    reads the metis graph (1st line = nb-img 2*nb-edges,
            other lines : neighbours of each image
    Output : list of lists (list[i] = neighbours of i)
    """
    graph = []
    try:
        f = open(file,'r')
        s = f.readline().strip()
        l = s.split()
        with_weights = False
        if len(l) == 3:
            with_weights = True
        while True:
            s = f.readline()
            if s == '':
                break
            l = []
            if with_weights:
                x = s.strip().split()
                for i in xrange(0,len(x),2):
                    l.append(int(x[i]) - 1)
            else:
                l = map(lambda x : int(x) - 1,s.strip().split())
            graph.append(l)
        f.close()
    except Exception:
        raise MyException("%s open error" %file)

    return graph

def read_adj_matrix(file):
    """
    reads the adjacency matrix (values = nb of matched points)
    Output : list of lists (list[i] = list of (j, nb-match) where j is a neighbour of i)
         list[i] is ordered according to nb-match
    """
    adj = []
    try:
        f = open(file,'r')
        lines = f.readlines()
        f.close()
    except Exception:
        raise MyException("%s open error" %file)
    for s in lines:
        l = []
        decode_line(s.strip(),l,func = int)
        neighbours = []
        for i in xrange(0,len(l)):
            if l[i] != 0:
                neighbours.append((i,l[i]))
        neighbours = sorted(neighbours,cmp = lambda (i1,nb1),(i2,nb2) : nb1 - nb2)
        adj.append(neighbours)
    
    return adj

def calc_cam_positions(cams,focals,R = None, t = None):
    poscams = []
    for i in xrange(0,len(cams)):
        if focals[i] == 0:
            poscams.append(None)
            continue
        m = cams[i]
        R0 = m[:,0:3]
        t0 = -m[:,3]
#        Rinv = np.linalg.inv(R0)
        X = np.dot(R0.T,t0)
##        print "XX %s, i= %d, t0 %s\nR0 %s\nR%s\nX %s" %(bundlefile,i,str(t0),str(R0),str(Rinv),str(X));quit()
        if R != None:
            X = np.dot(R,X) + t
        poscams.append((X[0],X[1],X[2]))
    return poscams
