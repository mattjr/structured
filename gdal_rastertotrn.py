#!/usr/bin/python

import sys
import numpy as np
from osgeo import gdal

def write_ply(filename, coordinates, triangles, uv,binary=True):
    template = "ply\n"
    if binary:
        template += "format binary_" + sys.byteorder + "_endian 1.0\n"
    else:
        template += "format ascii 1.0\n"
    template += """element vertex {nvertices:n}
property float x
property float y
property float z
element face {nfaces:n}
property list int int vertex_index
property list uchar float texcoord
end_header
"""

    context = {
     "nvertices": len(coordinates),
     "nfaces": len(triangles)
    }

    if binary:
        with  open(filename,'wb') as outfile:
            outfile.write(template.format(**context))
            coordinates = np.array(coordinates, dtype="float32")
            coordinates.tofile(outfile)

            triangles = np.hstack((np.ones([len(triangles),1], dtype="int") * 3,
                triangles))
            triangles = np.array(triangles, dtype="int32")
            triangles.tofile(outfile)
    else:
        with  open(filename,'w') as outfile:
            outfile.write(template.format(**context))
            np.savetxt(outfile, coordinates, fmt="%.3f")
            for i in range(0,triangles.shape[0]):
                str = '3 %i %i %i ' % tuple(triangles[i,:][::-1])
                str += '6 %f %f %f %f %f %f\n' %tuple(uv[triangles[i,:]].ravel())
#                print str
                outfile.write(str)
#            np.savetxt(outfile, np.hstack((triangles,uv)), fmt="3 %i %i %i 6 %f %f %f %f %f %f")

def write_obj(filename, vertices, triangles, uv=None):
    with  open(filename,'w') as outfile:
        np.savetxt(outfile, vertices, fmt="v %f %f %f")
        if not uv is None:
            np.savetxt(outfile, uv, 
                       fmt="vt %f %f")
        if triangles is not None and len(triangles) > 0:
                np.savetxt(outfile, np.dstack((triangles, triangles)).reshape((-1, 6)) + 1, 
                           fmt="f %d/%d %d/%d %d/%d")

def readraster(filename):
    raster = gdal.Open(filename)
    return raster


def createvertexarray(raster):
    transform = raster.GetGeoTransform()
    width = raster.RasterXSize
    height = raster.RasterYSize
    band = raster.GetRasterBand(1)

    nodata = band.GetNoDataValue()
    if nodata is None:
        nodata=255

    metric_width=100.0
    ratio=abs(transform[5])/abs(transform[1])
    metric_scale=metric_width/float(width)
    final_scale=metric_scale/transform[1]

    x = np.linspace(-(metric_width/2.0),(metric_width/2.0),num=width) 
    y = np.linspace(-((metric_width*ratio)/2.0),((metric_width*ratio)/2.0),num=height) 
 #   x = np.arange(0, width) * transform[1] + transform[0]
 #   y = np.arange(0, height) * transform[5] + transform[3]
    xx, yy = np.meshgrid(x, y)
    zz = raster.ReadAsArray()
#    print nodata
    
    vertices = np.vstack((xx,yy,zz)).reshape([3, -1]).transpose()
    vertices[vertices == nodata] = np.nan
    mask= ~np.isnan(vertices).any(1)
    vertices[:,2]=vertices[:,2]*final_scale
#    print vertices.shape,mask.shape
    
#    print np.count_nonzero(mask)
   # print vertices[mask,:].shape,mask
    #vertices=vertices[mask,:]
    return vertices,mask

def createuvarray(raster):
    transform = raster.GetGeoTransform()
    width = raster.RasterXSize
    height = raster.RasterYSize
    x = np.arange(0, width) #* transform[1] + transform[0]
    y = np.arange(0, height)# * transform[5] + transform[3]
    xx, yy = np.meshgrid(x, y)

    uu = xx / float(width)
    vv = 1.0-(yy / float(height))
    uv = np.vstack((uu,vv)).reshape([2, -1]).transpose()
    return uv

def createindexarray(raster):
    width = raster.RasterXSize
    height = raster.RasterYSize

    ai = np.arange(0, width - 1)
    aj = np.arange(0, height - 1)
    aii, ajj = np.meshgrid(ai, aj)
    a = aii + ajj * width
    a = a.flatten()

    tria = np.vstack((a, a + width, a + width + 1, a, a + width + 1, a + 1))
    tria = np.transpose(tria).reshape([-1, 3])
    return tria


def main(argv):
    inputfile = argv[0]
    outputfile = argv[1]

    raster = readraster(inputfile)
    vertices,mask = createvertexarray(raster)
    triangles = createindexarray(raster)
    #triangles = triangles[mask[0]]
    #triangles = triangles[triangles!=mask]
  #  print triangles.shape,mask.shape
    #triangles= triangles[np.dstack((mask,mask)),:]
   # print triangles.shape,mask.shape
#    print mask.shape,triangles.shape
    triangles_vals = ~np.isnan(np.take(vertices[:,2],triangles))
#    print triangles_vals.shape,triangles.shape
    triangles = triangles[np.where(np.all(triangles_vals,axis=1))]
#    print triangles_vals.shape,triangles.shape
    vertices[np.isnan(vertices)] = 0.0

    uv = createuvarray(raster)
    #write_obj(outputfile, vertices, triangles,uv)
    write_ply(outputfile, vertices, triangles, uv,binary=False)

if __name__ == "__main__":
    main(sys.argv[1:])
