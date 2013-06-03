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


try:
    from osgeo import gdal
    from osgeo import osr
    from osgeo import ogr
    gdal # pyflakes

except ImportError:
    import gdal

import sys, array,struct
def Usage():
    print('Usage: create_masked [--help-general] [-a_srs srs_def] [-a_ullr ulx uly lrx lry] dataset mask output')
    print('apply geotags onto existing GDAL dataset.')
    return -1

gdal.SetConfigOption( 'GDAL_TIFF_INTERNAL_MASK', 'YES' )
#gdal.SetConfigOption('GDAL_TIFF_INTERNAL_MASK_TO_8BIT', 'FALSE')
def gdal_edit(argv):
    
    argv = gdal.GeneralCmdLineProcessor( argv )
    if argv is None:
        return -1

    datasetname = None
    maskname = None
    outputname = None
    srs = None
    ulx = None
    uly = None
    lrx = None
    lry = None
    nodata = None
    xres = None
    yres = None
    unsetgt = False
    molist = []

    i = 1
    argc = len(argv)
    while i < argc:
        if argv[i] == '-a_srs' and i < len(argv)-1:
            srs = argv[i+1]
            i = i + 1
        elif argv[i] == '-a_ullr' and i < len(argv)-4:
            ulx = float(argv[i+1])
            i = i + 1
            uly = float(argv[i+1])
            i = i + 1
            lrx = float(argv[i+1])
            i = i + 1
            lry = float(argv[i+1])
            i = i + 1
        elif argv[i] == '-tr' and i < len(argv)-2:
            xres = float(argv[i+1])
            i = i + 1
            yres = float(argv[i+1])
            i = i + 1
        elif argv[i] == '-a_nodata' and i < len(argv)-1:
            nodata = float(argv[i+1])
            i = i + 1
        elif argv[i] == '-mo' and i < len(argv)-1:
            molist.append(argv[i+1])
            i = i + 1
        elif argv[i] == '-unsetgt' :
            unsetgt = True
        elif argv[i][0] == '-':
            sys.stderr.write('Unrecognized option : %s\n' % argv[i])
            return Usage()
        elif datasetname is None:
            datasetname = argv[i]
        elif maskname is None:
            maskname = argv[i]
        elif outputname is None:
            outputname = argv[i]
        else:
            sys.stderr.write('Unexpected option : %s\n' % argv[i])
            return Usage()

        i = i + 1

    if datasetname is None:
        return Usage()
    if maskname is None:
        return Usage()
    if outputname is None:
        return Usage()
    if srs is None and lry is None and yres is None and not unsetgt and nodata is None and len(molist) == 0:
        print('No option specified')
        print('')
        return Usage()

    exclusive_option = 0
    if lry is not None:
        exclusive_option = exclusive_option + 1
    if yres is not None:
        exclusive_option = exclusive_option + 1
    if unsetgt:
        exclusive_option = exclusive_option + 1
    if exclusive_option > 1:
        print('-a_ullr, -tr and -unsetgt options are exclusive.')
        print('')
        return Usage()

    source = gdal.Open(datasetname, gdal.GA_ReadOnly)
    alphaBand = gdal.Open(maskname, gdal.GA_ReadOnly)
    
    bands = [source.GetRasterBand(i+1) for i in range(source.RasterCount)]

    # check that bands 1-3 are RGB
    #assert [band.GetColorInterpretation() for band in bands] == [gdal.GCI_RedBand, gdal.GCI_GreenBand, gdal.GCI_BlueBand]

    target = gdal.GetDriverByName('GTiff').Create( outputname,
                                              source.RasterXSize,
                                              source.RasterYSize,
                                              3, gdal.GDT_Byte,
                                              [ 'PHOTOMETRIC=YCBCR',
                                                'COMPRESS=JPEG',
                                                'TILED=YES' ] ) 
    if target is None:
        return -1

    # copy georef
    target.SetProjection(source.GetProjection())
    target.SetGeoTransform(source.GetGeoTransform())

    if srs is not None:
        srs_real = osr.SpatialReference()
        srs_real.ImportFromProj4(srs)
        target.SetProjection(srs_real.ExportToWkt())

    if lry is not None:
        gt = [ ulx, (lrx - ulx) / target.RasterXSize, 0,
               uly, 0, (lry - uly) / target.RasterYSize ]
        target.SetGeoTransform(gt)

    if yres is not None:
        gt = target.GetGeoTransform()
        # Doh ! why is gt a tuple and not an array...
        gt = [ gt[i] for i in range(6) ]
        gt[1] = xres
        gt[5] = yres
        target.SetGeoTransform(gt)

    if unsetgt:
        target.SetGeoTransform([0,1,0,0,0,1])

    if nodata is not None:
        for i in range(target.RasterCount):
            target.GetRasterBand(1).SetNoDataValue(nodata)

    if len(molist) != 0:
        target.SetMetadata(molist)


    target_bands = [target.GetRasterBand(i+1) for i in range(3)]
    block_width, block_height = target_bands[0].GetBlockSize()

   

    # Use straight up black if neither alpha nor nodata is present
    nodata = [band.GetNoDataValue() for band in bands]
    use_alpha = True #(len(bands) > 4)
    use_nodata = False#(not use_alpha and len([n for n in nodata if n is not None]) == 3)

    print >>sys.stderr, "Generating mask from",
    if use_alpha:
        print >>sys.stderr, "alpha band."
    else:
        if use_nodata:
            print >>sys.stderr, "NODATA values."
            nodata = [chr(int(n)) for n in nodata]
        else:
            print >>sys.stderr, "black pixels."
            nodata = ["\0", "\0", "\0"]
            use_nodata = True

    target.CreateMaskBand(gdal.GMF_PER_DATASET)
    mask_band = target_bands[0].GetMaskBand()

    total_blocks = (target.RasterXSize * target.RasterYSize) / float(block_width * block_height)
    blocks_written = 0
    sys.stderr.write("0...")

    for col in range(0, target.RasterXSize, block_width):
        for row in range(0, target.RasterYSize, block_height):
            width = min(target.RasterXSize - col, block_width)
            height = min(target.RasterYSize - row, block_height)
            data = [band.ReadRaster(col, row, width, height) for band in bands[:3]]
            mask = array.array("B", (1 for x in range(len(data[0]))))
            alpha = alphaBand.ReadRaster(col, row, width, height)

            tuple= struct.unpack('B' * width*height ,alpha)
            alpha = None
            if use_alpha:
                alpha = alphaBand.ReadRaster(col, row, width, height)
    #            print len(alpha)
    #            print tuple
            for byte in range(len(data[0])):
                if use_alpha:
                    if not tuple[byte]:
                        mask[byte] = 0
                else:
                    if data[0][byte] == nodata[0] and data[1][byte] == nodata[1] and data[2][byte] == nodata[2]:
                        mask[byte] = 0
            for n, block in enumerate(data):
                target_bands[n].WriteRaster(col, row, width, height, block)
            #print mask.tostring()
            mask_band.WriteRaster(col, row, width, height, mask.tostring())

            if int(blocks_written / total_blocks * 10) != int((blocks_written + 1) / total_blocks * 10):
                count = int((blocks_written + 1) / total_blocks * 10)
                sys.stderr.write("done." if count == 10 else "%d0..." % count)
            blocks_written += 1

    sys.stderr.write("\n")
    mask_band = None
    target_bands = None
    target = None
    alphaBand = None
if __name__ == '__main__':
    sys.exit(gdal_edit(sys.argv))