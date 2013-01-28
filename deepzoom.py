#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
#  Deep Zoom Tools
#
#  Copyright (c) 2008-2011, OpenZoom <http://openzoom.org>
#  Copyright (c) 2008-2011, Daniel Gasienica <daniel@gasienica.ch>
#  Copyright (c) 2010, Boris Bluntschli <boris@bluntschli.ch>
#  Copyright (c) 2008, Kapil Thangavelu <kapil.foss@gmail.com>
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#      1. Redistributions of source code must retain the above copyright notice,
#         this list of conditions and the following disclaimer.
#
#      2. Redistributions in binary form must reproduce the above copyright
#         notice, this list of conditions and the following disclaimer in the
#         documentation and/or other materials provided with the distribution.
#
#      3. Neither the name of OpenZoom nor the names of its contributors may be used
#         to endorse or promote products derived from this software without
#         specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

import math
import optparse
import os
import PIL.Image
import shutil

try:
    import cStringIO
    StringIO = cStringIO
except ImportError:
    import StringIO

import sys
import time
import urllib
import warnings
import xml.dom.minidom
import numpy

from collections import deque
from osgeo import gdal
from osgeo import osr
import osgeo.gdal_array as gdalarray
import base64
import bz2
import struct

NS_DEEPZOOM = 'http://schemas.microsoft.com/deepzoom/2008'

DEFAULT_RESIZE_FILTER = PIL.Image.ANTIALIAS
DEFAULT_IMAGE_FORMAT = 'jpg'

RESIZE_FILTERS = {
    'cubic': PIL.Image.CUBIC,
    'bilinear': PIL.Image.BILINEAR,
    'bicubic': PIL.Image.BICUBIC,
    'nearest': PIL.Image.NEAREST,
    'antialias': PIL.Image.ANTIALIAS,
    }

IMAGE_FORMATS = {
    'jpg': 'jpg',
    'png': 'png',
    }

jsmin_str_basebz2="""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
KLM7uH/4u5IpwoSAmJIUGA=="""

class DeepZoomImageDescriptor(object):
    def __init__(self, width=None, height=None,
                 tile_size=254, tile_overlap=1, tile_format='jpg'):
        self.width = width
        self.height = height
        self.tile_size = tile_size
        self.tile_overlap = tile_overlap
        self.tile_format = tile_format
        self._num_levels = None

    def open(self, source):
        """Intialize descriptor from an existing descriptor file."""
        doc = xml.dom.minidom.parse(safe_open(source))
        image = doc.getElementsByTagName('Image')[0]
        size = doc.getElementsByTagName('Size')[0]
        self.width = int(size.getAttribute('Width'))
        self.height = int(size.getAttribute('Height'))
        self.tile_size = int(image.getAttribute('TileSize'))
        self.tile_overlap = int(image.getAttribute('Overlap'))
        self.tile_format = image.getAttribute('Format')

    def save(self, destination):
        """Save descriptor file."""
        file = open(destination, 'w')
        doc = xml.dom.minidom.Document()
        image = doc.createElementNS(NS_DEEPZOOM, 'Image')
        image.setAttribute('xmlns', NS_DEEPZOOM)
        image.setAttribute('TileSize', str(self.tile_size))
        image.setAttribute('Overlap', str(self.tile_overlap))
        image.setAttribute('Format', str(self.tile_format))
        size = doc.createElementNS(NS_DEEPZOOM, 'Size')
        size.setAttribute('Width', str(self.width))
        size.setAttribute('Height', str(self.height))
        image.appendChild(size)
        doc.appendChild(image)
        descriptor = doc.toxml(encoding='UTF-8')
        file.write(descriptor)
        file.close()

    @classmethod
    def remove(self, filename):
        """Remove descriptor file (DZI) and tiles folder."""
        _remove(filename)

    @property
    def num_levels(self):
        """Number of levels in the pyramid."""
        if self._num_levels is None:
            max_dimension = max(self.width, self.height)
            self._num_levels = int(math.ceil(math.log(max_dimension, 2))) + 1
        return self._num_levels

    def get_scale(self, level):
        """Scale of a pyramid level."""
        assert 0 <= level and level < self.num_levels, 'Invalid pyramid level'
        max_level = self.num_levels - 1
        return math.pow(0.5, max_level - level)

    def get_dimensions(self, level):
        """Dimensions of level (width, height)"""
        assert 0 <= level and level < self.num_levels, 'Invalid pyramid level'
        scale = self.get_scale(level)
        width = int(math.ceil(self.width * scale))
        height = int(math.ceil(self.height * scale))
        return (width, height)

    def get_num_tiles(self, level):
        """Number of tiles (columns, rows)"""
        assert 0 <= level and level < self.num_levels, 'Invalid pyramid level'
        w, h = self.get_dimensions( level )
        return (int(math.ceil(float(w) / self.tile_size)),
                int(math.ceil(float(h) / self.tile_size)))

    def get_tile_bounds(self, level, column, row):
        """Bounding box of the tile (x1, y1, x2, y2)"""
        assert 0 <= level and level < self.num_levels, 'Invalid pyramid level'
        offset_x = 0 if column == 0 else self.tile_overlap
        offset_y = 0 if row == 0 else self.tile_overlap
        x = (column * self.tile_size) - offset_x
        y = (row * self.tile_size) - offset_y
        level_width, level_height = self.get_dimensions(level)
        w = self.tile_size + (1 if column == 0 else 2) * self.tile_overlap
        h = self.tile_size + (1 if row == 0 else 2) * self.tile_overlap
        w = min(w, level_width  - x)
        h = min(h, level_height - y)
        return (x, y, x + w, y + h)

    

class DeepZoomCollection(object):
    def __init__(self, filename, image_quality=0.8, max_level=7,
                tile_size=256, tile_format='jpg', items=[]):
        self.source = filename
        self.image_quality = image_quality
        self.tile_size = tile_size
        self.max_level = max_level
        self.tile_format = tile_format
        self.items = deque(items)
        self.next_item_id = len(self.items)
        # XML
        self.doc = xml.dom.minidom.Document()
        collection = self.doc.createElementNS(NS_DEEPZOOM, 'Collection')
        collection.setAttribute('xmlns', NS_DEEPZOOM)
        collection.setAttribute('MaxLevel', str(self.max_level))
        collection.setAttribute('TileSize', str(self.tile_size))
        collection.setAttribute('Format', str(self.tile_format))
        collection.setAttribute('Quality', str(self.image_quality))
        items = self.doc.createElementNS(NS_DEEPZOOM, 'Items')
        collection.appendChild(items)
        collection.setAttribute('NextItemId', str(self.next_item_id))
        self.doc.appendChild(collection)

    @classmethod
    def from_file(self, filename):
        """Open collection descriptor."""
        doc = xml.dom.minidom.parse(safe_open(filename))
        collection = doc.getElementsByTagName('Collection')[0]
        image_quality = float(collection.getAttribute('Quality'))
        max_level = int(collection.getAttribute('MaxLevel'))
        tile_size = int(collection.getAttribute('TileSize'))
        tile_format = collection.getAttribute('Format')
        items = [DeepZoomCollectionItem.from_xml(item) for item
                    in doc.getElementsByTagName('I')]

        collection = DeepZoomCollection(filename,
                                        image_quality=image_quality,
                                        max_level=max_level,
                                        tile_size=tile_size,
                                        tile_format=tile_format,
                                        items=items)
        return collection


    @classmethod
    def remove(self, filename):
        """Remove collection file (DZC) and tiles folder."""
        _remove(filename)

    def append(self, source):
        descriptor = DeepZoomImageDescriptor()
        descriptor.open(source)
        item = DeepZoomCollectionItem(source, descriptor.width, descriptor.height,
                                     id=self.next_item_id)
        self.items.append(item)
        self.next_item_id += 1

    def save(self, pretty_print_xml=False):
        """Save collection descriptor."""
        collection = self.doc.getElementsByTagName('Collection')[0]
        items = self.doc.getElementsByTagName('Items')[0]
        while len(self.items) > 0:
            item = self.items.popleft()
            i = self.doc.createElementNS(NS_DEEPZOOM, 'I')
            i.setAttribute('Id', str(item.id))
            i.setAttribute('N', str(item.id))
            i.setAttribute('Source', item.source)
            # Size
            size = self.doc.createElementNS(NS_DEEPZOOM, 'Size')
            size.setAttribute('Width', str(item.width))
            size.setAttribute('Height', str(item.height))
            i.appendChild(size)
            items.appendChild(i)
            self._append_image(item.source, item.id)
        collection.setAttribute('NextItemId', str(self.next_item_id))
        with open(self.source, 'w') as f:
            if pretty_print_xml:
                xml = self.doc.toprettyxml(encoding='UTF-8')
            else:
                xml = self.doc.toxml(encoding='UTF-8')
            f.write(xml)
    
    def _append_image(self, path, i):
        descriptor = DeepZoomImageDescriptor()
        descriptor.open(path)
        files_path = _get_or_create_path(_get_files_path(self.source))
        for level in reversed(xrange(self.max_level + 1)):
            level_path = _get_or_create_path('%s/%s'%(files_path, level))
            level_size = 2**level
            images_per_tile = int(math.floor(self.tile_size / level_size))
            column, row = self.get_tile_position(i, level, self.tile_size)
            tile_path = '%s/%s_%s.%s'%(level_path, column, row, self.tile_format)
            if not os.path.exists(tile_path):
                tile_image = PIL.Image.new('RGB', (self.tile_size, self.tile_size))
                q = int(self.image_quality * 100)
                tile_image.save(tile_path, 'JPEG', quality=q)
            tile_image = PIL.Image.open(tile_path)
            source_path = '%s/%s/%s_%s.%s'%(_get_files_path(path), level, 0, 0,
                                            descriptor.tile_format)
            # Local
            if os.path.exists(source_path):
                try:
                    source_image = PIL.Image.open(safe_open(source_path))
                except IOError:
                    warnings.warn('Skipped invalid level: %s' % source_path)
                    continue
            # Remote
            else:
                if level == self.max_level:
                    try:
                        source_image = PIL.Image.open(safe_open(source_path))
                    except IOError:
                        warnings.warn('Skipped invalid image: %s' % source_path)
                        return
                    # Expected width & height of the tile
                    e_w, e_h = descriptor.get_dimensions(level)
                    # Actual width & height of the tile
                    w, h = source_image.size
                    # Correct tile because of IIP bug where low-level tiles
                    # have wrong dimensions (they are too large)
                    if w != e_w or h != e_h:
                        # Resize incorrect tile to correct size
                        source_image = source_image.resize((e_w, e_h), PIL.Image.ANTIALIAS)
                        # Store new dimensions
                        w, h = e_w, e_h
                else:
                    w = int(math.ceil(w * 0.5))
                    h = int(math.ceil(h * 0.5))
                    source_image.thumbnail((w, h), PIL.Image.ANTIALIAS)
            column, row = self.get_position(i)
            x = (column % images_per_tile) * level_size
            y = (row % images_per_tile) * level_size
            tile_image.paste(source_image, (x, y))
            tile_image.save(tile_path)

    def get_position(self, z_order):
        """Returns position (column, row) from given Z-order (Morton number.)"""
        column = 0
        row = 0
        for i in xrange(0, 32, 2):
            offset = i / 2
            # column
            column_offset = i
            column_mask = 1 << column_offset
            column_value = (z_order & column_mask) >> column_offset
            column |= column_value << offset
            #row
            row_offset = i + 1
            row_mask = 1 << row_offset
            row_value = (z_order & row_mask) >> row_offset
            row |= row_value << offset
        return int(column), int(row)

    def get_z_order(self, column, row):
        """Returns the Z-order (Morton number) from given position."""
        z_order = 0
        for i in xrange(32):
            z_order |= (column & 1 << i) << i | (row & 1 << i) << (i + 1)
        return z_order

    def get_tile_position(self, z_order, level, tile_size):
        level_size = 2**level
        x, y = self.get_position(z_order)
        return (int(math.floor((x * level_size) / tile_size)),
                int(math.floor((y * level_size) / tile_size)))


class DeepZoomCollectionItem(object):
    def __init__(self, source, width, height, id=0):
        self.id = id
        self.source = source
        self.width = width
        self.height = height

    @classmethod
    def from_xml(cls, xml):
        id = int(xml.getAttribute('Id'))
        source = xml.getAttribute('Source')
        size = xml.getElementsByTagName('Size')[0]
        width = int(size.getAttribute('Width'))
        height = int(size.getAttribute('Height'))
        return DeepZoomCollectionItem(source, width, height, id)


class ImageCreator(object):
    """Creates Deep Zoom images."""
    def __init__(self, tile_size=254, tile_overlap=1, tile_format='jpg',
                 image_quality=0.8, resize_filter=None, copy_metadata=False,clearcolor=None):
        self.tile_size = int(tile_size)
        self.clearcolor = clearcolor
        self.tile_format = tile_format
        self.tile_overlap = _clamp(int(tile_overlap), 0, 10)
        self.image_quality = _clamp(image_quality, 0, 1.0)
        if not tile_format in IMAGE_FORMATS:
            self.tile_format = DEFAULT_IMAGE_FORMAT
        self.resize_filter = resize_filter
        self.copy_metadata = copy_metadata
    def open_input(self):
		"""Initialization of the input raster, reprojection if necessary"""
		from optparse import OptionParser, OptionGroup
		usage = "Usage: %prog [options] input_file(s) [output]"

                p = OptionParser(usage)
                p.add_option("-v", "--verbose",
                             action="store_true", dest="verbose",
                             help="Print status messages to stdout")
                p.add_option('-a', '--srcnodata', dest="srcnodata", metavar="NODATA",
                                     help="NODATA transparency value to assign to the input data")
		gdal.AllRegister()
                self.options = p
                if self.clearcolor == None:
                    self.clearcolor= numpy.array([255,255,255])
                else:
                    if self.clearcolor[0] == '#':      
                        self.clearcolor=numpy.array(struct.unpack('BBB',self.clearcolor[1:].decode('hex')))
                        print 'Using clear color'
                        print self.clearcolor
                    else:
                        print 'invalid pass #hexstring failing!'
                        sys.exit(-1)
                self.options.verbose = 0
                self.options.srcnodata = None
                self.options.s_srs = None
                self.options.profile = None
                self.tilesize = 256
                self.tminz = None
		self.tmaxz = None
                self.kmz = None
                self.kml = None
                self.options.profile = 'raster'
		# Initialize necessary GDAL drivers
		
		self.out_drv = gdal.GetDriverByName( 'gtiff')#self.tiledriver )
		self.mem_drv = gdal.GetDriverByName( 'MEM' )
		
		if not self.out_drv:
			raise Exception("The '%s' driver was not found, is it available in this GDAL build?", self.tiledriver)
		if not self.mem_drv:
			raise Exception("The 'MEM' driver was not found, is it available in this GDAL build?")
		
		# Open the input file
		
		if self.input:
			self.in_ds = gdal.Open(self.input, gdal.GA_ReadOnly)
		else:
			raise Exception("No input file was specified")

		if self.options.verbose:
			print("Input file:", "( %sP x %sL - %s bands)" % (self.in_ds.RasterXSize, self.in_ds.RasterYSize, self.in_ds.RasterCount))

		if not self.in_ds:
			# Note: GDAL prints the ERROR message too
			self.error("It is not possible to open the input file '%s'." % self.input )
			
		# Read metadata from the input file
		if self.in_ds.RasterCount == 0:
			self.error( "Input file '%s' has no raster band" % self.input )
			
		if self.in_ds.GetRasterBand(1).GetRasterColorTable():
			# TODO: Process directly paletted dataset by generating VRT in memory
			self.error( "Please convert this file to RGB/RGBA and run gdal2tiles on the result.",
			"""From paletted file you can create RGBA file (temp.vrt) by:
gdal_translate -of vrt -expand rgba %s temp.vrt
then run:
gdal2tiles temp.vrt""" % self.input )

		# Get NODATA value
		self.in_nodata = []
		for i in range(1, self.in_ds.RasterCount+1):
			if self.in_ds.GetRasterBand(i).GetNoDataValue() != None:
				self.in_nodata.append( self.in_ds.GetRasterBand(i).GetNoDataValue() )
		if self.options.srcnodata:
			nds = list(map( float, self.options.srcnodata.split(',')))
			if len(nds) < self.in_ds.RasterCount:
				self.in_nodata = (nds * self.in_ds.RasterCount)[:self.in_ds.RasterCount]
			else:
				self.in_nodata = nds

		if self.options.verbose:
			print("NODATA: %s" % self.in_nodata)

		#
		# Here we should have RGBA input dataset opened in self.in_ds
		#

		if self.options.verbose:
			print("Preprocessed file:", "( %sP x %sL - %s bands)" % (self.in_ds.RasterXSize, self.in_ds.RasterYSize, self.in_ds.RasterCount))

		# Spatial Reference System of the input raster


		self.in_srs = None
		
		if self.options.s_srs:
			self.in_srs = osr.SpatialReference()
			self.in_srs.SetFromUserInput(self.options.s_srs)
			self.in_srs_wkt = self.in_srs.ExportToWkt()
		else:
			self.in_srs_wkt = self.in_ds.GetProjection()
			if not self.in_srs_wkt and self.in_ds.GetGCPCount() != 0:
				self.in_srs_wkt = self.in_ds.GetGCPProjection()
			if self.in_srs_wkt:
				self.in_srs = osr.SpatialReference()
				self.in_srs.ImportFromWkt(self.in_srs_wkt)
			#elif self.options.profile != 'raster':
			#	self.error("There is no spatial reference system info included in the input file.","You should run gdal2tiles with --s_srs EPSG:XXXX or similar.")

		# Spatial Reference System of tiles
		
		self.out_srs = osr.SpatialReference()

		if self.options.profile == 'mercator':
			self.out_srs.ImportFromEPSG(900913)
		elif self.options.profile == 'geodetic':
			self.out_srs.ImportFromEPSG(4326)
		else:
			self.out_srs = self.in_srs
		
		# Are the reference systems the same? Reproject if necessary.

		self.out_ds = None
		
		if self.options.profile in ('mercator', 'geodetic'):
						
			if (self.in_ds.GetGeoTransform() == (0.0, 1.0, 0.0, 0.0, 0.0, 1.0)) and (self.in_ds.GetGCPCount() == 0):
				self.error("There is no georeference - neither affine transformation (worldfile) nor GCPs. You can generate only 'raster' profile tiles.",
				"Either gdal2tiles with parameter -p 'raster' or use another GIS software for georeference e.g. gdal_transform -gcp / -a_ullr / -a_srs")
				
			if self.in_srs:
				
				if (self.in_srs.ExportToProj4() != self.out_srs.ExportToProj4()) or (self.in_ds.GetGCPCount() != 0):
					
					# Generation of VRT dataset in tile projection, default 'nearest neighbour' warping
					self.out_ds = gdal.AutoCreateWarpedVRT( self.in_ds, self.in_srs_wkt, self.out_srs.ExportToWkt() )
					
					# TODO: HIGH PRIORITY: Correction of AutoCreateWarpedVRT according the max zoomlevel for correct direct warping!!!
					
					if self.options.verbose:
						print("Warping of the raster by AutoCreateWarpedVRT (result saved into 'tiles.vrt')")
						self.out_ds.GetDriver().CreateCopy("tiles.vrt", self.out_ds)
						
					# Note: self.in_srs and self.in_srs_wkt contain still the non-warped reference system!!!

					# Correction of AutoCreateWarpedVRT for NODATA values
					if self.in_nodata != []:
						import tempfile
						tempfilename = tempfile.mktemp('-gdal2tiles.vrt')
						self.out_ds.GetDriver().CreateCopy(tempfilename, self.out_ds)
						# open as a text file
						s = open(tempfilename).read()
						# Add the warping options
						s = s.replace("""<GDALWarpOptions>""","""<GDALWarpOptions>
	  <Option name="INIT_DEST">NO_DATA</Option>
	  <Option name="UNIFIED_SRC_NODATA">YES</Option>""")
						# replace BandMapping tag for NODATA bands....
						for i in range(len(self.in_nodata)):
							s = s.replace("""<BandMapping src="%i" dst="%i"/>""" % ((i+1),(i+1)),"""<BandMapping src="%i" dst="%i">
	      <SrcNoDataReal>%i</SrcNoDataReal>
	      <SrcNoDataImag>0</SrcNoDataImag>
	      <DstNoDataReal>%i</DstNoDataReal>
	      <DstNoDataImag>0</DstNoDataImag>
	    </BandMapping>""" % ((i+1), (i+1), self.in_nodata[i], self.in_nodata[i])) # Or rewrite to white by: , 255 ))
						# save the corrected VRT
						open(tempfilename,"w").write(s)
						# open by GDAL as self.out_ds
						self.out_ds = gdal.Open(tempfilename) #, gdal.GA_ReadOnly)
						# delete the temporary file
						os.unlink(tempfilename)

						# set NODATA_VALUE metadata
						self.out_ds.SetMetadataItem('NODATA_VALUES','%i %i %i' % (self.in_nodata[0],self.in_nodata[1],self.in_nodata[2]))

						if self.options.verbose:
							print("Modified warping result saved into 'tiles1.vrt'")
							open("tiles1.vrt","w").write(s)

					# -----------------------------------
					# Correction of AutoCreateWarpedVRT for Mono (1 band) and RGB (3 bands) files without NODATA:
					# equivalent of gdalwarp -dstalpha
					if self.in_nodata == [] and self.out_ds.RasterCount in [1,3]:
						import tempfile
						tempfilename = tempfile.mktemp('-gdal2tiles.vrt')
						self.out_ds.GetDriver().CreateCopy(tempfilename, self.out_ds)
						# open as a text file
						s = open(tempfilename).read()
						# Add the warping options
						s = s.replace("""<BlockXSize>""","""<VRTRasterBand dataType="Byte" band="%i" subClass="VRTWarpedRasterBand">
    <ColorInterp>Alpha</ColorInterp>
  </VRTRasterBand>
  <BlockXSize>""" % (self.out_ds.RasterCount + 1))
						s = s.replace("""</GDALWarpOptions>""", """<DstAlphaBand>%i</DstAlphaBand>
  </GDALWarpOptions>""" % (self.out_ds.RasterCount + 1))
						s = s.replace("""</WorkingDataType>""", """</WorkingDataType>
    <Option name="INIT_DEST">0</Option>""")
						# save the corrected VRT
						open(tempfilename,"w").write(s)
						# open by GDAL as self.out_ds
						self.out_ds = gdal.Open(tempfilename) #, gdal.GA_ReadOnly)
						# delete the temporary file
						os.unlink(tempfilename)

						if self.options.verbose:
							print("Modified -dstalpha warping result saved into 'tiles1.vrt'")
							open("tiles1.vrt","w").write(s)
					s = '''
					'''
						
			else:
				self.error("Input file has unknown SRS.", "Use --s_srs ESPG:xyz (or similar) to provide source reference system." )

			if self.out_ds and self.options.verbose:
				print("Projected file:", "tiles.vrt", "( %sP x %sL - %s bands)" % (self.out_ds.RasterXSize, self.out_ds.RasterYSize, self.out_ds.RasterCount))
		
		if not self.out_ds:
			self.out_ds = self.in_ds

		#
		# Here we should have a raster (out_ds) in the correct Spatial Reference system
		#

		# Get alpha band (either directly or from NODATA value)
		self.alphaband = self.out_ds.GetRasterBand(1).GetMaskBand()
		if (self.alphaband.GetMaskFlags() & gdal.GMF_ALPHA) or self.out_ds.RasterCount==4 or self.out_ds.RasterCount==2:
			# TODO: Better test for alpha band in the dataset
			self.dataBandsCount = self.out_ds.RasterCount - 1
		else:
			self.dataBandsCount = self.out_ds.RasterCount

		# KML test
		self.isepsg4326 = False
		srs4326 = osr.SpatialReference()
		srs4326.ImportFromEPSG(4326)
		if self.out_srs and srs4326.ExportToProj4() == self.out_srs.ExportToProj4():
			self.kml = True
			self.isepsg4326 = True
			if self.options.verbose:
				print("KML autotest OK!")

		# Read the georeference 

		self.out_gt = self.out_ds.GetGeoTransform()
			
		#originX, originY = self.out_gt[0], self.out_gt[3]
		#pixelSize = self.out_gt[1] # = self.out_gt[5]
		
		# Test the size of the pixel
		
		# MAPTILER - COMMENTED
		#if self.out_gt[1] != (-1 * self.out_gt[5]) and self.options.profile != 'raster':
			# TODO: Process corectly coordinates with are have swichted Y axis (display in OpenLayers too)
			#self.error("Size of the pixel in the output differ for X and Y axes.")
			
		# Report error in case rotation/skew is in geotransform (possible only in 'raster' profile)
		if (self.out_gt[2], self.out_gt[4]) != (0,0):
			self.error("Georeference of the raster contains rotation or skew. Such raster is not supported. Please use gdalwarp first.")
			# TODO: Do the warping in this case automaticaly

		#
		# Here we expect: pixel is square, no rotation on the raster
		#

		# Output Bounds - coordinates in the output SRS
		self.ominx = self.out_gt[0]
		self.omaxx = self.out_gt[0]+self.out_ds.RasterXSize*self.out_gt[1]
		self.omaxy = self.out_gt[3]
		self.ominy = self.out_gt[3]-self.out_ds.RasterYSize*self.out_gt[1]
		# Note: maybe round(x, 14) to avoid the gdal_translate behaviour, when 0 becomes -1e-15

		if self.options.verbose:
			print("Bounds (output srs):", round(self.ominx, 13), self.ominy, self.omaxx, self.omaxy)

		#
		# Calculating ranges for tiles in different zoom levels
		#

		if self.options.profile == 'mercator':

			self.mercator = GlobalMercator() # from globalmaptiles.py
			
			# Function which generates SWNE in LatLong for given tile
			self.tileswne = self.mercator.TileLatLonBounds

			# Generate table with min max tile coordinates for all zoomlevels
			self.tminmax = list(range(0,32))
			for tz in range(0, 32):
				tminx, tminy = self.mercator.MetersToTile( self.ominx, self.ominy, tz )
				tmaxx, tmaxy = self.mercator.MetersToTile( self.omaxx, self.omaxy, tz )
				# crop tiles extending world limits (+-180,+-90)
				tminx, tminy = max(0, tminx), max(0, tminy)
				tmaxx, tmaxy = min(2**tz-1, tmaxx), min(2**tz-1, tmaxy)
				self.tminmax[tz] = (tminx, tminy, tmaxx, tmaxy)

			# TODO: Maps crossing 180E (Alaska?)

			# Get the minimal zoom level (map covers area equivalent to one tile) 
			if self.tminz == None:
				self.tminz = self.mercator.ZoomForPixelSize( self.out_gt[1] * max( self.out_ds.RasterXSize, self.out_ds.RasterYSize) / float(self.tilesize) )

			# Get the maximal zoom level (closest possible zoom level up on the resolution of raster)
			if self.tmaxz == None:
				self.tmaxz = self.mercator.ZoomForPixelSize( self.out_gt[1] )
			
			if self.options.verbose:
				print("Bounds (latlong):", self.mercator.MetersToLatLon( self.ominx, self.ominy), self.mercator.MetersToLatLon( self.omaxx, self.omaxy))
				print('MinZoomLevel:', self.tminz)
				print("MaxZoomLevel:", self.tmaxz, "(", self.mercator.Resolution( self.tmaxz ),")")

		if self.options.profile == 'geodetic':

			self.geodetic = GlobalGeodetic() # from globalmaptiles.py

			# Function which generates SWNE in LatLong for given tile
			self.tileswne = self.geodetic.TileLatLonBounds
			
			# Generate table with min max tile coordinates for all zoomlevels
			self.tminmax = list(range(0,32))
			for tz in range(0, 32):
				tminx, tminy = self.geodetic.LatLonToTile( self.ominx, self.ominy, tz )
				tmaxx, tmaxy = self.geodetic.LatLonToTile( self.omaxx, self.omaxy, tz )
				# crop tiles extending world limits (+-180,+-90)
				tminx, tminy = max(0, tminx), max(0, tminy)
				tmaxx, tmaxy = min(2**(tz+1)-1, tmaxx), min(2**tz-1, tmaxy)
				self.tminmax[tz] = (tminx, tminy, tmaxx, tmaxy)
				
			# TODO: Maps crossing 180E (Alaska?)

			# Get the maximal zoom level (closest possible zoom level up on the resolution of raster)
			if self.tminz == None:
				self.tminz = self.geodetic.ZoomForPixelSize( self.out_gt[1] * max( self.out_ds.RasterXSize, self.out_ds.RasterYSize) / float(self.tilesize) )

			# Get the maximal zoom level (closest possible zoom level up on the resolution of raster)
			if self.tmaxz == None:
				self.tmaxz = self.geodetic.ZoomForPixelSize( self.out_gt[1] )
			
			if self.options.verbose:
				print("Bounds (latlong):", self.ominx, self.ominy, self.omaxx, self.omaxy)
					
		if self.options.profile == 'raster':
			
			log2 = lambda x: math.log10(x) / math.log10(2) # log2 (base 2 logarithm)
			
			self.nativezoom = int(max( math.ceil(log2(self.out_ds.RasterXSize/float(self.tilesize))),
			                           math.ceil(log2(self.out_ds.RasterYSize/float(self.tilesize)))))
			self.width = self.out_ds.RasterXSize
			self.height = self.out_ds.RasterYSize

			if self.options.verbose:
				print("Native zoom of the raster:", self.nativezoom)

			# Get the minimal zoom level (whole raster in one tile)
			if self.tminz == None:
				self.tminz = 0

			# Get the maximal zoom level (native resolution of the raster)
			if self.tmaxz == None:
				self.tmaxz = self.nativezoom

			# Generate table with min max tile coordinates for all zoomlevels
			self.tminmax = list(range(0, self.tmaxz+1))
			self.tsize = list(range(0, self.tmaxz+1))
			for tz in range(0, self.tmaxz+1):
				tsize = 2.0**(self.nativezoom-tz)*self.tilesize
				tminx, tminy = 0, 0
				tmaxx = int(math.ceil( self.out_ds.RasterXSize / tsize )) - 1
				tmaxy = int(math.ceil( self.out_ds.RasterYSize / tsize )) - 1
				self.tsize[tz] = math.ceil(tsize)
				self.tminmax[tz] = (tminx, tminy, tmaxx, tmaxy)

			# Function which generates SWNE in LatLong for given tile
			if self.kml and self.in_srs_wkt:
				self.ct = osr.CoordinateTransformation(self.in_srs, srs4326)

				def rastertileswne(x,y,z):
					pixelsizex = (2**(self.tmaxz-z) * self.out_gt[1]) # X-pixel size in level
					pixelsizey = (2**(self.tmaxz-z) * self.out_gt[1]) # Y-pixel size in level (usually -1*pixelsizex)
					west = self.out_gt[0] + x*self.tilesize*pixelsizex
					east = west + self.tilesize*pixelsizex
					south = self.ominy + y*self.tilesize*pixelsizex
					north = south + self.tilesize*pixelsizex
					if not self.isepsg4326:
						# Transformation to EPSG:4326 (WGS84 datum)
						west, south = self.ct.TransformPoint(west, south)[:2]
						east, north = self.ct.TransformPoint(east, north)[:2]
					return south, west, north, east

				self.tileswne = rastertileswne
			else:
				self.tileswne = lambda x, y, z: (0,0,0,0)
					

    def get_image(self, level):
        """Returns the bitmap image at the given level."""
        assert 0 <= level and level < self.descriptor.num_levels, 'Invalid pyramid level'
        width, height = self.descriptor.get_dimensions(level)
	# Scaling by PIL (Python Imaging Library) - improved Lanczos
        array = numpy.zeros((self.descriptor.width, self.descriptor.height, 3), numpy.uint8)
        myNDVs=numpy.zeros((self.descriptor.width, self.descriptor.height,1), numpy.uint8)

        for i in range(3):
            array[:,:,i] = gdalarray.BandReadAsArray(self.in_ds.GetRasterBand(i+1), 0, 0, self.descriptor.width, self.descriptor.height)


        ndmask=(array[:,:,0]==self.in_nodata[0]) & (array[:,:,1]==self.in_nodata[1]) & (array[:,:,2]==self.in_nodata[2])
        ndx = (ndmask==1)
        array[ndx,0:3]=self.clearcolor
        im = PIL.Image.fromarray(array, 'RGB') # Always four bands
        # don't transform to what we already have
        if self.descriptor.width == width and self.descriptor.height == height:
            return im
        if (self.resize_filter is None) or (self.resize_filter not in RESIZE_FILTERS):
            return im.resize((width, height), PIL.Image.ANTIALIAS)
        return im.resize((width, height), RESIZE_FILTERS[self.resize_filter])

    def tiles(self, level):
        """Iterator for all tiles in the given level. Returns (column, row) of a tile."""
        columns, rows = self.descriptor.get_num_tiles(level)
        for column in xrange(columns):
            for row in xrange(rows):
                yield (column, row)
        
    def create(self, source, destination):
        """Creates Deep Zoom image from source file and saves it to destination."""
        self.input = source 
        # Opening and preprocessing of the input file
#	self.options.profile = 'raster'
        self.open_input()

        #self.image = PIL.Image.open(safe_open(source))
        width = self.out_ds.RasterXSize
        hieght = self.out_ds.RasterYSize # self.image.size
        self.descriptor = DeepZoomImageDescriptor(width=self.width,
                                                  height=self.height,
                                                  tile_size=self.tile_size,
                                                  tile_overlap=self.tile_overlap,
                                                  tile_format=self.tile_format)
        # Create tiles
        image_files = _get_or_create_path(_get_files_path(destination))
        for level in xrange(self.descriptor.num_levels):
            level_dir = _get_or_create_path(os.path.join(image_files, str(level)))
            level_image = self.get_image(level)
            for (column, row) in self.tiles(level):
                bounds = self.descriptor.get_tile_bounds(level, column, row)
                tile = level_image.crop(bounds)
                format = self.descriptor.tile_format
                tile_path = os.path.join(level_dir,
                                         '%s_%s.%s'%(column, row, format))
                tile_file = open(tile_path, 'wb')
                if self.descriptor.tile_format == 'jpg':
                    jpeg_quality = int(self.image_quality * 100)
                    tile.save(tile_file, 'JPEG', quality=jpeg_quality)
                else:
                    tile.save(tile_file)
        # Create descriptor
        file2 = open(os.path.dirname(destination)+'/seadragon-min.js','w')
        file2.write(bz2.decompress(base64.decodestring(jsmin_str_basebz2)))
        file2.close()
        self.descriptor.save(destination)
        file = open(os.path.dirname(destination)+'/sample.html', 'w')

        htmlstr="""<!doctype html>
        <html>
        <head>
            <title>Seadragon Ajax - Sample Page</title>
            <style>
                html {
                    font-family: Verdana, sans-serif;
                }
                #container {
                    width: 500px;
                    height: 400px;
                    background-color: black;
                    border: 1px solid black;
                    color: white;   /* text color for messages */
                }
            </style>
        </head>
        <body>
            <h1>Seadragon Ajax</h1>
            <div id="container"></div>
            <script type="text/javascript" src="seadragon-min.js"></script>
            <script>
                var viewer = new Seadragon.Viewer("container");
                viewer.openDzi("%s");
            </script>
        </body>
        </html>""" % os.path.basename(destination)
        file.write(htmlstr)
        file.close()
class CollectionCreator(object):
    """Creates Deep Zoom collections."""
    def __init__(self, image_quality=0.8, tile_size=256,
                 max_level=7, tile_format='jpg', copy_metadata=False):
        self.image_quality = image_quality
        self.tile_size = tile_size
        self.max_level = max_level
        self.tile_format = tile_format
        # TODO
        self.copy_metadata = copy_metadata

    def create(self, images, destination):
        """Creates a Deep Zoom collection from a list of images."""
        collection = DeepZoomCollection(destination,
                                        image_quality=self.image_quality,
                                        max_level=self.max_level,
                                        tile_size=self.tile_size,
                                        tile_format=self.tile_format)
        for image in images:
            collection.append(image)
        collection.save()

################################################################################

def retry(attempts, backoff=2):
    """Retries a function or method until it returns or
    the number of attempts has been reached."""

    if backoff <= 1:
        raise ValueError('backoff must be greater than 1')

    attempts = int(math.floor(attempts))
    if attempts < 0:
        raise ValueError('attempts must be 0 or greater')

    def deco_retry(f):
        def f_retry(*args, **kwargs):
            last_exception = None
            for _ in xrange(attempts):
                try:
                    return f(*args, **kwargs)
                except Exception as exception:
                    last_exception = exception
                    time.sleep(backoff**(attempts + 1))
            raise last_exception
        return f_retry
    return deco_retry

def _get_or_create_path(path):
    if not os.path.exists(path):
        os.makedirs(path)
    return path

def _clamp(val, min, max):
    if val < min:
        return min
    elif val > max:
        return max
    return val

def _get_files_path(path):
    return os.path.splitext(path)[0] + '_files'

def _remove(path):
    os.remove(path)
    tiles_path = _get_files_path(path)
    shutil.rmtree(tiles_path)

@retry(6)
def safe_open(path):
    return StringIO.StringIO(urllib.urlopen(path).read())

################################################################################

def main():
    parser = optparse.OptionParser(usage='Usage: %prog [options] filename')

    parser.add_option('-d', '--destination', dest='destination',
                      help='Set the destination of the output.')
    parser.add_option('-s', '--tile_size', dest='tile_size', type='int',
                      default=254, help='Size of the tiles. Default: 254')
    parser.add_option('-f', '--tile_format', dest='tile_format',
                      default=DEFAULT_IMAGE_FORMAT, help='Image format of the tiles (jpg or png). Default: jpg')
    parser.add_option('-o', '--tile_overlap', dest='tile_overlap', type='int',
                      default=1, help='Overlap of the tiles in pixels (0-10). Default: 1')
    parser.add_option('-q', '--image_quality', dest='image_quality', type='float',
                      default=0.8, help='Quality of the image output (0-1). Default: 0.8')
    parser.add_option('-r', '--resize_filter', dest='resize_filter', default=DEFAULT_RESIZE_FILTER,
                      help='Type of filter for resizing (bicubic, nearest, bilinear, antialias (best). Default: antialias')
    parser.add_option('-c', '--clear-color', dest='clearcolor', default=None)

    (options, args) = parser.parse_args()

    if not args:
        parser.print_help()
        sys.exit(1)

    source = args[0]

    if not options.destination:
        if os.path.exists(source):
            options.destination = os.path.splitext(source)[0] + '.dzi'
        else:
            options.destination = os.path.splitext(os.path.basename(source))[0] + '.dzi'
    if options.resize_filter and options.resize_filter in RESIZE_FILTERS:
        options.resize_filter = RESIZE_FILTERS[options.resize_filter]

    creator = ImageCreator(tile_size=options.tile_size,
                           tile_format=options.tile_format,
                           image_quality=options.image_quality,
                           resize_filter=options.resize_filter,
                           clearcolor=options.clearcolor)
    creator.create(source, options.destination)

if __name__ == '__main__':
    main()

