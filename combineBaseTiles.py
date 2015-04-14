#!/usr/bin/env python
#
#  mergeVirtualTextureTiles.py
#  VirtualTexturing
#
#  Created by Julian Mayer on 19.01.10.
#  Copyright (c) 2010 A. Julian Mayer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitationthe rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 

import sys, os, math, shutil, math

def print_exit(s): print s;	sys.exit()

try:		from PIL import Image
except:		print_exit("Error: Python Imaging Library or later not found")

border = 0
levels = 0
output = ""
inputfolders = []


try:
	for i in range(1, len(sys.argv)):
		if sys.argv[i].startswith("-b="):	border = int(sys.argv[i][3:])
		elif sys.argv[i].startswith("-l="):	levels = int(sys.argv[i][3:])

		else:
			inputfolders.append(sys.argv[i])
except e:
	print e
	print_exit("""Usage: %s -o=<output_folder> -l=<levels_the_inputfolders_have> -b=<border> input_folders (4, 16 or 64)""")

#if (levels == 0 or output == "" or (len(inputfolders) != 4 and len(inputfolders) != 16 and len(inputfolders) != 64)):
#	print_exit("""Usage: %s -o=<output_folder> -l=<levels_the_inputfolders_have> -b=<border> input_folders (4, 16 or 64)""")



maxTiles = 2**(levels - 1)
sideLen = maxTiles
tilesStr = "/tiles_b" + str(border) + "_level"
tileSize = Image.open(inputfolders[0] + tilesStr + "0/tile_0_0_0.jpg").size[0] - border*2


#fixup borders in existing files
maxTiles = 2**(levels + 2)
level=0

#generate new levels
size = tileSize*sideLen
print size
im = Image.new(mode='RGB', size=(size, size))
levels=1
for x in range(0, sideLen):
	for y in range(0, sideLen):
		sys.stdout.write('\r%04d/%04d'%(x,sideLen))
		sys.stdout.flush()
		tile = Image.open(inputfolders[0] + tilesStr + str(levels-1) + "/tile_" + str(levels-1)  + "_" + str(x) + "_" + str(y)  + ".jpg")
		im.paste(tile.crop((border, border, tileSize + border, tileSize + border)), (x*tileSize, y*tileSize, (x+1)*tileSize, (y+1)*tileSize))
im.save('output.jpg')