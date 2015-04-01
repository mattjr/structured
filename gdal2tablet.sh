#!/bin/bash
mkdir ipad
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
python $DIR/gdal_rastertotrn.py $1 ipad/tex-total.obj
$DIR/vcgapps/bin/texturedDecimator ipad/tex-total.obj ipad//ipad.ply 163810 -Oy -V -P ;/home/auv/git/structured/vcgapps/bin/splitForTablet ipad/ipad.ply -uipad/octree -s65534;cd ipad
n=`cat octree.txt`
nm=$[$n-1]
for i in `seq 0 $nm`;do
	filesuf=`printf "%04d" $i`
	$DIR/generateOctreeFromObj.py -o=vtex-$filesuf.octree octree-$filesuf.obj
done
gdalwarp -ts 8192 $1 image.tif
$DIR/generateVirtualTextureTiles.py image.tif
