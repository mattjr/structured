#!/bin/bash
mkdir ipad
CUR=$PWD
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
python $DIR/gdal_rastertotrn.py $1 ipad/tex-total.ply
$DIR/vcgapps/bin/texturedDecimator ipad/tex-total.ply ipad//ipad.ply 163810 -Oy -V -P ;/home/auv/git/structured/vcgapps/bin/splitForTablet ipad/ipad.ply -uipad/octree -s65534;cd ipad
n=`cat octree.txt`
nm=$[$n-1]
for i in `seq 0 $nm`;do
	filesuf=`printf "%04d" $i`
	$DIR/generateOctreeFromObj.py -o=vtex-$filesuf.octree octree-$filesuf.obj
done
#cd ipad
#$DIR/generateOctreeFromObj.py -o=vtex-0.octree tex-total.obj
gdalwarp -ts 8128 8128 ../$2 vtex.tif
$DIR/generateVirtualTextureTiles.py -b=1 -f=jpg vtex.tif
mkdir $3
ln -sf $PWD/vtex $PWD/$3/
n=`cat octree.txt`
nm=$[$n-1]
for i in `seq 0 $nm`;do
        filesuf=`printf "%04d" $i`
        ln -sf $PWD/vtex-$filesuf.octree $PWD/$3/m-$filesuf.octree
done
ln -sf $PWD/octree.txt $PWD/$3/cnt
gdalwarp ../$2 -ts 256 256 out.tif
convert $PWD/out.tif $3/m.jpg
rm -f $3/m.xml
tar cvfh $3.tar $3
bash $DIR/getmeta.sh $3 > $3/m.xml
tar rvf $3.tar $3/m.xml
echo -n $3 > tarname
