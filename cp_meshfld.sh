#!/bin/bash
if [ ! -d "$1" ]; then
    echo "$1 Does not exist bailing"
    exit -1
fi
if [ ! -d "$2" ]; then
    echo "$2 Does not exist creating"
    mkdir $2
fi
args="-p -d"
FILES=("stereo_pose_est.data" "localiser.cfg" "stereo.calib" "img")
for f in "${FILES[@]}"
do
    echo "Copying $f"
    cp $args $1/$f $2/$f

done
echo "Copying mesh.cfg"
cp `dirname $0`/mesh.cfg $2/

