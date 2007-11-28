#!/bin/sh

STEREO_CFG=../seabed_localisation/ningaloo_stereo.cfg
POSE_FILE=ng06_pose_est.data
IMAGE_DIR=~/data/ningaloo/ng_06_night_grid/ImagesEqualised/


COMMAND1="./threadedStereo $STEREO_CFG $POSE_FILE -f  $IMAGE_DIR --confply -d -t 4 --split 100 --dicevol 10" 
COMMAND2="./genTex $STEREO_CFG -f $IMAGE_DIR -t 4"


$COMMAND1
$COMMAND2

cp -rf mesh mesh-ng06-split100-dicevol10

echo COMMANDS:
echo $COMMAND1
echo $COMMAND2
