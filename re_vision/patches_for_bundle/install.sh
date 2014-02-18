#!/bin/bash
#
# 3D Model Creator
# install.sh
# 2010 Dorian Galvez-Lopez. University of Zaragoza
#
#
# Installs all the tools to create 3D models
#

# exit on error
set -e

. config.sh

echo "* Installing the 3D Model Creator" 
echo "+ Bundler path: $BUNDLER_BASE"
echo "+ PMVS path: $PMVS_BASE"

PATCH_DIR=.
CONTRAPTION_DIR=SurfContraption
SCALE_DIR=ScaleContraption
BACKGROUND_DIR=BackgroundContraption
CUR_DIR=`pwd`

echo "-- Applying patches to bundle"
cp $PATCH_DIR/Bundle2PMVS_cpp_patch.diff "$BUNDLER_SRC"

cd $BUNDLER_SRC
for s in `ls *.diff`; do
 patch < $s
done

echo "-- Building bundler"
make clean
cd $BUNDLER_BASE
make

cd $CUR_DIR

#echo 
#echo "-- Done. Run ./createModelUpToScale.sh <img dir> to create a model"


