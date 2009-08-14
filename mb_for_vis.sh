#!/bin/bash
EXISTS=`ls $1/*.gsf`
EXISTS2=`ls $PWD/mb_grd/grdfiles/*.grd`
if [ -n "$EXISTS" ]; then
    GSF_FLAG=
else
  GSF_FLAG="-GSF -CLN"
fi
if [ "$#" -gt 3 ]; then
    START_STRING="-START $4"
fi 

if [ "$#" -gt 4 ]; then
    STOP_STRING="-STOP $5"
fi 

if [ "$#" -gt 5 ]; then
    BKSTR="-HIALT -BK $6"
fi

if [ -n "$EXISTS2" ]; then
    echo "Allready have cached grd files delete mb_grd if you want to regen"
else
#    echo "y\n" | mbm_processDT.sh $GSF_FLAG -POS  -UNGRD -E $2/$2/m -SPLINE_DIST $3 -O $PWD/mb_grd $START_STRING $STOP_STRING $BKSTR $1
    mbm_processDT.sh  -E $2/$2/m -SPLINE_DIST $3 -O $PWD/mb_grd $START_STRING $STOP_STRING $BKSTR $1
fi
#-NP 
