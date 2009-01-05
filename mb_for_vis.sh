#!/bin/bash
EXISTS=`ls $PWD/mb_grd/*.gsf`
EXISTS2=`ls $PWD/mb_grd/grdfiles/*.grd`
if [ -n "$EXISTS" ]; then
    GSF_FLAG=
else
  GSF_FLAG="-GSF"
fi
if [ -n "$EXISTS2" ]; then
    echo "Allready have cached grd files delete mb_grd if you want to regen"
else
    echo "y\n" | ~/cvs/seabed_localisation/mbm_processDT.sh $1  $GSF_FLAG -UNGRD -NP -E 0.1/0.1/m -O $PWD/mb_grd -BK $1/background.grd
fi
