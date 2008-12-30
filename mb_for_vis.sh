#!/bin/bash
EXISTS=`ls $PWD/mb_grd/*.gsf`
if [ -n "$EXISTS" ]; then
    GSF_FLAG=
else
  GSF_FLAG="-GSF"
fi
echo "y\n" | ~/cvs/seabed_localisation/mbm_processDT.sh $1  $GSF_FLAG -GRD2XYZ -NP -E 0.1/0.1/m -O $PWD/mb_grd 
