#!/bin/bash
echo "y\n" | ~/cvs/seabed_localisation/mbm_processDT.sh $1  -UNGRD -NP -E 0.1/0.1/m -O $PWD/mb_grd 
