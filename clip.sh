#!/bin/bash
#REGION=147.97312/147.99698/-43.086517/-43.074583
#INC=1.0/1.0/m
PATH=$PATH:~/cvs/threadedStereo/
#~/cvs/threadedStereo/vrip/bin/plycrunch  -d 1.0  mesh-agg/merged.ply > simp.ply
~/cvs/threadedStereo/meshtohmap --m-pix=0.25 --gmt=$HOME/datasets/wa_20/renav20081014_DT/SEABED.localiser.cfg simp.ply vismask.grd
grdmaskgrd  mb_grd/GRD_0.1_0.1_m/20081014_0543-003.gsf.grd vismask.grd mb_grd/GRD_0.1_0.1_m/masked-20081014_0543-003.gsf.grd 2
 grd2ply mb_grd/GRD_0.1_0.1_m/masked-20081014_0543-003.gsf.grd  -S -F $HOME/datasets/wa_20/renav20081014_DT/SEABED.localiser.cfg l.ply
 grd2ply vismask.grd  -S -F $HOME/datasets/wa_20/renav20081014_DT/SEABED.localiser.cfg mask.ply

#~/cvs/threadedStereo/bin/meshtoxy ~/datasets/wa_20/renav20081014_DT/SEABED.localiser.cfg simp.ply simp.xy
#grdmath 1 vismask.grd ISNAN SUB = inv-mask.grd
#gmtset D_FORMAT %.30f
#info=(`grdinfo -C -M inv-mask.grd`)
#REGION="${info[1]}/${info[2]}/${info[3]}/${info[4]}"
#INTER="${info[7]}/${info[8]}"
#echo $REGION
#grdsample -R$REGION  tas20-interp.grd   -Gcut-tas.grd -I$INTER
#grdmath cut-tas.grd cut-tas.grd inv-mask.grd MUL NAN = masked-tas.grd
