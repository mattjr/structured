#!/bin/sh
./threadedStereo ng2_stereo.cfg ~/pose_est.data -f  /media/reef/r20070523_061146_ng2_13_legWestSouthEst/i20070523_061146_cv/ -n 2 --confply
/genTex ng2_stereo.cfg ~/pose_est.data -f  /media/reef/r20070523_061146_ng2_13_legWestSouthEst/i20070523_061146_cv/
