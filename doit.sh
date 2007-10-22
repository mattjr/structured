#!/bin/sh
./threadedStereo ng2_stereo.cfg ~/data/m13/pose_est.data -f  ~/data/m13/ng2-13/ -n 1 --confply
./genTex ng2_stereo.cfg ~/data/m13/pose_est.data -f   ~/data/m13/ng2-13
