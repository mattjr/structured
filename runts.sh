#!/bin/bash
COUNT=0
LOGDIR="/mnt/shared/log-tex"
BASEPATH="/home/mkj/cvs/threadedStereo/"
SUBVOLDIR="/mnt/shared/svol"
SHAREBASE="/mnt/shared/"

NUMPOSE=$((`grep PR_ \$1 |wc -l  | awk '{ print $1 }'` - 1))
# bash until loop
LAST=0
if [ $# -eq 5 ];then
    NUMPOSE=$5
fi

echo "Creating Stereo Meshes for $NUMPOSE frame pairs"
rm -f tscmds
while [ $COUNT -lt $NUMPOSE ]; do
check=$(($COUNT % $2));   
    if [ $check -eq 0 ]  && [ "$COUNT" -gt 0 ];then
        echo "cd $3;$BASEPATH/threadedStereo $BASEPATH/ng2_stereo.cfg -f /media//coral/r20071004_013437_gbr_05_noggin_grids/i20071004_013437_cv/ $1 --single-run $LAST $COUNT" >> tscmds
	LAST=$COUNT
    fi
        let COUNT=COUNT+1
done
if [ $check -gt 0 ];then
    final=$COUNT
    echo "cd $3;$BASEPATH/threadedStereo $BASEPATH/ng2_stereo.cfg -f /media//coral/r20071004_013437_gbr_05_noggin_grids/i20071004_013437_cv/ $1 --single-run $LAST $final" >> tscmds
fi
~/cvs/threadedStereo/vrip/bin/loadbalance ~/loadlimit tscmds -logdir /mnt/shared/log-ts/  -noxload
TARGET_DIR=/mnt/shared/svol
for i in `find $4 -name '*.ply'`; do
    cp "$i" $TARGET_DIR/
    echo  cp "$i" $TARGET_DIR/
done

#bash tscmds
find /mnt/shared/svol -name 'surface-*.ply' | sort  |  sed 's_.*/__' | awk '{print "bmesh " $0  " 0.033 1" }' > /mnt/shared/svol/surface.conf
find /mnt/shared/svol -name 'mb-*.ply' | sort  |  sed 's_.*/__' | awk '{print "bmesh " $0  " 0.1 0" }' >> /mnt/shared/svol/surface.conf

echo -e "#!/bin/bash\nOUTDIR=\$PWD\nVRIP_HOME=$BASEPATH/vrip\nexport VRIP_DIR=\$VRIP_HOME/src/vrip/\nPATH=\$PATH:\$VRIP_HOME/bin\ncd $SUBVOLDIR\n" > runvrip.sh

echo -e "$BASEPATH/vrip/bin/pvrip1 auto.vri \$OUTDIR/mesh-agg/total.ply surface.conf surface.conf  0.033 100M ~/loadlimit -logdir /mnt/shared/log -rampscale 300 -subvoldir $SUBVOLDIR -nocrunch -passtovrip -use_bigger_bbox\n" >> runvrip.sh
chmod 0777  runvrip.sh 
./runvrip.sh
STEREOCONFIG=$BASEPATH/ng2_stereo.cfg
IMGDIR=/media//coral/r20071004_013437_gbr_05_noggin_grids/i20071004_013437_cv/

SUBVOL=25;
EPS=0.1
echo -e "#!/bin/bash\necho 'Dicing...\\n'\nVRIP_HOME=$BASEPATH/vrip\nexport VRIP_DIR=\$VRIP_HOME/src/vrip/\nPATH=\$PATH:\$VRIP_HOME/bin\nDICEDIR=$SHAREBASE/mesh-agg/\nmkdir -p \$DICEDIR\ncd \$DICEDIR\n$BASEPATH/vrip/bin/plydice -writebbox range.txt -writebboxall bbtmp.txt -dice $SUBVOL $EPS diced total.ply | tee diced.txt\n" >  diced.sh

	
echo -e "rm -f gentexcmds\nNUMDICED=\$((\`wc -l diced.txt | awk '{ print \$1 }'\` - 1))\nfor i in \`seq 0 \$NUMDICED\`;\ndo\n\techo \"setenv DISPLAY :0.0;cd \$DICEDIR/..;$BASEPATH/genTex $STEREOCONFIG -f $IMGDIR --single-run \$i\" >> gentexcmds\ndone\n" >> diced.sh 

echo -e "cd $SUBVOLDIR\n$BASEPATH/vrip/bin/vripdicebbox surface.conf \$DICEDIR\n" >> diced.sh
	
echo -e "cd \$DICEDIR\n$BASEPATH/vrip/bin/loadbalance ~/loadlimit gentexcmds -logdir $LOGDIR\n" >> diced.sh
chmod 0777 diced.sh 
./diced.sh
echo -e "#!/bin/bash\necho 'LODGen...'\ncd $3;$BASEPATH/lodgen \n" > lodgen.sh
chmod 0777 lodgen.sh
./lodgen.sh
