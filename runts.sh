#!/bin/bash
LOGDIR="/mnt/shared/log-tex"
BASEPATH=`dirname $0`
SUBVOLDIR="/mnt/shared/mesh-agg"
MESHCACHE=$1/cache-mesh/
SHAREBASE="/mnt/shared/"
POSEFILE=$1/pose_est.data
IMGDIR=$1/img/
CACHEDIMGDIR=$1/cache-tex/
STEREOCONFIG=$1/stereo.cfg
DATADIR=$1
MBDIR="."
OUTPUTLOC="/mnt/shared/"
NUMPOSE=$((`grep PR_ \$1 |wc -l $POSEFILE | awk '{ print $1 }'` - 1))
mkdir -p $PWD/mesh-agg/
mkdir -p $PWD/mesh/
chown 0777  $PWD/mesh-agg/
chown 0777  $PWD/mesh/
usage()
{
cat << EOF
usage: $0 options

This script run the test1 or test2 over a machine.

OPTIONS:
   -h      Show this message
   -t      Test type, can be ‘test1′ or ‘test2′
   -r      Server address
   -p      Server root password
   -v      Verbose
EOF
}
SKIPVRIP=
SINGLEVRIP=
while getopts "ht:s:p:v" OPTION
do
     case $OPTION in
         h)
             usage
             exit 1
             ;;
         s)
             SINGLEVRIP=1
             ;;
         r)
             SKIPVRIP=1
             ;;
         p)
             NUMPOSE=$OPTARG
             ;;
         v)
             VERBOSE=1
             ;;
         ?)
             usage
             exit
             ;;
     esac
done



if [ $SKIPVRIP ] 
then

    echo "Creating Stereo Meshes for $NUMPOSE frame pairs"
    if [ $SINGLEVRIP ] 
    then
# bash until loop
	LAST=0
	COUNT=0
	SPLIT=500
	
	rm -f tscmds
	while [ $COUNT -lt $NUMPOSE ]; do
	    check=$(($COUNT % $SPLIT));   
	    if [ $check -eq 0 ]  && [ "$COUNT" -gt 0 ];then
		echo "cd $OUTPUTLOC;$BASEPATH/threadedStereo $DATADIR --single-run $LAST $COUNT" >> tscmds
		LAST=$COUNT
	    fi
            let COUNT=COUNT+1
	done
	if [ $check -gt 0 ];then
	    final=$COUNT
	    echo "cd $OUTPUTLOC;$BASEPATH/threadedStereo $DATADIR --single-run $LAST $final" >> tscmds
	fi
	$BASEPATH/vrip/bin/loadbalance ~/loadlimit tscmds -logdir /mnt/shared/log-ts/  -noxload
    TARGET_DIR=/mnt/shared/mesh-agg
    for i in `ls $MBDIR/mb*.ply`; do
	cp "$i" $TARGET_DIR/
	echo  cp "$i" $TARGET_DIR/
    done
    else
	echo "Single Vrip"
	$BASEPATH/threadedStereo $1 --single-run  0 $NUMPOSE -t 2
    fi
fi
#bash tscmds
find /mnt/shared/mesh-agg -name 'surface-*.ply' | sort  |  sed 's_.*/__' | awk '{print $0  " 0.033 1" }' > /mnt/shared/mesh-agg/surface.txt
find /mnt/shared/mesh-agg -name 'mb-*.ply' | sort  |  sed 's_.*/__' | awk '{print $0  " 0.1 0" }' >> /mnt/shared/mesh-agg/surface.txt

echo -e "#!/bin/bash\nOUTDIR=\$PWD\nVRIP_HOME=$BASEPATH/vrip\nexport VRIP_DIR=\$VRIP_HOME/src/vrip/\nPATH=\$PATH:\$VRIP_HOME/bin:$BASEPATH/tridecimator\ncd $MESHCACHE\n" > runvrip.sh

echo -e "$BASEPATH/vrip/bin/pvrip1 \$OUTDIR/mesh-agg/auto.vri \$OUTDIR/mesh-agg/total.ply surface.txt surface.txt  0.033 1000M ~/loadlimit -logdir /mnt/shared/log -rampscale 300 -subvoldir $SUBVOLDIR -nocrunch -passtovrip -use_bigger_bbox -dec -meshcache $MESHCACHE\n" >> runvrip.sh
chmod 0777  runvrip.sh 
./runvrip.sh



SUBVOL=25;
EPS=0.1
echo -e "#!/bin/bash\necho 'Dicing...\\n'\nVRIP_HOME=$BASEPATH/vrip\nexport VRIP_DIR=\$VRIP_HOME/src/vrip/\nPATH=\$PATH:\$VRIP_HOME/bin\nDICEDIR=$SHAREBASE/mesh-agg/\nmkdir -p \$DICEDIR\ncd \$DICEDIR\n$BASEPATH/vrip/bin/plydice -writebbox range.txt -writebboxall bbtmp.txt -dice $SUBVOL $EPS diced total.ply | tee diced.txt\n" >  diced.sh

	
echo -e "rm -f gentexcmds\nNUMDICED=\$((\`wc -l diced.txt | awk '{ print \$1 }'\` - 1))\nfor i in \`seq 0 \$NUMDICED\`;\ndo\n\techo \"setenv DISPLAY :0.0;cd \$DICEDIR/..;$BASEPATH/genTex $STEREOCONFIG -f $IMGDIR --single-run \$i\" >> gentexcmds\ndone\n" >> diced.sh 

echo -e "cd $MESHCACHE\n$BASEPATH/vrip/bin/vripdicebbox surface.txt \$DICEDIR\n" >> diced.sh
	
echo -e "cd \$DICEDIR\n$BASEPATH/vrip/bin/loadbalance ~/loadlimit gentexcmds -logdir $LOGDIR\n" >> diced.sh
chmod 0777 diced.sh 
./diced.sh
echo -e "#!/bin/bash\necho 'LODGen...'\ncd $OUTPUTLOC;$BASEPATH/lodgen \n" > lodgen.sh
chmod 0777 lodgen.sh
./lodgen.sh
