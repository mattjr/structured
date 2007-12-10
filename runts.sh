#!/bin/bash
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
SKIPVRIP=0
SINGLEVRIP=
NUMPOSE=
while getopts "rh:s:p:v:d:" OPTION
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
	 d)
	     BASEDIR=$OPTARG
             ;;
         ?)
             usage
             exit
             ;;
     esac
done

LOGDIR="/mnt/shared/log-tex"
BASEPATH=`dirname $0`
SUBVOLDIR="/mnt/shared/mesh-agg"
MESHCACHE=$BASEDIR/cache-mesh/
SHAREBASE="/mnt/shared/"
POSEFILE=$BASEDIR/pose_est.data
IMGDIR=$BASEDIR/img/
CACHEDIMGDIR=$BASEDIR/cache-tex/
STEREOCONFIG=$BASEDIR/stereo.cfg
DATADIR=$BASEDIR
MBDIR="."
OUTPUTLOC="/mnt/shared/"
if [ -z $NUMPOSE ]; then
    NUMPOSE=$((`grep PR_ $POSEFILE |wc -l` - 1))
fi
if [ $PWD = "/mnt/shared" ]; then
    echo "Clearing dirs"
    rm -rf $PWD/mesh-agg/
    rm -rf $PWD/mesh/
fi
mkdir -m777 -p $PWD/mesh-agg/
mkdir -m777 -p $PWD/mesh/

if [ $SKIPVRIP -eq 0 ] 
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
	$BASEPATH/threadedStereo $BASEDIR --single-run  0 $NUMPOSE -t 2
    fi
fi
#bash tscmds
echo "Creating $NUMPOSE meshes"

echo -e "#!/bin/bash\nOUTDIR=\$PWD\nVRIP_HOME=$BASEPATH/vrip\nexport VRIP_DIR=\$VRIP_HOME/src/vrip/\nPATH=\$PATH:\$VRIP_HOME/bin:$BASEPATH/tridecimator\ncd $MESHCACHE\n$BASEPATH/vrip/bin/vripxftrans meshlist.txt -subvoldir $SUBVOLDIR -n $NUMPOSE" > runvrip.sh
echo -e "cd $SUBVOLDIR\nfind . -name 'surface-*.ply' | sort  |  sed 's_.*/__' | awk '{print \$0  \" 0.033 1\" }' > surface.txt\nfind $SUBVOLDIR -name 'mb-*.ply' | sort  |  sed 's_.*/__' | awk '{print \$0  \" 0.1 0\" }' >> surface.txt" >> runvrip.sh

echo -e "cd $SUBVOLDIR\n$BASEPATH/vrip/bin/pvrip1 \$OUTDIR/mesh-agg/auto.vri \$OUTDIR/mesh-agg/total.ply surface.txt surface.txt  0.033 1000M ~/loadlimit -logdir /mnt/shared/log -rampscale 300 -subvoldir $SUBVOLDIR -nocrunch -passtovrip -use_bigger_bbox -dec -meshcache $SUBVOLDIR\n" >> runvrip.sh
chmod 0777  runvrip.sh 
bash runvrip.sh



SUBVOL=25;
EPS=0.1
echo -e "#!/bin/bash\necho 'Dicing...\\n'\nVRIP_HOME=$BASEPATH/vrip\nexport VRIP_DIR=\$VRIP_HOME/src/vrip/\nPATH=\$PATH:\$VRIP_HOME/bin\nDICEDIR=$SHAREBASE/mesh-agg/\nmkdir -p \$DICEDIR\ncd \$DICEDIR\n$BASEPATH/vrip/bin/plydice -writebbox range.txt -writebboxall bbtmp.txt -dice $SUBVOL $EPS diced total.ply | tee diced.txt\n" >  diced.sh

 echo -e "rm -f gentexcmds\nNUMDICED=\$((\`wc -l diced.txt | awk '{ print \$1 }'\` - 1))\nfor i in \`seq 0 \$NUMDICED\`;\ndo\n\techo \"setenv DISPLAY :0.0;cd \$DICEDIR/..;$BASEPATH/genTex $STEREOCONFIG -f $CACHEDIMGDIR --single-run \$i\" >> gentexcmds\ndone\n" >> diced.sh 

echo -e "cd $SUBVOLDIR\n$BASEPATH/vrip/bin/vripdicebbox surface.txt \$DICEDIR\n" >> diced.sh
	
echo -e "cd \$DICEDIR\n$BASEPATH/vrip/bin/loadbalance ~/loadlimit gentexcmds -logdir $LOGDIR\n" >> diced.sh
chmod 0777 diced.sh 
bash diced.sh
echo -e "#!/bin/bash\necho 'LODGen...'\ncd $OUTPUTLOC;$BASEPATH/lodgen \n" > lodgen.sh
chmod 0777 lodgen.sh
bash lodgen.sh
