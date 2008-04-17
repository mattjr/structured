#include "ShellCmd.h"

void ShellCmd::pos_simp_cmd(bool run){
  
  FILE *dicefp=fopen("./pos_simp.sh","w+");
  fprintf(dicefp,"#!/bin/bash\necho -e 'Simplifying...\\n'\nBASEPATH=%s/\nVRIP_HOME=$BASEPATH/vrip\nMESHAGG=$PWD/mesh-agg/\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\nPATH=$PATH:$VRIP_HOME/bin\nRUNDIR=$PWD\nDICEDIR=$PWD/mesh-pos/\nmkdir -p $DICEDIR\ncd $MESHAGG\n",basepath);
  fprintf(dicefp,"cd $DICEDIR\n");
  fprintf(dicefp,"NUMDICED=`wc -l diced.txt |cut -f1 -d\" \" `\n"  
	  "REDFACT=(0.01 %f %f)\n",0.01,0.01);
  if(dist_run){
  
    fprintf(dicefp, "LOGDIR=%s\n"
	    "if [[ -d $LOGDIR ]] ; then\n"
	    "find $LOGDIR -name 'loadbal*' | xargs rm &>/dev/null\nfi\n"
	    "if [[ -d $DICEDIR ]] ; then\n"
	    "find $DICEDIR -name '*lod*' | xargs rm &> /dev/null\nfi\n",pos_simp_log_dir);
  }
    fprintf(dicefp, 
	    "rm -f simpcmds\n"
	    "rm -f valid.txt\n"
	  "cat diced.txt | while read MESHNAME; do\n"
	  "FACES=`plyhead $MESHNAME | grep \"element face\" | cut -f 3 -d\" \"`\n"
	    "if [ $FACES == 0 ]; then\n continue;\n fi\n"
	    "echo $MESHNAME >> valid.txt\n"
	    "SIMPCMD=\"cd $DICEDIR/\" \n"
	    "\tfor f in `echo {0..2}`\n"
	    "\tdo\n"
	    "\t\tif [ $f == 0 ]; then\n"
	    "\t\t\tNEWNAME=`echo $MESHNAME | sed s/.ply/-lod$f.ply/g`\n"
	  "FLIPCMD=\"-F\"\n"
	  "\t\telse\n"
	  "FLIPCMD="
	  "\t\t\tNEWNAME=`echo $MESHNAME | sed s/-lod$(($f - 1 )).ply/-lod$f.ply/g`\n"
	  "\t\tfi\n"
	  "\t\tSIMPCMD=$SIMPCMD\";\"\"$BASEPATH/tridecimator/tridecimator $MESHNAME $NEWNAME ${REDFACT[$f]}r -b2.0 $FLIPCMD >& declog-$MESHNAME.txt ;chmod 0666 $NEWNAME  \"\n"
	  "MESHNAME=$NEWNAME\n"
	  "\tdone\n"
	  "echo $SIMPCMD >> simpcmds\n"
	  "done\n");
  
  
  if(dist_run){
    fprintf(dicefp,"cd $DICEDIR\n"
	    "time $BASEPATH/vrip/bin/loadbalance ~/loadlimit simpcmds -logdir $LOGDIR\n");
  } else {
    fprintf(dicefp,"time %s/runtp.py simpcmds\n",basepath);
  }

  fprintf(dicefp,"cat valid.txt | xargs plybbox > range.txt\n");
  fchmod(fileno(dicefp),0777);
  fclose(dicefp);
  if(run)
    system("./pos_simp.sh");
  
}
