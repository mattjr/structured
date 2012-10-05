#include "ShellCmd.h"
#include <sstream>
#include "MemUtils.h"
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

static const char *thrpool="runtasks.py";
static const char *serfile="localserver";

void ShellCmd::write_generic(string filename,string cmdfile,string cmdname,const vector<string> *precmds , const vector<string> *postcmds,int thread_override,string custom){
    FILE *fp=fopen(filename.c_str(),"w");
    fprintf(fp,"#!/usr/bin/python\n");
    fprintf(fp,"import sys\n");
    fprintf(fp,"import os\n");
    fprintf(fp,"import setupts\n");
    fprintf(fp,"setupts.setup(os)\n");
    fprintf(fp,"print 'Running %s...'\n",cmdname.c_str());
    int loc_num_threads=num_threads;
    if(thread_override >0)
        loc_num_threads=thread_override;
    if(precmds){
        for(int i=0; i< (int)precmds->size(); i++)
            fprintf(fp,"os.system('%s')\n",(*precmds)[i].c_str());
    }
    string path=osgDB::getFilePath(filename);
    path = path.size() == 0 ? "." : path;
    string loc_file = (path +"/"+string(serfile));

    if(!osgDB::fileExists(loc_file)){
        FILE *ffp=fopen(loc_file.c_str(),"w");
        fprintf(ffp,"LOCAL %d\n",loc_num_threads);
        fclose(ffp);
    }

    fprintf(fp,"os.system(setupts.basepath +'/%s %s %s %s')\n",
            thrpool,cmdfile.c_str(),loc_file.c_str(),cmdname.c_str());

    if(postcmds){
        for(int i=0; i<(int) postcmds->size(); i++)
            fprintf(fp,"os.system('%s')\n",(*postcmds)[i].c_str());
    }
    fprintf(fp,"%s\n",custom.c_str());

    fchmod(fileno(fp),0777);

    fclose(fp);
}
std::string createFileCheckPython(std::string cmd,std::string cwd,std::vector<std::string> files,std::string output,int size){
    std::stringstream ss;
    ss<<"finlst = []\n";
    ss<< "tmplst = [";
    for(int i=0; i<(int)files.size(); i++)
        ss<<"'"<<cwd+"/"+files[i]<< "',";
    ss<<"]\n";
    ss<<"for i in tmplst:\n";
    ss<<"\tif os.path.isfile(i) and os.path.getsize(i) > " << size << ":\n";
    ss<<"\t\tfinlst.append(i)\n";
    ss<<"if len(finlst) > 0:\n";
    ss<<"\tos.system('"<<cmd<<" '+\" \".join(finlst)+" <<"'"<<output<<"'"<< ")\n";
    ss<<"else:\n";
    ss<<"\tprint 'Not running commmand all files invalid ' + \" \".join(tmplst)\n";

    return ss.str();
}

void ShellCmd::write_setup(void){
    FILE *fp=fopen("setupts.py","w");
    fprintf(fp,"def setup(os):\n");
    fprintf(fp,"\tglobal basepath\n");
    fprintf(fp,"\tbasepath = '%s'\n",basepath);
    fprintf(fp,"\tdicedir =' %s'\n",dicedir);
    fprintf(fp,"\tvrip_path = basepath+'/vrip'\n");
    fprintf(fp,"\tMY_PATH = os.getenv('PATH', 'Error')\n");
    fprintf(fp,"\tos.environ[\"PATH\"] = MY_PATH + ':' + vrip_path + '/bin' +"
            "':'+basepath+'/poisson/'\n");
    fprintf(fp,"\tos.environ[\"BASEPATH\"] = basepath\n");
    fprintf(fp,"\tos.environ[\"VRIP_HOME\"] = vrip_path + '/src/vrip/'\n");
    fprintf(fp,"\tos.popen('mkdir -p ' + dicedir)\n");
    fclose(fp);
}



void ShellCmd::pos_simp_cmd2(bool run){
    int res;
    FILE *dicefp=fopen("./pos_simp.sh","w+");
    fprintf(dicefp,"#!/bin/bash\necho -e 'Simplifying...\\n'\nBASEPATH=%s/\nVRIP_HOME=$BASEPATH/vrip\nMESHAGG=$PWD/mesh-agg/\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\nPATH=$PATH:$VRIP_HOME/bin\nRUNDIR=$PWD\nDICEDIR=$PWD/mesh-pos/\nmkdir -p $DICEDIR\ncd $MESHAGG\n",basepath);
    fprintf(dicefp,"cd $DICEDIR\n");
    fprintf(dicefp,"NUMDICED=`wc -l diced.txt |cut -f1 -d\" \" `\n"
            "REDFACT=(0.01 %f %f)\n",0.1,.40);

    fprintf(dicefp, 
	    "rm -f simpcmds\n"
	    "rm -f valid.txt\n"
            "cat diced.txt | while read MESHNAME; do\n"
            "FACES=`plyhead $MESHNAME | grep \"element face\" | cut -f 3 -d\" \"`\n"
	    "if [ $FACES == 0 ]; then\n continue;\n fi\n"
	    "echo $MESHNAME >> valid.txt\n"
	    "SIMPCMD=\"cd $DICEDIR/\" \n"
	    "\tfor k in `echo {0..2}`\n"
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



    fprintf(dicefp,"%s/%s simpcmds\n",thrpool,basepath);


    fprintf(dicefp,"cat valid.txt | xargs plybbox > range.txt\n");
    fchmod(fileno(dicefp),0777);
    fclose(dicefp);
    if(run)
        res=system("./pos_simp.sh");

}
void ShellCmd::pos_simp_cmd(bool run){
    int res;
    FILE *dicefp=fopen("./pos_simp.sh","w+");
    fprintf(dicefp,"#!/bin/bash\necho -e 'Simplifying...\\n'\nBASEPATH=%s/\nVRIP_HOME=$BASEPATH/vrip\nMESHAGG=$PWD/mesh-agg/\nexport VRIP_DIR=$VRIP_HOME/src/vrip/\nPATH=$PATH:$VRIP_HOME/bin\nRUNDIR=$PWD\nDICEDIR=$PWD/mesh-pos/\nmkdir -p $DICEDIR\ncd $MESHAGG\n",basepath);
    fprintf(dicefp,"cd $DICEDIR\n");
    fprintf(dicefp,"NUMDICED=`wc -l diced.txt |cut -f1 -d\" \" `\n"
            "REDFACT=(0.01 %f %f)\n",0.1,.40);

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



    fprintf(dicefp,"time %s/%s simpcmds\n",thrpool,basepath);


    fprintf(dicefp,"cat valid.txt | xargs plybbox > range.txt\n");
    fchmod(fileno(dicefp),0777);
    fclose(dicefp);
    if(run)
        res=system("./pos_simp.sh");

}

string ShellCmd::generateMergeAndCleanCmd(vector<Cell_Data<Stereo_Pose_Data> >vrip_cells,string basename,string outname,double vrip_res,int lod){
    char tmp100[8096];
    string tcmd;
    float cleanPercentage=0.05;
    tcmd =basepath+string("/vrip/bin/plymerge ");
    for(int i=0; i <(int)vrip_cells.size(); i++){
        if(vrip_cells[i].poses.size() == 0)
            continue;
        if(lod>=0)
            sprintf(tmp100, " %s/%s-%08d-lod%d.ply ",diced_dir,basename.c_str(),i,lod);
        else
            sprintf(tmp100, " %s/%s-%08d.ply ",diced_dir,basename.c_str(),i);
        tcmd+=tmp100;
    }
    if(lod >=0)
        sprintf(tmp100, " > %s/%s-unmerged-lod%d.ply ;",diced_dir,outname.c_str(),lod);
    else
        sprintf(tmp100, " > %s/%s-unmerged.ply ;",diced_dir,outname.c_str());

    tcmd+= tmp100;
    if(lod >=0)
        sprintf(tmp100,"  %s/vcgapps/bin/mergeMesh %s/%s-unmerged-lod%d.ply -tex -thresh %f -out %s/%s-lod%d.ply;",basepath,diced_dir,outname.c_str(),lod,0.9*vrip_res,diced_dir,outname.c_str(),lod);
    else
        sprintf(tmp100,"  %s/vcgapps/bin/mergeMesh %s/%s-unmerged.ply -color -cleansize %f -thresh %f -out %s/%s.ply;",basepath,diced_dir,outname.c_str(),cleanPercentage,0.9*vrip_res,diced_dir,outname.c_str());

    tcmd+=tmp100;

    sprintf(tmp100,"export DISPLAY=:0.0;time %s/vcgapps/bin/shadevis -n128  -f %s/%s.ply >%s/shadevislog.txt;",basepath,diced_dir,outname.c_str(),diced_dir);
    tcmd+=tmp100;


    return tcmd;
}
