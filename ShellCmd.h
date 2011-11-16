#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h> 
#include <stdlib.h>
#include <vector>
#include "stereo_cells.hpp"
using namespace std;
class ShellCmd{
public:
    ShellCmd(const char *basepath,float simp_mult,const char *pos_simp_log_dir,const char *cwd,const char *aggdir,const char *dicedir,int num_threads):basepath(basepath),simp_mult(simp_mult),pos_simp_log_dir(pos_simp_log_dir),cwd(cwd),aggdir(aggdir),dicedir(dicedir),num_threads(num_threads){};
    void pos_dice(vector<Cell_Data<Stereo_Pose_Data> >cells,float eps,bool run=true);
    void write_setup(void);
    void write_generic(string filename, string cmdfile,string cmdname="Cmds",const vector<string> *precmds =NULL, const vector<string> *postcmds=NULL,int thread_override=0,std::string custom="");
    void pos_simp_cmd(bool run);
    void pos_simp_cmd2(bool run);
    std::string generateMergeAndCleanCmd(std::vector<Cell_Data<Stereo_Pose_Data> >vrip_cells,std::string basename,std::string outname,double vrip_res,int lod=-1);

    const char *basepath;
    float simp_mult;
    const char *pos_simp_log_dir;

    const char *cwd,*aggdir,*dicedir;
    int num_threads;
};

void save_bbox_frame (const osg::BoundingBox &bb, FILE * fptr);
std::string createFileCheckPython(std::string cmd,std::string cwd,std::vector<std::string> files,std::string output,int size);
