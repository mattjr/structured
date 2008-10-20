#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h> 
#include <stdlib.h>
#include <vector>
#include "stereo_cells.hpp"
using namespace std;
class ShellCmd{
public:
  ShellCmd(const char *basepath,float simp_mult,const char *pos_simp_log_dir,bool dist_run,const char *cwd,const char *aggdir,const char *dicedir,bool have_mb_ply,int num_threads):basepath(basepath),simp_mult(simp_mult),pos_simp_log_dir(pos_simp_log_dir),dist_run(dist_run),cwd(cwd),aggdir(aggdir),dicedir(dicedir),have_mb_ply(have_mb_ply),num_threads(num_threads){};
  void pos_dice(vector<Cell_Data>cells,float eps,bool run=true);
  void write_setup(void);
  void write_generic(string filename, string cmdfile,const vector<string> *precmds =NULL, const vector<string> *postcmds=NULL);
  void pos_simp_cmd(bool run);
 void pos_simp_cmd2(bool run);
  const char *basepath;
  float simp_mult;
  const char *pos_simp_log_dir;

  bool dist_run;
  const char *cwd,*aggdir,*dicedir;
  bool have_mb_ply;
  int num_threads;
};

void save_bbox_frame (GtsBBox * bb, FILE * fptr);
