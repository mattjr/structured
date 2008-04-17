#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h> 
#include <stdlib.h>

class ShellCmd{
public:
  ShellCmd(const char *basepath,float simp_mult,const char *pos_simp_log_dir,bool dist_run):basepath(basepath),simp_mult(simp_mult),pos_simp_log_dir(pos_simp_log_dir),dist_run(dist_run){};
  void pos_simp_cmd(bool run);
  const char *basepath;
  float simp_mult;
  const char *pos_simp_log_dir;
  bool dist_run;
};
