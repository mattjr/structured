/* * structured - Tools for the Generation and Visualization of Large-scale
 * Three-dimensional Reconstructions from Image Data. This software includes
 * source code from other projects, which is subject to different licensing,
 * see COPYING for details. If this project is used for research see COPYING
 * for making the appropriate citations.
 * Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
 *
 * This file is part of structured.
 *
 * structured is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * structured is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with structured.  If not, see <http://www.gnu.org/licenses/>.
 */

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
