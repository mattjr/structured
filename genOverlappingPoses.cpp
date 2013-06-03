//
// structured - Tools for the Generation and Visualization of Large-scale
// Three-dimensional Reconstructions from Image Data. This software includes
// source code from other projects, which is subject to different licensing,
// see COPYING for details. If this project is used for research see COPYING
// for making the appropriate citations.
// Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
//
// This file is part of structured.
//
// structured is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// structured is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with structured.  If not, see <http://www.gnu.org/licenses/>.
//

#include "SeaBedIO.h"

#include <math.h>
enum{ POSE_INDEX_X,
      POSE_INDEX_Y,
      POSE_INDEX_Z,
      POSE_INDEX_PHI,
      POSE_INDEX_THETA,
      POSE_INDEX_PSI,
      NUM_POSE_STATES } ;
bool does_overlap(const Stereo_Pose &p1,const Stereo_Pose &p2){
    double larger =std::max(p1.image_footprint_radius,p2.image_footprint_radius);
    double dx=p1.pose_est[POSE_INDEX_X] - p2.pose_est[POSE_INDEX_X];
    double dy=p1.pose_est[POSE_INDEX_Y] - p2.pose_est[POSE_INDEX_Y];
    double dist=sqrt(pow(dx,2.0)+pow(dy,2.0));
    return dist < larger;
}

int main( int argc, char *argv[ ] )
{
    if(argc < 4){
        fprintf(stderr,"%s: src_stereo_pose_file stereo_pose_file_overlap outputfile\n",argv[0]);
        return -1;
    }

    Stereo_Pose_File src_pose_data=read_stereo_pose_est_file(argv[1] );
    Stereo_Pose_File overlap_pose_data=read_stereo_pose_est_file(argv[2] );
    if(src_pose_data.origin_latitude != overlap_pose_data.origin_latitude || src_pose_data.origin_longitude != overlap_pose_data.origin_longitude){
        fprintf(stderr, "Can't perform this on differing lat long orgins\n");
        return -1;
    }
    Stereo_Pose_File output;
    output.origin_latitude=src_pose_data.origin_latitude;
    output.origin_longitude=src_pose_data.origin_longitude;

    for(int i=0; i<(int)src_pose_data.poses.size(); i++){
        for(int j=0; j<(int)overlap_pose_data.poses.size(); j++){
            if(does_overlap(src_pose_data.poses[i],overlap_pose_data.poses[j])){
                output.poses.push_back(src_pose_data.poses[i]);
                int newid=output.poses.size();
                output.poses.back().pose_id=newid;
                break;
            }
        }
    }
    write_stereo_pose_est_file(argv[3],"",output);

    printf("Stored %d/%d to %s from filter of %d\n",(int)output.poses.size(), (int)src_pose_data.poses.size(),argv[3],(int)overlap_pose_data.poses.size());
}
