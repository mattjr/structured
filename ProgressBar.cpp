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

#include "ProgressBar.h"
#include <string>
#include <sys/ioctl.h>
#include <sstream>
#include <math.h>
#include <iostream>
#include <iomanip>

#include <stdio.h>
#include <assert.h>
using namespace std;
std::string formatBarString(string name,osg::Timer_t startTick,osg::Timer_t tick,unsigned int count,unsigned int totalCount){
    double currentTime = osg::Timer::instance()->delta_s(startTick, tick);
    struct winsize w;
    ioctl(0, TIOCGWINSZ, &w);
    int term_width=w.ws_col;
    if(w.ws_col > 2800)
        term_width=2;
    std::stringstream tmp;
    tmp<<totalCount;
    int countlength=tmp.str().size();
    std::stringstream time;
    double percentage=(count/(double)totalCount);

    if(count == 0){
        time <<"ETA: --:--:--";
    }else{
        if(count == totalCount){
            time << "Time: ";
        } else{
            time <<"ETA:  ";
            currentTime= currentTime*totalCount/count - currentTime;
        }
        double hours=floor(currentTime/60.0/60.0);
        double mins=floor((currentTime-(hours*60.0*60.0))/60.0);
        double secs=floor(currentTime-(hours*60.0*60.0)-(mins*60.0));

        time   <<  setfill('0') << setw(2) <<(int)(hours) <<":"<<setfill('0') << setw(2) <<(int)(mins)<< ":"<<setfill('0') << setw(2) <<secs;
    }

    std::stringstream bar;

    bar << name << " " <<  setw(3)<<(int)(round(100.0*percentage))<<"% " <<setfill('0')<<setw(countlength+1)<< count <<"/"<<setfill('0')<<setw(countlength+1)<<totalCount;
    bar <<" |";
    int length=term_width-bar.str().size()-time.str().size()-2;
    for(int i=0; i< length; i++){
        if(i < length*percentage)
            bar<<"#";
        else
            bar <<" ";
    }
    bar << "| "<<time.str();
    // if(count==totalCount)
    //   bar<<"\n";
    return bar.str();
}

void formatBar(string name,osg::Timer_t startTick,unsigned int count,unsigned int totalCount){
    osg::Timer_t tick = osg::Timer::instance()->tick();

    string str=formatBarString(name,startTick,tick,count,totalCount);
    printf("\r%s",str.c_str());
    fflush(stdout);
}
#define ASCII_ESC 27

void formatBarMultiLevel(vector<std::string> &name,std::vector<osg::Timer_t> &startTick,std::vector<osg::Timer_t> &endTick,std::vector<unsigned int> &count,std::vector<unsigned int> &totalCount){
    assert(name.size() == startTick.size() && count.size() == totalCount.size() && endTick.size() == startTick.size());
    bool isDone=true;
    for(int i=0; i < (int)name.size(); i++){
        string str=formatBarString(name[i],startTick[i],endTick[i],count[i],totalCount[i]);
        if(count[i] != totalCount[i])
            isDone=false;
        printf("%s\n",str.c_str());
    }
    if(!isDone){
        printf( "%c[%dA",ASCII_ESC,(int)name.size());
        fflush(stdout);
    }
   //
}
