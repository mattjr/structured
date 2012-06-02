#ifndef PROGRESSBAR_H
#define PROGRESSBAR_H
#include <osg/Timer>
#include <string>
#include <vector>
void formatBar(std::string name,osg::Timer_t startTick,unsigned int count,unsigned int totalCount);
void formatBarMultiLevel(std::vector<std::string> &name,std::vector<osg::Timer_t> &startTick,std::vector<osg::Timer_t> &endTick,std::vector<unsigned int> &count,std::vector<unsigned int> &totalCount);

#endif // PROGRESSBAR_H
