#ifndef VPBINTERFACE_HPP
#define VPBINTERFACE_HPP
#include "stereo_cells.hpp"
#include "Extents.h"
#include <vpb/Commandline>
#include <vector>
#include <string>
void doQuadTreeVPB(std::vector<std::vector<std::string> > datalist_lod,Bounds bounds,Camera_Calib &calib);

#endif // VPBINTERFACE_HPP
