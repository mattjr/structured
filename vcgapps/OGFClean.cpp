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


#include <OGF/cells/types/cells_library.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/map_builder.h>

#include <OGF/cells/map/map_editor.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/cells/map_algos/atlas_generator.h>
#include <OGF/cells/map_algos/pm_manager.h>
#include <OGF/image/types/image.h>
#include <OGF/cells/map_algos/variational_map_splitter.h>
#include <OGF/image/types/image_library.h>
#include <OGF/image/algos/rasterizer.h>
#include <OGF/image/algos/morpho_math.h>
#include <OGF/image/io/image_serializer_ppm.h>
#include <OGF/basic/os/file_system.h>
#include <OGF/cells/io/map_serializer_obj.h>
#include <osgDB/ReadFile>
#include <osg/NodeVisitor>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgUtil/Optimizer>
#include <osg/io_utils>
#include <osg/MatrixTransform>
#include <iostream>
#include "GenParam.h"
using namespace std;
int main(int argc, char **argv){
    osg::ArgumentParser arguments(&argc,argv);

    osg::ref_ptr<osg::Node> model = osgDB::readNodeFiles(arguments);
    osg::DrawElementsUInt *tri; osg::Vec3Array *verts;
    osg::Geode *geode= dynamic_cast<osg::Geode*>(model.get());
    if(!geode)
        geode=model->asGroup()->getChild(0)->asGeode();
    if(geode && geode->getNumDrawables()){


        osg::Drawable *drawable = geode->getDrawable(0);
        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        verts=static_cast< osg::Vec3Array*>(geom->getVertexArray());

        tri= (osg::DrawElementsUInt*)geom->getPrimitiveSet(0);

        OGF::Map the_map ;

        LoaderOSG2OGF map_builder(&the_map,verts,tri);

        if(!map_builder.build()) {
            std::cerr << "Could not open proces model" << std::endl ;
            exit(-1) ;
        }
        std::cerr << "nb facets: " << the_map.size_of_facets() << " nb vertices:" << the_map.size_of_vertices() << std::endl ;
        std::cerr << std::endl ;
        the_map.compute_normals() ;
        OGF::MapSerializer_obj mso;
        ofstream f("temp.obj");
        mso.serialize_write(&the_map,f);
    }else
        fprintf(stderr,"Can't load moedl\n");

}
