/* -*-c++-*- OpenSceneGraph Cookbook
 * Chapter 8 Recipe 7
 * Author: Wang Rui <wangray84 at gmail dot com>
*/

#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include "OctreeBuilder.h"
template <typename CellType>
osg::LOD* OctreeBuilder<CellType>::createNewLevel( int level, const osg::Vec3& center, float radius )
{
    osg::ref_ptr<osg::LOD> lod = new osg::LOD;
    lod->setCenterMode( osg::LOD::USER_DEFINED_CENTER );
    lod->setCenter( center );
    lod->setRadius( radius );
    lod->setRange( 0, radius * 5.0f, FLT_MAX );
    lod->setRange( 1, 0.0f, radius * 5.0f );

    if ( _maxLevel<level ) _maxLevel = level;
    return lod.release();
}
template <typename CellType>
osg::Node* OctreeBuilder<CellType>::createElement( const std::string& id, const osg::Vec3& center, float radius )
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( new osg::ShapeDrawable(new osg::Sphere(center, radius)) );
    geode->setName( id );
    return geode.release();
}
template <typename CellType>

osg::Geode* OctreeBuilder<CellType>::createBoxForDebug( const osg::Vec3& max, const osg::Vec3& min )
{
    osg::Vec3 dir = max - min;
    osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array(10);
    (*va)[0] = min + osg::Vec3(0.0f, 0.0f, 0.0f);
    (*va)[1] = min + osg::Vec3(0.0f, 0.0f, dir[2]);
    (*va)[2] = min + osg::Vec3(dir[0], 0.0f, 0.0f);
    (*va)[3] = min + osg::Vec3(dir[0], 0.0f, dir[2]);
    (*va)[4] = min + osg::Vec3(dir[0], dir[1], 0.0f);
    (*va)[5] = min + osg::Vec3(dir[0], dir[1], dir[2]);
    (*va)[6] = min + osg::Vec3(0.0f, dir[1], 0.0f);
    (*va)[7] = min + osg::Vec3(0.0f, dir[1], dir[2]);
    (*va)[8] = min + osg::Vec3(0.0f, 0.0f, 0.0f);
    (*va)[9] = min + osg::Vec3(0.0f, 0.0f, dir[2]);
    
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setVertexArray( va.get() );
    geom->addPrimitiveSet( new osg::DrawArrays(GL_QUAD_STRIP, 0, 10) );
    
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( geom.get() );
    geode->getOrCreateStateSet()->setAttribute(
        new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE) );
    geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    return geode.release();
}
