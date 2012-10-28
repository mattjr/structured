/* -*-c++-*- OpenSceneGraph Cookbook
 * Chapter 8 Recipe 7
 * Author: Wang Rui <wangray84 at gmail dot com>
*/

#ifndef H_COOKBOOK_CH8_OCTREEBUILDER
#define H_COOKBOOK_CH8_OCTREEBUILDER

#include <osg/Geode>
#include <osg/LOD>
#include <stdio.h>
#include <iostream>
#include <osg/io_utils>
template <typename CellType>
class OctreeBuilder
{
public:
    OctreeBuilder() : _maxChildNumber(16), _maxTreeDepth(32), _maxLevel(0),_maxFaces(10000) {}
    int getMaxLevel() const { return _maxLevel; }
    
    void setMaxChildNumber( int max ) { _maxChildNumber = max; }
    int getMaxChildNumber() const { return _maxChildNumber; }
    
    void setMaxTreeDepth( int max ) { _maxTreeDepth = max; }
    int getMaxTreeDepth() const { return _maxTreeDepth; }
    void setMaxFaces( int max ) { _maxFaces = max; }
    int getMaxFaces() const { return _maxFaces; }
    typedef std::pair<std::vector<CellType> , osg::BoundingBox> OctNode;
    void build(int *finaldepth,std::vector<OctNode> &group, int *depth, const osg::BoundingBox& total,
                      const std::vector<CellType>& elements );
    
protected:
    osg::LOD* createNewLevel( int level, const osg::Vec3& center, float radius );
    osg::Node* createElement( const std::string& id, const osg::Vec3& center, float radius );
    osg::Geode* createBoxForDebug( const osg::Vec3& max, const osg::Vec3& min );
    
    int _maxChildNumber;
    int _maxTreeDepth;
    int _maxLevel;
    int _maxFaces;
};

template <typename CellType>
void OctreeBuilder<CellType>::build(int *finaldepth, std::vector<OctNode> &group ,int *depth, const osg::BoundingBox& total,
                                  const std::vector<CellType>& elements )
{
    int s[3];  // axis sides (0 or 1)
    osg::Vec3 extentSet[3] = {
        total._min,
        (total._max + total._min) * 0.5f,
        total._max
    };
    int faces=0;
    std::vector<CellType> childData;
    for ( unsigned int i=0; i<elements.size(); ++i )
    {
        const CellType& obj = elements[i];
        if ( total.contains(obj.bbox._min) && total.contains(obj.bbox._max) )
            childData.push_back( obj );
        else if ( total.intersects(obj.bbox) )
        {
            osg::Vec3 center = (obj.bbox._max + obj.bbox._min) * 0.5f;
            if ( total.contains(center) ) childData.push_back( obj );
        }
    }

    if(childData.size() == 0)
        return;

    for(unsigned int i=0;i<childData.size(); i++){
        faces+=childData[i].faces;
    }

    bool isLeafNode;

    if(faces < _maxFaces){
        isLeafNode = true;
    }else if ( (int)childData.size()<=_maxChildNumber || depth[0]>_maxTreeDepth  || depth[1]>_maxTreeDepth || depth[2]>_maxTreeDepth)
        isLeafNode = true;
    else
        isLeafNode = false;

    if ( !isLeafNode )
    {
       // osg::ref_ptr<osg::Group> childNodes[8];
     /*   for ( s[0]=0; s[0]<2; ++s[0] )
        {
            for ( s[1]=0; s[1]<2; ++s[1] )
            {
                for ( s[2]=0; s[2]<2; ++s[2] )
                {
                    // Calculate the child extent
                    osg::Vec3 min, max;
                    for ( int a=0; a<3; ++a )
                    {
                        min[a] = (extentSet[s[a] + 0])[a];
                        max[a] = (extentSet[s[a] + 1])[a];
                    }

                    //int id = s[0] + (2 * s[1]) + (4 * s[2]);
                    build(largestDepthReached,group, depth+1, osg::BoundingBox(min, max), childData );
                }
            }
        }*/

        // Calculate the child extent
        osg::Vec3 min, max;
        min = total._min;
        max = total._max;
        float largestAxis=-FLT_MAX;
        int axis=0;
        for(int i=0; i<3; i++){
            double range=max[i]-min[i];
            if(range > largestAxis){
                largestAxis=range;
                axis=i;
            }

        }
        //printf("Largest Axis %d %d %d\n",axis,faces,childData.size());
        //std::cout << total._min<< " " << total._max<<std::endl;
        double half=((max[axis]-min[axis])/2.0);
        int newdepth[]={depth[0],depth[1],depth[2]};
        newdepth[axis]++;
        max[axis]= min[axis]+half;
        build(finaldepth,group, newdepth, osg::BoundingBox(min, max), childData );
        min[axis]= max[axis];
        max[axis]=min[axis]+half;
        build(finaldepth,group, newdepth, osg::BoundingBox(min, max), childData );


    }
    else
    {
      /*  for ( unsigned int i=0; i<childData.size(); ++i )
        {
            const CellType& obj = childData[i];
            osg::Vec3 center = (obj.bbox._max + obj.bbox._min) * 0.5;
            float radius = (obj.bbox._max - obj.bbox._min).length() * 0.5f;
        }*/
     //   if(depth > largestDepthReached)
       //     largestDepthReached=depth;
        for ( unsigned int i=0; i<3; ++i )
        if(finaldepth[i]<depth[i])
            finaldepth[i]=depth[i];

        group.push_back(std::make_pair<std::vector<CellType> , osg::BoundingBox>(childData,total) );

    }

  /*  osg::Vec3 center = (total._max + total._min) * 0.5;
    float radius = (total._max - total._min).length() * 0.5f;
    osg::LOD* level = createNewLevel( depth, center, radius );
    level->insertChild( 0, createBoxForDebug(total._max, total._min) );  // For debug use
    level->insertChild( 1, group.get() );*/
    //return ;
}

#if 0
template <typename CellType>
void OctreeBuilder<CellType>::build( std::vector<OctNode> &group ,int depth, const osg::BoundingBox& total,
                                  const std::vector<CellType>& elements )
{
    int s[3];  // axis sides (0 or 1)
    osg::Vec3 extentSet[3] = {
        total._min,
        (total._max + total._min) * 0.5f,
        total._max
    };
    int faces=0;
    std::vector<CellType> childData;
    for ( unsigned int i=0; i<elements.size(); ++i )
    {
        const CellType& obj = elements[i];
        if ( total.contains(obj.bbox._min) && total.contains(obj.bbox._max) )
            childData.push_back( obj );
        else if ( total.intersects(obj.bbox) )
        {
            osg::Vec3 center = (obj.bbox._max + obj.bbox._min) * 0.5f;
            if ( total.contains(center) ) childData.push_back( obj );
        }
    }

    if(childData.size() == 0)
        return;

    for(unsigned int i=0;i<childData.size(); i++){
        faces+=childData[i].faces;
    }

    bool isLeafNode;

    if(faces < _maxFaces){
        isLeafNode = true;
    }else if ( (int)childData.size()<=_maxChildNumber || depth>_maxTreeDepth )
        isLeafNode = true;
    else
        isLeafNode = false;

    if ( !isLeafNode )
    {
       // osg::ref_ptr<osg::Group> childNodes[8];
        for ( s[0]=0; s[0]<2; ++s[0] )
        {
            for ( s[1]=0; s[1]<2; ++s[1] )
            {
                for ( s[2]=0; s[2]<2; ++s[2] )
                {
                    // Calculate the child extent
                    osg::Vec3 min, max;
                    for ( int a=0; a<3; ++a )
                    {
                        min[a] = (extentSet[s[a] + 0])[a];
                        max[a] = (extentSet[s[a] + 1])[a];
                    }

                    //int id = s[0] + (2 * s[1]) + (4 * s[2]);
                    build(group, depth+1, osg::BoundingBox(min, max), childData );
                }
            }
        }

      /*  for ( unsigned int i=0; i<8; ++i )
        {
            if ( childNodes[i] && childNodes[i]->getNumChildren() )
                group->addChild( childNodes[i] );
        }*/
    }
    else
    {
      /*  for ( unsigned int i=0; i<childData.size(); ++i )
        {
            const CellType& obj = childData[i];
            osg::Vec3 center = (obj.bbox._max + obj.bbox._min) * 0.5;
            float radius = (obj.bbox._max - obj.bbox._min).length() * 0.5f;
        }*/
        group.push_back(std::make_pair<std::vector<CellType> , osg::BoundingBox>(childData,total) );

    }

  /*  osg::Vec3 center = (total._max + total._min) * 0.5;
    float radius = (total._max - total._min).length() * 0.5f;
    osg::LOD* level = createNewLevel( depth, center, radius );
    level->insertChild( 0, createBoxForDebug(total._max, total._min) );  // For debug use
    level->insertChild( 1, group.get() );*/
    //return ;
}
#endif

#endif
