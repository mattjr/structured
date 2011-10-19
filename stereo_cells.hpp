//
// Functions to read in the stereo pose data produced by seabed_slam, and divide
// the poses into a set of cells while avoiding segmenting the overlap regions.
//

#ifndef AUV_STEREO_CELLS_HPP
#define AUV_STEREO_CELLS_HPP

#include <string>
#include <vector>
#include <osg/Matrix>
#include <osg/BoundingBox>
#include <iostream>
#include <stdlib.h>
enum{ POSE_INDEX_X,
     POSE_INDEX_Y,
     POSE_INDEX_Z,
     POSE_INDEX_PHI,
     POSE_INDEX_THETA,
     POSE_INDEX_PSI,
     NUM_POSE_STATES } ;


//
// Parameters
//

extern unsigned int max_cell_poses;
extern unsigned int max_tree_depth;
class Tex_Pose_Data
{
public:
  unsigned int id;
  std::string file_name;
  std::string dir;
  osg::Matrixd mat;
  osg::BoundingBox bbox;
  bool valid;
};


class Stereo_Pose_Data : public Tex_Pose_Data
{
public:
  double time;
  double pose[6];
  std::string left_name;
  std::string mesh_name;
  std::string right_name;
  double alt;
  double radius;
  bool overlap;

};



class Bounds
{
public:
   Bounds( );
   Bounds( const std::vector<Tex_Pose_Data> &poses );
   Bounds( const std::vector<Stereo_Pose_Data> &poses );

   void set( double min_x, double max_x, double min_y, double max_y );
   double area( void ) const;

   osg::BoundingBox bbox;

};


enum{AUV_SPLIT,EVEN_SPLIT};

template <class A>
class Cell_Data
{
public:

   Cell_Data<A>( );

   Cell_Data<A>( const std::vector<const A*> &poses,
              const Bounds                               &bounds ,
              const std::pair<int,int> &idx);
              
   Cell_Data<A>( const std::vector<const A*> &poses,
              const Bounds                               &bounds );
  Bounds bounds;
   std::vector<const A*> poses;
   std::pair<int,int> idx;
   bool valid;

};


//
// Public functions
//

//----------------------------------------------------------------------------//
//   Class Cell_Data                                                          //
//----------------------------------------------------------------------------//
template <class A>
Cell_Data<A>::Cell_Data( ): valid(false)
{

}
template <class A>
        Cell_Data<A>::Cell_Data( const std::vector<const A*> &poses,
                     const Bounds                          &bounds,
                     const std::pair<int,int> &idx)
    : bounds( bounds),
      poses( poses ),
      idx(idx),valid(poses.size() > 0)
{

}
template <class A>
        Cell_Data<A>::Cell_Data( const std::vector<const A*> &poses,
                     const Bounds                          &bounds)
    : bounds( bounds),
      poses( poses ),valid(poses.size() > 0)
{

}


std::vector<Stereo_Pose_Data> load_stereo_pose_file( const std::string &file_name );
template <class A>
        static void split_data( const std::vector<const A *> &poses,
                       const Bounds                           &bounds,
                       unsigned int                            split_axis,
                       double                                  split_value,
                       std::vector<const A *>       &poses1,
                       Bounds                                 &bounds1,
                       std::vector<const A *>       &poses2,
                       Bounds                                 &bounds2 )
{
    // Split the data - here we don't just consider if the center of the stereo
    // is in a cell, but if any part of the footprint is. Therefore a pose can
    // be in multiple cells.



    // Create the bounds for the split data sets
    if( split_axis == POSE_INDEX_X )
    {
        bounds1.set( bounds.bbox.xMin(), split_value , bounds.bbox.yMin(), bounds.bbox.yMax() );
        bounds2.set( split_value , bounds.bbox.xMax(), bounds.bbox.yMin(), bounds.bbox.yMax() );
    }
    else
    {
        bounds1.set( bounds.bbox.xMin(), bounds.bbox.xMax(), bounds.bbox.yMin(), split_value );
        bounds2.set( bounds.bbox.xMin(), bounds.bbox.xMax(), split_value , bounds.bbox.yMax() );
    }

    for( unsigned int i=0 ; i<poses.size() ; i++ )
    {
        if(poses[i]->bbox.radius2() <= 0 || !poses[i]->valid || !poses[i]->bbox.valid())
            continue;
        if( bounds1.bbox.intersects((poses[i]->bbox )))
            poses1.push_back( poses[i] );

        if( bounds2.bbox.intersects((poses[i]->bbox )))
            poses2.push_back( poses[i] );

    }
}

//template<class A>
//std::vector<Cell_Data<A> > calc_cells( const std::vector<A> &poses ,int max_poses);
//std::vector<Cell_Data> calc_cells( const std::vector<Tex_Pose_Data> &poses,double x1,double x2,double y1 ,double y2,double area );
template <class A>
        static void recursive_split_poses( const std::vector<const A *> &poses,
                                  const Bounds &bounds,
                                  unsigned int depth,
                                  std::vector<Cell_Data<A> > &cells)
{
    if( depth > max_tree_depth )
    {
        std::cerr << "ERROR - maximum recursion level reached in recursive_split" << std::endl;
        exit(1);
    }



    if(poses.size() < max_cell_poses ){
        Cell_Data<A> new_cell( poses, bounds );
        cells.push_back( new_cell );
        return ;
    }
    unsigned int best_axis;
    double best_split_point;

    if(bounds.bbox.xMax()-bounds.bbox.xMin()  > bounds.bbox.yMax()-bounds.bbox.yMin()){
        best_axis= POSE_INDEX_X ;
        best_split_point=(bounds.bbox.xMax()+bounds.bbox.xMin())/2;
    }else{
        best_axis= POSE_INDEX_Y ;
        best_split_point=(bounds.bbox.yMax()+bounds.bbox.yMin())/2;
    }
    // Perform the split
    std::vector<const A*> poses1, poses2;
    Bounds bounds1, bounds2;
    split_data( poses, bounds, best_axis, best_split_point,
               poses1, bounds1, poses2, bounds2 );


    // Check if the subsets need to be recursively split
    if( poses1.size() > max_cell_poses)
    {
        recursive_split_poses( poses1, bounds1, depth+1 ,cells );
    }
    else
    {
        Cell_Data<A> new_cell( poses1, bounds1 );
        cells.push_back( new_cell  );
#if VERBOSE
        printf( "Area Cell %d at depth %d area: %f %f %f %f %f, poses: %d %04d_%04d\n", cells.size(), depth,
               bounds1.area(), bounds1.bbox.xMin(),bounds1.bbox.xMax(),bounds1.bbox.yMin(),bounds1.bbox.yMax(),poses2.size() ,row,col);
#endif
    }


    if( poses2.size() > max_cell_poses)
    {
        recursive_split_poses( poses2, bounds2, depth+1,cells );
    }
    else
    {
        Cell_Data<A> new_cell( poses2, bounds2  );
        cells.push_back( new_cell );
#if VERBOSE
        printf( "Area Cell %d at depth %d area: %f %f %f %f %f, poses: %d %04d_%04d\n", cells.size(), depth,
               bounds2.area(), bounds2.bbox.xMin(),bounds2.bbox.xMax(),bounds2.bbox.yMin(),bounds2.bbox.yMax(),poses2.size() ,row,col);
#endif
    }

}
template <class A>
        std::vector<Cell_Data<A> > calc_cells( const std::vector<A> &poses,int max_poses )
{
   // Calculate the bounds of the stereo data
   Bounds bounds( poses );
#if VERBOSE
   printf( "Bounds: %f %f %f %f\n", bounds.bbox.xMin(), bounds.bbox.xMax() ,
          bounds.bbox.yMin(), bounds.bbox.yMax() );
#endif

   // m->_buildDestination(true);

   // vpb::Commandline cl;


   // Root node of the binary tree has all poses
   std::vector<const A *> node_poses( poses.size() );
   for( unsigned int i=0 ; i<poses.size() ; i++ )
       node_poses[i] = &poses[i];

   max_cell_poses= max_poses;


   // Perform binary division until termination criteria
   std::vector<Cell_Data<A> > cells;

   recursive_split_poses( node_poses, bounds,0, cells );

   return cells;
}
#endif // !AUV_STEREO_CELLS_HPP



