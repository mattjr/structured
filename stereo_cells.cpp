//
// stereo_cells.cpp
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <cstdlib>

#include "stereo_cells.hpp"

#include <osgDB/ReadFile>
using namespace std;

#define VERBOSE 0

#define POSE_INDEX_X 0 
#define POSE_INDEX_Y 1


//
// Parameters
//

static unsigned int max_cell_poses    = 300;
static unsigned int max_tree_depth    = 1000;  // Max recursion level 



// The spherical bound radii could be wrong due to vehicle roll or pitch,
// or a non-flat seafloor. The radii are increased to be more conservative
// FIXME: should this be done in seabed_slam? That would mean that the
//        detection of overlapping poses would also be more conservative



//----------------------------------------------------------------------------//
//   Class Bounds                                                             //
//----------------------------------------------------------------------------//

Bounds::Bounds( )
{

}


Bounds::Bounds( const vector<Tex_Pose_Data> &poses )
{
    for( unsigned int i=0; i<poses.size( ) ; i++ )
    {
        bbox.expandBy(poses[i].bbox);
    }
    bbox._min[2]=-FLT_MAX;
    bbox._max[2]=FLT_MAX;

}
Bounds::Bounds( const vector<Stereo_Pose_Data> &poses )
{
    for( unsigned int i=0; i<poses.size( ) ; i++ )
    {
        bbox.expandBy(poses[i].bbox);
    }
    bbox._min[2]=-FLT_MAX;
    bbox._max[2]=FLT_MAX;

}
void Bounds::set( double min_x, double max_x, double min_y, double max_y )
{
     bbox._min.x()=min_x;
     bbox._min.y()=min_y;
     bbox._max.x()=max_x;
     bbox._max.y()=max_y;

     bbox._min.z() = -FLT_MAX;
     bbox._max.z() = FLT_MAX;


}


double Bounds::area( void ) const
{
   return (bbox.xMax()-bbox.xMin())*(bbox.yMax()-bbox.yMin());

 }

//----------------------------------------------------------------------------//
//   Class Cell_Data                                                          //
//----------------------------------------------------------------------------//
template <class A>
Cell_Data<A>::Cell_Data( ): valid(false)
{ 

}
template <class A>
Cell_Data<A>::Cell_Data( const vector<const A*> &poses,
                     const Bounds                          &bounds,
                     const std::pair<int,int> &idx)
    : bounds( bounds),
      poses( poses ),
      idx(idx),valid(poses.size() > 0)
{ 

}               
template <class A>
Cell_Data<A>::Cell_Data( const vector<const A*> &poses,
                     const Bounds                          &bounds)
    : bounds( bounds),
      poses( poses ),valid(poses.size() > 0)
{

}


//----------------------------------------------------------------------------//
//  Private Helper Functions                                                  //
//----------------------------------------------------------------------------//
template <class A>
static void split_data( const vector<const A *> &poses,
                       const Bounds                           &bounds,
                       unsigned int                            split_axis,
                       double                                  split_value,
                       vector<const A *>       &poses1,
                       Bounds                                 &bounds1,
                       vector<const A *>       &poses2,
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

#if 0

static vector<double> sample_cost_func(
    const vector<const Stereo_Pose_Data *> &poses,
    const Bounds &bounds,
    unsigned int axis,
    const vector<double> &samples )
{
    vector<double> costs( samples.size(), 0 );

    for( unsigned int i=0 ; i < poses.size() ; i++ )
    {
        for( unsigned int j=0 ; j < samples.size() ; j++ )
        {
            double dist = fabs( poses[i]->pose[axis] - samples[j] );
            if( dist <= radius_multiplier*poses[i]->radius )
            {
                if( poses[i]->overlap )
                    costs[j] += overlap_cost;
                else
                    costs[j] += pose_cost;
            }
        }
    }

    // Modify the weights for how evenly the samples are splitting up the area
    double bounds_min, bounds_max;
    if( axis == POSE_INDEX_X )
    {
        bounds_min = bounds.bbox.xMin();
        bounds_max = bounds.bbox.xMax();
    }
    else
    {
        bounds_min = bounds.bbox.yMin();
        bounds_max = bounds.bbox.yMax();
    }
    for( unsigned int i=0; i<samples.size(); i++ )
    {
        double ratio1 = (samples[i]-bounds_min)/(bounds_max-bounds_min);
        double ratio2 = (bounds_max-samples[i])/(bounds_max-bounds_min);
        double max_ratio = max( ratio1, ratio2 );
        costs[i] += (max_ratio-0.5)*area_cost_multiplier;
    }


    // Slightly favour chunky squares instead of long thin things
    // this will mainly be for empty boxes that are too big
    for( unsigned int i=0 ; i<costs.size() ; i++ )
    {
        if( axis == POSE_INDEX_X )
            costs[i] += 1000*(bounds.bbox.yMax()-bounds.bbox.yMin())/(bounds.bbox.xMax()-bounds.bbox.xMin());
        else
            costs[i] += 1000*(bounds.bbox.xMax()-bounds.bbox.xMin())/(bounds.bbox.yMax()-bounds.bbox.yMin());
    }

    // FIXME: Modify the weights for how evenly the samples are splitting up the
    //        number of poses?



    return costs;
}

static void recursive_split( const vector<const Stereo_Pose_Data *> &poses,
                            const Bounds &bounds,
                            unsigned int depth,
                            vector<Cell_Data> &cells )
{
    if( depth > max_tree_depth )
    {
        cerr << "ERROR - maximum recursion level reached in recursive_split" << endl;
        exit(1);
    }

    if(bounds.area() < max_cell_area && poses.size() < max_cell_poses ){
        Cell_Data new_cell( poses, bounds );
        cells.push_back( new_cell );
        return ;
    }

    // Sample the cost function at various X and Y axis values
    vector<double> x_samples( num_split_samples );
    vector<double> y_samples( num_split_samples );
    double x_inc = (bounds.bbox.xMax()-bounds.bbox.xMin())/(num_split_samples+1);
    double y_inc = (bounds.bbox.yMax()-bounds.bbox.yMin())/(num_split_samples+1);
    for( unsigned int i=0 ; i<num_split_samples ; i++ )
    {
        x_samples[i] = bounds.bbox.xMin() + (i+1)*x_inc;
        y_samples[i] = bounds.bbox.yMin() + (i+1)*y_inc;
    }

    vector<double> x_costs;
    vector<double> y_costs;
    x_costs = sample_cost_func( poses, bounds, POSE_INDEX_X, x_samples );
    y_costs = sample_cost_func( poses, bounds, POSE_INDEX_Y, y_samples );


    // Find the best split point
    unsigned int best_axis=0;
    double best_split_point=0.0;
    double min_cost=DBL_MAX;
    for( unsigned int i=0; i < num_split_samples ; i++ )
    {
        if( i==0 || x_costs[i] < min_cost )
        {
            best_axis = POSE_INDEX_X;
            best_split_point = x_samples[i];
            min_cost = x_costs[i];
        }
        if( y_costs[i] < min_cost )
        {
            best_axis = POSE_INDEX_Y;
            best_split_point = y_samples[i];
            min_cost = y_costs[i];
        }
    }

#if VERBOSE
    if( best_axis == POSE_INDEX_X )
        printf( "Splitting on X axis at depth %d with value %f cost %f\n", depth, best_split_point, min_cost );
    else
        printf( "Splitting on Y axis at depth %d with value %f cost %f\n", depth, best_split_point, min_cost );
#endif

    // Perform the split
    vector<const Stereo_Pose_Data*> poses1, poses2;
    Bounds bounds1, bounds2;
    split_data( poses, bounds, best_axis, best_split_point,
               poses1, bounds1, poses2, bounds2 );


    // Check if the subsets need to be recursively split
    if( bounds1.area() > max_cell_area || poses1.size() > max_cell_poses )
    {
        recursive_split( poses1, bounds1, depth+1, cells );
    }
    else
    {
        Cell_Data new_cell( poses1, bounds1 );
        cells.push_back( new_cell );
#if VERBOSE
        printf( "Cell %d at depth %d area: %f, poses: %d\n", cells.size(), depth,
               bounds1.area(), poses1.size() );
#endif
    }


    if( bounds2.area() > max_cell_area || poses2.size() > max_cell_poses )
    {
        recursive_split( poses2, bounds2, depth+1, cells );
    }
    else
    {
        Cell_Data new_cell( poses2, bounds2 );
        cells.push_back( new_cell );
#if VERBOSE
        printf( "Cell %d at depth %d area: %f, poses: %d\n", cells.size(), depth,
               bounds2.area(), poses2.size() );
#endif
    }

}


static void recursive_split_even( const vector<const Stereo_Pose_Data *> &poses,
                                 const Bounds &bounds,
                                 unsigned int depth,
                                 vector<Cell_Data> &cells )
{
    if( depth > max_tree_depth )
    {
        cerr << "ERROR - maximum recursion level reached in recursive_split" << endl;
        exit(1);
    }



    if(bounds.area() < max_cell_area && poses.size() < max_cell_poses ){
        Cell_Data new_cell( poses, bounds );
        cells.push_back( new_cell );
        return ;
    }

    unsigned int best_axis= POSE_INDEX_X ;
    double best_split_point=(bounds.bbox.xMax()-bounds.bbox.xMin())/2;

    // Perform the split
    vector<const Stereo_Pose_Data*> poses1, poses2;
    Bounds bounds1, bounds2;
    split_data( poses, bounds, best_axis, best_split_point,
               poses1, bounds1, poses2, bounds2 );


    // Check if the subsets need to be recursively split
    if( bounds1.area() > max_cell_area || poses1.size() > max_cell_poses )
    {
        recursive_split( poses1, bounds1, depth+1, cells );
    }
    else
    {
        Cell_Data new_cell( poses1, bounds1 );
        cells.push_back( new_cell );
#if VERBOSE
        printf( "Cell %d at depth %d area: %f, poses: %d\n", cells.size(), depth,
               bounds1.area(), poses1.size() );
#endif
    }


    if( bounds2.area() > max_cell_area || poses2.size() > max_cell_poses )
    {
        recursive_split( poses2, bounds2, depth+1, cells );
    }
    else
    {
        Cell_Data new_cell( poses2, bounds2 );
        cells.push_back( new_cell );
#if VERBOSE
        printf( "Cell %d at depth %d area: %f, poses: %d\n", cells.size(), depth,
               bounds2.area(), poses2.size() );
#endif
    }

}


#endif
template <class A>
static void recursive_split_poses( const vector<const A *> &poses,
                                  const Bounds &bounds,
                                  unsigned int depth,
                                  vector<Cell_Data<A> > &cells)
{
    if( depth > max_tree_depth )
    {
        cerr << "ERROR - maximum recursion level reached in recursive_split" << endl;
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
    vector<const A*> poses1, poses2;
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

//----------------------------------------------------------------------------//
//  Public Functions                                                          //
//----------------------------------------------------------------------------//


vector<Stereo_Pose_Data> load_stereo_pose_file( const string &file_name )
{
    vector<Stereo_Pose_Data> poses;

    ifstream in_file( file_name.c_str() );
    if( !in_file )
    {
        cerr << "ERROR - Unable to load stereo pose file '" << file_name << "'"
             << endl;
        exit(1);
    }

    bool done = false;
    while( done == false )
    {
        Stereo_Pose_Data new_pose;

        if( in_file >> new_pose.id
                >> new_pose.time
                >> new_pose.pose[0]
                >> new_pose.pose[1]
                >> new_pose.pose[2]
                >> new_pose.pose[3]
                >> new_pose.pose[4]
                >> new_pose.pose[5]
                >> new_pose.left_name
                >> new_pose.right_name
                >> new_pose.alt
                >> new_pose.radius
                >> new_pose.overlap )
        {
            new_pose.file_name=new_pose.left_name;
            poses.push_back( new_pose );
        }
        else
        {
            done = true;
        }
    }

    return poses;
}



 template <class A>
vector<Cell_Data<A> > calc_cells( const vector<A> &poses,int max_poses )
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
    vector<const A *> node_poses( poses.size() );
    for( unsigned int i=0 ; i<poses.size() ; i++ )
        node_poses[i] = &poses[i];

    max_cell_poses= max_poses;


    // Perform binary division until termination criteria
    vector<Cell_Data<A> > cells;

    recursive_split_poses( node_poses, bounds,0, cells );

    return cells;
}
 void tmp(void){
     std::vector<Tex_Pose_Data> tasks;
     std::vector<Cell_Data<Tex_Pose_Data> > vrip_cells;
     vrip_cells=calc_cells<Tex_Pose_Data> (tasks,100);

     std::vector<Stereo_Pose_Data> tasks2;
     std::vector<Cell_Data<Stereo_Pose_Data> > vrip_cells2;
     vrip_cells2=calc_cells<Stereo_Pose_Data> (tasks2,100);
 }
