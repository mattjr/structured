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

using namespace std;

#define VERBOSE 0

#define POSE_INDEX_X 0 
#define POSE_INDEX_Y 1


//
// Parameters
//

static unsigned int num_split_samples = 100;
static double       max_cell_area     = 2500;  // square meters
static unsigned int max_cell_poses    = 300;
static unsigned int max_tree_depth    = 1000;  // Max recursion level 

static unsigned int max_cell_area_even    = 400;  
static unsigned int max_cell_poses_even    = 300;  

// Splitting cost heuristic parameters
static double pose_cost = 1;
static double overlap_cost = 50;
static double area_cost_multiplier = 1000;

// The spherical bound radii could be wrong due to vehicle roll or pitch,
// or a non-flat seafloor. The radii are increased to be more conservative
// FIXME: should this be done in seabed_slam? That would mean that the
//        detection of overlapping poses would also be more conservative
static double radius_multiplier = 3.0; 



//----------------------------------------------------------------------------//
//   Class Bounds                                                             //
//----------------------------------------------------------------------------//

Bounds::Bounds( )
{

}


Bounds::Bounds( const vector<Stereo_Pose_Data> &poses )
{
   for( unsigned int i=0; i<poses.size( ) ; i++ )
   {
      double x1 = poses[i].pose[0]-radius_multiplier*poses[i].radius;
      double x2 = poses[i].pose[0]+radius_multiplier*poses[i].radius;
      double y1 = poses[i].pose[1]-radius_multiplier*poses[i].radius;
      double y2 = poses[i].pose[1]+radius_multiplier*poses[i].radius;
 
      if( i==0 || x1 < min_x )
         min_x = x1;
      if( i==0 || x2 > max_x )
         max_x = x2;
      if( i==0 || y1 < min_y )
         min_y = y1;
      if( i==0 || y2 > max_y )
         max_y = y2;
   }
}

void Bounds::set( double min_x, double max_x, double min_y, double max_y )
{ 
   this->min_x = min_x;
   this->max_x = max_x;
   this->min_y = min_y;
   this->max_y = max_y;
}


double Bounds::area( void ) const
{
   return (max_x-min_x)*(max_y-min_y);
}

//----------------------------------------------------------------------------//
//   Class Cell_Data                                                          //
//----------------------------------------------------------------------------//

Cell_Data::Cell_Data( )
{ 

}

Cell_Data::Cell_Data( const vector<const Stereo_Pose_Data*> &poses,
                      const Bounds                          &bounds )
  : bounds( bounds),
    poses( poses )
{ 

}               



//----------------------------------------------------------------------------//
//  Private Helper Functions                                                  //
//----------------------------------------------------------------------------//


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
      bounds_min = bounds.min_x;
      bounds_max = bounds.max_x;
   }
   else
   {
      bounds_min = bounds.min_y;
      bounds_max = bounds.max_y;   
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
         costs[i] += 1000*(bounds.max_y-bounds.min_y)/(bounds.max_x-bounds.min_x);
      else
         costs[i] += 1000*(bounds.max_x-bounds.min_x)/(bounds.max_y-bounds.min_y);
   }
   
   // FIXME: Modify the weights for how evenly the samples are splitting up the 
   //        number of poses?



   return costs;
}                             

static void split_data( const vector<const Stereo_Pose_Data *> &poses,
                        const Bounds                           &bounds,
                        unsigned int                            split_axis,
                        double                                  split_value,
                        vector<const Stereo_Pose_Data *>       &poses1,
                        Bounds                                 &bounds1,
                        vector<const Stereo_Pose_Data *>       &poses2,
                        Bounds                                 &bounds2 )
{
   // Split the data - here we don't just consider if the center of the stereo
   // is in a cell, but if any part of the footprint is. Therefore a pose can
   // be in multiple cells.
   for( unsigned int i=0 ; i<poses.size() ; i++ )
   {
      if( poses[i]->pose[split_axis]-radius_multiplier*poses[i]->radius < split_value ) 
         poses1.push_back( poses[i] );

      if( poses[i]->pose[split_axis]+radius_multiplier*poses[i]->radius >= split_value )
         poses2.push_back( poses[i] );
   }


   // Create the bounds for the split data sets
   if( split_axis == POSE_INDEX_X )
   {
      bounds1.set( bounds.min_x, split_value , bounds.min_y, bounds.max_y );
      bounds2.set( split_value , bounds.max_x, bounds.min_y, bounds.max_y );
   }
   else
   {
      bounds1.set( bounds.min_x, bounds.max_x, bounds.min_y, split_value );
      bounds2.set( bounds.min_x, bounds.max_x, split_value , bounds.max_y );
   }
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
   double x_inc = (bounds.max_x-bounds.min_x)/(num_split_samples+1);
   double y_inc = (bounds.max_y-bounds.min_y)/(num_split_samples+1);
   for( unsigned int i=0 ; i<num_split_samples ; i++ )
   { 
      x_samples[i] = bounds.min_x + (i+1)*x_inc;
      y_samples[i] = bounds.min_y + (i+1)*y_inc;
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
   double best_split_point=(bounds.max_x-bounds.min_x)/2;
 
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



static void recursive_split_area( const vector<const Stereo_Pose_Data *> &poses,
                             const Bounds &bounds,
                             unsigned int depth,
                             vector<Cell_Data> &cells )
{
   if( depth > max_tree_depth )
   {
      cerr << "ERROR - maximum recursion level reached in recursive_split" << endl;
      exit(1);
   }



   if(bounds.area() < max_cell_area_even ){
     Cell_Data new_cell( poses, bounds );
     cells.push_back( new_cell );
     return ;
   }
   unsigned int best_axis;
   double best_split_point;

   if(bounds.max_x-bounds.min_x  > bounds.max_y-bounds.min_y){
     best_axis= POSE_INDEX_X ;
     best_split_point=(bounds.max_x+bounds.min_x)/2;
   }else{
     best_axis= POSE_INDEX_Y ;
     best_split_point=(bounds.max_y+bounds.min_y)/2;
   }
   // Perform the split
   vector<const Stereo_Pose_Data*> poses1, poses2;
   Bounds bounds1, bounds2;
   split_data( poses, bounds, best_axis, best_split_point, 
               poses1, bounds1, poses2, bounds2 );


   // Check if the subsets need to be recursively split
   if( bounds1.area() > max_cell_area_even )
   {   
      recursive_split_area( poses1, bounds1, depth+1, cells );
   }
   else
   {
      Cell_Data new_cell( poses1, bounds1 );
      cells.push_back( new_cell );
#if VERBOSE
      printf( "Area Cell %d at depth %d area: %f, poses: %d\n", cells.size(), depth,
              bounds1.area(), poses1.size() );
#endif
   }


   if( bounds2.area() > max_cell_area_even )
   {   
      recursive_split_area( poses2, bounds2, depth+1, cells );
   }
   else
   {
      Cell_Data new_cell( poses2, bounds2 );
      cells.push_back( new_cell );
#if VERBOSE
      printf( "Area Cell %d at depth %d area: %f, poses: %d\n", cells.size(), depth,
              bounds2.area(), poses2.size() );
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
         poses.push_back( new_pose );
      }
      else
      {
         done = true; 
      }
   }
   
   return poses;
} 




vector<Cell_Data> calc_cells( const vector<Stereo_Pose_Data> &poses,int method,double cell_scale ) 
{
   // Calculate the bounds of the stereo data
   Bounds bounds( poses );
#if VERBOSE
   printf( "Bounds: %f %f %f %f\n", bounds.min_x, bounds.max_x ,
                                    bounds.min_y, bounds.max_y );
#endif

   // Root node of the binary tree has all poses
   vector<const Stereo_Pose_Data *> node_poses( poses.size() );
   for( unsigned int i=0 ; i<poses.size() ; i++ )
       node_poses[i] = &poses[i];

   max_cell_area*=cell_scale;
   max_cell_poses*=cell_scale;

   // Perform binary division until termination criteria
   vector<Cell_Data> cells;
   if(method==AUV_SPLIT)
     recursive_split( node_poses, bounds, 1, cells );
   else
     recursive_split_area( node_poses, bounds, 1, cells );
   
   return cells;
}
                   

vector<Cell_Data> calc_cells( const vector<Stereo_Pose_Data> &poses,double x1,double x2,double y1 ,double y2,int numcells ) 
{
   // Calculate the bounds of the stereo data
   Bounds bounds;
   bounds.set(x1,x2,y1,y2);
#if VERBOSE
   printf( "Bounds: %f %f %f %f\n", bounds.min_x, bounds.max_x ,
                                    bounds.min_y, bounds.max_y );
#endif

  
   // Root node of the binary tree has all poses
   vector<const Stereo_Pose_Data *> node_poses( poses.size() );
   for( unsigned int i=0 ; i<poses.size() ; i++ )
       node_poses[i] = &poses[i];


   max_cell_area_even=   bounds.area()/numcells;

   // Perform binary division until termination criteria
   vector<Cell_Data> cells;
 
   recursive_split_area( node_poses, bounds, 1, cells );

   return cells;
}
