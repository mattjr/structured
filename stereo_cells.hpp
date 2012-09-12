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



class Bounds  : public osg::BoundingBoxd
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

std::vector<Stereo_Pose_Data> load_stereo_pose_file( const std::string &file_name );

//template<class A>
//std::vector<Cell_Data<A> > calc_cells( const std::vector<A> &poses ,int max_poses);
//std::vector<Cell_Data> calc_cells( const std::vector<Tex_Pose_Data> &poses,double x1,double x2,double y1 ,double y2,double area );
template <class A>
        std::vector<Cell_Data<A> > calc_cells( const std::vector<A> &poses,int max_poses );

#endif // !AUV_STEREO_CELLS_HPP



