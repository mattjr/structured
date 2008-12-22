#include "quadtree.hpp"
//#define PLOTTING
#ifdef PLOTTING
#include <plplot/plplot.h>
#include <plplot/plstream.h>
#endif 
#include <math.h>
#include <ostream>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PolygonMode>
#include <osgUtil/DelaunayTriangulator>
#include <osg/Timer>
 class terrain_data{
public:
   double x,y,z;
friend  std::ostream& operator << (std::ostream& os, const terrain_data &data)
  {
    return os<<data.z<<std::endl;
  }

};
typedef quadtree_node<terrain_data> terrain_node;
class terrain_tree:public quadtree<terrain_data>{
public:
  terrain_tree(const Envelope<double>& extent,int maxdepth,double ratio,double min_extent_size):quadtree<terrain_data>(extent,maxdepth,ratio),min_extent_size_(min_extent_size)
  {
    osg_root = new osg::Group();
    tree_geode =new osg::Geode();
    tree_box_geometry = new osg::Geometry();
    tree_point_geometry = new osg::Geometry();
    terrain_geometry = new osg::Geometry();
    tree_geode->addDrawable(tree_point_geometry); 
    tree_geode->addDrawable(tree_box_geometry); 
    tree_geode->addDrawable(terrain_geometry); 
    osg_root->addChild(tree_geode);
    osg::StateSet *state = osg_root->getOrCreateStateSet();
    osg::PolygonMode *polyModeObj;
    polyModeObj = dynamic_cast< osg::PolygonMode* >(state->getAttribute( osg::StateAttribute::POLYGONMODE ));
    if ( !polyModeObj ) {
      polyModeObj = new osg::PolygonMode;
      state->setAttribute( polyModeObj );    
    }
    polyModeObj->setMode(  osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
    

 
    tree_box_vertices=  new osg::Vec3Array;
    terrain_vertices=  new osg::Vec3Array;
    tree_point_vertices=  new osg::Vec3Array;
#ifdef PLOTTING
    pls = new plstream();
 
    f_name ="plot.ps";

    PLOptionTable options[] = {
  

{
    "save",			/* For saving in postscript */
    NULL,
    NULL,
    &f_name,
    PL_OPT_STRING,
    "-save filename",
      "Save sombrero plot in color postscript `filename'" },
{
    NULL,			/* option */
    NULL,			/* handler */
    NULL,			/* client data */
    NULL,			/* address of variable to set */
    0,				/* mode flag */
    NULL,			/* short syntax */
    NULL }			/* long syntax */
};
const    char *argv[1];
argv[0]= "quadmerge" ;
// Parse an process command line arguments.
//pls->MergeOpts(options, "x20c options", NULL);
//pls->parseopts( 0, argv, PL_PARSE_FULL );

#endif
}

#ifdef PLOTTING
   void insert(const terrain_data& data,const Envelope<double>& item_ext)
    {
        insert(data,item_ext,root_);
    }
  
 
 void draw() const
    {
      pls->init();
	pls->adv (0);
	pls->vpor (0.0, 1.0, 0.0, 1.0);
	pls->wind (root_->ext_.minx(),root_->ext_.maxx(),
		   root_->ext_.miny(),root_->ext_.maxy());
	pls->col0 (0);
		pls->box ("", 10.0, 0, "", 10.0, 0);
	
	
	//	pls->box("bc", 1.0, 0, "bcnv", 10.0, 0);


		draw(root_);
    }
  void draw(const terrain_node *node,int level=0) const
    {
     
        if (node)
        {
	  PLFLT px[] = {node->ext_.minx(),node->ext_.minx(),
			node->ext_.maxx(),node->ext_.maxx()};

	  PLFLT py[] = {node->ext_.miny(),node->ext_.maxy(),
			node->ext_.miny(),node->ext_.maxy()};

	  if(level > 0)
	    pls->fill (4, px, py);

            for (int i=0;i<4;++i)
            {
	      draw(node->children_[i],level+1);
            }
	    
        }
    }
#endif
void insert(const terrain_data& data,double x,double y)
    {
      insert(data, Envelope<double>(x,y,x,y),root_);
    }
void insert(const terrain_data &data,const Envelope<double>& item_ext,quadtree_node<terrain_data>*  node )
    {
        if (node && node->ext_.contains(item_ext))
        {
            coord2d c=node->ext_.center();

            double width=node->ext_.width();
            double height=node->ext_.height();

            double lox=node->ext_.minx();
            double loy=node->ext_.miny();
            double hix=node->ext_.maxx();
            double hiy=node->ext_.maxy();
	    
            Envelope<double> ext[4];
            ext[0]=Envelope<double>(lox,loy,lox + width * ratio_,loy + height * ratio_);
            ext[1]=Envelope<double>(hix - width * ratio_,loy,hix,loy + height * ratio_);
            ext[2]=Envelope<double>(lox,hiy - height*ratio_,lox + width * ratio_,hiy);
            ext[3]=Envelope<double>(hix - width * ratio_,hiy - height*ratio_,hix,hiy);

            if (width > min_extent_size_ || height > min_extent_size_)
            {
                for (int i=0;i<4;++i)
                {
                   
                        if (!node->children_[i])
			{
                            node->children_[i]=new terrain_node(ext[i]);
                        }

		}
		for(int i=0; i<4; ++i){
		  if (ext[i].contains(item_ext)) {
		    insert(data,item_ext,node->children_[i]);
		    return;
		  }
		  
		}

            }
	 
            node->data_.push_back(data);
        }
    }
  void render_tree() const {
    
		render_tree(root_);
		//	osg::DrawElementsUInt* tree_base = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, tree_box_vertices->size());
		
		tree_box_geometry->setVertexArray(tree_box_vertices);
	       	tree_box_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,tree_box_vertices->size()));

		tree_point_geometry->setVertexArray(tree_point_vertices);
	       	tree_point_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,tree_point_vertices->size()));
	
    }
  void render_tree(const terrain_node *node,int level=0) const{
    
    if (node){
      std::vector<terrain_data>::const_iterator itr=node->data_.begin();
      tree_box_vertices->push_back( osg::Vec3(node->ext_.minx(),node->ext_.miny(),0.0));
      tree_box_vertices->push_back( osg::Vec3( node->ext_.minx(),node->ext_.maxy(),0.0));
      tree_box_vertices->push_back( osg::Vec3( node->ext_.maxx(),node->ext_.maxy(),0.0));
      tree_box_vertices->push_back (osg::Vec3( node->ext_.maxx(),node->ext_.miny(),0.0));
      while(itr!=node->data_.end()) {
	tree_point_vertices->push_back(osg::Vec3(itr->x,itr->y,0.0));
	++itr;
      }
      for (int i=0;i<4;++i){
	render_tree(node->children_[i],level+1);
      }
    }
  }



  void render_terrain() const {
    
		render_terrain(root_);
		//	osg::DrawElementsUInt* tree_base = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, tree_box_vertices->size());
		// create triangulator and set the points as the area
		osg::ref_ptr<osgUtil::DelaunayTriangulator> trig = new osgUtil::DelaunayTriangulator();
		trig->setInputPointArray(terrain_vertices);
		printf("Triangulating %d pts...\n",(int)terrain_vertices->size());
		const osg::Timer& timer = *osg::Timer::instance();
		osg::Timer_t start_tick = timer.tick();
		
	   
		trig->triangulate();
		         
		double duration = timer.delta_s(start_tick,timer.tick());
		printf("Elapsed time for Triangulate %f\n",duration);
		terrain_geometry->setVertexArray(terrain_vertices);
		terrain_geometry->addPrimitiveSet(trig->getTriangles());

 
		//		terrain_geometry->setVertexArray(terrain_vertices);
		// 	terrain_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,terrain_vertices->size()));
		
    }
osg::Vec3 get_estimated_pos(const std::vector<terrain_data> &data) const{
  osg::Vec3 sum;
    unsigned int i=0;
    if(data.size() ==0)
      return osg::Vec3(0,0,0);
    for(i=0; i < data.size(); i++){
      sum+=      osg::Vec3(data[i].x,data[i].y,data[i].z);
    }
    sum[0]/=(double)i;
    sum[1]/=(double)i;
    sum[2]/=(double)i;
    return sum;

  }
  void render_terrain(const terrain_node *node) const{
    
 
    
    if (node ){
      if(node->children_[0] == NULL && node->data_.size()){

	terrain_vertices->push_back( get_estimated_pos(node->data_));
      }
    
      for (int i=0;i<4;++i){
	render_terrain(node->children_[i]);
      }
    }
  }
private:
#ifdef PLOTTING
   plstream *pls;
const char *f_name;
#endif
  double zmin,zmax,min_extent_size_;
osg::Geode* tree_geode;
osg::Geometry *tree_box_geometry;
osg::Geometry *tree_point_geometry;
osg::Geometry *terrain_geometry;
osg::Vec3Array* terrain_vertices;
osg::Vec3Array* tree_box_vertices;
osg::Vec3Array* tree_point_vertices;
public :
osg::Group* osg_root;
};

