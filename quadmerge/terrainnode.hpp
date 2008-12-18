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
 class terrain_data{
public:
  double z;
friend  std::ostream& operator << (std::ostream& os, const terrain_data &data)
  {
    return os<<data.z<<std::endl;
  }

};
typedef quadtree_node<terrain_data> terrain_node;
class terrain_tree:public quadtree<terrain_data>{
public:
  terrain_tree(const Envelope<double>& extent,int maxdepth,double ratio):quadtree<terrain_data>(extent,maxdepth,ratio)
  {
    osg_root = new osg::Group();
    tree_geode =new osg::Geode();
    tree_geometry = new osg::Geometry();
    tree_geode->addDrawable(tree_geometry); 
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
void render_tree() const
    {
    
		render_tree(root_);
		//	osg::DrawElementsUInt* tree_base = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, tree_box_vertices->size());
		
		tree_geometry->setVertexArray(tree_box_vertices);
	       	tree_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,tree_box_vertices->size()));//	tree_geometry->addPrimitiveSet(tree_base);
	
    }
  void render_tree(const terrain_node *node,int level=0) const{
    
    if (node){
      tree_box_vertices->push_back( osg::Vec3(node->ext_.minx(),node->ext_.miny(),0.0));
      tree_box_vertices->push_back( osg::Vec3( node->ext_.minx(),node->ext_.maxy(),0.0));
      tree_box_vertices->push_back( osg::Vec3( node->ext_.maxx(),node->ext_.maxy(),0.0));
      tree_box_vertices->push_back (osg::Vec3( node->ext_.maxx(),node->ext_.miny(),0.0));
      
      for (int i=0;i<4;++i){
	render_tree(node->children_[i],level+1);
      }
    }
  }
private:
#ifdef PLOTTING
   plstream *pls;
const char *f_name;
#endif

osg::Geode* tree_geode;
osg::Geometry *tree_geometry;

osg::Vec3Array* tree_box_vertices;
public :
osg::Group* osg_root;
};

