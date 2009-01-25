// quadtree.hpp	-thatcher 9/15/1999 Copyright 1999-2000 Thatcher Ulrich

// Data structures for quadtree terrain storage.

// This code may be freely modified and redistributed.  I make no
// warrantees about it; use at your own risk.  If you do incorporate
// this code into a project, I'd appreciate a mention in the credits.
//
// Thatcher Ulrich <tu@tulrich.com>



#ifndef QUADTREE_HPP
#define QUADTREE_HPP


#include "clip.hpp"
#include "geometry.hpp"
#include "colormap.hpp"
#include <ostream>
#include <limits.h>
#include <vector>
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef short int16;
typedef int int32;
#define UINT16_MAX_MINUS_ONE (USHRT_MAX-2)
class global_extents{
public:
  double min[3],max[3],range[3];
  double cell_size;
  int max_Level;
  double get_cell_size(int level){
    int	whole_cell_int_size = 2 << level;
    double tree_max_size=std::max(range[0],range[1]);
    return tree_max_size/whole_cell_int_size;
  }
  float toUINTz(double z){
    return z; //    return  std::min((int)((UINT16_MAX_MINUS_ONE)* ((z-min[2])/(range[2]))) ,UINT16_MAX_MINUS_ONE) +1;
      }
  
  float fromUINTz(float Z){
    return Z;    
//    return (((Z-1)/(float)UINT16_MAX_MINUS_ONE) *(range[2]))+ min[2];
  }

 double fromUINTzLocal(unsigned short Z){
    return (((Z-1)/(float)UINT16_MAX_MINUS_ONE) *(range[2]));
  }
  int get_in_cells(double p,int level){
    double cell_size=get_cell_size(level);
    return ((int)((p/cell_size) + 0.5));
  }
  void get_closest_res_level(double target_res,int &level, double &res){
    for(level=max_Level; level > 0; level--){
      res=get_cell_size(level);
      //printf("%f= %d\n",res,level);
      if(res > target_res){
	level=max_Level-level;
	break;      
      }
    }
  }
  friend  std::ostream& operator << (std::ostream& os, const global_extents &data){
    return os<<"Min X: "<< data.min[0] << " Y: "<< data.min[1] << " Z: "<< data.min[2] <<"\nMax X: "<< data.max[0] << " Y: "<< data.max[1] << " Z: " <<data.max[2] << "\nCell Size " << data.cell_size <<std::endl;
  }

  
};

extern global_extents ge;
struct HeightMapInfo {
	float*	Data;
	int	x_origin, y_origin;
	int	XSize, YSize;
	int	RowWidth;
	int	Scale;
  int index;
	float	Sample(int x, int y) const;
};


struct	VertInfo {
  //	uint16	Z;
  float  Z;
  unsigned char num_samples;
  float *Zsamples;
  unsigned short *Zsource;
  unsigned char shadowed;
  //	unsigned char	Lightness;	// For simple precomputed vertex lighting for purposes of the demo.  It's a waste of 2 bytes if we're texturing.

};
typedef struct _pt_3 {
  float x,y;
  float s[5];
}pt_3;

class quadsquare;


// A structure used during recursive traversal of the tree to hold
// relevant but transitory data.
struct quadcornerdata {
	const quadcornerdata*	Parent;
	quadsquare*	Square;
	int	ChildIndex;
	int	Level;
	int	xorg, yorg;
	VertInfo	Verts[4];	// ne, nw, sw, se
};


struct quadsquare {
	quadsquare*	Child[4];

	VertInfo	Vertex[5];	// center, e, n, w, s
	float	Error[6];	// e, s, children: ne, nw, sw, se
	float	MinZ, MaxZ;	// Bounds for frustum culling and error testing.
	unsigned char	EnabledFlags;	// bits 0-7: e, n, w, s, ne, nw, sw, se
	unsigned char	SubEnabledCount[2];	// e, s enabled reference counts.
	bool	Static;
	bool	Dirty;	// Set when vertex data has changed, but error/enabled data has not been recalculated.

// public:
	quadsquare(quadcornerdata* pcd);
	~quadsquare();

  void StaticUpdate(const quadcornerdata& cd, float Detail);
  void	AddHeightMap(const quadcornerdata& cd, const HeightMapInfo& hm,bool insert_sparse=false);
void AddShadowMap(const quadcornerdata& cd, const HeightMapInfo& hm,bool insert_sparse=false);
  void	AddPts(const quadcornerdata& cd,pt_3 *pts,int npts);
  void	AddPtsAux(const quadcornerdata& cd, pt_3 &pt,int minScale);
  void	StaticCullData(const quadcornerdata& cd, float ThresholdDetail);	void UpdateStats(const quadcornerdata& cd);
	float	RecomputeErrorAndLighting(const quadcornerdata& cd);
	int	CountNodes();
	
	void	Update(const quadcornerdata& cd, const float ViewerLocation[3], float Detail);
	int	Render(const quadcornerdata& cd, bool Textured);

	float	GetHeight(const quadcornerdata& cd, float x, float z);
	int	  RenderToWF(const quadcornerdata& cd);
private:
  bool check_valid(const quadcornerdata& cd, const HeightMapInfo& hm);
	void	EnableEdgeVertex(int index, bool IncrementCount, const quadcornerdata& cd);
	quadsquare*	EnableDescendant(int count, int stack[], const quadcornerdata& cd);
	void	EnableChild(int index, const quadcornerdata& cd);
	void	NotifyChildDisable(const quadcornerdata& cd, int index);

	void	ResetTree();
	void	StaticCullAux(const quadcornerdata& cd, float ThresholdDetail, int TargetLevel);
  void StaticUpdateAux(const quadcornerdata& cd,float CenterError);

	quadsquare*	GetNeighbor(int dir, const quadcornerdata& cd);
	void	CreateChild(int index, const quadcornerdata& cd);
  void	SetupCornerData(quadcornerdata* q, const quadcornerdata& pd, int ChildIndex);

  void RenderToWFAux(const quadcornerdata& cd);
	void	UpdateAux(const quadcornerdata& cd, const float ViewerLocation[3], float CenterError);
	void	RenderAux(const quadcornerdata& cd, bool Textured, Clip::Visibility vis);
	void	SetStatic(const quadcornerdata& cd);


class nFlatTriangleCorner {
public:
    int x,y;            // polar x/z 
    const VertInfo *vi;       // vertex info containing y coord, normal and index reuse data
  ul::vector pos;        // the triangle's corner position in model space
    bool pos_ok;        // false: pos noch nicht updated    
    nFlatTriangleCorner() {
        pos_ok = false;
    };
};
void AddTriangleToWF(quadsquare * /* usused qs */, 
                                   nFlatTriangleCorner *tc0, 
                                   nFlatTriangleCorner *tc1,
		     nFlatTriangleCorner *tc2);

};
extern int color_metric;
extern bool apply_color_wf;
extern bool render_no_data;
extern char *wf_fname;
extern double max_stat_val;
extern double min_stat_val;
extern std::vector<double> stat_vals;
extern bool save_stats;
enum {Z_SAMPLES,Z_ERR,SHADOWED,SIGNED_ERR};

#endif // QUADTREE_HPP
