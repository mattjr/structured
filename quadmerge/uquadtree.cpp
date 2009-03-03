// quadtree.cpp	-thatcher 9/15/1999 Copyright 1999-2000 Thatcher Ulrich

// Code for quadtree terrain manipulation, meshing, and display.

// This code may be freely modified and redistributed.  I make no
// warrantees about it; use at your own risk.  If you do incorporate
// this code into a project, I'd appreciate a mention in the credits.
//
// Thatcher Ulrich <tu@tulrich.com>


#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>

#else
#include <GL/glx.h> /* this includes the necessary X headers */
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#endif
#include <limits.h>
#include <float.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "uquadtree.hpp"
#include "geometry.hpp"
#include "sample.hpp"
#include "auv_mesh_utils.hpp"
#include "colormap.hpp"
#include "robust/RobustStats.hpp"

#define NO_DATA 0
//using ul::vector;
#include "fileio.hpp"
global_extents ge;
int color_metric=Z_ERR;
using namespace ul;
bool render_no_data=false;
bool apply_color_wf=false;
//
// quadsquare functions.
//
char *wf_fname;
bool save_stats=false;
std::vector<double> stat_vals;
double	max_stat_val=DBL_MIN;
double	min_stat_val=DBL_MAX;
quadsquare::quadsquare(quadcornerdata* pcd)
// Constructor.
{
	pcd->Square = this;
	
	// Set static to true if/when this node contains real data, and
	// not just interpolated values.  When static == false, a node
	// can be deleted by the Update() function if none of its
	// vertices or children are enabled.
	Static = false;
	
	int	i;
	for (i = 0; i < 4; i++) {
		Child[i] = NULL;
	}

	EnabledFlags = 0;
	
	for (i = 0; i < 2; i++) {
		SubEnabledCount[i] = 0;
	}


	for(i=0; i<5; i++){
	  Vertex[i].Zsamples=NULL;
	  Vertex[i].Zsource=NULL;
	  Vertex[i].num_samples=0;
	}
	  
	// Set default vertex positions by interpolating from given corners.
	// Just bilinear interpolation.
	bool isnodata=false;
	for(int i=0; i< 4; i++)
	  if(pcd->Verts[i].Z == NO_DATA)
	    isnodata=true;
	
	if(isnodata){
	  for(int i=0; i < 5; i++)
	    Vertex[i].Z=NO_DATA;
	}else{
	  Vertex[0].Z = 0.25 * (pcd->Verts[0].Z + pcd->Verts[1].Z + pcd->Verts[2].Z + pcd->Verts[3].Z);
	  Vertex[1].Z = 0.5 * (pcd->Verts[3].Z + pcd->Verts[0].Z);
	  Vertex[2].Z = 0.5 * (pcd->Verts[0].Z + pcd->Verts[1].Z);
	  Vertex[3].Z = 0.5 * (pcd->Verts[1].Z + pcd->Verts[2].Z);
	  Vertex[4].Z = 0.5 * (pcd->Verts[2].Z + pcd->Verts[3].Z);
	}
	for (i = 0; i < 2; i++) {
		Error[i] = 0;
	}
	for (i = 0; i < 4; i++) {
		Error[i+2] = fabs((Vertex[0].Z + pcd->Verts[i].Z) - (Vertex[i+1].Z + Vertex[((i+1)&3) + 1].Z)) * 0.25;
	}

	// Compute MinY/MaxZ based on corner verts.
	MinZ = MaxZ = pcd->Verts[0].Z;
	for (i = 1; i < 4; i++) {
		float	y = pcd->Verts[i].Z;
		if (y < MinZ) MinZ = y;
		if (y > MaxZ) MaxZ = y;
	}
	
	// Initialize colors by interpolating from corners.
	/*Vertex[0].Lightness = (pcd->Verts[0].Lightness + pcd->Verts[1].Lightness + pcd->Verts[2].Lightness + pcd->Verts[3].Lightness) >> 2;
	Vertex[1].Lightness = (pcd->Verts[3].Lightness + pcd->Verts[0].Lightness) >> 1;
	Vertex[2].Lightness = (pcd->Verts[0].Lightness + pcd->Verts[1].Lightness) >> 1;
	Vertex[3].Lightness = (pcd->Verts[1].Lightness + pcd->Verts[2].Lightness) >> 1;
	Vertex[4].Lightness = (pcd->Verts[2].Lightness + pcd->Verts[3].Lightness) >> 1;*/

	Vertex[0].aux = std::max(std::max(pcd->Verts[0].aux , pcd->Verts[1].aux ),std::max(pcd->Verts[2].aux , pcd->Verts[3].aux));
	Vertex[1].aux = std::max(pcd->Verts[3].aux , pcd->Verts[0].aux);
	Vertex[2].aux = std::max(pcd->Verts[0].aux , pcd->Verts[1].aux);
	Vertex[3].aux = std::max (pcd->Verts[1].aux , pcd->Verts[2].aux);
	Vertex[4].aux = std::max (pcd->Verts[2].aux ,pcd->Verts[3].aux);
}


quadsquare::~quadsquare()
// Destructor.
{
	// Recursively delete sub-trees.
	int	i;
	for (i = 0; i < 4; i++) {
		if (Child[i]) delete Child[i];
		Child[i] = NULL;
	}
}


void	quadsquare::SetStatic(const quadcornerdata& cd)
// Sets this node's static flag to true.  If static == true, then the
// node or its children is considered to contain significant height data
// and shouldn't be deleted.
{
	if (Static == false) {
		Static = true;
		
		// Propagate static status to ancestor nodes.
		if (cd.Parent && cd.Parent->Square) {
			cd.Parent->Square->SetStatic(*cd.Parent);
		}
	}
}


int	quadsquare::CountNodes()
// Debugging function.  Counts the number of nodes in this subtree.
{
	int	count = 1;	// Count ourself.

	// Count descendants.
	for (int i = 0; i < 4; i++) {
		if (Child[i]) count += Child[i]->CountNodes();
	}

	return count;
}


float	quadsquare::GetHeight(const quadcornerdata& cd, float x, float z)
// Returns the height of the heightfield at the specified x,z coordinates.
{
	int	half = 1 << cd.Level;

	float	lx = (x - cd.xorg) / float(half);
	float	lz = (z - cd.yorg) / float(half);
	
	int	ix = floor(lx);
	int	iz = floor(lz);

	// Clamp.
	if (ix < 0) ix = 0;
	if (ix > 1) ix = 1;
	if (iz < 0) iz = 0;
	if (iz > 1) iz = 1;

	int	index = ix ^ (iz ^ 1) + (iz << 1);

	if (Child[index] && Child[index]->Static) {
		// Pass the query down to the child which contains it.
		quadcornerdata	q;
		SetupCornerData(&q, cd, index);
		return Child[index]->GetHeight(q, x, z);
	}

	// Bilinear interpolation.
	lx -= ix;
	if (lx < 0) lx = 0;
	if (lx > 1) lx = 1;
	
	lz -= iz;
	if (lx < 0) lz = 0;
	if (lz > 1) lz = 1;

	float	s00, s01, s10, s11;
	switch (index) {
	default:
	case 0:
		s00 = Vertex[2].Z;
		s01 = cd.Verts[0].Z;
		s10 = Vertex[0].Z;
		s11 = Vertex[1].Z;
		break;
	case 1:
		s00 = cd.Verts[1].Z;
		s01 = Vertex[2].Z;
		s10 = Vertex[3].Z;
		s11 = Vertex[0].Z;
		break;
	case 2:
		s00 = Vertex[3].Z;
		s01 = Vertex[0].Z;
		s10 = cd.Verts[2].Z;
		s11 = Vertex[4].Z;
		break;
	case 3:
		s00 = Vertex[0].Z;
		s01 = Vertex[1].Z;
		s10 = Vertex[4].Z;
		s11 = cd.Verts[3].Z;
		break;
	}

	return (s00 * (1-lx) + s01 * lx) * (1 - lz) + (s10 * (1-lx) + s11 * lx) * lz;
}


quadsquare*	quadsquare::GetNeighbor(int dir, const quadcornerdata& cd)
// Traverses the tree in search of the quadsquare neighboring this square to the
// specified direction.  0-3 --> { E, N, W, S }.
// Returns NULL if the neighbor is outside the bounds of the tree.
{
	// If we don't have a parent, then we don't have a neighbor.
	// (Actually, we could have inter-tree connectivity at this level
	// for connecting separate trees together.)
	if (cd.Parent == 0) return 0;
	
	// Find the parent and the child-index of the square we want to locate or create.
	quadsquare*	p = 0;
	
	int	index = cd.ChildIndex ^ 1 ^ ((dir & 1) << 1);
	bool	SameParent = ((dir - cd.ChildIndex) & 2) ? true : false;
	
	if (SameParent) {
		p = cd.Parent->Square;
	} else {
		p = cd.Parent->Square->GetNeighbor(dir, *cd.Parent);
		
		if (p == 0) return 0;
	}
	
	quadsquare*	n = p->Child[index];
	
	return n;
}


static ul::vector	SunVector(0.0705, -0.9875, -0.1411);	// For demo lighting.  Pick some unit vector pointing roughly downward.


unsigned char	MakeLightness(float xslope, float zslope)
// Generates an 8-bit lightness value, given a surface slope.
{
	ul::vector	norm(-xslope, -1, -zslope);	// Actually the negative of the surface slope.
	norm.normalize();

	float	dot = norm * SunVector;

	int	c = 255 - (1 - dot) * 300;
	if (c < 0) c = 0;
	if (c > 255) c = 255;
	c *= 0x010101;
	c |= 0xFF000000;

	return c;
}


unsigned int	MakeColor(unsigned char Lightness)
// Makes an ARGB color, given an 8-bit lightness value.
// Just replicates the components and uses FF for alpha.
{
	return 0xFF000000 | (Lightness << 16) | (Lightness << 8) | Lightness;
}


float	quadsquare::RecomputeErrorAndLighting(const quadcornerdata& cd)
// Recomputes the error values for this tree.  Returns the
// max error.
// Also updates MinZ & MaxZ.
// Also computes quick & dirty vertex lighting for the demo.
{
	int	i;
	
	// Measure error of center and edge vertices.
	float	maxerror = 0;

	// Compute error of center vert.
	float	e;
	if (cd.ChildIndex & 1) {
		e = fabs(Vertex[0].Z - (cd.Verts[1].Z + cd.Verts[3].Z) * 0.5);
	} else {
		e = fabs(Vertex[0].Z - (cd.Verts[0].Z + cd.Verts[2].Z) * 0.5);
	}
	if (e > maxerror) maxerror = e;

	// Initial min/max.
	MaxZ = Vertex[0].Z;
	MinZ = Vertex[0].Z;

	// Check min/max of corners.
	for (i = 0; i < 4; i++) {
		float	y = cd.Verts[i].Z;
		if (y < MinZ) MinZ = y;
		if (y > MaxZ) MaxZ = y;
	}
	
	// Edge verts.
	e = fabs(Vertex[1].Z - (cd.Verts[0].Z + cd.Verts[3].Z) * 0.5);
	if (e > maxerror) maxerror = e;
	Error[0] = e;
	
	e = fabs(Vertex[4].Z - (cd.Verts[2].Z + cd.Verts[3].Z) * 0.5);
	if (e > maxerror) maxerror = e;
	Error[1] = e;

	// Min/max of edge verts.
	for (i = 0; i < 4; i++) {
		float	y = Vertex[1 + i].Z;
		if (y < MinZ) MinZ = y;
		if (y > MaxZ) MaxZ = y;
	}
	
	// Check child squares.
	for (i = 0; i < 4; i++) {
		quadcornerdata	q;
		if (Child[i]) {
			SetupCornerData(&q, cd, i);
			Error[i+2] = Child[i]->RecomputeErrorAndLighting(q);

			if (Child[i]->MinZ < MinZ) MinZ = Child[i]->MinZ;
			if (Child[i]->MaxZ > MaxZ) MaxZ = Child[i]->MaxZ;
		} else {
			// Compute difference between bilinear average at child center, and diagonal edge approximation.
			Error[i+2] = fabs((Vertex[0].Z + cd.Verts[i].Z) - (Vertex[i+1].Z + Vertex[((i+1)&3) + 1].Z)) * 0.25;
		}
		if (Error[i+2] > maxerror) maxerror = Error[i+2];
	}


	//
	// Compute quickie demo lighting.
	//
	/*
	float	OneOverSize = 1.0 / (2 << cd.Level);
	Vertex[0].Lightness = MakeLightness((Vertex[1].Z - Vertex[3].Z) * OneOverSize,
					    (Vertex[4].Z - Vertex[2].Z) * OneOverSize);

	float	v;
	quadsquare*	s = GetNeighbor(0, cd);
	if (s) v = s->Vertex[0].Z; else v = Vertex[1].Z;
	Vertex[1].Lightness = MakeLightness((v - Vertex[0].Z) * OneOverSize,
				    (cd.Verts[3].Z - cd.Verts[0].Z) * OneOverSize);
	
	s = GetNeighbor(1, cd);
	if (s) v = s->Vertex[0].Z; else v = Vertex[2].Z;
	Vertex[2].Lightness = MakeLightness((cd.Verts[0].Z - cd.Verts[1].Z) * OneOverSize,
				    (Vertex[0].Z - v) * OneOverSize);
	
	s = GetNeighbor(2, cd);
	if (s) v = s->Vertex[0].Z; else v = Vertex[3].Z;
	Vertex[3].Lightness = MakeLightness((Vertex[0].Z - v) * OneOverSize,
				    (cd.Verts[2].Z - cd.Verts[1].Z) * OneOverSize);
	
	s = GetNeighbor(3, cd);
	if (s) v = s->Vertex[0].Z; else v = Vertex[4].Z;
	Vertex[4].Lightness = MakeLightness((cd.Verts[3].Z - cd.Verts[2].Z) * OneOverSize,
				    (v - Vertex[0].Z) * OneOverSize);
	*/

	for(int i=0; i <5; i++)
	  Vertex[i].aux=0;

	// The error, MinZ/MaxZ, and lighting values for this node and descendants are correct now.
	Dirty = false;
	
	return maxerror;
}


void	quadsquare::ResetTree()
// Clear all enabled flags, and delete all non-static child nodes.
{
	int	i;
	for (i = 0; i < 4; i++) {
		if (Child[i]) {
			Child[i]->ResetTree();
			if (Child[i]->Static == false) {
				delete Child[i];
				Child[i] = 0;
			}
		}
	}
	EnabledFlags = 0;
	SubEnabledCount[0] = 0;
	SubEnabledCount[1] = 0;
	Dirty = true;
}


void	quadsquare::StaticCullData(const quadcornerdata& cd, float ThresholdDetail)
// Examine the tree and remove nodes which don't contain necessary
// detail.  Necessary detail is defined as vertex data with a
// edge-length to height ratio less than ThresholdDetail.
{
	// First, clean non-static nodes out of the tree.
	ResetTree();

	// Make sure error values are up-to-date.
	if (Dirty) RecomputeErrorAndLighting(cd);
	
	// Recursively check all the nodes and do necessary removal.
	// We must start at the bottom of the tree, and do one level of
	// the tree at a time, to ensure the dependencies are accounted
	// for properly.
	int	level;
	for (level = 0; level < 15; level++) {
		StaticCullAux(cd, ThresholdDetail, level);
	}
}


void	quadsquare::StaticCullAux(const quadcornerdata& cd, float ThresholdDetail, int TargetLevel)
// Check this node and its descendents, and remove nodes which don't contain
// necessary detail.
{
	int	i, j;
	quadcornerdata	q;

	if (cd.Level > TargetLevel) {
		// Just recurse to child nodes.
		for (j = 0; j < 4; j++) {
			if (j < 2) i = 1 - j;
			else i = j;

			if (Child[i]) {
				SetupCornerData(&q, cd, i);
				Child[i]->StaticCullAux(q, ThresholdDetail, TargetLevel);
			}
		}
		return;
	}

	// We're at the target level.  Check this node to see if it's OK to delete it.

	// Check edge vertices to see if they're necessary.
	float	size = 2 << cd.Level;	// Edge length.
	if (Child[0] == NULL && Child[3] == NULL && Error[0] * ThresholdDetail < size) {
		quadsquare*	s = GetNeighbor(0, cd);
		if (s == NULL || (s->Child[1] == NULL && s->Child[2] == NULL)) {

			// Force vertex height to the edge value.
			float	y = (cd.Verts[0].Z + cd.Verts[3].Z) * 0.5;
			Vertex[1].Z = y;
			Error[0] = 0;
			
			// Force alias vertex to match.
			if (s) s->Vertex[3].Z = y;
			
			Dirty = true;
		}
	}

	if (Child[2] == NULL && Child[3] == NULL && Error[1] * ThresholdDetail < size) {
		quadsquare*	s = GetNeighbor(3, cd);
		if (s == NULL || (s->Child[0] == NULL && s->Child[1] == NULL)) {
			float	y = (cd.Verts[2].Z + cd.Verts[3].Z) * 0.5;
			Vertex[4].Z = y;
			Error[1] = 0;
			
			if (s) s->Vertex[2].Z = y;
			
			Dirty = true;
		}
	}

	// See if we have child nodes.
	bool	StaticChildren = false;
	for (i = 0; i < 4; i++) {
		if (Child[i]) {
			StaticChildren = true;
			if (Child[i]->Dirty) Dirty = true;
		}
	}

	// If we have no children and no necessary edges, then see if we can delete ourself.
	if (StaticChildren == false && cd.Parent != NULL) {
		bool	NecessaryEdges = false;
		for (i = 0; i < 4; i++) {
			// See if vertex deviates from edge between corners.
			float	diff = fabs(Vertex[i+1].Z - (cd.Verts[i].Z + cd.Verts[(i+3)&3].Z) * 0.5);
			if (diff > 0.00001) {
				NecessaryEdges = true;
			}
		}

		if (!NecessaryEdges) {
			size *= 1.414213562;	// sqrt(2), because diagonal is longer than side.
			if (cd.Parent->Square->Error[2 + cd.ChildIndex] * ThresholdDetail < size) {
				delete cd.Parent->Square->Child[cd.ChildIndex];	// Delete this.
				cd.Parent->Square->Child[cd.ChildIndex] = 0;	// Clear the pointer.
			}
		}
	}
}


int	MaxCreateDepth = 0;


void	quadsquare::EnableEdgeVertex(int index, bool IncrementCount, const quadcornerdata& cd)
// Enable the specified edge vertex.  Indices go { e, n, w, s }.
// Increments the appropriate reference-count if IncrementCount is true.
{
	if ((EnabledFlags & (1 << index)) && IncrementCount == false) return;
	
	//static const int	Inc[4] = { 1, 0, 0, 8 };

	// Turn on flag and deal with reference count.
	EnabledFlags |= 1 << index;
	if (IncrementCount == true && (index == 0 || index == 3)) {
		SubEnabledCount[index & 1]++;
	}

	// Now we need to enable the opposite edge vertex of the adjacent square (i.e. the alias vertex).

	// This is a little tricky, since the desired neighbor node may not exist, in which
	// case we have to create it, in order to prevent cracks.  Creating it may in turn cause
	// further edge vertices to be enabled, propagating updates through the tree.

	// The sticking point is the quadcornerdata list, which
	// conceptually is just a linked list of activation structures.
	// In this function, however, we will introduce branching into
	// the "list", making it in actuality a tree.  This is all kind
	// of obscure and hard to explain in words, but basically what
	// it means is that our implementation has to be properly
	// recursive.

	// Travel upwards through the tree, looking for the parent in common with our desired neighbor.
	// Remember the path through the tree, so we can travel down the complementary path to get to the neighbor.
	quadsquare*	p = this;
	const quadcornerdata*	pcd = &cd;
	int	ct = 0;
	int	stack[32];
	for (;;) {
		int	ci = pcd->ChildIndex;

		if (pcd->Parent == NULL || pcd->Parent->Square == NULL) {
			// Neighbor doesn't exist (it's outside the tree), so there's no alias vertex to enable.
			return;
		}
		p = pcd->Parent->Square;
		pcd = pcd->Parent;

		bool	SameParent = ((index - ci) & 2) ? true : false;
		
		ci = ci ^ 1 ^ ((index & 1) << 1);	// Child index of neighbor node.

		stack[ct] = ci;
		ct++;
		
		if (SameParent) break;
	}

	// Get a pointer to our neighbor (create if necessary), by walking down
	// the quadtree from our shared ancestor.
	p = p->EnableDescendant(ct, stack, *pcd);
	
/*
	// Travel down the tree towards our neighbor, enabling and creating nodes as necessary.  We'll
	// follow the complement of the path we used on the way up the tree.
	quadcornerdata	d[16];
	int	i;
	for (i = 0; i < ct; i++) {
		int	ci = stack[ct-i-1];

		if (p->Child[ci] == NULL && CreateDepth == 0) CreateDepth = ct-i;	//xxxxxxx
		
		if ((p->EnabledFlags & (16 << ci)) == 0) {
			p->EnableChild(ci, *pcd);
		}
		p->SetupCornerData(&d[i], *pcd, ci);
		p = p->Child[ci];
		pcd = &d[i];
	}
*/

	// Finally: enable the vertex on the opposite edge of our neighbor, the alias of the original vertex.
	index ^= 2;
	p->EnabledFlags |= (1 << index);
	if (IncrementCount == true && (index == 0 || index == 3)) {
		p->SubEnabledCount[index & 1]++;
	}
}


quadsquare*	quadsquare::EnableDescendant(int count, int path[], const quadcornerdata& cd)
// This function enables the descendant node 'count' generations below
// us, located by following the list of child indices in path[].
// Creates the node if necessary, and returns a pointer to it.
{
	count--;
	int	ChildIndex = path[count];

	if ((EnabledFlags & (16 << ChildIndex)) == 0) {
		EnableChild(ChildIndex, cd);
	}
	
	if (count > 0) {
		quadcornerdata	q;
		SetupCornerData(&q, cd, ChildIndex);
		return Child[ChildIndex]->EnableDescendant(count, path, q);
	} else {
		return Child[ChildIndex];
	}
}


void	quadsquare::CreateChild(int index, const quadcornerdata& cd)
// Creates a child square at the specified index.
{
	if (Child[index] == 0) {
		quadcornerdata	q;
		SetupCornerData(&q, cd, index);
		
		Child[index] = new quadsquare(&q);
	}
}


void	quadsquare::EnableChild(int index, const quadcornerdata& cd)
// Enable the indexed child node.  { ne, nw, sw, se }
// Causes dependent edge vertices to be enabled.
{
//	if (Enabled[index + 4] == false) {
	if ((EnabledFlags & (16 << index)) == 0) {
//		Enabled[index + 4] = true;
		EnabledFlags |= (16 << index);
		EnableEdgeVertex(index, true, cd);
		EnableEdgeVertex((index + 1) & 3, true, cd);
		
		if (Child[index] == 0) {
			CreateChild(index, cd);
		}
	}
}


int	BlockDeleteCount = 0;	//xxxxx
int	BlockUpdateCount = 0;	//xxxxx


void	quadsquare::NotifyChildDisable(const quadcornerdata& cd, int index)
// Marks the indexed child quadrant as disabled.  Deletes the child node
// if it isn't static.
{
	// Clear enabled flag for the child.
	EnabledFlags &= ~(16 << index);
	
	// Update child enabled counts for the affected edge verts.
	quadsquare*	s;
	
	if (index & 2) s = this;
	else s = GetNeighbor(1, cd);
	if (s) {
		s->SubEnabledCount[1]--;
	}
	
	if (index == 1 || index == 2) s = GetNeighbor(2, cd);
	else s = this;
	if (s) {
		s->SubEnabledCount[0]--;
	}
	
	if (Child[index]->Static == false) {
		delete Child[index];
		Child[index] = 0;

		BlockDeleteCount++;//xxxxx
	}
}


static float	DetailThreshold = 100;


bool	VertexTest(float x, float y, float z, float error, const float Viewer[3])
// Returns true if the vertex at (x,z) with the given world-space error between
// its interpolated location and its true location, should be enabled, given that
// the viewpoint is located at Viewer[].
{
	float	dx = fabs(x - Viewer[0]);
	float	dy = fabs(y - Viewer[1]);
	float	dz = fabs(z - Viewer[2]);
	float	d = dx;
	if (dy > d) d = dy;
	if (dz > d) d = dz;

	return (error * DetailThreshold) > d;
}


bool	BoxTest(float x, float z, float size, float miny, float maxy, float error, const float Viewer[3])
// Returns true if any vertex within the specified box (origin at x,z,
// edges of length size) with the given error value could be enabled
// based on the given viewer location.
{
	// Find the minimum distance to the box.
	float	half = size * 0.5;
	float	dx = fabs(x + half - Viewer[0]) - half;
	float	dy = fabs((miny + maxy) * 0.5 - Viewer[1]) - (maxy - miny) * 0.5;
	float	dz = fabs(z + half - Viewer[2]) - half;
	float	d = dx;
	if (dy > d) d = dy;
	if (dz > d) d = dz;

	return (error * DetailThreshold) > d;
}


//const float	VERTICAL_SCALE = 1.0 / 8.0;
const float	VERTICAL_SCALE = 1.0;

void	quadsquare::StaticUpdate(const quadcornerdata& cd, float Detail)
// Refresh the vertex enabled states in the tree, according to the
// location of the viewer.  May force creation or deletion of qsquares
// in areas which need to be interpolated.
{
	DetailThreshold = Detail;
	
	StaticUpdateAux(cd,0);
}

bool	DetailTest(int level,float error)
// edge-length to height ratio less than ThresholdDetail.
{
  //float	size = 2 << level;	// Edge length.
	//	return (error * DetailThreshold) > size;
	//	if(level > 10)
	  return false;
}
void	quadsquare::StaticUpdateAux(const quadcornerdata& cd,float CenterError)
// Does the actual work of updating enabled states and tree growing/shrinking.
{
	BlockUpdateCount++;	//xxxxx
	
	// Make sure error values are current.
	if (Dirty) {
		RecomputeErrorAndLighting(cd);
	}

	int	half = 1 << cd.Level;
	int	whole = half << 1;

	// See about enabling child verts.
	/*	if ((EnabledFlags & 1) == 0 && VertexTest(cd.xorg + whole, Vertex[1].Z, cd.yorg + half, Error[0], ViewerLocation) == true) EnableEdgeVertex(0, false, cd);	// East vert.
	if ((EnabledFlags & 8) == 0 && VertexTest(cd.xorg + half, Vertex[4].Z, cd.yorg + whole, Error[1], ViewerLocation) == true) EnableEdgeVertex(3, false, cd);	// South vert.
	if (cd.Level > 0) {
		if ((EnabledFlags & 32) == 0) {
			if (BoxTest(cd.xorg, cd.yorg, half, MinZ, MaxZ, Error[3], ViewerLocation) == true) EnableChild(1, cd);	// nw child.er
		}
		if ((EnabledFlags & 16) == 0) {
			if (BoxTest(cd.xorg + half, cd.yorg, half, MinZ, MaxZ, Error[2], ViewerLocation) == true) EnableChild(0, cd);	// ne child.
		}
		if ((EnabledFlags & 64) == 0) {
			if (BoxTest(cd.xorg, cd.yorg + half, half, MinZ, MaxZ, Error[4], ViewerLocation) == true) EnableChild(2, cd);	// sw child.
		}
		if ((EnabledFlags & 128) == 0) {
			if (BoxTest(cd.xorg + half, cd.yorg + half, half, MinZ, MaxZ, Error[5], ViewerLocation) == true) EnableChild(3, cd);	// se child.
		}
	*/
	float ViewerLocation[3]={0,0,0};
	if (cd.Level > 0) {

		// Recurse into child quadrants as necessary.
		quadcornerdata	q;
		
		if (EnabledFlags & 32) {
			SetupCornerData(&q, cd, 1);
			Child[1]->StaticUpdateAux(q, Error[3]);
		}
		if (EnabledFlags & 16) {
			SetupCornerData(&q, cd, 0);
			Child[0]->StaticUpdateAux(q,  Error[2]);
		}
		if (EnabledFlags & 64) {
			SetupCornerData(&q, cd, 2);
			Child[2]->StaticUpdateAux(q,  Error[4]);
		}
		if (EnabledFlags & 128) {
			SetupCornerData(&q, cd, 3);
			Child[3]->StaticUpdateAux(q,  Error[5]);
		}
	}
	
	// Test for disabling.  East, South, and center.
	if ((EnabledFlags & 1) && SubEnabledCount[0] == 0 && VertexTest(cd.xorg + whole, Vertex[1].Z, cd.yorg + half, Error[0], ViewerLocation) == false) {
		EnabledFlags &= ~1;
		quadsquare*	s = GetNeighbor(0, cd);
		if (s) s->EnabledFlags &= ~4;
	}
	if ((EnabledFlags & 8) && SubEnabledCount[1] == 0 && VertexTest(cd.xorg + half, Vertex[4].Z, cd.yorg + whole, Error[1], ViewerLocation) == false) {
		EnabledFlags &= ~8;
		quadsquare*	s = GetNeighbor(3, cd);
		if (s) s->EnabledFlags &= ~2;
	}
	if (EnabledFlags == 0 &&
	    cd.Parent != NULL &&
	    BoxTest(cd.xorg, cd.yorg, whole, MinZ, MaxZ, CenterError, ViewerLocation) == false)
	{
		// Disable ourself.
		cd.Parent->Square->NotifyChildDisable(*cd.Parent, cd.ChildIndex);	// nb: possibly deletes 'this'.
	}

}


void	quadsquare::Update(const quadcornerdata& cd, const float ViewerLocation[3], float Detail)
// Refresh the vertex enabled states in the tree, according to the
// location of the viewer.  May force creation or deletion of qsquares
// in areas which need to be interpolated.
{
	DetailThreshold = Detail * VERTICAL_SCALE;
	
	UpdateAux(cd, ViewerLocation, 0);
}


void	quadsquare::UpdateAux(const quadcornerdata& cd, const float ViewerLocation[3], float CenterError)
// Does the actual work of updating enabled states and tree growing/shrinking.
{
	BlockUpdateCount++;	//xxxxx
	
	// Make sure error values are current.
	if (Dirty) {
		RecomputeErrorAndLighting(cd);
	}

	int	half = 1 << cd.Level;
	int	whole = half << 1;
	//	float worldhalf=half*ge.cell_size;
	//float worldwhole=whole*ge.cell_size;

	// See about enabling child verts.
	if ((EnabledFlags & 1) == 0 && VertexTest(cd.xorg + whole, Vertex[1].Z, cd.yorg + half, Error[0], ViewerLocation) == true) EnableEdgeVertex(0, false, cd);	// East vert.
	if ((EnabledFlags & 8) == 0 && VertexTest(cd.xorg + half, Vertex[4].Z, cd.yorg + whole, Error[1], ViewerLocation) == true) EnableEdgeVertex(3, false, cd);	// South vert.
	if (cd.Level > 0) {
		if ((EnabledFlags & 32) == 0) {
			if (BoxTest(cd.xorg, cd.yorg, half, MinZ, MaxZ, Error[3], ViewerLocation) == true) EnableChild(1, cd);	// nw child.er
		}
		if ((EnabledFlags & 16) == 0) {
			if (BoxTest(cd.xorg + half, cd.yorg, half, MinZ, MaxZ, Error[2], ViewerLocation) == true) EnableChild(0, cd);	// ne child.
		}
		if ((EnabledFlags & 64) == 0) {
			if (BoxTest(cd.xorg, cd.yorg + half, half, MinZ, MaxZ, Error[4], ViewerLocation) == true) EnableChild(2, cd);	// sw child.
		}
		if ((EnabledFlags & 128) == 0) {
			if (BoxTest(cd.xorg + half, cd.yorg + half, half, MinZ, MaxZ, Error[5], ViewerLocation) == true) EnableChild(3, cd);	// se child.
		}
		
		// Recurse into child quadrants as necessary.
		quadcornerdata	q;
		
		if (EnabledFlags & 32) {
			SetupCornerData(&q, cd, 1);
			Child[1]->UpdateAux(q, ViewerLocation, Error[3]);
		}
		if (EnabledFlags & 16) {
			SetupCornerData(&q, cd, 0);
			Child[0]->UpdateAux(q, ViewerLocation, Error[2]);
		}
		if (EnabledFlags & 64) {
			SetupCornerData(&q, cd, 2);
			Child[2]->UpdateAux(q, ViewerLocation, Error[4]);
		}
		if (EnabledFlags & 128) {
			SetupCornerData(&q, cd, 3);
			Child[3]->UpdateAux(q, ViewerLocation, Error[5]);
		}
	}
	
	// Test for disabling.  East, South, and center.
	if ((EnabledFlags & 1) && SubEnabledCount[0] == 0 && VertexTest(cd.xorg + whole, Vertex[1].Z, cd.yorg + half, Error[0], ViewerLocation) == false) {
		EnabledFlags &= ~1;
		quadsquare*	s = GetNeighbor(0, cd);
		if (s) s->EnabledFlags &= ~4;
	}
	if ((EnabledFlags & 8) && SubEnabledCount[1] == 0 && VertexTest(cd.xorg + half, Vertex[4].Z, cd.yorg + whole, Error[1], ViewerLocation) == false) {
		EnabledFlags &= ~8;
		quadsquare*	s = GetNeighbor(3, cd);
		if (s) s->EnabledFlags &= ~2;
	}
	if (EnabledFlags == 0 &&
	    cd.Parent != NULL &&
	    BoxTest(cd.xorg, cd.yorg, whole, MinZ, MaxZ, CenterError, ViewerLocation) == false)
	{
		// Disable ourself.
		cd.Parent->Square->NotifyChildDisable(*cd.Parent, cd.ChildIndex);	// nb: possibly deletes 'this'.
	}
}





float	VertexArray[9 * 3];
unsigned int	ColorArray[9];
unsigned char	VertList[24];
int	TriCount = 0;
int wf_num_tris=0;
FILE *wf_fp=NULL;

static void	InitVert(int index, float x, float y, float z)
// Initializes the indexed vertex of VertexArray[] with the
// given values.
{
	int	i = index * 3;
	VertexArray[i] = x*ge.cell_size;
	//printf("%f y, %f  \n",y,((y/(float)UINT16_MAX_MINUS_ONE) *zrange)+ zmin);
       	//VertexArray[i+1] = ((y/(float)UINT16_MAX_MINUS_ONE) *(ge.range[2]))+ ge.min[2];
	VertexArray[i+1] = ge.fromUINTz(y);//((y/(float)UINT16_MAX_MINUS_ONE) *(ge.range[2]))+ ge.min[2];
	VertexArray[i+2] = z*ge.cell_size;
}


int	quadsquare::Render(const quadcornerdata& cd, bool Textured)
// Draws the heightfield represented by this tree.
// Returns the number of triangles rendered.
{
	TriCount = 0;

	// Do some initial setup, then do all the work in RenderAux().
//	gl....();
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, VertexArray);

	//xxxx
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glScalef(1.0, VERTICAL_SCALE, 1.0);
	
	if (!Textured) {
		// No texture; use crummy vertex lighting.
		glEnableClientState(GL_COLOR_ARRAY);
		glColorPointer(4, GL_UNSIGNED_BYTE, 0, ColorArray);
	} else {
		// Set up automatic texture-coordinate generation.
		// Basically we're just stretching the current texture over the entire 64K x 64K terrain.
		glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		float	p[4] = { 1.0 / 65536, 0, 0, 0 };
		glTexGenfv(GL_S, GL_OBJECT_PLANE, p);

		glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
		p[0] = 0;	p[2] = 1.0 / 65536;
		glTexGenfv(GL_T, GL_OBJECT_PLANE, p);

		glEnable(GL_TEXTURE_GEN_S);
		glEnable(GL_TEXTURE_GEN_T);
	}

	RenderAux(cd, Textured, Clip::NO_CLIP);

	glDisable(GL_TEXTURE_GEN_S);
	glDisable(GL_TEXTURE_GEN_T);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
//	gl...();

	glPopMatrix();

	return TriCount;
}


void	quadsquare::RenderAux(const quadcornerdata& cd, bool Textured, Clip::Visibility vis)
// Does the work of rendering this square.  Uses the enabled vertices only.
// Recurses as necessary.
{
  
	int	half = 1 << cd.Level;
	int	whole = 2 << cd.Level;

	
  

	
	//	printf("Level %f %d\n",cellsizex,cd.Level);
	//	printf("Level %d h %d whole %d %d %d\n",cd.Level,half,whole,1 << 1 , 1<< 0);
	// If this square is outside the frustum, then don't render it.
	/*if (vis != Clip::NO_CLIP) {
		float	min[3], max[3];
		min[0] = cd.xorg;
		min[1] = MinZ * VERTICAL_SCALE;
		min[2] = cd.yorg;
		max[0] = cd.xorg + whole;
		max[1] = MaxZ * VERTICAL_SCALE;
		max[2] = cd.yorg + whole;
		vis = Clip::ComputeBoxVisibility(min, max);
		if (vis == Clip::NOT_VISIBLE) {
			// This square is completely outside the view frustum.
			return;
		}
		// else vis is either NO_CLIP or SOME_CLIP.  If it's NO_CLIP, then child
		// squares won't have to bother with the frustum check.
		}*/
	
	int	i;

	int	flags = 0;
	int	mask = 1;
	quadcornerdata	q;
	for (i = 0; i < 4; i++, mask <<= 1) {
		if (EnabledFlags & (16 << i)) {
			SetupCornerData(&q, cd, i);
			Child[i]->RenderAux(q, Textured, vis);
		} else {
			flags |= mask;
		}
	}

	if (flags == 0) return;

//	// xxx debug color.
//	glColor3f(cd.Level * 10 / 255.0, ((cd.Level & 3) * 60 + ((cd.yorg >> cd.Level) & 255)) / 255.0, ((cd.Level & 7) * 30 + ((cd.xorg >> cd.Level) & 255)) / 255.0);
	
	// Init vertex data.
		InitVert(0, cd.xorg + half, Vertex[0].Z, cd.yorg + half);
	InitVert(1, cd.xorg + whole, Vertex[1].Z, cd.yorg + half);
	InitVert(2, cd.xorg + whole, cd.Verts[0].Z, cd.yorg);
	InitVert(3, cd.xorg + half, Vertex[2].Z, cd.yorg);
	InitVert(4, cd.xorg, cd.Verts[1].Z, cd.yorg);
	InitVert(5, cd.xorg, Vertex[3].Z, cd.yorg + half);
	InitVert(6, cd.xorg, cd.Verts[2].Z, cd.yorg + whole);
	InitVert(7, cd.xorg + half, Vertex[4].Z, cd.yorg + whole);
	InitVert(8, cd.xorg + whole, cd.Verts[3].Z, cd.yorg + whole);

	/*	InitVert(0, cd.xorg + worldhalfx, Vertex[0].Z, cd.yorg + worldhalfy);
	InitVert(1, cd.xorg + worldwholex, Vertex[1].Z, cd.yorg + worldhalfy);
	InitVert(2, cd.xorg + worldwholex, cd.Verts[0].Z, cd.yorg);
	InitVert(3, cd.xorg + worldhalfx, Vertex[2].Z, cd.yorg);
	InitVert(4, cd.xorg, cd.Verts[1].Z, cd.yorg);
	InitVert(5, cd.xorg, Vertex[3].Z, cd.yorg + worldhalfy);
	InitVert(6, cd.xorg, cd.Verts[2].Z, cd.yorg + worldwholey);
	InitVert(7, cd.xorg + worldhalfx, Vertex[4].Z, cd.yorg + worldwholey);
	InitVert(8, cd.xorg + worldwholex, cd.Verts[3].Z, cd.yorg + worldwholey);*/
	for(int i=0; i< 9; i++)
	  ColorArray[i]=0xFFFFFFFF;
	 
	/*	if (!Textured) {
		ColorArray[0] = MakeColor(Vertex[0].Lightness);
		ColorArray[1] = MakeColor(Vertex[1].Lightness);
		ColorArray[2] = MakeColor(cd.Verts[0].Lightness);
		ColorArray[3] = MakeColor(Vertex[2].Lightness);
		ColorArray[4] = MakeColor(cd.Verts[1].Lightness);
		ColorArray[5] = MakeColor(Vertex[3].Lightness);
		ColorArray[6] = MakeColor(cd.Verts[2].Lightness);
		ColorArray[7] = MakeColor(Vertex[4].Lightness);
		ColorArray[8] = MakeColor(cd.Verts[3].Lightness);
	}
	*/
	int	vcount = 0;
	
// Local macro to make the triangle logic shorter & hopefully clearer.
#define tri(a,b,c) ( VertList[vcount++] = a, VertList[vcount++] = b, VertList[vcount++] = c )

	// Make the list of triangles to draw.
	if ((EnabledFlags & 1) == 0) tri(0, 8, 2);
	else {
		if (flags & 8) tri(0, 8, 1);
		if (flags & 1) tri(0, 1, 2);
	}
	if ((EnabledFlags & 2) == 0) tri(0, 2, 4);
	else {
		if (flags & 1) tri(0, 2, 3);
		if (flags & 2) tri(0, 3, 4);
	}
	if ((EnabledFlags & 4) == 0) tri(0, 4, 6);
	else {
		if (flags & 2) tri(0, 4, 5);
		if (flags & 4) tri(0, 5, 6);
	}
	if ((EnabledFlags & 8) == 0) tri(0, 6, 8);
	else {
		if (flags & 4) tri(0, 6, 7);
		if (flags & 8) tri(0, 7, 8);
	}

	// Draw 'em.
	glDrawElements(GL_TRIANGLES, vcount, GL_UNSIGNED_BYTE, VertList);

	// Count 'em.
	TriCount += vcount / 3;
}


void	quadsquare::SetupCornerData(quadcornerdata* q, const quadcornerdata& cd, int ChildIndex)
// Fills the given structure with the appropriate corner values for the
// specified child block, given our own vertex data and our corner
// vertex data from cd.
//
// ChildIndex mapping:
// +-+-+
// |1|0|
// +-+-+
// |2|3|
// +-+-+
//
// Verts mapping:
// 1-0
// | |
// 2-3
//
// Vertex mapping:
// +-2-+
// | | |
// 3-0-1
// | | |
// +-4-+
{
	int	half = 1 << cd.Level;

	q->Parent = &cd;
	q->Square = Child[ChildIndex];
	q->Level = cd.Level - 1;
	q->ChildIndex = ChildIndex;
	
	switch (ChildIndex) {
	default:
	case 0:
		q->xorg = cd.xorg + half;
		q->yorg = cd.yorg;
		q->Verts[0] = cd.Verts[0];
		q->Verts[1] = Vertex[2];
		q->Verts[2] = Vertex[0];
		q->Verts[3] = Vertex[1];
		break;

	case 1:
		q->xorg = cd.xorg;
		q->yorg = cd.yorg;
		q->Verts[0] = Vertex[2];
		q->Verts[1] = cd.Verts[1];
		q->Verts[2] = Vertex[3];
		q->Verts[3] = Vertex[0];
		break;

	case 2:
		q->xorg = cd.xorg;
		q->yorg = cd.yorg + half;
		q->Verts[0] = Vertex[0];
		q->Verts[1] = Vertex[3];
		q->Verts[2] = cd.Verts[2];
		q->Verts[3] = Vertex[4];
		break;

	case 3:
		q->xorg = cd.xorg + half;
		q->yorg = cd.yorg + half;
		q->Verts[0] = Vertex[1];
		q->Verts[1] = Vertex[0];
		q->Verts[2] = Vertex[4];
		q->Verts[3] = cd.Verts[3];
		break;
	}	
}



void	quadsquare::AddHeightMap(const quadcornerdata& cd, const HeightMapInfo& hm,bool insert_sparse)
// Sets the height of all samples within the specified rectangular
// region using the given array of floats.  Extends the tree to the
// level of detail defined by (1 << hm.Scale) as necessary.
{
	// If block is outside rectangle, then don't bother.
	int	BlockSize = 2 << cd.Level;
	int source_idx=hm.index;

	if (cd.xorg > hm.x_origin + ((hm.XSize + 2) << hm.Scale) ||
	    cd.xorg + BlockSize < hm.x_origin - (1 << hm.Scale) ||
	    cd.yorg > hm.y_origin + ((hm.YSize + 2) << hm.Scale) ||
	    cd.yorg + BlockSize < hm.y_origin - (1 << hm.Scale))
	{
	  // This square does not touch the given height array area; no need to modify this square or descendants.
		return;
	}

	if (cd.Parent && cd.Parent->Square) {
		cd.Parent->Square->EnableChild(cd.ChildIndex, *cd.Parent);	// causes parent edge verts to be enabled, possibly causing neighbor blocks to be created.
	}
	bool children_valid=(Child[0]!=NULL &&Child[1]!=NULL &&Child[2]!=NULL &&Child[3]!=NULL);
	if(insert_sparse && (!children_valid && !check_valid(cd,hm) ))
	  return;
	int	i;
	
	int	half = 1 << cd.Level;

	// Create and update child nodes.
	for (i = 0; i < 4; i++) {
		quadcornerdata	q;
		SetupCornerData(&q, cd, i);
				
		if (Child[i] == NULL && cd.Level > hm.Scale) {
			// Create child node w/ current (unmodified) values for corner verts.
			Child[i] = new quadsquare(&q);
		}
		
		// Recurse.
		if (Child[i]) {
		  Child[i]->AddHeightMap(q, hm,insert_sparse);
		}
	}
	
	// Deviate vertex heights based on data sampled from heightmap.
	float	s[5];
	s[0] = hm.Sample(cd.xorg + half, cd.yorg + half);
	s[1] = hm.Sample(cd.xorg + half*2, cd.yorg + half);
	s[2] = hm.Sample(cd.xorg + half, cd.yorg);
	s[3] = hm.Sample(cd.xorg, cd.yorg + half);
	s[4] = hm.Sample(cd.xorg + half, cd.yorg + half*2);

	// Modify the vertex heights if necessary, and set the dirty
	// flag if any modifications occur, so that we know we need to
	// recompute error data later.
	for (i = 0; i < 5; i++) {
		if (s[i] != 0) {
			Dirty = true;
			Vertex[i].num_samples++;
			Vertex[i].Zsamples=(float *)realloc(Vertex[i].Zsamples,Vertex[i].num_samples*sizeof(float));
			Vertex[i].Zsamples[Vertex[i].num_samples-1]=s[i];
			Vertex[i].Zsource=(unsigned short *)realloc(Vertex[i].Zsource,Vertex[i].num_samples*sizeof(unsigned short));
			Vertex[i].Zsource[Vertex[i].num_samples-1]=source_idx;
			  
			Vertex[i].Z = s[i];
		
		
		}
	}

	if (!Dirty) {
		// Check to see if any child nodes are dirty, and set the dirty flag if so.
		for (i = 0; i < 4; i++) {
			if (Child[i] && Child[i]->Dirty) {
				Dirty = true;
				break;
			}
		}
	}

	if (Dirty) SetStatic(cd);
}

bool quadsquare::check_valid(const quadcornerdata& cd, const HeightMapInfo& hm){
	int	BlockSize = 2 << cd.Level;
	int ix,iy;

	for(int i=cd.xorg; i < cd.xorg + BlockSize; i++){
	  for(int j=cd.yorg; j < cd.yorg + BlockSize; j++){
	    ix = (i - hm.x_origin) >> hm.Scale;
	    iy = (j - hm.y_origin) >> hm.Scale;
	    if(ix >= hm.XSize || ix < 0 || iy >= hm.YSize || iy < 0)
	      	      continue;
	    if(hm.Data[ix + iy *hm.RowWidth] != 0)
	      return true;
	      
	  }
	}
	return false;
}

void	quadsquare::AddShadowMap(const quadcornerdata& cd, const HeightMapInfo& hm,bool insert_sparse)
// Sets the height of all samples within the specified rectangular
// region using the given array of floats.  Extends the tree to the
// level of detail defined by (1 << hm.Scale) as necessary.
{
	// If block is outside rectangle, then don't bother.
	int	BlockSize = 2 << cd.Level;
	if (cd.xorg > hm.x_origin + ((hm.XSize + 2) << hm.Scale) ||
	    cd.xorg + BlockSize < hm.x_origin - (1 << hm.Scale) ||
	    cd.yorg > hm.y_origin + ((hm.YSize + 2) << hm.Scale) ||
	    cd.yorg + BlockSize < hm.y_origin - (1 << hm.Scale))
	{
	  // This square does not touch the given height array area; no need to modify this square or descendants.
		return;
	}

	if (cd.Parent && cd.Parent->Square) {
		cd.Parent->Square->EnableChild(cd.ChildIndex, *cd.Parent);	// causes parent edge verts to be enabled, possibly causing neighbor blocks to be created.
	}
	bool children_valid=(Child[0]!=NULL &&Child[1]!=NULL &&Child[2]!=NULL &&Child[3]!=NULL);
	if(insert_sparse && (!children_valid && !check_valid(cd,hm) ))
	  return;
	int	i;
	
	int	half = 1 << cd.Level;

	// Create and update child nodes.
	for (i = 0; i < 4; i++) {
		quadcornerdata	q;
		SetupCornerData(&q, cd, i);	
		// Recurse.
		if (Child[i]) {
		  Child[i]->AddShadowMap(q, hm,insert_sparse);
		}
	}
	
	// Deviate vertex heights based on data sampled from heightmap.
	unsigned char	s[5];
	s[0] = hm.Sample(cd.xorg + half, cd.yorg + half);
	s[1] = hm.Sample(cd.xorg + half*2, cd.yorg + half);
	s[2] = hm.Sample(cd.xorg + half, cd.yorg);
	s[3] = hm.Sample(cd.xorg, cd.yorg + half);
	s[4] = hm.Sample(cd.xorg + half, cd.yorg + half*2);

	// Modify the vertex heights if necessary, and set the dirty
	// flag if any modifications occur, so that we know we need to
	// recompute error data later.
	for (i = 0; i < 5; i++) {
		if (s[i] != 0) {
		  //	Dirty = true;					  
			Vertex[i].aux = s[i];
		
		
		}
	}

	/*	if (!Dirty) {
		// Check to see if any child nodes are dirty, and set the dirty flag if so.
		for (i = 0; i < 4; i++) {
			if (Child[i] && Child[i]->Dirty) {
				Dirty = true;
				break;
			}
		}
	}
	*/
	//	if (Dirty) SetStatic(cd);
}

//
// HeightMapInfo
//


float	HeightMapInfo::Sample(int x, int z) const
// Returns the height (y-value) of a point in this heightmap.  The given (x,z) are in
// world coordinates.  Heights outside this heightmap are considered to be 0.  Heights
// between sample points are bilinearly interpolated from surrounding points.
// xxx deal with edges: either force to 0 or over-size the query region....
{
	// Break coordinates into grid-relative coords (ix,iz) and remainder (rx,rz).
	
	int	ix = (x - x_origin) >> Scale;
	int	iz = (z - y_origin) >> Scale;

	int	mask = (1 << Scale) - 1;

	int	rx = (x - x_origin) & mask;
	int	rz = (z - y_origin) & mask;
 
	if (ix < 0 || ix >= XSize-1 || iz < 0 || iz >= YSize-1) return 0;	// Outside the grid.

	float	fx = float(rx) / (mask + 1);
	float	fz = float(rz) / (mask + 1);

	float	s00 = Data[ix + iz * RowWidth];
	float	s01 = Data[(ix+1) + iz * RowWidth];
	float	s10 = Data[ix + (iz+1) * RowWidth];
	float	s11 = Data[(ix+1) + (iz+1) * RowWidth];

	//Don't interpolate from no data
	if(s00 ==NO_DATA || s01 == NO_DATA ||s10 == NO_DATA ||s11==NO_DATA)
	  return 0;
	
	return (s00 * (1-fx) + s01 * fx) * (1-fz) +
		(s10 * (1-fx) + s11 * fx) * fz;
}
void quadsquare::UpdateStats(const quadcornerdata& cd)
{
    

    // recursively go down to child node
    int i;
    int flags = 0;
    int mask = 1;
    quadcornerdata q;
    for (i=0; i<4; i++, mask<<=1) {
        if (EnabledFlags & (16<<i)) {
            	SetupCornerData(&q, cd, i);
			Child[i]->UpdateStats(q);
		} else {
			flags |= mask;
		}
	}


    if (flags == 0) return;
    for(i=0; i< 5; i++){
      double val=0.0;
      if(!Vertex[i].Zsamples)
	continue;

      if(color_metric == Z_SAMPLES){
	val = Vertex[i].num_samples;
      }else if(color_metric == Z_ERR){
	val=stddev(Vertex[i].Zsamples,Vertex[i].num_samples);
      }
      else if(color_metric == Z_VAR){
	//	val=var(Vertex[i].Zsamples,Vertex[i].num_samples);
	val=square_err(Vertex[i].Zsamples,Vertex[i].Zsource,
		       Vertex[i].num_samples);
      }else if(color_metric == SIGNED_ERR){
	val=signed_err(Vertex[i].Zsamples,Vertex[i].Zsource,
		       Vertex[i].num_samples);
      }

      if(Vertex[i].num_samples  >1 && save_stats)
	stat_vals.push_back(val);
      if( val > max_stat_val )
	max_stat_val=val;
      if(val < min_stat_val)
	min_stat_val=val; 
    }
    
}

void quadsquare::UpdateDiffs(const quadcornerdata& cd)
{
    

    // recursively go down to child node
    int i;
    int flags = 0;
    int mask = 1;
    quadcornerdata q;
    for (i=0; i<4; i++, mask<<=1) {
        if (EnabledFlags & (16<<i)) {
            	SetupCornerData(&q, cd, i);
			Child[i]->UpdateDiffs(q);
		} else {
			flags |= mask;
		}
	}


    if (flags == 0) return;
  
    float	v;
    double diffX,diffY;
    diffX=0.0;
    diffY=0.0;
    if(Vertex[0].Z ==0 || Vertex[1].Z ==0  || Vertex[2].Z == 0 || Vertex[3].Z == 0  || Vertex[4].Z ==0 )
      return;
    float vert0=mest(Vertex[0]);
    quadsquare*	s = GetNeighbor(0, cd);
    if (s) v = mest(s->Vertex[0]); else v = mest(Vertex[1]);
    //E
        if(Vertex[1].num_samples && Vertex[0].num_samples )
      diffX+=pow(vert0-v,2);
    
    s = GetNeighbor(1, cd);
    if (s) v = mest(s->Vertex[0]); else v = mest(Vertex[2]);
    //N
        if(Vertex[2].num_samples && Vertex[0].num_samples  )
    diffY+=pow(vert0-v,2);
    
    s = GetNeighbor(2, cd);
    if (s) v = mest(s->Vertex[0]); else v = mest(Vertex[3]);
    //W
        if(Vertex[3].num_samples && Vertex[0].num_samples )
    diffX+=pow(vert0-v,2);
    
    s = GetNeighbor(3, cd);
    if (s) v = mest(s->Vertex[0]); else v = mest(Vertex[4]);
    //S
        if(Vertex[4].num_samples  && Vertex[0].num_samples)
      diffY+=pow(vert0-v,2);


	printf("%f %f %d %d %d %d %d\n",diffX,diffY,Vertex[0].num_samples,
	       Vertex[1].num_samples,
	       Vertex[2].num_samples,
	       Vertex[3].num_samples,
	       Vertex[4].num_samples);
    Vertex[0].aux=diffX+diffY;
    if(Vertex[0].num_samples+
	       Vertex[1].num_samples+
	       Vertex[2].num_samples+
	       Vertex[3].num_samples+
       Vertex[4].num_samples < 6)
      return;
  if(  Vertex[0].aux > max_stat_val )
    max_stat_val= Vertex[0].aux ;
      if( Vertex[0].aux  < min_stat_val)
	min_stat_val= Vertex[0].aux ; 


    
    
}

//-------------------------------------------------------------------
//  RenderToWF()
//  Render quadtree to a wavefront file. You should run a few
//  Update()'s over the tree with an appropriate detail error
//  and the viewer positioned at (0,0,0) before calling
//  RenderToWF().
//  20-Jul-00   floh    created
//-------------------------------------------------------------------
int	quadsquare::RenderToWF(const quadcornerdata& cd)
{
  wf_num_tris=0;
  int vnum=0;
  wf_fp=fopen(wf_fname,"wb");
  ply_header(wf_fp,wf_num_tris,vnum,false,apply_color_wf);
  RenderToWFAux(cd);
  vnum = wf_num_tris * 3;
  int i;
  int buf[3];
  for (i=0; i<vnum; i+=3) {
    //    fprintf(wf_fp,"f %d %d %d\n",i,i+1,i+2);
    unsigned char fc=3;
    fwrite((char *)&fc,sizeof(unsigned char),1,wf_fp);
    buf[0]=i+2;
    buf[1]=i+1;
    buf[2]=i+0;
    fwrite((char *)buf,sizeof(int),3,wf_fp);

    //  fp->PutS(buf);
  }
  ply_header(wf_fp,wf_num_tris,vnum,false,apply_color_wf);
  fclose(wf_fp);
  printf("Wrote %d Faces %d Vertcies\n",wf_num_tris,vnum);
  return 0;
}


void quadsquare::RenderToWFAux(const quadcornerdata& cd)
{
    int half = 1 << cd.Level;
    int whole = 2 << cd.Level;

    // recursively go down to child node
    int i;
    int flags = 0;
    int mask = 1;
    quadcornerdata q;
    for (i=0; i<4; i++, mask<<=1) {
        if (EnabledFlags & (16<<i)) {
            	SetupCornerData(&q, cd, i);
			Child[i]->RenderToWFAux(q);
		} else {
			flags |= mask;
		}
	}


    if (flags == 0) return;
    //    if(!Static && !render_non_static) return;
    nFlatTriangleCorner tc[9];
    int x0 = cd.xorg;
    int x1 = cd.xorg + half;
    int x2 = cd.xorg + whole;
    int z0 = cd.yorg;
    int z1 = cd.yorg + half;
    int z2 = cd.yorg + whole;
    if(color_metric == RUGOSITY){
      for(int i=1; i < 5; i++)
	Vertex[i].aux=Vertex[0].aux;
    }

    tc[0].x=x1; tc[0].y=z1; tc[0].vi=&(Vertex[0]);
    tc[1].x=x2; tc[1].y=z1; tc[1].vi=&(Vertex[1]);
    tc[2].x=x2; tc[2].y=z0; tc[2].vi=&(cd.Verts[0]);
    tc[3].x=x1; tc[3].y=z0; tc[3].vi=&(Vertex[2]);
    tc[4].x=x0; tc[4].y=z0; tc[4].vi=&(cd.Verts[1]);
    tc[5].x=x0; tc[5].y=z1; tc[5].vi=&(Vertex[3]);
    tc[6].x=x0; tc[6].y=z2; tc[6].vi=&(cd.Verts[2]);
    tc[7].x=x1; tc[7].y=z2; tc[7].vi=&(Vertex[4]);
    tc[8].x=x2; tc[8].y=z2; tc[8].vi=&(cd.Verts[3]);

#undef tri
#define tri(a,b,c) (AddTriangleToWF(this,&(tc[c]),&(tc[b]),&(tc[a])))

    // Make the list of triangles to draw.
    if ((EnabledFlags & 1) == 0) tri(0, 8, 2);
    else {
        if (flags & 8) tri(0, 8, 1);
        if (flags & 1) tri(0, 1, 2);
    }
    if ((EnabledFlags & 2) == 0) tri(0, 2, 4);
    else {
        if (flags & 1) tri(0, 2, 3);
        if (flags & 2) tri(0, 3, 4);
    }
    if ((EnabledFlags & 4) == 0) tri(0, 4, 6);
    else {
        if (flags & 2) tri(0, 4, 5);
        if (flags & 4) tri(0, 5, 6);
    }
    if ((EnabledFlags & 8) == 0) tri(0, 6, 8);
    else {
        if (flags & 4) tri(0, 6, 7);
        if (flags & 8) tri(0, 7, 8);
    }
}




float get_avg_Z(float *samples,int num){
  double avg=0;
  for(int i=0; i < num; i++)
    avg+=samples[i];
      
      return (avg/num);

}

float get_clipped_avg_Z(float *samples,int num,float val,float dist){
  double avg=0;
  int counted=0;
  for(int i=0; i < num; i++){
    if(fabs(samples[i] - val) < dist){
      avg+=samples[i];
      counted++;
    }
  }
  if(counted == 0)
    return val;
  
  return (avg/counted);

}

float get_max_Z(float *samples,int num){
  double max=DBL_MIN;
  for(int i=0; i < num; i++)
    if(samples[i] > max)
      max=samples[i];
      
      return (max);

}


float get_min_Z(float *samples,int num){
  double min=DBL_MAX;
  for(int i=0; i < num; i++)
    if(samples[i] < min)
      min=samples[i];
      
      return (min);

}
float mest(const VertInfo &vert){
  if(vert.num_samples == 0)
    return 0.0;

  if(vert.num_samples < 2){
   return vert.Zsamples[0];
  }

  float median,mad,mest,mean;
  std::vector<float> weights;
  weights.resize(vert.num_samples);
  try{
    
    gpstk::QSort<float>(vert.Zsamples,(int)vert.num_samples);
    mean=get_avg_Z(vert.Zsamples,vert.num_samples);
    mad = gpstk::Robust::MedianAbsoluteDeviation<float>(vert.Zsamples,vert.num_samples, median);
    mest = gpstk::Robust::MEstimate<float>(vert.Zsamples,vert.num_samples, median, mad, &weights[0]);
    //		std::cout << "median: "<< median<< " mean: "<< mean << " mest: "<<mest  <<std::endl;//	    gpstk::Robust::StemLeafPlot<float>(std::cout,vert.Zsamples,(long)vert.num_samples,"S");
    
	    }catch (gpstk::Exception& e) {e.dump(std::cerr);}
  return mest;
}
void quadsquare::AddTriangleToWF(quadsquare * /* usused qs */, 
                                   nFlatTriangleCorner *tc0, 
                                   nFlatTriangleCorner *tc1,
                                   nFlatTriangleCorner *tc2)
{
    
    nFlatTriangleCorner *tc_array[3];
    tc_array[0] = tc0;
    tc_array[1] = tc1;
    tc_array[2] = tc2;

    int i;
    float buf[3];
    
    //if(!render_non_static){
    if(!render_no_data){ for (i=0; i<3; i++) {
	nFlatTriangleCorner *tc = tc_array[i];
	if(tc->vi->Z == 0 )
	  return;
      }
      }
    for (i=0; i<3; i++) {
        nFlatTriangleCorner *tc = tc_array[i];
	ul::vector p = ul::vector(float(tc->x),float(tc->y),tc->vi->Z);
	p.SetX(ge.min[0]+(tc->x*ge.cell_size));
	float Z=0;
	
	if(tc->vi->Zsamples){
	  //	  for(int i=0; i< tc->vi->num_samples; i++)
	  //printf("%f ",tc->vi->Zsamples[i]);
	  // printf("\n"); 

	
	
	    if(tc->vi->num_samples > 2){

	      Z=mest(*tc->vi);
	    }else{
	  double maxZ=get_max_Z(tc->vi->Zsamples,tc->vi->num_samples);//get_avg_Z(tc->vi->Zsamples,tc->vi->num_samples);
	  Z=get_clipped_avg_Z(tc->vi->Zsamples,tc->vi->num_samples,maxZ,ge.toUINTz(100.0));
	  //	  printf("%f\n",Z);
	    }
	}else{
	  Z=tc->vi->Z;
	}
	if(Z==0 && !render_no_data){
	  	  fprintf(stderr, "Shouldn't be zero I don't think uquadtree line 1372\n");
	  Z=1;
	}
	 

	p.SetZ(ge.fromUINTz(Z));
       	//printf("Z %f\n",((Z-1)/(float)UINT16_MAX_MINUS_ONE) );
	p.SetY(ge.min[1]+(tc->y*ge.cell_size));
	//        fprintf(wf_fp,"v %f %f %f\n",p.X(),p.Z(),-p.Y());
	buf[0]=p.X();
	buf[2]=p.Z();
	buf[1]=p.Y();
	fwrite((char *)buf,sizeof(float),3,wf_fp);

	if(apply_color_wf){
	  float r,g,b;
	  unsigned char rgb[3];
	  float val=0.0;
	  switch(color_metric){ 
	  case Z_SAMPLES:
	    val=tc->vi->num_samples;
	    break;
	  case Z_ERR:
	    val=stddev(tc->vi->Zsamples,tc->vi->num_samples);

	    break;
	  case Z_VAR:
	    	    val=square_err(tc->vi->Zsamples,tc->vi->Zsource,
				   tc->vi->num_samples);
		    //val=var(tc->vi->Zsamples,tc->vi->num_samples);
	    break;
	    
	  case SIGNED_ERR:
	    val=signed_err(tc->vi->Zsamples,tc->vi->Zsource,
			   tc->vi->num_samples);
	    break;

	  case RUGOSITY:
	    val=tc->vi->aux;
	    break;
	  
	  case SHADOWED:
	    r=tc->vi->aux;
	    g=tc->vi->aux;
	    b=tc->vi->aux;
	    break;


	    
	  }
	  
	  if(color_metric != SHADOWED){
	    //  printf("%f %f\n",min_stat_val,max_stat_val);
	    ColorMap::jetColorMap(rgb,val,min_stat_val,max_stat_val);
	    r=rgb[0]/255.0;
	    g=rgb[1]/255.0;
	    b=rgb[2]/255.0;
	  }

	  buf[0]=r;
	  buf[1]=g;
	  buf[2]=b;
	  fwrite((char *)buf,sizeof(float),3,wf_fp);

	}
	//}
	
	//        fp->PutS(buf);
    }
    wf_num_tris++;
}


#if USE_OSG



void	DrawQuadTree(osg::Node* root,	RECT	r, quadsquare* quad, int xorigin, int yorigin, int BlockPixels, const quadcornerdata& pd, bool NoClip)
// Draws the given quadtree height data as a contour map, with the
// quadtree's surface data as blocks of color in the background.
{
	int	i, j;
	
	if (!NoClip) {
	
		//dc->GetClipBox(&r);	//r.left,r.right,r.top,r.bottom;
		if (xorigin >= r.right || xorigin + BlockPixels < r.left || yorigin >= r.bottom || yorigin + BlockPixels < r.top) {
			// This square does not touch the rendering area; no need to draw.
			return;
		} else if (xorigin >= r.left && xorigin + BlockPixels < r.right && yorigin >= r.top && yorigin + BlockPixels < r.bottom) {
			// Square is completely inside rendering
			// rectangle; so any child blocks will also be
			// within the rendering area and we don't need
			// to check them.
			NoClip = true;
		}
	}

	for (j = 0; j < 2; j++) {
		for (i = 0; i < 2; i++) {
			int	ChildIndex = 2 * j + (j ? i : 1 - i);

			quadcornerdata	q;
			quad->SetupCornerData(&q, pd, ChildIndex);

			int	half = BlockPixels >> 1;
			int	x = xorigin + half * i;
			int	y = yorigin + half * j;
			
			if (BlockPixels > 8 /* lower limit for block size */ && quad->Child[ChildIndex] != NULL) {
				// Recurse to child block.
			  DrawQuadTree(root,r, quad->Child[ChildIndex], x, y, half, q, NoClip);
			} else {
				// Draw this square directly.
//				draw surface type background;
				
				DrawContourBox(root, x, y, half, q.Verts);
			}
		}
	}
}


const int	PENCOUNT = 6;
struct ContourParams {
	float	OneOverInterval;

	bool	PensCreated;
        osg::Vec4	pen[PENCOUNT];

	ContourParams() {
		PensCreated = false;
	}
} cp;


void	DrawHeightField(osg::Node* root,RECT &r, int xorigin, int yorigin, int Size, quadsquare* heightfield, float ContourInterval)
// Renders a height field into the given device context.
{
	int	i;
	
#ifdef NOT
	// Draw surface-type background.
	int	x, y;
	y = yorigin;
	float*	RowStart = DataGrid;
	unsigned char*	TypeRowStart = TypeDataGrid;
	for (int j = 0; j < zsize; j++) {
		float*	p = RowStart;
		unsigned char*	t = TypeRowStart;
		x = xorigin;
		for (int i = 0; i < xsize; i++) {
			// Show surface type behind the contour lines.
			pDC->FillSolidRect(x, y, BlockPixels, BlockPixels, TypeColor(*t));

			p++;
			t++;
			x += BlockPixels;
		}
		RowStart += GridPitch;
		TypeRowStart += GridPitch;
		y += BlockPixels;
	}
#endif // NOT
	
	//
	// Draw contour lines.
	//

	cp.OneOverInterval = 1.0 / ContourInterval;

	// Setup pens.
	if (cp.PensCreated == false) {
		for (i = 0; i < PENCOUNT; i++) {
		  cp.pen[i] = osg::Vec4((0 + 40 * i)/255.0,( 0 + 40 * i)/255.0,( 20 + 40 * i)/255.0,1.0);
		}
	}

	// Select a new pen into the DC.
	//CPen*	OldPen = dc->SelectObject(&cp.pen[0]);

	// Draw quadtree.
	quadcornerdata	q;
	q.Level = 15;
	q.ChildIndex = 0;
	q.xorg = q.yorg = 0;
	q.Verts[1].Z = 0;
	q.Verts[2].Z = 0;
	q.Verts[3].Z = 0;
	q.Verts[4].Z = 0;
	
	DrawQuadTree(root,r, heightfield, xorigin, yorigin, Size >> 1, q, false);

	// Restore the original pen.
	//dc->SelectObject(OldPen);

}


void	DrawContourBox(osg::Node *root, int x, int y, int BlockPixels, VertInfo samples[4])
// Draws the contour lines on a single square sub-element of a contour
// map, given the heights of the four corners and some setup parameters
// in the ContourParams cp structure.
{
	// Draw contour lines.
	float	samp[4];
	samp[0] = samples[0].Z * cp.OneOverInterval;	// upper left
	samp[1] = samples[1].Z * cp.OneOverInterval;	// upper right
	samp[2] = samples[2].Z * cp.OneOverInterval;	// lower right
	samp[3] = samples[3].Z * cp.OneOverInterval;	// lower left

	int	min = 100000;
	int	max = -100000;
	int	isamp[4];
	for (int k = 0; k < 4; k++) {
		// Keep samples within reasonable bounds.
		if (samp[k] < -300) samp[k] = -300;
		if (samp[k] > 10000) samp[k] = 10000;
		
		// Convert to integer.
		isamp[k] = floor(samp[k]);
		
		// Compute min & max.
		if (isamp[k] < min) min = isamp[k];
		if (isamp[k] > max) max = isamp[k];
	}

	// For each interval crossing within the range of values, draw the
	// appropriate line across the box.
	for (int k = min + 1; k <= max; k++) {
		int	EdgePix[4];
		
		// Choose the appropriate pen.
		//dc->SelectObject(&cp.pen[k % PENCOUNT]);
		
		// Check each edge to see if this contour
		// crosses it.
		for (int l = 0; l < 4; l++) {
			if ((isamp[(l+3)&3] < k && isamp[l] >= k) ||
			    (isamp[(l+3)&3] >= k && isamp[l] < k))
			{
				float	num = k - samp[(l+3)&3];
				if (l == 0 || l == 3) {
					EdgePix[l] = BlockPixels * (1 - num / (samp[l] - samp[(l+3)&3]));
				} else {
					EdgePix[l] = BlockPixels * num / (samp[l] - samp[(l+3)&3]);
				}
			} else {
				EdgePix[l] = -1;
			}
		}

		// Connect the edges.
		for (int l = 0; l < 4; l++) {
			if (EdgePix[l] == -1) continue;
			for (int m = l+1; m < 4; m++) {
				if (EdgePix[m] != -1) {
					// Connect edge l with m.
					int	xs, ys;
					if (l & 1) {
						xs = x + EdgePix[l];
						ys = y + (l == 3 ? BlockPixels : 0);
					} else {
						xs = x + (l == 2 ? BlockPixels : 0);
						ys = y + EdgePix[l];
					}
					
					int	xe, ye;
					if (m & 1) {
						xe = x + EdgePix[m];
						ye = y + (m == 3 ? BlockPixels : 0);
					} else {
						xe = x + (m == 2 ? BlockPixels : 0);
						ye = y + EdgePix[m];
					}
					
					if (l == 2 && m == 3) {
					  //dc->MoveTo(xe, ye);
					  //dc->LineTo(xs, ys);
					} else {
					  //dc->MoveTo(xs, ys);
					  //dc->LineTo(xe, ye);
					}
					
					if (m > l) l = m;
					break;
				}
			}
		}
	}
}


#endif // NOT
