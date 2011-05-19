#if 0

#include<vcg/complex/complex.h>
#include<vcg/simplex/vertex/base.h>
#include<vcg/simplex/face/base.h>
#include<vcg/simplex/face/topology.h>

#include<vcg/simplex/vertex/component_ocf.h>
#include<vcg/simplex/face/component_ocf.h>



class CFace;
class CFaceOcf;

class CVertex;
class CVertexOcf;


struct MyUsedTypes:		 public	vcg::UsedTypes<vcg::Use<CVertex>::AsVertexType,vcg::Use<CFace>::AsFaceType>{};
struct MyUsedTypesOcf: public vcg::UsedTypes<vcg::Use<CVertexOcf>::AsVertexType,vcg::Use<CFaceOcf>::AsFaceType>{};

// Optional stuff has two suffixes:
// OCF Optional Component Fast
// OCC Optional Component Compact

class CVertex     : public vcg::Vertex<	MyUsedTypes,  vcg::vertex::Coord3f, vcg::vertex::BitFlags,vcg::vertex::Normal3f >{};
class CVertexOcf  : public vcg::Vertex< MyUsedTypesOcf,vcg::vertex::InfoOcf,vcg::vertex::Coord3f,vcg::vertex::QualityfOcf, vcg::vertex::BitFlags,vcg::vertex::Normal3f,vcg::vertex::RadiusfOcf >{};
class CFace       : public vcg::Face< MyUsedTypes,    vcg::face::FFAdj,    vcg::face::VertexRef, vcg::face::BitFlags, vcg::face::Normal3f > {};
class CFaceOcf    : public vcg::Face< MyUsedTypesOcf, vcg::face::InfoOcf, vcg::face::FFAdjOcf, vcg::face::VertexRef, vcg::face::BitFlags, vcg::face::Normal3fOcf > {};

class CMesh       : public vcg::tri::TriMesh<     std::vector<CVertex   >,           std::vector<CFace   > > {};
class CMeshOcf    : public vcg::tri::TriMesh<     vcg::vertex::vector_ocf<CVertexOcf>, vcg::face::vector_ocf<CFaceOcf> > {};
#define CMeshO CMesh
#define CVertexO CVertex
#define CFaceO CFace
class CEdge;
// Declaration of the semantic of the used types
class CUsedTypesO: public vcg::UsedTypes < vcg::Use<CVertexO>::AsVertexType,
                                           vcg::Use<CEdge   >::AsEdgeType,
                                           vcg::Use<CFaceO  >::AsFaceType >{};
// The Main Edge Class
// Currently it does not contains anything.
class CEdge : public vcg::Edge<CUsedTypesO, vcg::edge::EVAdj> {
public:
        inline CEdge(){};
  inline CEdge( CVertexO * v0, CVertexO * v1){ V(0)= v0 ; V(1)= v1;};
  static inline CEdge OrderedEdge(CVertexO* v0,CVertexO* v1){
   if(v0<v1) return CEdge(v0,v1);
   else return CEdge(v1,v0);
        }
};
#endif

#if 1
#define _MESH_DEF_

#include<vcg/complex/complex.h>
#include<vcg/simplex/vertex/base.h>
#include<vcg/simplex/face/base.h>
#include<vcg/simplex/face/topology.h>
#include <vcg/complex/algorithms/update/color.h>

class CVertexO;
class CEdge;
class CFaceO;

// Declaration of the semantic of the used types
class CUsedTypesO: public vcg::UsedTypes < vcg::Use<CVertexO>::AsVertexType,
                                           vcg::Use<CEdge   >::AsEdgeType,
                                           vcg::Use<CFaceO  >::AsFaceType >{};


// The Main Vertex Class
// Most of the attributes are optional and must be enabled before use.
// Each vertex needs 40 byte, on 32bit arch. and 44 byte on 64bit arch.
class CVertexO  : public vcg::Vertex< CUsedTypesO,
  vcg::vertex::InfoOcf,           /*  4b */
  vcg::vertex::Coord3f,           /* 12b */
  vcg::vertex::BitFlags,          /*  4b */
  vcg::vertex::Normal3f,          /* 12b */
  vcg::vertex::Qualityf,          /*  4b */
  vcg::vertex::Color4b,           /*  4b */
  vcg::vertex::VFAdj,          /*  0b */
  vcg::vertex::Mark,           /*  0b */
  vcg::vertex::TexCoordfOcf,      /*  0b */
  vcg::vertex::CurvaturefOcf,     /*  0b */
  vcg::vertex::CurvatureDirfOcf,  /*  0b */
  vcg::vertex::RadiusfOcf         /*  0b */
  >{
  public:
    typedef int  MarkType ;

};


// The Main Edge Class
// Currently it does not contains anything.
class CEdge : public vcg::Edge<CUsedTypesO, vcg::edge::EVAdj> {
public:
	inline CEdge(){};
  inline CEdge( CVertexO * v0, CVertexO * v1){ V(0)= v0 ; V(1)= v1;};
  static inline CEdge OrderedEdge(CVertexO* v0,CVertexO* v1){
   if(v0<v1) return CEdge(v0,v1);
   else return CEdge(v1,v0);
	}
};

// Each face needs 32 byte, on 32bit arch. and 48 byte on 64bit arch.
class CFaceO    : public vcg::Face<  CUsedTypesO,
      vcg::face::InfoOcf,              /* 4b */
      vcg::face::VertexRef,            /*12b */
      vcg::face::BitFlags,             /* 4b */
      vcg::face::Normal3f,             /*12b */
      vcg::face::QualityfOcf,          /* 0b */
      vcg::face::Mark,              /* 0b */
      vcg::face::FFAdj,             /* 0b */
      vcg::face::VFAdj,             /* 0b */
      vcg::face::WedgeTexCoord2f     /* 0b */
    > {};

class CMeshO    : public vcg::tri::TriMesh< vcg::vertex::vector_ocf<CVertexO>, vcg::face::vector_ocf<CFaceO> > {
public :
  int sfn; //The number of selected faces.
  int svn; //The number of selected faces.
  vcg::Matrix44f Tr; // Usually it is the identity. It is applied in rendering and filters can or cannot use it. (most of the filter will ignore this)

  const vcg::Box3f &trBB()
  {
    static vcg::Box3f bb;
    bb.SetNull();
    bb.Add(Tr,bbox);
		return bb;
	}
};
#endif
