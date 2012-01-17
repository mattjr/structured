/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004                                                \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *   
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/
/****************************************************************************
  History

$Log: not supported by cvs2svn $
Revision 1.1  2009-01-07 22:03:13  m.roberson
shadevis

Revision 1.9  2005/11/12 06:47:18  cignoni
Added Enhancement, removed type warnings,
started to refactor code in order to remove the unnecessary generality of the class.

Revision 1.8  2004/09/28 09:45:17  cignoni
Added MapFalseColor

Revision 1.7  2004/09/16 14:23:57  ponchio
fixed gcc template compatibility issues.

Revision 1.6  2004/09/10 14:02:20  cignoni
Added Cone directions

Revision 1.5  2004/09/09 22:59:21  cignoni
Removed many small warnings

Revision 1.4  2004/09/09 22:37:48  cignoni
Integrated lost modifications...

Revision 1.3  2004/09/09 14:35:54  ponchio
Various changes for gcc compatibility

Revision 1.2  2004/07/11 22:13:30  cignoni
Added GPL comments


****************************************************************************/

#ifndef __VCG_MESH_VISIBILITY
#define __VCG_MESH_VISIBILITY

#include <stdlib.h>

#include <bitset>
#include <vcg/math/matrix44.h>
#ifdef USEGL
#include <wrap/gl/math.h>
#endif
#include "simplepic.h"
#include "gen_normal.h"
#include "../swrender/render_utils.h"
#include "../swrender/render_utils.h"
#include "../swrender/democommon.h"
#include "../swrender/renderer/geometry_processor.h"
#include "../swrender/renderer/rasterizer_subdivaffine.h"
#include "../swrender/renderer/span.h"
#include "../swrender/fixedpoint/fixed_func.h"
#include "../swrender/generic_fragment_shader.h"
#include "../swrender/generic_vertex_shader.h"
int cnter=0;
//#define USEGL 1
#define SetRotate SetRotateRad
namespace vcg {
  // Base Class che definisce le varie interfaccie;
template <class MESH_TYPE, int MAXVIS=2048> class VisShader 
{
	public :
		enum {VisMax=MAXVIS};
	VisShader(MESH_TYPE &me):m(me)
	{
			CullFlag= false;
			IsClosedFlag = false;
			ZTWIST=1e-3;
      SplitNum=1;
			
			CameraViewing=false;
	}

		typedef Point3<typename MESH_TYPE::ScalarType> Point3x;

    typedef typename MESH_TYPE::CoordType CoordType;
    typedef typename MESH_TYPE::ScalarType ScalarType;
		typedef typename MESH_TYPE::VertexType  VertexType;
    typedef typename MESH_TYPE::VertexPointer  VertexPointer;
    typedef typename MESH_TYPE::VertexIterator VertexIterator;
    typedef typename MESH_TYPE::FaceIterator   FaceIterator;
    typedef typename MESH_TYPE::FaceType   FaceType;
	typedef Matrix44<ScalarType> Matrix44x;
	typedef Box3<ScalarType> Box3x;

// The Basic Data the mesh and its wrapper;
	MESH_TYPE &m;

  std::vector<MESH_TYPE *> OMV;  // Occluder Mesh Vector;

// la visibilita' e' in float, per ogni entita' 
// 1 significa che e' totalmente visibile per una data direzione.

	std::vector<float> VV;
	std::vector< Point3x > VN;				 // Vettore delle normali che ho usato per calcolare la mask e i float in W;

	 // User defined parameters and flags
	 bool IsClosedFlag;
	 float ZTWIST;
	 bool CullFlag;   // Enable the frustum culling. Useful when the splitting value is larger than 2 
	 int SplitNum;
	 protected:
	 bool CameraViewing;
	 //Camera<ScalarType> Cam;
	 public: 

/********************************************************/
// Generic functions with Specialized code for every subclass
	 virtual void MapVisibility(float Gamma=1, float LowPass=0, float HighPass=1,float Scale=1.0)=0;
   //virtual void ApplyLightingEnvironment(std::vector<float> &W, float Gamma);
	 
	 virtual int GLAccumPixel(	std::vector<int> &PixSeen)=0;
	 
	 virtual bool ReadVisibility(const char * /*filename*/){assert( 0); return false;}
	 virtual bool WriteVisibility(const char * /*filename*/){assert( 0); return false;}

/********************************************************/
// Generic functions with same code for every subclass

	void Clear() {		
	fill(VV.begin(),VV.end(),0); }

	void InitGL() 
		{
		  glPushAttrib(GL_COLOR_BUFFER_BIT );
      ::glClearColor (1.0, 1.0, 1.0, 0.0);
			glMatrixMode (GL_PROJECTION);   			
			glPushMatrix();
			glMatrixMode (GL_MODELVIEW);    
			glPushMatrix();
		}

	void RestoreGL()
		{
			glMatrixMode (GL_PROJECTION);   			
			glPopMatrix();
			glMatrixMode (GL_MODELVIEW);    
			glPopMatrix();
			glPopAttrib();
		}


/*
 Funzione principale di conversione in visibilita'
 Dati i due vettori PixSeen e PixNotSeen che indicano per ogni entita' (vertice o faccia) 
 quanti sono, rispettivamente,  i pixel visibili e occlusi,
 questa funzione calcola un valore float per ogni entita' che indica quanto  e' visibile lungo una data direzione camera 
 == 1 significa completamente visibile
 == 0 significa completamente occluso.

*/
	void AddPixelCount(std::vector<float> &_VV, const std::vector<int> &PixSeen)
		{
		assert(_VV.size()==PixSeen.size());
			for(unsigned int i=0;i<PixSeen.size();++i)
				if(PixSeen[i]>0) _VV[i]+= 1;
		}
 

 //void SetVisibilityMask(std::vector< std::bitset<MAXVIS> > &_VM, const std::vector<int> &PixSeen, const int dir)
	//	{
	//	assert(_VM.size()==PixSeen.size());
	//		for(int i=0;i<PixSeen.size();++i)
	//			if(PixSeen[i]>0) _VM[i][dir]=true;
	//	}

/*******************************
Funzioni ad alto livello che computano le Visibility Mask per varie distribuzioni di direzioni


*******************************/

// Funzione Generica 
// Calcola l'occlusion in base all'insieme VN di direzioni.

void Compute( CallBack *cb)
{
  //cb(buf.format("Start to compute %i dir\n",VN.size()));
	InitGL();
  int t00=clock();
	VV.resize(m.vert.size());
  std::vector<int> PixSeen(VV.size(),0);
	int TotRay=0,HitRay=0;

        for(unsigned int i=0;i<VN.size();++i)
		{
      int t0=clock(); 
			fill(PixSeen.begin(),PixSeen.end(),0);
      int added=SplittedRendering(VN[i], PixSeen,cb);	
		  AddPixelCount(VV,PixSeen);
      int t1=clock();
      HitRay+=added;
      TotRay+=VV.size();
      printf("%3i/%i : %i msec -- TotRays %i, HitRays %i, ray/sec %3.1fk \n ",i,VN.size(),t1-t0,TotRay,HitRay,float(TotRay)/(clock()-t00));
                }
  
  printf("Tot Time %i msec TotRays %i, HitRays %i, ray/sec %3.1fk \n ",clock()-t00,TotRay,HitRay,float(TotRay)/(clock()-t00));
	RestoreGL();
}

void ComputeHalf(int nn, Point3x &dir, CallBack *cb)
{
	std::string buf;
	
	VN.clear();
	std::vector<Point3x> nvt;
	assert(0 && "This is only my guess (to compile). (Ponchio)");
	assert(0 && "Was: GenNormal(nn*2, nvt);");
	GenNormal<ScalarType>::Uniform(nn*2,nvt);
	for(int i=0;i<nvt.size();++i)
		if(dir*nvt[i]>0) VN.push_back(nvt[i]);
 
	printf("Asked %i normal, got %i normals\n",nn,VN.size());
  Compute(cb); 
}

void ComputeUniformCone(int nn, std::vector<Point3x> &vv, ScalarType AngleRad, Point3x &ConeDir, CallBack *cb)
{
	VN.clear();
  GenNormal<ScalarType>::UniformCone(nn,VN,AngleRad,ConeDir);
  typename std::vector<Point3x>::iterator vi;
  for(vi=VN.begin();vi!=VN.end();++vi) 
    vv.push_back(*vi); 
  
	char buf[256];
	sprintf(buf,"Asked %i normal, got %i normals\n",nn,VN.size());
  cb(buf);
  Compute(cb); 
}
void ComputeUniform(int nn, std::vector<Point3x> &vv, CallBack *cb)
{
	VN.clear();
  GenNormal<ScalarType>::Uniform(nn,VN);
  typename std::vector<Point3x>::iterator vi;
  for(vi=VN.begin();vi!=VN.end();++vi) 
    vv.push_back(*vi); 
  
	char buf[256];
	sprintf(buf,"Asked %i normal, got %i normals\n",nn,VN.size());
  cb(buf);
  Compute(cb); 
}

void ComputeSingle(Point3x &dir, std::vector<Point3x> &vv,CallBack *cb)
{
	VN.clear();
	VN.push_back(dir);
  vv.push_back(dir);
	printf("Computing one direction (%f %f %f)\n",dir[0],dir[1],dir[2]);
  Compute(cb); 
}

/**********************************************************/

int SplittedRendering(Point3x &ViewDir, std::vector<int> &PixSeen, CallBack *cb=DummyCallBack)
{
  int tt=0;
  int i,j;
  for(i=0;i<SplitNum;++i)
    for(j=0;j<SplitNum;++j){
      Matrix44d viewproj;
        SetupOrthoViewMatrix2(ViewDir, i,j,SplitNum,viewproj);
              tt+=GLAccumPixel(PixSeen,viewproj);
    }
    return tt;
}

// Compute a rotation matrix that bring Axis parallel to Z. 
void GenMatrix(Matrix44d &a, Point3d Axis, double angle)
{
	const double eps=1e-3;
	Point3d RotAx   = Axis ^ Point3d(0,0,1);
  double RotAngle = Angle(Axis,Point3d(0,0,1));

  if(math::Abs(RotAx.Norm())<eps) { // in questo caso Axis e' collineare con l'asse z
			RotAx=Axis ^ Point3d(0,1,0);
      double RotAngle = Angle(Axis,Point3d(0,1,0));
		}
  
  //printf("Rotating around (%5.3f %5.3f %5.3f) %5.3f\n",RotAx[0],RotAx[1],RotAx[2],RotAngle);
  RotAx.Normalize();
  a.SetRotate(RotAngle,RotAx);
	//Matrix44d rr;
  //rr.SetRotate(-angle, Point3d(0,0,1));
	//a=rr*a;
}


// Genera la matrice di proj e model nel caso di un rendering ortogonale.
// subx e suby indicano la sottoparte che si vuole
#ifdef USEGL
void SetupOrthoViewMatrix1(Point3x &ViewDir, int subx, int suby,int LocSplit)
{
	glMatrixMode (GL_PROJECTION);   			
	glLoadIdentity (); 
  float dlt=2.0f/LocSplit;

  glOrtho(-1+subx*dlt, -1+(subx+1)*dlt, -1+suby*dlt, -1+(suby+1)*dlt,-2,2);
	glMatrixMode (GL_MODELVIEW);    
	glLoadIdentity ();  
	Matrix44d rot;
	Point3d qq; qq.Import(ViewDir);
	GenMatrix(rot,qq,0);
  glMultMatrix(rot);
	double d=2.0/m.bbox.Diag();
  glScalef(d,d,d);
	glTranslate(-m.bbox.Center());
}

#endif
void ComputeSingleDirection(Point3x BaseDir, std::vector<int> &PixSeen, CallBack *cb=DummyCallBack)
{
	int t0=clock();
	std::string buf;
 
	int added=SplittedRendering(BaseDir, PixSeen,cb);	
  int t1=clock();
	printf("ComputeSingleDir %i msec\n",t1-t0);
}
/*
void ComputeAverageVisibilityDirection()
{
	int i,j;
	VD.resize(VM.size());
	for(j=0;j<VM.size();++j)
		{
			Point3x &nn=VD[j];
			nn=Point3x(0,0,0);
			bitset<VisMax> &msk=VM[j];
				for(i=0;i<VN.size();++i)
				    if(msk[i]) nn+=VN[i];
		}
		for(j=0;j<VM.size();++j)
			 VD[j].Normalize();
		
}
*/
// calcola un LightingEnvironment direzionale, cioe'un vettore di pesi per l'insieme di normali 
// corrente tale che 
// mette a 1 tutti i vettori che sono entro un angolo DegAngle1 
// a 0 tutti quelli oltre DegAngle2 e
// sfuma linearmente nel mezzo. 
void DirectionalLightingEnvironment(std::vector<float> &LE, Point3x dir, ScalarType DegAngle1, ScalarType DegAngle2)
{
	LE.clear();
	LE.resize(VN.size(),0);
	int i;
	for(i=0;i<VN.size();++i)
		{
			ScalarType a=ToDeg(Angle(dir,VN[i]));
			if(a<DegAngle1) { LE[i]=1; continue; }
			if(a>DegAngle2) { LE[i]=0; continue; }
			LE[i] = 1.0-(a-DegAngle1)/(DegAngle2-DegAngle1);

		}
	// last step normalize the weights;
	ScalarType sum=0;
	for(i=0;i<VN.size();++i)
		sum+=LE[i];
	for(i=0;i<VN.size();++i)
		LE[i]/=sum;
}


};
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

template <class MESH_TYPE,int MAXVIS=2048> class VertexVisShader //: public VisShader<MESH_TYPE>
{
	public :
	enum {VisMax=MAXVIS};

			VertexVisShader(MESH_TYPE &me):m(me)
	{
     // la mesh DEVE avere colore per vertice
		CullFlag= false;
			IsClosedFlag = false;
			ZTWIST=1e-3;
      SplitNum=1;
	CameraViewing=false;
			if(! m.HasPerVertexColor()) assert(0);
	}

	void Init()  {		VV.resize(m.vert.size()); }
	void Compute(int nn);
#ifdef USEGL
void DrawFill (MESH_TYPE &mm)
{
  static GLuint dl=0;
  if(mm.face.empty())
  { AMesh::VertexIterator vi;
    glBegin(GL_POINTS);
    for(vi=mm.vert.begin();vi!=mm.vert.end();++vi)
      {
        //if(ColorFlag)
	  glColor((*vi).C()); 
        glVertex((*vi).P());
      }
    glEnd();
  }
  else
  {
    glBegin(GL_TRIANGLES);
    FaceIterator fi;
    for(fi=mm.face.begin();fi!=mm.face.end();++fi)
    {
      glVertex((*fi).V(0)->P());
      glVertex((*fi).V(1)->P());
      glVertex((*fi).V(2)->P());
    }
    glEnd();
  }
}

void printmatrix(GLdouble thematrix[16])
{
   int i, j;  // MS-Vis C++ 6.0 sometimes chokes on var's
              //  declared in for's.

   for (i=0; i<4; ++i)     // i = row
   {
      for (j=0; j<4; ++j)  // j = column
      {
          cout << fixed << std::setprecision(4) << setw(7)
               << thematrix[j*4+i] << " ";
      }
      cout << endl;
   }
}
void printmatrix(mat4x mat){

for(int i=0; i<4; i++){
    for(int j=0; j<4; j++){
        cout << fixed << std::setprecision(4) << setw(7)<<fix2float<16>(mat.elem[i][j].intValue) << " ";
    }
    cout <<endl;
}
}
#endif
// Genera la matrice di proj e model nel caso di un rendering ortogonale.
// subx e suby indicano la sottoparte che si vuole

void DrawSWFill(MESH_TYPE &mm,const F2X_RenderSurface &surf,const swr::GeometryProcessor::CullMode &cullmode){
    vec4x tmp_vertices[3];

    // the indices we need for rendering
    unsigned indices[] = {0, 1, 2};

    // create a rasterizer class that will be used to rasterize primitives
    swr::RasterizerSubdivAffine r;
    // create a geometry processor class used to feed vertex data.
    swr::GeometryProcessor g(&r);
    // it is necessary to set the viewport
    g.viewport(0, 0, surf.width, surf.height);
    // set the cull mode (CW is already the default mode)
    g.cull_mode(cullmode);
    //Set glDepthRange(0.0,1.0);
    float zNear=2.0*ZTWIST;
    float zFar=1.0;
    g.depth_range(
            fixedpoint::fixmul<16>(float2fix<16>(zNear), 0x3fffffff),
            fixedpoint::fixmul<16>(float2fix<16>(zFar), 0x3fffffff)
    );

//printf("AAA %f %d %f\n",2.0*ZTWIST,0x3fff-0x7fff,0x3fffffff/(float)0x7fffffff);

    // it is also necessary to set the clipping rectangle
    r.clip_rect(0, 0, surf.width, surf.height);

    // set the vertex and fragment shaders


    // specify where out data lies in memory
    g.vertex_attrib_pointer(0, sizeof(vec4x), tmp_vertices);
    g.vertex_shader<GenericVertexShader>();
    r.fragment_shader<GenericFragmentShader>();
    GenericFragmentShader::s_depth_buffer =surf;
    static GLuint dl=0;


      FaceIterator fi;
      for(fi=mm.face.begin();fi!=mm.face.end();++fi)
      {

          tmp_vertices[0].x=(*fi).V(0)->P()[0];
          tmp_vertices[0].y=(*fi).V(0)->P()[1];
          tmp_vertices[0].z=(*fi).V(0)->P()[2];
          tmp_vertices[0].w=1.0;

          tmp_vertices[1].x=(*fi).V(1)->P()[0];
          tmp_vertices[1].y=(*fi).V(1)->P()[1];
          tmp_vertices[1].z=(*fi).V(1)->P()[2];
          tmp_vertices[1].w=1.0;

          tmp_vertices[2].x=(*fi).V(2)->P()[0];
          tmp_vertices[2].y=(*fi).V(2)->P()[1];
          tmp_vertices[2].z=(*fi).V(2)->P()[2];
          tmp_vertices[2].w=1.0;
          g.draw_triangles(3, indices);

      }


}
/***************************************************************************/

//VertexVisibility
// Funzione Principale restituisce per ogni entita' quanti px si vedono o no.

int GLAccumPixel(	std::vector<int> &PixSeen,      int width,int height,Matrix44d viewproj)
{
#if USEGL
        SimplePic<float> snapZ;
	SimplePic<Color4b> snapC;
#endif
        GenericVertexShader::modelview_projection_matrix_d=viewproj;

        GenericFragmentShader::depth_test_func =GL_LEQUAL;// GL_GREATER;//GL_LEQUAL;
        F2X_RenderSurface surf;
        surf.width=width;
        surf.height=height;
        surf.pitch=width*sizeof(float);
        surf.data=new float[width*height];
        for(int i=0; i<surf.height*surf.width; i++){
            static_cast<float*>(surf.data)[i]=1.0;
        }

        F2X_RenderSurface color;
        color.width=width;
        color.height=height;
        color.pitch=width*sizeof(unsigned char);
        color.data=new unsigned char[width*height];
        bzero(color.data,height*width*sizeof(unsigned char));
        GenericFragmentShader::s_color_buffer=color;
#ifdef USEGL
        glClearColor(Color4b::Black);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_POLYGON_BIT );
	glDisable(GL_LIGHTING);	
	glDepthRange(0.0f,1.0f);
	glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
	glDepthMask(GL_TRUE);
	glDrawBuffer(GL_BACK);
	glReadBuffer(GL_BACK);
	
  /////** Si disegnano le front face  **/////
  glDepthRange(2.0*ZTWIST,1.0f);
  if(IsClosedFlag) glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glColor(Color4b::Red);
	DrawFill(m);


#endif
        SimplePic<float> depthZ;
        depthZ.Create(width,height);

       // SimplePic<Color4b> colorb;
        //colorb.Create(width,height);
        for(int i=0; i<depthZ.sx;i++)
            for(int j=0; j<depthZ.sy; j++){
            depthZ.Pix(i,j)=1.0;
        }

        GenericFragmentShader::depthZ=&(depthZ.img[0]);
        GenericFragmentShader::flatcolor=0xff;
        DrawSWFill(m,surf,swr::GeometryProcessor::CULL_CW);



  if(!IsClosedFlag) {
#ifdef USEGL
      glCullFace(GL_FRONT);
            glColor(Color4b::Black);
            DrawFill(m);
            snapC.OpenGLSnap();
#endif
            GenericFragmentShader::flatcolor=0x00;
            DrawSWFill(m,surf,swr::GeometryProcessor::CULL_CCW);
  }
 /* for(int i=0; i<colorb.sx;i++)
      for(int j=0; j<colorb.sy; j++){
      unsigned char c=(( unsigned char*)color.data)[color.width*j+i];
      colorb.Pix(i,j)=Color4b(c,0,0,0 );
  }
  char tmpc[1024];
 sprintf(tmpc,"but-%03d-O.ppm",cnter);
  snapC.SavePPM(tmpc);
  sprintf(tmpc,"but-%03d-N.ppm",cnter);

          colorb.SavePPM(tmpc);*/
 /* sprintf(tmpc,"but-O.ppm",cnter);
  snapC.SavePPM(tmpc);
  sprintf(tmpc,"but-N.ppm",cnter);

          colorb.SavePPM(tmpc);*/
       //a64l()  exit(0);
int cnt=0;

#ifdef USEGL
  snapZ.OpenGLSnap(GL_DEPTH_COMPONENT);


       //  snapZ.SavePFM("assy.pfm");
         //tmpC2.SavePFM("assy2.pfm");
       // snapC.SavePPM("but.ppm");
       // colorb.SavePPM("but2.ppm");

       // exit(-1);


         glDepthRange(0,1.0f-2.0*ZTWIST);
  double MM[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,MM);
	double MP[16];
  glGetDoublev(GL_PROJECTION_MATRIX,MP);
	int VP[4];
	glGetIntegerv(GL_VIEWPORT,(GLint*)VP);
#endif
        double tx,ty,tz;

	for(unsigned int i=0;i<m.vert.size();++i)
	{
                /*gluProject(m.vert[i].P()[0],m.vert[i].P()[1],m.vert[i].P()[2],
			   MM,MP,(GLint*)VP,
			&tx,&ty,&tz);
                cout << tx<<" "<<ty<< " "<<tz<<endl;*/
                vcg::Point4d v=viewproj* vcg::Point4d(m.vert[i].P()[0],m.vert[i].P()[1],m.vert[i].P()[2],1.0);
                v.X()/=v.W();
                v.Y()/=v.W();
                v.Z()/=v.W();

                v.X() = v.X() * 0.5 + 0.5;
                v.Y() = v.Y() * 0.5 + 0.5;

                v.Z() = v.Z() * 0.5 + 0.5;

                v.X() *= width;
                v.Y() *= height;
                tx=v.X();
                ty=v.Y();
                tz=v.Z();
             //   cout << tx<<" "<<ty<< " "<<tz<<endl;

    int col=1;
		    
    if(tx>=0 && tx<depthZ.sx && ty>=0 && ty<depthZ.sy)
    {
		    int txi=floor(tx),tyi=floor(ty);
                    //float sd=snapZ.Pix(tx,ty);
                   float sd=depthZ.Pix(tx,ty);
                  //  printf("%f %f %f\n",sd-p,sd,p);
    		if(!IsClosedFlag) {
      //    col = std::max( std::max(snapC.Pix(txi+0,tyi+0)[0],snapC.Pix(txi+1,tyi+0)[0]),
        //                                                   std::max(snapC.Pix(txi+0,tyi+1)[0],snapC.Pix(txi+1,tyi+1)[0]));
                  unsigned char c00=(( unsigned char*)color.data)[color.width*(tyi+0)+(txi+0)];
                  unsigned char c10=(( unsigned char*)color.data)[color.width*(tyi+0)+(txi+1)];
                  unsigned char c01=(( unsigned char*)color.data)[color.width*(tyi+1)+(txi+0)];
                  unsigned char c11=(( unsigned char*)color.data)[color.width*(tyi+1)+(txi+1)];
//              int    col2 =std::max( std::max( c00,c10),std::max(c01,c11));
            //  if(col != col2)
           //   printf("col %d %d\n",col,col2);
col=std::max( std::max( c00,c10),std::max(c01,c11));//col2;
        // col=snapC.Pix(txi+0,tyi+0)[0];
        }
        if(col!=0 && tz<sd) {
			    PixSeen[i]++;
			    cnt++;
		    }
	    }
  }
  	glPopAttrib();
        delete []    surf.data;
        delete []color.data;
//printf("Seen %i vertexes on %i\n",cnt,m.vert.size());
return cnt;
}

void SmoothVisibility(bool Enhance=false)
{
	FaceIterator fi;
	std::vector<float> VV2;
	std::vector<int> VC(VV.size(),1);
	VV2=VV;
	for(fi=m.face.begin();fi!=m.face.end();++fi)
		for(int i=0;i<3;++i)
		{
			VV2[(*fi).V(i)-&*m.vert.begin()] += VV[(*fi).V1(i)-&*m.vert.begin()];
			++VC[(*fi).V(i)-&*m.vert.begin()];
		}

 if(!Enhance)
	  for(unsigned int i=0;i<VV2.size();++i)
    		VV[i]=VV2[i]/VC[i];
 else
	  for(unsigned int i=0;i<VV2.size();++i)
		    VV[i]=VV[i]+ (VV[i]-VV2[i]/VC[i])*.5;
}


void MapFalseColor()
{
  float minv=*min_element(VV.begin(),VV.end());
	float maxv=*max_element(VV.begin(),VV.end());
	printf("Visibility Range %f %f\n", minv,maxv);
  MapFalseColor(minv, maxv);
}

void MapFalseColor(float minv, float maxv)
{
	VertexIterator vi;
	for(vi=m.vert.begin();vi!=m.vert.end();++vi){
			    float gval=(VV[vi-m.vert.begin()]-minv)/(maxv-minv);
          math::Clamp(gval,0.0f,1.0f);
				  (*vi).C().ColorRamp(1.0,0.0,gval);
    	}
}

/*
The visibility is mapped in [0..1]
then clamped to [low,high]
this value is mapped again in [0.1] and gamma corrected;
and at the end is scaled for 'Scale'
*/

void MapVisibility(float Gamma=1, float LowPass=0, float HighPass=1, float Scale= 1.0)
{
	float minv=*min_element(VV.begin(),VV.end());
	float maxv=*max_element(VV.begin(),VV.end());
	printf("Visibility Range %f %f\n", minv,maxv);
    minv=0.0;
    maxv=255.0;
    printf("Clamped Visibility Range %f %f\n", minv,maxv);

	VertexIterator vi;
			for(vi=m.vert.begin();vi!=m.vert.end();++vi){
				float gval=(VV[vi-m.vert.begin()]-minv)/(maxv-minv);
				if(gval<LowPass) gval=LowPass;
				if(gval>HighPass) gval=HighPass;
				(*vi).C().SetGrayShade(Scale*pow((gval-LowPass)/(HighPass-LowPass),Gamma));
			}
}

//void ApplyLightingEnvironment(std::vector<float> &W, float Gamma=1)
//	{
//		assert(W.size()==VN.size());
//		MESH_TYPE::VertexIterator vi;
//	
//		for(vi=m.vert.begin();vi!=m.vert.end();++vi)
//		{
//		float gray=0;
//		bitset<VisMax> &msk=VM[vi-m.vert.begin()];
//			for(int i=0;i<VN.size();++i)
//				if(msk[i]) gray+=W[i];
//			
//			(*vi).C().SetGrayShade(gray);
//		}
//	}
		typedef Point3<typename MESH_TYPE::ScalarType> Point3x;

    typedef typename MESH_TYPE::CoordType CoordType;
    typedef typename MESH_TYPE::ScalarType ScalarType;
		typedef typename MESH_TYPE::VertexType  VertexType;
    typedef typename MESH_TYPE::VertexPointer  VertexPointer;
    typedef typename MESH_TYPE::VertexIterator VertexIterator;
    typedef typename MESH_TYPE::FaceIterator   FaceIterator;
    typedef typename MESH_TYPE::FaceType   FaceType;
	typedef Matrix44<ScalarType> Matrix44x;
	typedef Box3<ScalarType> Box3x;

// The Basic Data the mesh and its wrapper;
	MESH_TYPE &m;

  std::vector<MESH_TYPE *> OMV;  // Occluder Mesh Vector;

// la visibilita' e' in float, per ogni entita' 
// 1 significa che e' totalmente visibile per una data direzione.

	std::vector<float> VV;
	std::vector< Point3x > VN;				 // Vettore delle normali che ho usato per calcolare la mask e i float in W;

	 // User defined parameters and flags
	 bool IsClosedFlag;
	 float ZTWIST;
	 bool CullFlag;   // Enable the frustum culling. Useful when the splitting value is larger than 2 
	 int SplitNum;
	 protected:
	 bool CameraViewing;
	 //Camera<ScalarType> Cam;
	 public: 

/********************************************************/
// Generic functions with Specialized code for every subclass
//  virtual void MapVisibility(float Gamma=1, float LowPass=0, float HighPass=1,float Scale=1.0);
   //virtual void ApplyLightingEnvironment(std::vector<float> &W, float Gamma);
	 
  //	 virtual int GLAccumPixel(	std::vector<int> &PixSeen);
	 
	 virtual bool ReadVisibility(const char * /*filename*/){assert( 0); return false;}
	 virtual bool WriteVisibility(const char * /*filename*/){assert( 0); return false;}

/********************************************************/
// Generic functions with same code for every subclass

	void Clear() {		
	fill(VV.begin(),VV.end(),0); }

	void InitGL() 
		{
		  glPushAttrib(GL_COLOR_BUFFER_BIT );
      ::glClearColor (1.0, 1.0, 1.0, 0.0);
			glMatrixMode (GL_PROJECTION);   			
			glPushMatrix();
			glMatrixMode (GL_MODELVIEW);    
			glPushMatrix();
		}

	void RestoreGL()
		{
			glMatrixMode (GL_PROJECTION);   			
			glPopMatrix();
			glMatrixMode (GL_MODELVIEW);    
			glPopMatrix();
			glPopAttrib();
		}


/*
 Funzione principale di conversione in visibilita'
 Dati i due vettori PixSeen e PixNotSeen che indicano per ogni entita' (vertice o faccia) 
 quanti sono, rispettivamente,  i pixel visibili e occlusi,
 questa funzione calcola un valore float per ogni entita' che indica quanto  e' visibile lungo una data direzione camera 
 == 1 significa completamente visibile
 == 0 significa completamente occluso.

*/
	void AddPixelCount(std::vector<float> &_VV, const std::vector<int> &PixSeen)
		{
		assert(_VV.size()==PixSeen.size());
			for(unsigned int i=0;i<PixSeen.size();++i)
				if(PixSeen[i]>0) _VV[i]+= 1;
		}
 

 //void SetVisibilityMask(std::vector< std::bitset<MAXVIS> > &_VM, const std::vector<int> &PixSeen, const int dir)
	//	{
	//	assert(_VM.size()==PixSeen.size());
	//		for(int i=0;i<PixSeen.size();++i)
	//			if(PixSeen[i]>0) _VM[i][dir]=true;
	//	}

/*******************************
Funzioni ad alto livello che computano le Visibility Mask per varie distribuzioni di direzioni


*******************************/

// Funzione Generica 
// Calcola l'occlusion in base all'insieme VN di direzioni.

void Compute(int height,int width, CallBack *cb)
{
  //cb(buf.format("Start to compute %i dir\n",VN.size()));
	InitGL();
  int t00=clock();
	VV.resize(m.vert.size());
  std::vector<int> PixSeen(VV.size(),0);
	int TotRay=0,HitRay=0;
        for(unsigned int i=0;i<VN.size();++i)
		{
      int t0=clock(); 
      cnter=i;
			fill(PixSeen.begin(),PixSeen.end(),0);
      int added=SplittedRendering(height,width,VN[i], PixSeen,cb);
		  AddPixelCount(VV,PixSeen);
      int t1=clock();
      HitRay+=added;
      TotRay+=VV.size();
      printf("%3i/%i : %i msec -- TotRays %i, HitRays %i, ray/sec %3.1fk \n ",i,VN.size(),t1-t0,TotRay,HitRay,float(TotRay)/(clock()-t00));
                }
  
        printf("Tot Time %s TotRays %i, HitRays %i, ray/sec %3.1fk \n ",format_elapsed((clock()-t00)/(double)CLOCKS_PER_SEC).c_str(),TotRay,HitRay,float(TotRay)/(clock()-t00));
        RestoreGL();
}

void ComputeHalf(int nn, Point3x &dir, CallBack *cb)
{
	std::string buf;
	
	VN.clear();
	std::vector<Point3x> nvt;
	assert(0 && "This is only my guess (to compile). (Ponchio)");
	assert(0 && "Was: GenNormal(nn*2, nvt);");
	GenNormal<ScalarType>::Uniform(nn*2,nvt);
	for(int i=0;i<nvt.size();++i)
		if(dir*nvt[i]>0) VN.push_back(nvt[i]);
 
	printf("Asked %i normal, got %i normals\n",nn,VN.size());
  Compute(cb); 
}

void ComputeUniformCone(int height,int width,int nn, std::vector<Point3x> &vv, ScalarType AngleRad, Point3x &ConeDir, CallBack *cb)
{
	VN.clear();
  GenNormal<ScalarType>::UniformCone(nn,VN,AngleRad,ConeDir);
  typename std::vector<Point3x>::iterator vi;
  for(vi=VN.begin();vi!=VN.end();++vi) 
    vv.push_back(*vi); 
  
	char buf[256];
	sprintf(buf,"Asked %i normal, got %i normals\n",nn,VN.size());
  cb(buf);
  Compute(height,width,cb);
}
void ComputeUniform(int nn, std::vector<Point3x> &vv, CallBack *cb)
{
	VN.clear();
  GenNormal<ScalarType>::Uniform(nn,VN);
  typename std::vector<Point3x>::iterator vi;
  for(vi=VN.begin();vi!=VN.end();++vi) 
    vv.push_back(*vi); 
  
	char buf[256];
	sprintf(buf,"Asked %i normal, got %i normals\n",nn,VN.size());
  cb(buf);
  Compute(cb); 
}

void ComputeSingle(int height,int width,Point3x &dir, std::vector<Point3x> &vv,CallBack *cb)
{
	VN.clear();
	VN.push_back(dir);
  vv.push_back(dir);
	printf("Computing one direction (%f %f %f)\n",dir[0],dir[1],dir[2]);
  Compute( height, width,cb);
}

/**********************************************************/

int SplittedRendering( int height,int width ,Point3x &ViewDir, std::vector<int> &PixSeen, CallBack *cb=DummyCallBack)
{
  int tt=0;
  int i,j;
  for(i=0;i<SplitNum;++i)
    for(j=0;j<SplitNum;++j){
      Matrix44d viewproj;

        SetupOrthoViewMatrix2(ViewDir, i,j,SplitNum,viewproj);
              tt+=GLAccumPixel(PixSeen,height,width,viewproj);
    }
    return tt;
}

// Compute a rotation matrix that bring Axis parallel to Z. 
void GenMatrix(Matrix44d &a, Point3d Axis, double angle)
{
	const double eps=1e-3;
	Point3d RotAx   = Axis ^ Point3d(0,0,1);
  double RotAngle = Angle(Axis,Point3d(0,0,1));

  if(math::Abs(RotAx.Norm())<eps) { // in questo caso Axis e' collineare con l'asse z
			RotAx=Axis ^ Point3d(0,1,0);
      double RotAngle = Angle(Axis,Point3d(0,1,0));
		}
  
  //printf("Rotating around (%5.3f %5.3f %5.3f) %5.3f\n",RotAx[0],RotAx[1],RotAx[2],RotAngle);
  RotAx.Normalize();
  a.SetRotate(RotAngle,RotAx);
	//Matrix44d rr;
  //rr.SetRotate(-angle, Point3d(0,0,1));
	//a=rr*a;
}

void SetupOrthoViewMatrix2(Point3x &ViewDir, int subx, int suby,int LocSplit,Matrix44d &viewproj)
{
    //printf("AAAAASDASDASD\n");
    mat4x projA;
     mat4x modelviewprojection_matrix=identity4<fixed16_t>();
#ifdef USEGL
        glMatrixMode (GL_PROJECTION);
        glLoadIdentity ();
#endif
  float dlt=2.0f/LocSplit;
#ifdef USEGL

  glOrtho(-1+subx*dlt, -1+(subx+1)*dlt, -1+suby*dlt, -1+(suby+1)*dlt,-2,2);
#endif
   projA=ortho_matrix<fixed16_t>(-1+subx*dlt, -1+(subx+1)*dlt, -1+suby*dlt, -1+(suby+1)*dlt,-2,2);
Matrix44d projB;
#ifdef USEGL
        glMatrixMode (GL_MODELVIEW);
        glLoadIdentity ();
#endif
        Matrix44d rot,viewTMP,scalingM,tranM;
        Point3d qq; qq.Import(ViewDir);
        GenMatrix(rot,qq,0);

        mat4<fixed16_t> rotA;
        for(int i=0; i<4; i++)
            for(int j=0; j<4; j++){
            rotA.elem[i][j]=rot.ElementAt(i,j);
            projB.ElementAt(i,j)=fix2float<16>(projA.elem[i][j].intValue);
        }
#ifdef USEGL

  glMultMatrix(rot);
#endif
        double d=2.0/m.bbox.Diag();
#ifdef USEGL
  glScalef(d,d,d);
        glTranslate(-m.bbox.Center());
#endif
        scalingM.SetScale(d,d,d);
        tranM.SetTranslate(-m.bbox.Center()[0],-m.bbox.Center()[1],-m.bbox.Center()[2]);
        viewTMP=rot*scalingM*tranM;//scaling_matrix<fixed16_t>(d,d,d)*translation_matrix<fixed16_t>(-m.bbox.Center()[0],-m.bbox.Center()[1],-m.bbox.Center()[2]);

viewproj=projB*viewTMP;
 /*       glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrixd);
cout <<"OV\n";
printmatrix(mvmatrixd);
cout <<"\nOP\n";
glGetDoublev(GL_PROJECTION_MATRIX, mvmatrixd);
printmatrix(mvmatrixd);
cout <<"\nSV\n";
printmatrix(viewA);
cout <<"\nSP\n";
printmatrix(projA);
cout <<"\n";*/

}

// Genera la matrice di proj e model nel caso di un rendering ortogonale.
// subx e suby indicano la sottoparte che si vuole
#ifdef USEGL

void SetupOrthoViewMatrix(Point3x &ViewDir, int subx, int suby,int LocSplit)
{
	glMatrixMode (GL_PROJECTION);   			
	glLoadIdentity (); 
  float dlt=2.0f/LocSplit;

  glOrtho(-1+subx*dlt, -1+(subx+1)*dlt, -1+suby*dlt, -1+(suby+1)*dlt,-2,2);
	glMatrixMode (GL_MODELVIEW);    
	glLoadIdentity ();  
	Matrix44d rot;
	Point3d qq; qq.Import(ViewDir);
	GenMatrix(rot,qq,0);
  glMultMatrix(rot);
	double d=2.0/m.bbox.Diag();
  glScalef(d,d,d);
	glTranslate(-m.bbox.Center());
}
#endif
void ComputeSingleDirection(Point3x BaseDir, std::vector<int> &PixSeen, CallBack *cb=DummyCallBack)
{
	int t0=clock();
	std::string buf;
 
	int added=SplittedRendering(BaseDir, PixSeen,cb);	
  int t1=clock();
	printf("ComputeSingleDir %i msec\n",t1-t0);
}
/*
void ComputeAverageVisibilityDirection()
{
	int i,j;
	VD.resize(VM.size());
	for(j=0;j<VM.size();++j)
		{
			Point3x &nn=VD[j];
			nn=Point3x(0,0,0);
			bitset<VisMax> &msk=VM[j];
				for(i=0;i<VN.size();++i)
				    if(msk[i]) nn+=VN[i];
		}
		for(j=0;j<VM.size();++j)
			 VD[j].Normalize();
		
}
*/
// calcola un LightingEnvironment direzionale, cioe'un vettore di pesi per l'insieme di normali 
// corrente tale che 
// mette a 1 tutti i vettori che sono entro un angolo DegAngle1 
// a 0 tutti quelli oltre DegAngle2 e
// sfuma linearmente nel mezzo. 
void DirectionalLightingEnvironment(std::vector<float> &LE, Point3x dir, ScalarType DegAngle1, ScalarType DegAngle2)
{
	LE.clear();
	LE.resize(VN.size(),0);
	int i;
	for(i=0;i<VN.size();++i)
		{
			ScalarType a=ToDeg(Angle(dir,VN[i]));
			if(a<DegAngle1) { LE[i]=1; continue; }
			if(a>DegAngle2) { LE[i]=0; continue; }
			LE[i] = 1.0-(a-DegAngle1)/(DegAngle2-DegAngle1);

		}
	// last step normalize the weights;
	ScalarType sum=0;
	for(i=0;i<VN.size();++i)
		sum+=LE[i];
	for(i=0;i<VN.size();++i)
		LE[i]/=sum;
}

};



}
#endif // __VCG_MESH_VISIBILITY
