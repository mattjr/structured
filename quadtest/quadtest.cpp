// quadtest.cpp	-thatcher 1/8/2000 Copyright Thatcher Ulrich

// Test program for quadtree adaptive heightfield.

// This code may be freely modified and redistributed.  I make no
// warrantees about it; use at your own risk.  If you do incorporate
// this code into a project, I'd appreciate a mention in the credits.
//
// Thatcher Ulrich <tu@tulrich.com>


#include <stdio.h>
#include <math.h>
#include "geometry.hpp"
#include "clip.hpp"
#include "quadtree.hpp"

#include <stdlib.h> 
#include <string.h>
#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>

#else

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#endif




#define PI 3.141592654


void	DisplayReshape(int w, int h);
void	Display();
void	Idle();
void	MouseHandler(int button, int state, int x, int y);
void	MotionHandler(int x, int y);
void	SpecialKeyHandler(int key, int x, int y);
void	KeyHandler(unsigned char key, int x, int y);
void	LoadData(char  *filename);
void	LoadData();


quadsquare*	root = NULL;
quadcornerdata	RootCornerData = { NULL, NULL, 0, 15, 0, 0, { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } } };


vector	ViewerDir(1, 0, 0);
vector	ViewerUp(0, 1, 0);
float	ViewerTheta = 0;
float	ViewerPhi = 0;
float	ViewerHeight = 0;
vector	ViewerLoc(30000, 9550, 30000);
float	Speed = (1 << 5);
bool	PinToGround = false;
bool	MoveForward = false;

int	TriangleCounter = 0;

static float	Detail = 140;


int	main(int argc, char *argv[])
{
	// Print a summary of the user-interface.
	printf("Welcome to the Adaptive Quadtree Meshing demo.\n");
	printf("by Thatcher Ulrich <tu@tulrich.com> Copyright 2000\n\n");
	printf("Use the mouse and keyboard to control the program.  Here's a summary:\n");
	printf(" * mouse-left plus mouse-move rotates the view orientation\n");
	printf(" * mouse-right plus mouse-move moves the viewpoint\n");
	printf(" * both mouse buttons plus mouse-move moves the viewpoint differently\n");
	printf(" * 'w' toggles wireframe mode.  Good for checking out the LOD\n");
	printf(" * '0'-'9' controls the speed of viewpoint motion\n");
	printf(" * 't' toggles texturing (off by default).  Please try it\n");
	printf(" * 'c' toggles backface culling\n");
	printf(" * 'p' fixes the viewpoint at a certain altitude above the terrain\n");
	printf(" * 'm' toggles motion mode\n");
	printf(" * 'd' runs a 1-second benchmark and displays some performance data\n");
	printf(" * '=' increases terrain detail\n");
	printf(" * '-' decreases terrain detail\n");
	printf("\n");
	
	int	i;
	int cnt=0;
	root = new quadsquare(&RootCornerData);
	/*	FILE* fp;
#define DIMENSION 3
	float c[2*DIMENSION];
	fp=fopen(argv[1],"rb");
	double min[3],max[3];
	while(1){
	  if(fread(c,sizeof(float),2*DIMENSION,fp)!=6){break;}
	  for(i=0;i<DIMENSION;i++){
	    if(!cnt || c[i]<min[i]){min[i]=c[i];}
	    if(!cnt || c[i]>max[i]){max[i]=c[i];}
	  }
	  cnt++;
	}
	fseek(fp,SEEK_SET,0);
	float pts[cnt*DIMENSION];
	float norms[cnt*DIMENSION];
	int idx=0;
	while(1){
	  if(fread(c,sizeof(float),2*DIMENSION,fp)!=6){break;}
	  memcpy(&pts[idx],c,sizeof(float)*3);
	  memcpy(&norms[idx],c,sizeof(float)*3);
	  idx++;
	}
	fclose(fp);
	printf("%d count %f %f %f %f %f %f\n",cnt,min[0],min[1],min[2],max[0],max[1],max[2]);*/
	
		LoadData(argv[1]);
	
	// Debug info.
	printf("nodes = %d\n", root->CountNodes());
	printf("max error = %g\n", root->RecomputeErrorAndLighting(RootCornerData));

	// Get rid of unnecessary nodes in flat-ish areas.
	printf("Culling unnecessary nodes (detail factor = 25)...\n");
	root->StaticCullData(RootCornerData, 25);

	// Post-cull debug info.
	printf("nodes = %d\n", root->CountNodes());
	printf("max error = %g\n", root->RecomputeErrorAndLighting(RootCornerData));


	// Run the update function a few times before we start rendering
	// to disable unnecessary quadsquares, so the first frame won't
	// be overloaded with tons of triangles.
	for (i = 0; i < 10; i++) {
		root->Update(RootCornerData, (const float*) ViewerLoc, Detail);
	}

	
	//
	// Set up glut.
	//
	glutInit(&argc, argv);

	glutInitWindowSize(640, 480);
	glutInitWindowPosition(0, 0);

	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

	int	win_id = glutCreateWindow(argv[0]);
	if (win_id < 0) {
		printf("could not create window...exiting\n");
		exit(1);
	}

	glutReshapeFunc(DisplayReshape);
	glutKeyboardFunc(KeyHandler);
	glutSpecialFunc(SpecialKeyHandler);
	glutMouseFunc(MouseHandler);
	glutMotionFunc(MotionHandler);
	glutIdleFunc(Idle);
	glutDisplayFunc(Display);

	glutMainLoop();

	delete root;

	return 0;
}
void load_hm_file(HeightMapInfo *hm,const char *filename){
  FILE *fp= fopen(filename,"rb");
  if(!fp){
    fprintf(stderr,"Cannot open %s\n",filename);
    return;
  }
	float data[2];
	int idata[2];
	
	fread((char *)data,sizeof(float),2,fp);
	hm->XOrigin=data[0];
	hm->ZOrigin=data[1];

	fread((char *)data,sizeof(float),2,fp);
	float min=data[0];
	float max=data[1];
	float zrange = max-min;
	fread((char *)idata,sizeof(int),2,fp);
	hm->ZSize=idata[0];
	hm->XSize=idata[1];

	hm->RowWidth=hm->XSize;
	hm->Scale=5;
	hm->Data = new int16[hm->XSize * hm->ZSize];
	float tmp;
	int range = (int) pow(2,8) - 1;
	for(int i=0; i < hm->XSize * hm->ZSize; i++){
	  fread((char *)&tmp,sizeof(float),1,fp);
	  if(isinf(tmp))
	    //	    hm->Data[i]=100;//	    
	    tmp=min;
	    //	    hm->Data[i]=0;
	    // else
	    //	  else
	    // hm->Data[i]=0;
	    hm->Data[i]=((uint16)(((tmp-min)/zrange)*(range)));
	      //printf("%f %f %f %d %d\n",tmp, (tmp-min)/zrange,zrange,range,hm->Data[i]);
	}
	hm->XOrigin=24576;
	  hm->ZOrigin=24576;

}

void	LoadData(char *filename)
// Load some data and put it into the quadtree.
{

  HeightMapInfo	hm;
  load_hm_file(&hm,filename);
  root->AddHeightMap(RootCornerData, hm);
	

}
void	LoadData()
// Load some data and put it into the quadtree.
{
	
	
	HeightMapInfo	hm;
	hm.XOrigin = 0;
	hm.ZOrigin = 0;
	hm.XSize = 512;
	hm.ZSize = 512;
	hm.RowWidth = hm.XSize;
	hm.Scale = 7;
	hm.Data = new int16[hm.XSize * hm.ZSize];

	printf("Loading height grids...\n");

	// Big coarse data, at 128 meter sample spacing.
	FILE*	fp = fopen("demdata/gc16at128.raw", "rb");
	fread(hm.Data, sizeof(uint16), hm.XSize * hm.ZSize, fp);
	fclose(fp);
	printf("Building quadtree data...\n");
	root->AddHeightMap(RootCornerData, hm);
	
	// More detailed data at 64 meter spacing, covering the middle of the terrain.
	hm.XOrigin = 16384;
	hm.ZOrigin = 16384;
	hm.Scale = 6;
	fp = fopen("demdata/gc16at64.raw", "rb");
	fread(hm.Data, sizeof(uint16), hm.XSize * hm.ZSize, fp);
	fclose(fp);
	printf("Adding quadtree data...\n");
	root->AddHeightMap(RootCornerData, hm);
	
	// Even more detailed data, at 32 meter spacing, covering a smaller area near the middle.
	hm.XOrigin = 24576;
	hm.ZOrigin = 24576;
	hm.Scale = 5;
	fp = fopen("demdata/gc16at32.raw", "rb");
	fread(hm.Data, sizeof(uint16), hm.XSize * hm.ZSize, fp);
	fclose(fp);
	printf("Adding quadtree data...\n");
	root->AddHeightMap(RootCornerData, hm);
	
	delete [] hm.Data;
}


struct PlaneInfo {
	vector	Normal;
	float	D;

	void	Set(float nx, float ny, float nz, float d) {
		Normal.SetXYZ(nx, ny, nz);
		D = d;
	}
};
static PlaneInfo	FrustumPlane[6];
static PlaneInfo	TransformedFrustumPlane[6];


void	DisplayReshape(int w, int h)
// Called when window changes dimensions.  Set up viewport & projection matrix.
{
	glViewport(0, 0, w, h);

	//
	// Set up projection matrix.
	// This is slightly non-standard, since I prefer right-handed view coordinates, while
	// OpenGL defaults to a left-handed system.
	//
	float	nearz = 4.0;
	float	farz = 80000;

	float	AspectRatio = float(h) / float(w);
	float	ViewAngleH = 90 * (PI / 180);
	float	ViewAngleV = atan(tan(ViewAngleH/2) * AspectRatio) * 2;
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	float	m[16];
	int	i;
	for (i = 0; i < 16; i++) m[i] = 0;
	m[0] = -1.0 / tan(ViewAngleH / 2);
	m[5] = -m[0] / AspectRatio;
	m[10] = (farz + nearz) / (farz - nearz);
	m[11] = 1;
	m[14] = - 2 * farz * nearz / (farz - nearz);
	
	glMultMatrixf(m);

	glMatrixMode(GL_MODELVIEW);

	// Compute values for the frustum planes.
	FrustumPlane[0].Set(0, 0, 1, nearz);	// near.
	FrustumPlane[1].Set(0, 0, -1, -farz);	// far.
	FrustumPlane[2].Set(-cos(ViewAngleH/2), 0, sin(ViewAngleH/2), 0);	// left.
	FrustumPlane[3].Set(cos(ViewAngleH/2), 0, sin(ViewAngleH/2), 0);	// right.
	FrustumPlane[4].Set(0, -cos(ViewAngleV/2), sin(ViewAngleV/2), 0);	// top.
	FrustumPlane[5].Set(0, cos(ViewAngleV/2), sin(ViewAngleV/2), 0);	// bottom.
}


void	OGLViewMatrix(const matrix& m)
// Sets the OpenGL modelview matrix.
{
	float	mat[16];

	// Copy to the 4x4 layout.
	for (int col = 0; col < 4; col++) {
		for (int row = 0; row < 3; row++) {
			mat[col * 4 + row] = m.GetColumn(col).Get(row);
		}
		if (col < 3) {
			mat[col * 4 + 3] = 0;
		} else {
			mat[col * 4 + 3] = 1;
		}
	}

	// Apply to the current OpenGL matrix.
	glMultMatrixf(mat);
}


int	LastMouseX = 0, LastMouseY = 0;
bool	LeftButton = false, RightButton = false;

bool	Textured = false;



void	Display()
// Display function.
{
	// Turn on z-buffering.
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	// Set up the view matrix.
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	matrix	ViewMatrix;
	ViewMatrix.View(ViewerDir, ViewerUp, ViewerLoc);
	OGLViewMatrix(ViewMatrix);

	// Transform the frustum planes from view coords into world coords.
	int	i;
	for (i = 0; i < 6; i++) {
		// Rotate the plane from view coords into world coords.  I'm pretty sure this is not a slick
		// way to do this :)
		PlaneInfo&	tp = TransformedFrustumPlane[i];
		PlaneInfo&	p = FrustumPlane[i];
		ViewMatrix.ApplyInverseRotation(&tp.Normal, p.Normal);
		vector	v;
		ViewMatrix.ApplyInverse(&v, p.Normal * p.D);
		tp.D = v * tp.Normal;
	}
	
	// Clear buffers.
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw the quadtree.
	if (root) {
		root->Update(RootCornerData, (const float*) ViewerLoc, Detail);
		TriangleCounter += root->Render(RootCornerData, Textured);
	}

	// Show frame.
	glutSwapBuffers();
}


void	Idle()
{
	static int	lastticks = 0;

	int	ticks = glutGet(GLUT_ELAPSED_TIME);
	int	DeltaTicks = ticks - lastticks;
	if (DeltaTicks < 10) return;

	lastticks = ticks;

	// Move stuff.

	if (MoveForward) {
		// Move the viewer in the view direction, according to the current speed.
		ViewerLoc += ViewerDir * Speed * 10 * (DeltaTicks / 1000.0);
	}

	if (PinToGround) {
		// Force the viewer's height to be a certain distance above the ground.
		ViewerLoc.SetY(root->GetHeight(RootCornerData, ViewerLoc.X(), ViewerLoc.Z()) + ViewerHeight);
	}

	// Refresh the display.
	Display();
}


void	MouseHandler(int button, int state, int x, int y)
// Handles mouse button events.
{
	if (button == 0) {
		if (state == 0) {
			LeftButton = true;
		} else {
			LeftButton = false;
		}
	}
	if (button == 2) {
		if (state == 0) {
			RightButton = true;
		} else {
			RightButton = false;
		}
	}
	LastMouseX = x;
	LastMouseY = y;
}


void	MotionHandler(int x, int y)
// Handles mouse motion.
{
	float	dx = x - LastMouseX;
	float	dy = y - LastMouseY;
	LastMouseX = x;
	LastMouseY = y;
	
	if (LeftButton && RightButton) {
		// Translate in the view plane.
		vector	right = ViewerDir.cross(ViewerUp);
		ViewerLoc += right * dx / 3 * Speed + ViewerUp * -dy / 3 * Speed;
		
	} else if (LeftButton) {
		// Rotate the viewer.
		ViewerTheta += -dx / 100;
		while (ViewerTheta < 0) ViewerTheta += 2 * PI;
		while (ViewerTheta >= 2 * PI) ViewerTheta -= 2 * PI;

		ViewerPhi += -dy / 100;
		const float	plimit = PI / 2;
		if (ViewerPhi > plimit) ViewerPhi = plimit;
		if (ViewerPhi < -plimit) ViewerPhi = -plimit;

		ViewerDir = vector(1, 0, 0);
		ViewerUp = vector(0, 1, 0);
		
		ViewerDir = Geometry::Rotate(ViewerTheta, ViewerUp, ViewerDir);
		vector	right = ViewerDir.cross(ViewerUp);
		ViewerDir = Geometry::Rotate(ViewerPhi, right, ViewerDir);
		ViewerUp = Geometry::Rotate(ViewerPhi, right, ViewerUp);
		
	} else if (RightButton) {
		// Translate the viewer.
		vector	right = ViewerDir.cross(ViewerUp);
		ViewerLoc += right * dx / 3 * Speed + ViewerDir * -dy / 3 * Speed;
	}
}


void	SpecialKeyHandler(int key, int x, int y)
// Receives notification of function and arrow-key presses.
{
}


void	KeyHandler(unsigned char key, int x, int y)
{
	// Set the speed level of the viewer.
	if (key >= '0' && key <= '9') {
		Speed = 1 << (key - '0');
	}

	// Toggle wireframe on 'w'.
	static bool	Wireframe = false;
	if (key == 'w') {
		Wireframe = !Wireframe;
		if (Wireframe) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		} else {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}
	}

	// Toggle backface culling on 'c'.
	static bool	BackfaceCulling = false;
	if (key == 'c') {
		BackfaceCulling = !BackfaceCulling;
		if (BackfaceCulling) {
			glCullFace(GL_BACK);
			glEnable(GL_CULL_FACE);
		} else {
			glDisable(GL_CULL_FACE);
		}
	}
	
	// Toggle texturing on 't' input.
	static unsigned int	TextureName = 0;
	if (key == 't') {
		Textured = !Textured;

		if (Textured) {
			if (TextureName == 0) {
				// Load and create texture.
				glGenTextures(1, &TextureName);
				glBindTexture(GL_TEXTURE_2D, TextureName);

				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
				
				char*	data = new char[1024 * 1024 * 3];
				FILE*	fp = fopen("demdata/gctexcolored.raw", "rb");
				fread(data, 1, 1024 * 1024 * 3, fp);
				fclose(fp);
				
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
				gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, 1024, 1024, GL_RGB, GL_UNSIGNED_BYTE, data);

				delete [] data;
				
				glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			}

			glBindTexture(GL_TEXTURE_2D, TextureName);
			glEnable(GL_TEXTURE_2D);

			glColor3f(1, 1, 1);
		} else {
			glDisable(GL_TEXTURE_2D);
		}
	}

	// Toggle PinToGround mode on 'p'.
	if (key == 'p') {
		PinToGround = !PinToGround;

		if (PinToGround) {
			ViewerHeight = ViewerLoc.Y() - root->GetHeight(RootCornerData, ViewerLoc.X(), ViewerLoc.Z());
		}
	}

	// Toggle MoveForward mode on 'm'.
	if (key == 'm') {
		MoveForward = !MoveForward;
	}


	// =/- keys adjust the detail threshold.
	if (key == '-') {
		Detail *= 0.9;
		if (Detail < 10) Detail = 10;
		printf("DetailThreshold = %g\n", Detail);
	}
	if (key == '=') {
		Detail *= 1.11111111;
		if (Detail > 1500) Detail = 1500;
		printf("DetailThreshold = %g\n", Detail);
	}
	

	// On 'd', hijack the app for a second while we run some
	// performance tests.  Because of limitations in GLUT, Idle()
	// only gets called up to 18 times a second, so we can't get
	// good performance numbers that way.
	if (key == 'd') {
		int	StartTicks = glutGet(GLUT_ELAPSED_TIME);
		int	ticks;
		TriangleCounter = 0;
		int	FrameCounter = 0;
		int	TrisPerFrame = 0;

		// For approximately one second, render frames as fast as we can.
		for (;;) {
			Display();
			FrameCounter++;
			if (FrameCounter == 1) TrisPerFrame = TriangleCounter;
			
			ticks = glutGet(GLUT_ELAPSED_TIME);
			if (ticks - StartTicks > 1000) break;
		}

		// Show the fps and tps results.
		float	dt = (ticks - StartTicks) / 1000.0;
		printf("Rendered %0.1f frames/sec, %d tris/frame, %d tris/sec\n", FrameCounter / dt, TrisPerFrame, int(TriangleCounter / dt));
	}
}



//
// Frustum culling code.
//

namespace Clip {
;


Visibility	ComputeBoxVisibility(const float min[3], const float max[3])
// Returns a visibility code indicating whether the axis-aligned box defined by {min, max} is
// completely inside the frustum, completely outside the frustum, or partially in.
{
	// Doesn't do a perfect test for NOT_VISIBLE.  Just checks to
	// see if all box vertices are outside at least one frustum
	// plane.  Some pathological boxes could return SOME_CLIP even
	// though they're really fully outside the frustum.  But that
	// won't hurt us too much if it isn't a common case; the
	// contents will just be culled/clipped at a later stage in the
	// pipeline.
	
	// Check each vertex of the box against the view frustum, and compute
	// bit codes for whether the point is outside each plane.
	int	OrCodes = 0, AndCodes = ~0;

	for (int i = 0; i < 8; i++) {
		
		vector	v(min[0], min[1], min[2]);
		if (i & 1) v.SetX(max[0]);
		if (i & 2) v.SetY(max[1]);
		if (i & 4) v.SetZ(max[2]);
		
		// Now check against the frustum planes.
		int	Code = 0;
		int	Bit = 1;
		for (int j = 0; j < 6; j++, Bit <<= 1) {
			const PlaneInfo&	p = TransformedFrustumPlane[j];
			if (v * p.Normal - p.D < 0) {
				// The point is outside this plane.
				Code |= Bit;
			}
		}

		OrCodes |= Code;
		AndCodes &= Code;
	}

	// Based on bit-codes, return culling results.
	if (OrCodes == 0) {
		// The box is completely within the frustum.
		return NO_CLIP;
	} else if (AndCodes != 0) {
		// All the points are outside one of the frustum planes.
		return NOT_VISIBLE;
	} else {
		return SOME_CLIP;
	}
}


}	// end namespace Clip.

