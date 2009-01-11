#include "uquadtree.hpp"
#include <math.h>
void IntersectMap(quadsquare *heightmap,const quadcornerdata& cd, unsigned char *lightmap, unsigned char shadowColor[3], int sizex,int sizey, float lightDir[3]);
void IntersectMap(float *heightmap, unsigned char *lightmap, unsigned char shadowColor[3],int sizex,int sizey,  float lightDir[3]);
unsigned char *lsc_calcShadows(unsigned char *heightMap, int width,int height,
			       ul::vector scale, ul::vector dir);
void addTerrainShadowsToLightmap(unsigned char* heightMap,unsigned char* lightMap,size_t width, size_t height, ul::vector& lightDir, int color);
void calc_shadow(unsigned char *heightmap,unsigned char *shadowmap, int MapWidth,int MapHeight,ul::vector Sun);
