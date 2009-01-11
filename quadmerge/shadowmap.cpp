#include "shadowmap.hpp"
#include <stdlib.h>
#include <iostream>
#include <string.h>
void IntersectMap(float *heightmap, unsigned char *lightmap, unsigned char shadowColor[3], int sizex,int sizey, float lightDir[3])
 {
        // create flag buffer to indicate where we've been
        float *flagMap = new float[sizex*sizey];
        for(int i = 0; i < sizex*sizey; i++) 
            flagMap[i] = 0;
  
        int *X, *Y;
        int iX, iY;
        int dirX, dirY;
  	float px, py, height, distance;
	int index;
         // calculate absolute values for light direction
        float lightDirXMagnitude = lightDir[0];
        float lightDirZMagnitude = lightDir[2];
        if(lightDirXMagnitude < 0) lightDirXMagnitude *= -1;
        if(lightDirZMagnitude < 0) lightDirZMagnitude *= -1;
        
        // decide which loop will come first, the y loop or x loop
        // based on direction of light, makes calculations faster
        if(lightDirXMagnitude > lightDirZMagnitude)
        {
                Y = &iX;
                X = &iY;
                
                if(lightDir[0] < 0)
                {
                        iY = sizey-1;
                        dirY = -1;
                }
                else
                {
                        iY = 0;
                        dirY = 1;
                }
                   
                if(lightDir[2] < 0)
                {
                        iX = sizex-1;
                        dirX = -1;
                }
                else
                {
                        iX = 0;
                        dirX = 1;
                }
        }
        else
        {
                Y = &iY;
                X = &iX;
                 
                if(lightDir[0] < 0)
                {
                        iX = sizex-1;
                        dirX = -1;
                }
                else
                {
                        iX = 0;
                        dirX = 1;
                }
                    
                if(lightDir[2] < 0)
                {
                        iY = sizey-1;
                        dirY = -1;
                }
                else
                {
                        iY = 0;
                        dirY = 1;
                }
        }
   
       // outer loop
       while(1)
       {
           // inner loop
           while(1)
           {
               // travel along the terrain until we:
               // (1) intersect another point
               // (2) find another point with previous collision data
               // (3) or reach the edge of the map
               px = *X;
               py = *Y;
               index = (*Y) * sizex + (*X);
  
               // travel along ray
               while(1)
               {
                   px -= lightDir[0];
                   py -= lightDir[2];
  
                  // check if we've reached the boundary
                  if(px < 0 || px >= sizex || py < 0 || py >= sizey)
                  {  
                      flagMap[index] = -1;
                      break;
                  }
  
                  // calculate interpolated values
                  static int x0, x1, y0, y1;
                  static float du, dv;
                  static float interpolatedHeight, interpolatedFlagMap;
                  static float heights[4];
                  static float pixels[4];
                  static float invdu, invdv;
                  static float w0, w1, w2, w3;
  
                  x0 = floor(px);
                  x1 = ceil(px);
                  y0 = floor(py);
                  y1 = ceil(py);
  
                 du = px - x0;
                 dv = py - y0;
              
                 invdu = 1.0 - du;
                 invdv = 1.0 - dv;
                 w0 = invdu * invdv;
                 w1 = invdu * dv;
                 w2 = du * invdv;
                 w3 = du * dv;
  
                 // compute interpolated height value from the heightmap direction below ray
                 heights[0] = heightmap[y0*sizex+x0];
                 heights[1] = heightmap[y1*sizex+x0];
                 heights[2] = heightmap[y0*sizex+x1];
                 heights[3] = heightmap[y1*sizex+x1];
                 interpolatedHeight = w0*heights[0] + w1*heights[1] + w2*heights[2] + w3*heights[3];
  
                 // compute interpolated flagmap value from point directly below ray
                 pixels[0] = flagMap[y0*sizex+x0];
                 pixels[1] = flagMap[y1*sizex+x0];
                 pixels[2] = flagMap[y0*sizex+x1];
                 pixels[3] = flagMap[y1*sizex+x1];
                 interpolatedFlagMap = w0*pixels[0] + w1*pixels[1] + w2*pixels[2] + w3*pixels[3];
  
                 // get distance from original point to current point
                 distance = sqrt( (px-*X)*(px-*X) + (py-*Y)*(py-*Y) );
  
                 // get height at current point while traveling along light ray
                 height = heightmap[index] + lightDir[1]*distance;
  
                 // check intersection with either terrain or flagMap
                 // if interpolatedHeight is less than interpolatedFlagMap that means we need to use the flagMap value instead
                 // else use the height value
                 static float val;
                 val = interpolatedHeight;
                 if(interpolatedHeight < interpolatedFlagMap) val = interpolatedFlagMap;
                 if(height < val)
                 {
                     flagMap[index] = val - height;
  
                     lightmap[index*3+0] = shadowColor[0];
                     lightmap[index*3+1] = shadowColor[1];
                     lightmap[index*3+2] = shadowColor[2];
                
                     break;
                 }
  
                 // check if pixel we've moved to is unshadowed
                 // since the flagMap value we're using is interpolated, we will be in between shadowed and unshadowed areas
                 // to compensate for this, simply define some epsilon value and use this as an offset from -1 to decide
                 // if current point under the ray is unshadowed
                 static float epsilon = 0.5f;
                 if(interpolatedFlagMap < -1.0f+epsilon && interpolatedFlagMap > -1.0f-epsilon)
                 {
                     flagMap[index] = -1.0f;
                     break;
                 }   
             }
       
             // update inner loop variable
             if(dirY < 0)
             {
                 iY--;
                 if(iY < 0) break;
             }
             else
             {
                 iY++;
                 if(iY >= sizey) break;
             }
         }
  
         // reset inner loop starting point
         if(dirY < 0) iY = sizey - 1;
         else iY = 0;
  
         // update outer loop variable
         if(dirX < 0)
         {
             iX--;
             if(iX < 0) break;
         }
         else
         {
             iX++;
             if(iX >= sizex) break;
         }
     }
   
     delete [] flagMap;
 }

void IntersectMap2(quadsquare *heightmap,const quadcornerdata& cd, unsigned char *lightmap, unsigned char shadowColor[3], int size, float lightDir[3])
 {
        // create flag buffer to indicate where we've been
        float *flagMap = new float[size*size];
        for(int i = 0; i < size*size; i++) 
            flagMap[i] = 0;
  
        int *X, *Y;
        int iX, iY;
        int dirX, dirY;
  
         // calculate absolute values for light direction
        float lightDirXMagnitude = lightDir[0];
        float lightDirZMagnitude = lightDir[2];
        if(lightDirXMagnitude < 0) lightDirXMagnitude *= -1;
        if(lightDirZMagnitude < 0) lightDirZMagnitude *= -1;
        
        // decide which loop will come first, the y loop or x loop
        // based on direction of light, makes calculations faster
        if(lightDirXMagnitude > lightDirZMagnitude)
        {
                Y = &iX;
                X = &iY;
                
                if(lightDir[0] < 0)
                {
                        iY = size-1;
                        dirY = -1;
                }
                else
                {
                        iY = 0;
                        dirY = 1;
                }
                   
                if(lightDir[2] < 0)
                {
                        iX = size-1;
                        dirX = -1;
                }
                else
                {
                        iX = 0;
                        dirX = 1;
                }
        }
        else
        {
                Y = &iY;
                X = &iX;
                 
                if(lightDir[0] < 0)
                {
                        iX = size-1;
                        dirX = -1;
                }
                else
                {
                        iX = 0;
                        dirX = 1;
                }
                    
                if(lightDir[2] < 0)
                {
                        iY = size-1;
                        dirY = -1;
                }
                else
                {
                        iY = 0;
                        dirY = 1;
                }
        }
	float px, py, height, distance;
	int index;
	int indexx,indexy;
       // outer loop
       while(1)
       {
           // inner loop
           while(1)
           {
               // travel along the terrain until we:
               // (1) intersect another point
               // (2) find another point with previous collision data
               // (3) or reach the edge of the map
               px = *X;
               py = *Y;
               index = (*Y) * size + (*X);
	       indexx= (*X);
	       indexy= (*Y);
               // travel along ray
               while(1)
               {
                   px -= lightDir[0];
                   py -= lightDir[2];
  
                  // check if we've reached the boundary
                  if(px < 0 || px >= size || py < 0 || py >= size)
                  {  
                      flagMap[index] = -1;
                      break;
                  }
  
                  // calculate interpolated values
                  static int x0, x1, y0, y1;
                  static float du, dv;
                  static float interpolatedHeight, interpolatedFlagMap;
                  static float heights[4];
                  static float pixels[4];
                  static float invdu, invdv;
                  static float w0, w1, w2, w3;
  
                  x0 = floor(px);
                  x1 = ceil(px);
                  y0 = floor(py);
                  y1 = ceil(py);
  
                 du = px - x0;
                 dv = py - y0;
              
                 invdu = 1.0 - du;
                 invdv = 1.0 - dv;
                 w0 = invdu * invdv;
                 w1 = invdu * dv;
                 w2 = du * invdv;
                 w3 = du * dv;
  
                 // compute interpolated height value from the heightmap direction below ray
		 /*                 heights[0] = heightmap[y0*size+x0];
                 heights[1] = heightmap[y1*size+x0];
                 heights[2] = heightmap[y0*size+x1];
                 heights[3] = heightmap[y1*size+x1];*/

		 heights[0] = heightmap->GetHeight(cd,cd.xorg+x0,cd.yorg+y0);
		 heights[1] = heightmap->GetHeight(cd,cd.xorg+x0,cd.yorg+y1);
		 heights[2] = heightmap->GetHeight(cd,cd.xorg+x1,cd.yorg+y0);
		 heights[3] = heightmap->GetHeight(cd,cd.xorg+x1,cd.yorg+y1);
                 interpolatedHeight = w0*heights[0] + w1*heights[1] + w2*heights[2] + w3*heights[3];
  
                 // compute interpolated flagmap value from point directly below ray
                 pixels[0] = flagMap[y0*size+x0];
                 pixels[1] = flagMap[y1*size+x0];
                 pixels[2] = flagMap[y0*size+x1];
                 pixels[3] = flagMap[y1*size+x1];
                 interpolatedFlagMap = w0*pixels[0] + w1*pixels[1] + w2*pixels[2] + w3*pixels[3];
  
                 // get distance from original point to current point
                 distance = sqrt( (px-*X)*(px-*X) + (py-*Y)*(py-*Y) );
  
                 // get height at current point while traveling along light ray
                 height = heightmap->GetHeight(cd,indexx,indexy) + lightDir[1]*distance;
  
                 // check intersection with either terrain or flagMap
                 // if interpolatedHeight is less than interpolatedFlagMap that means we need to use the flagMap value instead
                 // else use the height value
                 static float val;
                 val = interpolatedHeight;
                 if(interpolatedHeight < interpolatedFlagMap) val = interpolatedFlagMap;
                 if(height < val)
                 {
                     flagMap[index] = val - height;
  
                     lightmap[index*3+0] = shadowColor[0];
                     lightmap[index*3+1] = shadowColor[1];
                     lightmap[index*3+2] = shadowColor[2];
                
                     break;
                 }
  
                 // check if pixel we've moved to is unshadowed
                 // since the flagMap value we're using is interpolated, we will be in between shadowed and unshadowed areas
                 // to compensate for this, simply define some epsilon value and use this as an offset from -1 to decide
                 // if current point under the ray is unshadowed
                 static float epsilon = 0.5f;
                 if(interpolatedFlagMap < -1.0f+epsilon && interpolatedFlagMap > -1.0f-epsilon)
                 {
                     flagMap[index] = -1.0f;
                     break;
                 }   
             }
       
             // update inner loop variable
             if(dirY < 0)
             {
                 iY--;
                 if(iY < 0) break;
             }
             else
             {
                 iY++;
                 if(iY >= size) break;
             }
         }
  
         // reset inner loop starting point
         if(dirY < 0) iY = size - 1;
         else iY = 0;
  
         // update outer loop variable
         if(dirX < 0)
         {
             iX--;
             if(iX < 0) break;
         }
         else
         {
             iX++;
             if(iX >= size) break;
         }
     }
   
     delete [] flagMap;
 }
/*
  shadowMapPut()

  This is a helper function for drawing into the shadow map. This function
  is used when casting a ray from a heightmap point. If the height of the
  heightmap at point x, y is less than the height z of the ray at point x, y
  then a full darkness shadow pixel is drawn.
*/
static void lsc_shadowMapPut(unsigned char *shadowMap,
			     unsigned char *heightMap, int width,int height,
                                                         int x, int y, float z)
{
        /* Assume size is a power of two */
        x = x & (width - 1);
	y = y & (height- 1);
        if (heightMap[x + y*(width + 1)] <= z)
                shadowMap[x + y*width] = 255;
	
}
 
void addTerrainShadowsToLightmap(unsigned char* heightMap,unsigned char* lightMap,size_t width, size_t height, ul::vector& lightDir, int color)
    {
      // algorithm for ray traced shadow map as described here:
      // http://gpwiki.org/index.php/Faster_Ray_Traced_Terrain_Shadow_Maps
      size_t i, j;
      size_t *x, *z;
      int iDir, jDir;
      size_t iSize, jSize;
      float lDirXAbs = fabs(lightDir[0]);
      float lDirZAbs = fabs(lightDir[2]);

      // based on the direction of light, decide in which order to traverse
      // to speed up calculations
      if (lDirXAbs > lDirZAbs)
      {
        z = &i;
        x = &j;
        iSize = height;
        jSize = width;
        if (lightDir[0] < 0)
        {
          j = jSize - 1;
          jDir = -1;
        }
        else
        {
          j = 0;
          jDir = 1;
        }
        if (lightDir[2] < 0)
        {
          i = iSize - 1;
          iDir = -1;
        }
        else
        {
          i = 0;
          iDir = 1;
        }
      }
      else
      {
        x = &i;
        z = &j;
        jSize = height;
        iSize = width;
        if (lightDir[0] < 0)
        {
          i = iSize - 1;
          iDir = -1;
        }
        else
        {
          i = 0;
          iDir = 1;
        }
        if (lightDir[2] < 0)
        {
          j = jSize - 1;
          jDir = -1;
        }
        else
        {
          j = 0;
          jDir = 1;
        }
      }

      // calculate the step size to use
      /*AxisAlignedBox extents = info.getExtents();
      Vector3 pos = extents.getMinimum();
      Vector3 step = extents.getMaximum() - extents.getMinimum();
      step.x /= width;
      step.z /= height;
      */
      int stepx=1,stepz=1;
      float* flagMap = new float[width*height];
      memset(flagMap, 0, width*height*sizeof(float));

      while (1)
      {
        while (1)
        {
          // travel along terrain until we:
          // (1) intersect another point
          // (2) find another point with previous collision data
          // (3) reach the edge of the map
          float px = *x;
          float pz = *z;
          size_t index = (*z) * width + (*x);

          // travel along ray
          while (1)
          {
            px -= lightDir[0];
            pz -= lightDir[2];

            // check if we've reached the boundary
            if (px < 0 || px >= width || pz < 0 || pz >= height)
            {
              flagMap[index] = -1.0f;
              break;
            }

            // calculate interpolated values
            int x0 = (int)floor(px);
            int x1 = (int)ceil(px);
            int z0 = (int)floor(pz);
            int z1 = (int)ceil(pz);

            float du = px - x0;
            float dv = pz - z0;
            float invdu = 1.0 - du;
            float invdv = 1.0 - dv;
            float w0 = invdu * invdv;
            float w1 = invdu * dv;
            float w2 = du * invdv;
            float w3 = du * dv;

            // get interpolated height at position
	    //ul::vector curPos = pos + ul::vector(px*stepx, 0, pz*stepz);
	    // float ipHeight = heightMap[pz*width + px];//info.getHeightAt(curPos[0], curPos[2]) - pos.y;
	    float heights[4];
	    heights[0] = heightMap[z0*width+x0];
            heights[1] = heightMap[z1*width+x0];
            heights[2] = heightMap[z0*width+x1];
            heights[3] = heightMap[z1*width+x1];

	    float ipHeight= w0*heights[0] + w1*heights[1] + w2*heights[2] + w3*heights[3];


            // compute interpolated flagmap value
            float pixels[4];
            pixels[0] = flagMap[z0*width+x0];
            pixels[1] = flagMap[z1*width+x0];
            pixels[2] = flagMap[z0*width+x1];
            pixels[3] = flagMap[z1*width+x1];
            float ipFlag = w0*pixels[0] + w1*pixels[1] + w2*pixels[2] + w3*pixels[3];

            // get distance from original point to current point
            float realXDist = (px - *x) * stepx;
            float realZDist = (pz - *z) * stepz;
            float distance = sqrt(realXDist*realXDist + realZDist*realZDist);

            // calculate ray height at current point
	    // float height =  // info.getHeightAt(pos[0] + (*x)*stepx, pos[2] + (*z)*stepz) - pos[1] - lightDir[1]*distance;
	    height = heightMap[index] + lightDir[1]*distance;
  

            // check intersection with either terrain or flagMap
            // if ipHeight < ipFlag check against flagMap value
            float val = (ipHeight < ipFlag ? ipFlag : ipHeight);
	    //	    printf("%d %d\n",i,j);
            if (height < val)
            {
              // point in shadow
              flagMap[index] = val - height;
	      /*   lightMap[index*3+0] = (uchar) (255*ambient.r);
              lightMap[index*3+1] = (uchar) (255*ambient.g);
              lightMap[index*3+2] = (uchar) (255*ambient.b);*/
	      lightMap[index] = color;
              break;
            }

            // check if pixel we moved to is unshadowed
            // since the flagMap value is interpolated, we use an epsilon value to check
            // if it's close enough to -1 to indicate non-shadow
            const float epsilon = 0.5f;
            if (ipFlag < -1.0f+epsilon && ipFlag > -1.0f-epsilon)
            {
              flagMap[index] = -1.0f;
              break;
            }
          }

          // update inner loop
          j += jDir;
          if (j >= jSize) // due to size_t, if j < 0, will wrap around and be > jSize ;)
            break;
        }

        // reset inner loop starting point
        if (jDir < 0)
          j = jSize-1;
        else
          j = 0;

        // update outer loop variable
        i += iDir;
        if (i >= iSize)
          break;

      }

      delete[] flagMap;
    }



/*
  calcShadows()

  For each point in the heightmap we cast a ray in the light direction. Any
  points intersected in x-y which are below the ray in z are deemed to be in
  shadow.
*/
unsigned char *lsc_calcShadows(unsigned char *heightMap, int width,int height,
                                                                                        ul::vector scale, ul::vector dir)
{
        const float scaleNormal = 1.0f / 127.0f;
        unsigned char *shadowMap;
        ul::vector L;
        int X, Y;

        printf("Calculating shadow map");
        fflush(stdout);

        /* Allocate space for the shadow map */
        shadowMap = (unsigned char*)malloc(height *width);
        if (!shadowMap) {
	  fprintf(stderr,"lsc_calcShadows: cannot allocate %d bytes", width*height);
                return shadowMap;
        }
        memset(shadowMap, 0,height *width);

        /* Make sure the light source is normalised */
        L=dir;//vec3_cpy(dir, L);
        L.SetX(L.X()/scale.X()); // vec3_div(L, scale, L);
        L.SetY(L.Y()/scale.Y()); // vec3_div(L, scale, L);
        L.SetZ(L.Z()/scale.Z()); // vec3_div(L, scale, L);
	//        vec3_norm(L);
	L.normalize();
        if (L.Z() == 0.0f) {
                /* Pathological case */
	  fprintf(stderr,"lsc_calcShadows: light vector horizontal");
                return shadowMap;
        }

        /* For each heightmap vertex */
        for (Y = 0; Y < height; Y++) {

                /* Show progress */
                if (!(Y & 0x3F)) {
                        printf(".");
                        fflush(stdout);
                }

                for (X = 0; X < width; X++) {
                        float z;

                        /* If vertex already in shadow ignore it */
                        if (shadowMap[X + Y*width])
                                continue;

                        /* Step along a line through the vertex in the direction of L */

                        z = (float)heightMap[X + Y*(width + 1)];
                        if (fabs(L[0]) < fabs(L[1])) {
                                float incx = L[0] / L[1];
                                float incz = L[2] / L[1];
                                int y, incy = 1;
                                float x;
                                if (L[1] < 0) {
                                        incx = -incx; incy = -incy, incz = -incz;
                                }
                                x = X + incx;
                                z += incz;
                                for (y = Y + incy; 1; x += incx, y += incy, z += incz) {
                                        if (z < 0.0f)
                                                break;
                                        lsc_shadowMapPut(shadowMap, heightMap, width,height, (int)x, y, z);
                                }
                        }
                        else {
                                float incy = L[1] / L[0];
                                float incz = L[2] / L[0];
                                int x, incx = 1;
                                float y;
                                if (L[0] < 0) {
                                        incx = -incx; incy = -incy, incz = -incz;
                                }
                                y = Y + incy;
                                z += incz;
                                for (x = X + incx; 1; x += incx, y += incy, z += incz) {
                                        if (z < 0.0f)
                                                break;
                                        lsc_shadowMapPut(shadowMap, heightMap, width,height, x, (int)y, z);
                                }
                        }
                }
        }

        printf("\n");
        fflush(stdout);

        return shadowMap;
}
static void blurLightMap(unsigned char *lightMap, int width,int height, ul::vector dir)
{
        unsigned char *lightMap2;
        int blurMap[9] = { 64,  64,  64,
                                           64, 255,  64,
                                           64,  64,  64 };
        int X, Y, i, divisor = 0;

        lightMap2 = (unsigned char*)malloc(width*height);
        if (!lightMap2) {
	  fprintf(stderr,"lsc_blurLightMap: cannot allocate %d bytes", width*height);
                return;
        }
        memset(lightMap2, 0, width*height);

        if (dir[0] > 0.6) {
                blurMap[2] = blurMap[5] = blurMap[8] = 128;
        }
        else if (dir[0] < 0.6) {
                blurMap[0] = blurMap[3] = blurMap[6] = 128;
        }
        if (dir[1] > 0.6) {
                blurMap[0] = blurMap[1] = blurMap[2] = 128;
        }
        else if (dir[1] < 0.6) {
                blurMap[6] = blurMap[7] = blurMap[8] = 128;
        }

        for (i = 0; i < 9; i++)
                divisor += blurMap[i];

        /* For each heightmap vertex */
        for (Y = 0; Y < height; Y++) {
                for (X = 0; X < width; X++) {
                        int x, y, accum = 0;
                        for (y = 0; y < 3; y++) {
                                for (x = 0; x < 3; x++) {
                                        accum += blurMap[x + 3*y]
					  * lightMap[(X+x-1)+((Y+y-1) *width)];
                                }
                        }
                        accum /= divisor;
                        if (accum > 255)
                                accum = 255;
			lightMap2[X+ Y*width]=(unsigned char)accum;
                }
        }
        memcpy(lightMap, lightMap2, height*width);
        free(lightMap2);
}

void calc_shadow(unsigned char *heightmap,unsigned char *shadowmap, int MapWidth,int MapHeight,ul::vector Sun){

  
  ul::vector CurrentPos;
  ul::vector LightDir;
  
  
  
  int LerpX, LerpZ;
  
  // const int MapWidth = hmap.GetWidth(), MapHeight = hmap.GetHeight();
  
  //Initialize new shadow map
  //ShadowMap.Create(MapWidth, MapHeight);
  
  std::cout << "Status = ";
  
  //For every pixel on the map
  for (size_t z=0; z<MapHeight; ++z)
  {
    for (size_t x=0; x<MapWidth; ++x)
    {
      //Set current position in terrain
      CurrentPos.SetXYZ((float)x, heightmap[x+ z*MapWidth], (float)z);
  
      //Calc new direction of lightray
      LightDir = Sun - CurrentPos;
      LightDir.normalize();
  
      shadowmap[x+ z*MapWidth]= 255;
  
      //Start the test
      while ( CurrentPos.X() >= 0 &&
          CurrentPos.X() < MapWidth && 
          CurrentPos.Z() >= 0 && 
          CurrentPos.Z() < MapHeight && 
          CurrentPos != Sun && CurrentPos.Y() < 255 )
      {
        CurrentPos+=LightDir;
    
        LerpX = round(CurrentPos.X());
        LerpZ = round(CurrentPos.Z());
  
        //Hit?
        if(CurrentPos.Y() <= heightmap[LerpX+ LerpZ*MapWidth])
        { 
          shadowmap[x+ z*MapWidth]=0;
          break;
        }
      }
    }
    if (!(z%32)) std::cout << "0";
    fflush(stdout);
  }
 std::cout << std::endl;
 std::cout <<"Blurring...\n";
 blurLightMap(shadowmap, MapWidth, MapHeight, Sun);
}  
 
  
