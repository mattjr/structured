#include "novelty.h"
osg::TextureRectangle*  getTextureHists(  CvHistogram *&finalhist){

   
   
   //long double total=0;
  
   float *tmpV= new float[16*16*16];
   osg::Image *dataImg= new osg::Image;
   dataImg->allocateImage(64,64,1,GL_LUMINANCE,GL_UNSIGNED_BYTE);
   //int count=0;
   float min=FLT_MAX;
   float max=FLT_MIN;
   float *ptr=tmpV;
   for(int i=0; i< 16; i++)
     for(int j=0; j< 16; j++)
       for(int k=0; k< 16; k++){
	 float val=cvQueryHistValue_3D(finalhist,i,j,k);


	 if(val == 0.0){
	   *ptr=FLT_MIN;
	   ptr++;
	   continue;
	 }
	 float logV=(-log(val));
	 if(logV< min)
	   min=logV;

	 if(logV> max)
	   max=logV;

	 *ptr=logV;
	 ptr++;
     
	 
	 /* cout << cvQueryHistValue_3D(finalhist,i,j,k) << " ";
	    total +=cvQueryHistValue_3D(finalhist,i,j,k);*/
       }
   printf("Min %f\n",min);
   printf("Max %f\n",max);
   float range=max-min;
   ptr=tmpV;
   for(int i=0;i < 64; i++)
     for(int j=0; j<64; j++){
       if(*ptr==FLT_MIN){
	 *ptr=0.0;
	 ptr++;
	 continue;
       }
      
       *ptr = (*ptr-min)*(255.0/range);
       //    printf("i: %d j: %d %f\n",i,j,*ptr); 
       ptr++;
     }
     cout << endl;//"\nTotal " << total <<endl;*/
  
     unsigned char *imgPtr=(unsigned char *)dataImg->data();
     ptr=tmpV;
     for(int i=0;i < 64; i++){
       for(int j=0; j<64; j++){
	 *imgPtr=(int)(*ptr);
   if(i ==1 && j==0 )
     *imgPtr=255;
	 //printf("i: %d j: %d %d\n",i,j,*imgPtr); 
	 ptr++;
	 imgPtr++;
       }
     }
     
  

   osg::TextureRectangle* texture = new osg::TextureRectangle(dataImg);
   return texture;

}
