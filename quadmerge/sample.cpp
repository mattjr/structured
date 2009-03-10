#include <math.h>
#include "sample.hpp"
#include <stdio.h>
#include <limits.h>
#include "uquadtree.hpp"
#include <float.h>
float *convert_to_z(float *samples, int numsamples){
  float *new_samples=new float[numsamples];
  for(int i=0; i < numsamples; i++){
    new_samples[i]=ge.fromUINTz(samples[i]);
  }
  return new_samples;
}
double sum(std::vector<double> & xList)
{
        unsigned int items = xList.size();
        double sum = 0;
        for (unsigned int i=0; i<items; i++)
        {
                sum += xList[i];
        }
        return sum;
}


double mean(std::vector<double> & xList)
{
        return sum(xList)/xList.size();
}

double median(std::vector<double> & xList)
{
        unsigned int items = xList.size();
	if( xList.size() ==0)
	  return 0.0;
        if (items % 2 == 0) //even number of items
	  return (xList[(items+1)/2]+xList[(items-1)/2])/2;
        else
	  return xList[(items+1)/2-1];
}
/*Assumes first source is the reference */
double signed_err(float *data,unsigned short *sources, int samples){
  unsigned short min_source=SHRT_MAX;
  if(samples < 2)
    return 0.0;
  else if(samples > 2)
    fprintf(stderr, "warning not defined for more then two samples\n");

  for(int i=0; i < samples; i++){
    if(sources[i] < min_source  )
      min_source=sources[i];
  }
  
  if(min_source == SHRT_MAX){
    fprintf(stderr,"Weird error shouldn't calulate signed err when there is no referece mesh %d %d\n", samples,sources[0]);
    return 0.0;
  }
  double ref_mean=0.0;
  int mean_cnt=0;
  for(int i=0; i < samples; i++){
    if(sources[i] == min_source){
      ref_mean+=ge.fromUINTz(data[i]);
      mean_cnt++;
    }
  }

  if(mean_cnt > 0)
    ref_mean/=(double)mean_cnt;

  double other_mean=0.0;
  int other_cnt=0;
  double err=0.0;
  for(int i=0; i < samples; i++){
    if(sources[i] != min_source){
      other_mean += ge.fromUINTz(data[i]);
      other_cnt++;
    }
  }
  other_mean /= (double) other_cnt;
  return ref_mean-other_mean;
}



    
double square_err(float *data,unsigned short *sources, int samples){
  double se=sqrt(pow(signed_err(data,sources,samples),2));

  if(std::isnan(se))
    return 0.0;
  return se;
}

double stddev(float *data, int samples)
{


if(samples <= 1) return 0;
 double sum2=0.0;
 double sum=0.0;
 for(int i=0; i < samples; i++){
   sum+=data[i];
   sum2+=data[i]*data[i];
 }

 double dev= sqrt((sum2 - (sum*sum)/samples)/(samples-1));
 if(std::isnan(dev))
   dev=0.0;
 //printf("%f %f %f %f\n",data[0],data[1],sum/samples,dev); 
 return dev;


}/* end function stddev */


double var(float *data, int samples)
{


if(samples <= 1) return 0;
 double sum2=0.0;
 double sum=0.0;
 for(int i=0; i < samples; i++){
   sum+=data[i];
   sum2+=data[i]*data[i];
 }
 double mean=sum/samples;
 double var= (sum2 - samples*mean*mean)/(samples-1);
 if(std::isnan(var))
   var=0.0;
 //printf("%f %f %f %f\n",data[0],data[1],sum/samples,dev); 
 return var;


}/* end function var */
