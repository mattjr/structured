#include <math.h>
#include "sample.hpp"
#include <stdio.h>
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
        if (items % 2 == 0) //even number of items
                return (xList[(items+1)/2]+xList[(items-1)/2])/2;
        else
                return xList[(items+1)/2-1];
}



bool debug_flag=false;
double var(float *data, int samples)
{
int i;
double Xt; /* percentage price change */
double variance;
double x, x2;

if(debug_flag)
        {
        fprintf(stdout, "var(): arg data=%lu samples=%d\n",\
        (long)data, samples);
        }

/* argument check */
if(samples < 2) return 0;
if(! data) return 0;

x = 0;
x2 = 0;
for(i = 1; i < samples; i++)
        {
        if(data[i] <= 0) continue;
        if(data[i - 1] <= 0) continue;
        if(debug_flag)
                {
                fprintf(stdout, "var(): data[%d]=%.2f\n", i, (float)data[i]);
                }

        Xt = log(data[i] / data[i - 1]);

        x += Xt;
        x2 += Xt * Xt;
        }

variance = (x2 - ( (x * x) / samples) )  / (samples - 1);

if(debug_flag) printf("var(): variance=%.6f\n", (float)variance);

return variance;
}/* end function standard_deviation */


double stddev(float *data, int samples)
{
double variance;
double standard_deviation;

if(debug_flag)
        {
        fprintf(stdout, "stddev(): arg data=%lu samples=%d\n",\
        (long)data, samples);
        }

if(samples <= 1) return 0;

variance = var(data, samples);
if(variance <= 0) return 0;

standard_deviation = sqrt(variance);

if(debug_flag)
        {
        printf("stddev: standard_deviation=%.6f\n", (float)standard_deviation);
        }

return standard_deviation;
}/* end function stddev */


