#include <math.h>
#include "sample.hpp"
#include <stdio.h>
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


