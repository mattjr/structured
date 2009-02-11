#include <vector>
double var(float *data, int samples);
double stddev(float *data, int samples);
double sum(std::vector<double> & xList);
double mean(std::vector<double> & xList);
double median(std::vector<double> & xList);
double signed_err(float *data,unsigned short *sources ,int samples);
float *convert_to_z(float *samples, int numsamples);
double square_err(float *data,unsigned short *sources, int samples);
