#ifndef MEMUTILS_H
#define MEMUTILS_H
#include <string>
#include <fstream>
void process_mem_usage(double& vm_usage, double& resident_set);
std::string get_size_string(double kb);

#endif // MEMUTILS_H
