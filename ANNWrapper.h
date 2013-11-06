#ifndef __ANNWRAPPER_H__
#define __ANNWRAPPER_H__

#include <vector>
#include <ANN/ANN.h>
#include "DataStruct.h"

class ANNWrapper
{
public:
    ANNWrapper();
    ~ANNWrapper();
    void Free();
    void SetPoints(const std::vector <Vertex> &P);
    void FindClosest(const Vertex &P, double *sq_distance, int *index);
	void SetResults(int a);

private:
	int			    m_npts;					// actual number of data points
	ANNpointArray	m_data_pts;				// data points
	ANNpoint		m_query_pt;				// query point
	ANNidxArray		m_nnidx;					// near neighbor indices
	ANNdistArray	m_dists;					// near neighbor distances
	ANNkd_tree*		m_kdtree;					// search structure

    static const int dim = 3;
    int m_k;
    double m_eps;
    bool m_anything_to_free;
};


#endif
