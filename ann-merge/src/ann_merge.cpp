//----------------------------------------------------------------------
//		File:			ann_sample.cpp
//		Programmer:		Sunil Arya and David Mount
//		Last modified:	03/04/98 (Release 0.1)
//		Description:	Sample program for ANN
//----------------------------------------------------------------------
// Copyright (c) 1997-2005 University of Maryland and Sunil Arya and
// David Mount.  All Rights Reserved.
// 
// This software and related documentation is part of the Approximate
// Nearest Neighbor Library (ANN).  This software is provided under
// the provisions of the Lesser GNU Public License (LGPL).  See the
// file ../ReadMe.txt for further information.
// 
// The University of Maryland (U.M.) and the authors make no
// representations about the suitability or fitness of this software for
// any purpose.  It is provided "as is" without express or implied
// warranty.
//----------------------------------------------------------------------

#include <cstdlib>						// C standard library
#include <cstdio>						// C I/O (for sscanf)
#include <cstring>						// string manipulation
#include <fstream>						// file I/O
#include "/home/mkj/exp/ann_1.1.1/include/ANN/ANN.h"					// ANN declarations

#include "/home/mkj/exp/ann_1.1.1/include/ANN/ANNperf.h"					// ANN declarations

#include "/home/mkj/exp/ann_1.1.1/src/kd_tree.h"
#include "/home/mkj/exp/ann_1.1.1/src/kd_split.h"
using namespace std;					// make std:: accessible
void conv_to_ply(ANNkd_node *node,ostream &out);
void conv_to_ply(ANNkd_tree *the_tree,
		 ostream &out);
void conv_to_ply(ANNkd_leaf leaf,
	  ostream &out);
//----------------------------------------------------------------------
// ann_sample
//
// This is a simple sample program for the ANN library.	 After compiling,
// it can be run as follows.
// 
// ann_sample  [-e eps] [-df data] [-of output]
//
// where

//		eps				is the error bound (default = 0.0)
//		data			file containing data points

//
// Results are sent to the standard output.
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//	Parameters that are set in getArgs()
//----------------------------------------------------------------------
void getArgs(int argc, char **argv);			// get command-line arguments

int				k				= 1;			// number of nearest neighbors
int				dim				= 2;			// dimension
double			eps				= 0;			// error bound
int				maxPts			= 1000;			// maximum number of data points

istream*		dataIn			= NULL;			// input for data points
ofstream*		plyOut			= NULL;			// input for query points

bool readPt(istream &in, ANNpoint p)			// read point (false on EOF)
{
    float buffer[6];
    if(in.read((char *)buffer,sizeof(float)*6)){
      p[0]=buffer[0];
      p[1]=buffer[1];
      return true;
    }else
      return false;
}

void printPt(ostream &out, ANNpoint p)			// print point
{
	out << "(" << p[0];
	for (int i = 1; i < dim; i++) {
		out << ", " << p[i];
	}
	out << ")\n";
}
//----------------------------------------------------------------------
//	ANN kd- and bd-tree Dump Format
//		The dump file begins with a header containing the version of
//		ANN, an optional section containing the points, followed by
//		a description of the tree.	The tree is printed in preorder.
//
//		Format:
//		#ANN <version number> <comments> [END_OF_LINE]
//		points <dim> <n_pts>			(point coordinates: this is optional)
//		0 <xxx> <xxx> ... <xxx>			(point indices and coordinates)
//		1 <xxx> <xxx> ... <xxx>
//		  ...
//		tree <dim> <n_pts> <bkt_size>
//		<xxx> <xxx> ... <xxx>			(lower end of bounding box)
//		<xxx> <xxx> ... <xxx>			(upper end of bounding box)
//				If the tree is null, then a single line "null" is
//				output.	 Otherwise the nodes of the tree are printed
//				one per line in preorder.  Leaves and splitting nodes 
//				have the following formats:
//		Leaf node:
//				leaf <n_pts> <bkt[0]> <bkt[1]> ... <bkt[n-1]>
//		Splitting nodes:
//				split <cut_dim> <cut_val> <lo_bound> <hi_bound>
//
//		For bd-trees:
//
//		Shrinking nodes:
//				shrink <n_bnds>
//						<cut_dim> <cut_val> <side>
//						<cut_dim> <cut_val> <side>
//						... (repeated n_bnds times)
//----------------------------------------------------------------------

void ANNkd_tree::conv_to_ply(std::vector<face> *faces,ostream &out)		
// output stream
{
  ANNkdStats st;	
  getStats(st);
  int num_cells=st.n_lf;
  /*	annPrintPt(bnd_box_lo, dim, out);	// print lower bound
	out << "\n";
	annPrintPt(bnd_box_hi, dim, out);	// print upper bound
	out << "\n";
  */
  if(num_cells == 0)
    return;

  root->conv_to_ply(faces,out);				// invoke printing at root
}

void ANNkd_split::conv_to_ply(ANNkd_node *node,ostream &out)					// dump a splitting node
{
  if(!node)
    return;
  ANNkd_split *split_node=dynamic_cast<ANNkd_split *>(node);
  if(split_node){
    conv_to_ply(split_node->child[ANN_LO],out);			// print low child
    conv_to_ply(split_node->child[ANN_HI],out);			// print low child
    return;
  }else{
    ANNkd_leaf *leaf_node=dynamic_cast<ANNkd_leaf *>(node);
    if(leaf_node)
      conv_to_ply(leaf_node,out);
    return;
  }
}

void convert_to_ply(ANNkd_leaf *leaf,					// dump a leaf node
		ostream &out)					// output stream
{
  if (leaf == KD_TRIVIAL) {			// canonical trivial leaf node
	  return;				// leaf no points
	}
	else{
	  out << "leaf " << leaf->n_pts;
		for (int j = 0; j < leaf->n_pts; j++) {
			out << " " << leaf->bkt[j];
		}
		out << "\n";
	}
}
/*
void ANNbd_shrink::dump(				// dump a shrinking node
		ostream &out)					// output stream
{
	out << "shrink " << n_bnds << "\n";
	for (int j = 0; j < n_bnds; j++) {
		out << bnds[j].cd << " " << bnds[j].cv << " " << bnds[j].sd << "\n";
	}
	child[ANN_IN]->dump(out);			// print in-child
	child[ANN_OUT]->dump(out);			// print out-child
}
*/
int main(int argc, char **argv)
{
	int					nPts;
	// actual number of data points
	ANNpointArray		dataPts;				
	// data points
	
	// query point
	ANNidxArray			nnIdx;					
	// near neighbor indices
	ANNdistArray		dists;
	// near neighbor distances
	ANNkd_tree*			kdTree;					
	// search structure

	getArgs(argc, argv);
#define PTDIMENSION 3
	//float c[2*PTDIMENSION];
	// get length of file:
	int length;
	dataIn->seekg (0, ios::end);
	length = dataIn->tellg(); 
	dataIn->seekg (0, ios::beg);

	maxPts=length/(sizeof(float)*6);
	printf("File contains %d pts\n",maxPts);
	// read command-line arguments

	dataPts = annAllocPts(maxPts, dim);	
	// allocate data points
	nnIdx = new ANNidx[k];
	// allocate near neigh indices
	dists = new ANNdist[k];
	// allocate near neighbor dists

	nPts = 0;									// read data points

	cout << "Data Points:\n";
	while (nPts < maxPts && readPt(*dataIn, dataPts[nPts])) {
	  //	printPt(cout, dataPts[nPts]);
		nPts++;
		
	}

	kdTree = new ANNkd_tree(					// build search structure
					dataPts,					// the data points
					nPts,						// number of points
					dim);						// dimension of space

	conv_to_ply(kdTree,*plyOut);
	/*	ofstream out_dump_file("dump.dmp");
	if (!out_dump_file) {
	  cerr << "File name: " << "dump.dmp" << "\n";
	  // Error("Cannot open dump file", ANNabort);
	}
	// dump the tree and points
	kdTree->Dump(ANNtrue, out_dump_file);
	out_dump_file.close();*/
	plyOut->close();
    delete [] nnIdx;							// clean things up
    delete [] dists;
    delete kdTree;
	annClose();									// done with ANN

	return EXIT_SUCCESS;
}

//----------------------------------------------------------------------
//	getArgs - get command line arguments
//----------------------------------------------------------------------

void getArgs(int argc, char **argv)
{
	static ifstream dataStream;					// data file stream
	static ofstream queryStream;				// query file stream

	if (argc <= 1) {							// no arguments
		cerr << "Usage:\n\n"
		<< "  ann_sample [-d dim] [-max m] [-nn k] [-e eps] [-df data]"
		   " [-qf query]\n\n"
		<< "  where:\n"
		<< "    dim      dimension of the space (default = 2)\n"
		<< "    m        maximum number of data points (default = 1000)\n"
		<< "    k        number of nearest neighbors per query (default 1)\n"
		<< "    eps      the error bound (default = 0.0)\n"
		<< "    data     name of file containing data points\n"
		<< "    query    name of file containing query points\n\n"
		<< " Results are sent to the standard output.\n"
		<< "\n"
		<< " To run this demo use:\n"
		<< "    ann_sample -df data.pts -qf query.pts\n";
		exit(0);
	}
	int i = 1;
	while (i < argc) {							// read arguments
		if (!strcmp(argv[i], "-d")) {			// -d option
			dim = atoi(argv[++i]);				// get dimension to dump
		}
		else if (!strcmp(argv[i], "-max")) {	// -max option
			maxPts = atoi(argv[++i]);			// get max number of points
		}
		else if (!strcmp(argv[i], "-nn")) {		// -nn option
			k = atoi(argv[++i]);				// get number of near neighbors
		}
		else if (!strcmp(argv[i], "-e")) {		// -e option
			sscanf(argv[++i], "%lf", &eps);		// get error bound
		}
		else if (!strcmp(argv[i], "-df")) {		// -df option
		  dataStream.open(argv[++i], ios::in | ios::binary);// open data file
			if (!dataStream) {
				cerr << "Cannot open data file\n";
				exit(1);
			}
			dataIn = &dataStream;				// make this the data stream
		}
		else if (!strcmp(argv[i],  "-qf")) {		// -qf option
			queryStream.open(argv[++i], ios::out);// open query file
			if (!queryStream) {
			  cerr << "Cannot open query file " <<argv[i]<<endl ;
			exit(1); 
				}
		       plyOut = &queryStream;			// make this query stream
		}
		else {									// illegal syntax
			cerr << "Unrecognized option.\n";
			exit(1);
		}
		i++;
	}
	if (dataIn == NULL && plyOut == NULL) {
		cerr << "-df and -qf options must be specified\n";
		exit(1);
	}
}
