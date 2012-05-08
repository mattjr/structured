#include <stdlib.h>


#include "defs.h"
#include "structs.h"


int greater_eq(int i, int j, struct vtx_data ** graph) {
	return graph[i]->nedges >= graph[j]->nedges;
}

int smaller_eq(int i, int j, struct vtx_data ** graph) {
	return graph[i]->nedges <= graph[j]->nedges;
}

void split(struct vtx_data ** graph, int *nodes, int first,int last,int *middle) {
	int splitter=rand()*(last-first)/RAND_MAX+first;
	int val;
	int left=first+1;
	int right=last;
	int temp;

	val=nodes[splitter];  nodes[splitter]=nodes[first]; nodes[first]=val;

	while (left<right) {
		while (left<right &&  greater_eq(nodes[left],val,graph)) 
			left++;
		while (left<right && smaller_eq(nodes[right],val,graph)) 
			right--;
		if (left<right) {
			temp=nodes[left]; nodes[left]=nodes[right]; nodes[right]=temp; 
			left++; right--; // (1)
							 
		}
	}
	// in this point either, left==right (meeting), or left=right+1 (because of (1))
	// we have to decide to which part the meeting point (or left) belongs.
	
	if (!greater_eq(nodes[left],val,graph))
		left=left-1; // notice that always left>first, because of its initialization
	*middle=left;
	nodes[first]=nodes[*middle];
	nodes[*middle]=val;
}

void quicksort(struct vtx_data ** graph, int *nodes, int first,int last) {
	if (first<last) {
		int middle;
		split(graph,nodes,first,last,&middle);
		quicksort(graph,nodes,first,middle-1);
		quicksort(graph,nodes,middle+1,last);
	}
}

