/*Header for a class which creates quadtrees (for RQT)*/

#ifndef __SQUARELIST_H_
#define __SQUARELIST_H_

#include "assert.h"
typedef  unsigned int UINT;
#include <math.h>
#include <string.h>
#include <list>
#include <stdlib.h>
using namespace std;





class CD3DFXSquareList
{
public:
	struct SSquare
	{
		UINT nSideSize;
		UINT nUpperLeftX;
		UINT nUpperLeftY;

		double fScore;

		SSquare* rgpChildren[4];//NW, NE, SE, SW
		SSquare* rgpNeighbors[8];//left, up, right, down
	};

	
	struct SSquareWrapper
	{
	public:
		SSquare* pData;

		bool operator< (const SSquareWrapper& One)
		{
			return pData->fScore > One.pData->fScore;
		}
	};

	typedef list< SSquareWrapper > SquareList;

	enum NEIGHBOR_INDICES
	{
		NEIGHBOR_LEFT = 0,
		NEIGHBOR_UP = 2,
		NEIGHBOR_RIGHT = 4,
		NEIGHBOR_DOWN = 6
	};

	typedef  void (*ForEachConstIterator)(const SSquare*, const CD3DFXSquareList*, void*);
	typedef  void (*ForEachIterator)(SSquare*, const CD3DFXSquareList*, void*);

public:
	CD3DFXSquareList();
	~CD3DFXSquareList();

	/**
	* Initialize the quadtree.  This discards any data held in the quadtree
	* @param The number of vertices on the side of the square.  Must be a valid grid size as defined by D3DFXIsValidGridSize.
	*/
	bool Initialize(UINT nSideSize);

	/**
	* Traverse the data structure and call a user defined callback for every leaf node.
	*/
	void ForEach(ForEachConstIterator pCallback, void* pUserData) const;

	/**
	* Set the function to score each square.  The square with the highest score is tesselated.
	*/
	void SetScoreFunction(double (*pScorer)(const SSquare&, const CD3DFXSquareList&));

	/**
	* Returns the side size of the root square
	*/
	UINT GetSideSize() const;

	/**
	* Returns the number of leaf squares in the structure
	*/
	UINT GetSquareCount() const;

	/**
	* Returns the maximum number of triangles this structure could be tesselated into
	*/
	UINT GetMaxTriangleCount() const;

	/**
	* Tesselate the highest scoring square into 4 smaller squares (while making sure that each quadtree block is at most 1 level different from its neighbors).
	*/
	bool Tesselate();
	
	/**
	* Trianglate this structure.  Returns the number of triangles created.
	* @param pIndicesOut An array of indices to be filled with triangle indices.  The size of this array must be at least GetMaxTriangleCount() * 3
	*/
	UINT ConvertToTriangleList(UINT* pIndicesOut) const;

	inline const SSquare* GetRootNode() const
	{
		return m_pRoot;
	}


public:


private:
	/**
	* Erase the structure from memory
	*/
	void Cleanup();

	/**
	* Recursive implementation of FindNeighbors
	*/
	UINT FindNeighborsRecursive(const CD3DFXSquareList::SSquare* pCurrent,
								const CD3DFXSquareList::SSquare& Target, 
								const CD3DFXSquareList::SSquare** ppNeighbors) const;

	/**
	* Recursive implementation of ForEach
	*/
	void ForEachRecursive(ForEachConstIterator pCallBack, 
							void* pUserData, 
							const CD3DFXSquareList::SSquare* pCurrent) const;

	/**
	* Recursive implementation of NonConstForEach
	*/
	void ForEachNonConstRecursive(ForEachIterator pCallBack, 
									void* pUserData, 
									CD3DFXSquareList::SSquare* pCurrent);

	/**
	* Non const version of ForEach()
	*/
	void ForEachNonConst(ForEachIterator pCallback, void* pUserData);

	/**
	* Recursively deletes the quadtree.
	*/
	void DeleteTreeRecursive(CD3DFXSquareList::SSquare* pCurrent);

	/**
	* Subdivde a square into 4 smaller squares
	*/
	bool TesselateSquare(CD3DFXSquareList::SSquare* pTarget);

	/** 
	* Returns the highest bit set in the input
	*/
	UINT GetHighBitSet(UINT nTest) const;


public:
	/**
	* Returns true if the input square is has no children.
	*/
	bool IsLeafNode(const CD3DFXSquareList::SSquare& Square) const;

	/**
	* Finds all the neighbors of the input square.
	* @param Start The square to find the neighbors of.
	* @param pNeighbors An array of 8 squares which will be filled with the neighbors of the input square.
	*/
	UINT FindNeighbors(const CD3DFXSquareList::SSquare& Start, 
						const CD3DFXSquareList::SSquare** ppNeighbors) const;

	/**
	* Returns the maximum and minimum levels of the input square's neighbors.
	* @param Start The square to find the neighbors of.
	* @param pMinOut Pointer to an integer which will be filled with the minimum level of the input Square's neighbors
	* @param pMaxOut Pointer to an integer which will be filled with the maximum level of the input Square's neighbors
	*/
	void GetNeighborLevelMinMax(const CD3DFXSquareList::SSquare& Start, UINT* pMinOut, UINT* pMaxOut) const;
	
	
	/**
	* Returns true if the input square is large enough, and has the correct neighbors to be tesselated
	*/
	bool CanTesselateSquare(const CD3DFXSquareList::SSquare& Square) const;

	/**
	* Returns the tesselation level of the specified neighbor.
	*/
	/*UINT GetNeighborLevel(const CD3DFXSquareList::SSquare& Square,
							NEIGHBOR_TYPE Neighbor) const;*/





	/**
	* Returns true if the two input squares are neighbors.
	*/
	bool AreSquaresNeighbors(const CD3DFXSquareList::SSquare& SquareOne,
							const CD3DFXSquareList::SSquare& SquareTwo) const;

	/**
	* Returns true if SquareInner is inside of SquareOuter
	*/
	bool IsSquareInside(const CD3DFXSquareList::SSquare& SquareInner,
						const CD3DFXSquareList::SSquare& SquareOuter) const;

	/**
	* Returns the score of the input square.
	*/
	double ScoreSquare(const CD3DFXSquareList::SSquare& Square) const;

	/**
	* Returns the tesselation level of the input square.  0 == widest level
	*/
	UINT GetSquareLevel(const CD3DFXSquareList::SSquare& Square) const;


private:
	UINT		m_nLeafCount;

	SSquare*	m_pRoot;

	double (*m_pScoreFunction)(const SSquare&, const CD3DFXSquareList&);

	SquareList m_SortedList;
};

//closer to the center yields higher score
double DefaultScoreFunction(const CD3DFXSquareList::SSquare& Square, const CD3DFXSquareList& SquareList);






#endif//__SQUARELIST_H_
