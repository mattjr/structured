#include "D3DFXSquareList.h"










void ToTriangleIterator(const CD3DFXSquareList::SSquare*,
						const CD3DFXSquareList*,
						void* pUserData);




void Scorer(CD3DFXSquareList::SSquare* CurrentSquare, 
			const CD3DFXSquareList* List, 
			void* pSortedList);


CD3DFXSquareList::CD3DFXSquareList()
{
	m_pRoot = NULL;
	m_nLeafCount = 0;

	m_pScoreFunction = DefaultScoreFunction;
}

CD3DFXSquareList::~CD3DFXSquareList()
{
	Cleanup();
}

void CD3DFXSquareList::Cleanup()
{
	DeleteTreeRecursive(m_pRoot);
	m_pRoot = NULL;
	m_nLeafCount = 0;

	if(m_SortedList.size()>0)
	{
		m_SortedList.clear();
	}


}


//public interface
//not 64-bit safe
UINT CD3DFXSquareList::ConvertToTriangleList(UINT* pIndicesOut) const
{
	assert(NULL != pIndicesOut);

	if(NULL == m_pRoot)
	{
		return false;
	}

	UINT* pOriginalIndex = pIndicesOut;

	UINT* pCurrentIndex = pIndicesOut;

	ForEach(ToTriangleIterator, &pCurrentIndex);
	
	long int nIndicesMade = (((UINT)pCurrentIndex) - ((UINT)pOriginalIndex)) / sizeof(UINT);

	return nIndicesMade;
}

bool CD3DFXSquareList::Initialize(UINT nSideSize)
{
	Cleanup();

	m_pRoot = new SSquare;
	if(NULL == m_pRoot)
	{
		return false;
	}

	
	memset(m_pRoot, 0, sizeof(SSquare));
	
	m_pRoot->nSideSize = nSideSize;
	m_pRoot->nUpperLeftX = 0; 
	m_pRoot->nUpperLeftY = 0;

	m_nLeafCount = 1;

	SSquareWrapper RootNode;

	RootNode.pData = m_pRoot;

	try
	{
		m_SortedList.push_back(RootNode);
	}
	catch(...)
	{
		return false;
	}

	
	return true;
}






void CD3DFXSquareList::ForEach(ForEachConstIterator pCallBack, 
							   void* pUserData) const
{
	assert(NULL != pCallBack);

	ForEachRecursive(pCallBack,
					pUserData,
					m_pRoot);
}

void CD3DFXSquareList::SetScoreFunction(double (*pScorer)(const SSquare&, const CD3DFXSquareList&))
{
	assert(NULL != pScorer);
	m_pScoreFunction = pScorer;
}

UINT CD3DFXSquareList::GetSideSize() const
{
	assert(NULL != m_pRoot);

	return m_pRoot->nSideSize;
}

UINT CD3DFXSquareList::GetSquareCount() const
{
	return m_nLeafCount;
}


UINT CD3DFXSquareList::GetMaxTriangleCount() const
{
	return m_nLeafCount * 8;
}





bool CD3DFXSquareList::Tesselate()
{
	CD3DFXSquareList::SSquare* pTarget = NULL;

	SquareList::iterator Best;

	try
	{
		//sort all squares, construct list of posibilities
		
		Best = m_SortedList.begin();

		while(NULL == pTarget)
		{
			if(Best == m_SortedList.end())
			{
				return false;//no ok squares to tesselate
			}

			CD3DFXSquareList::SSquare* pSquare = (*Best).pData;

			if(CanTesselateSquare(*pSquare))
			{
				pTarget = pSquare;
			}
			else
			{
				++Best;
			}
		}
	}
	catch(...)
	{
		return false;
	}


	if(TesselateSquare(pTarget))
	{
		try
		{
			m_nLeafCount+=3;

			m_SortedList.erase(Best);

			for(UINT nChild = 0; nChild<4; nChild++)
			{
				SquareList NewList;

				SSquareWrapper Child;
				Child.pData = pTarget->rgpChildren[nChild];

				NewList.insert(NewList.begin(), Child);
				m_SortedList.merge(NewList);
			}

			return true;
		}
		catch(...)
		{
			return false;
		}	 

	}
	else
	{
		return false;
	}
}



void CD3DFXSquareList::ForEachNonConst(ForEachIterator pCallBack, 
										void* pUserData)
{
	assert(NULL != pCallBack);

	ForEachNonConstRecursive(pCallBack,
							pUserData,
							m_pRoot);
}

//Recursive functions



void CD3DFXSquareList::ForEachRecursive(ForEachConstIterator pCallBack, 
									  void* pUserData,
									  const CD3DFXSquareList::SSquare* pCurrent) const
{
	assert(NULL != pCallBack);

	if(IsLeafNode(*pCurrent))
	{
		pCallBack(pCurrent, this, pUserData);
	}
	else
	{
		for(UINT nCtr = 0; nCtr<4; nCtr++)
		{
			ForEachRecursive(pCallBack, pUserData, pCurrent->rgpChildren[nCtr]);
		}
	}
}


void CD3DFXSquareList::ForEachNonConstRecursive(ForEachIterator pCallBack, 
												  void* pUserData,
												  CD3DFXSquareList::SSquare* pCurrent)
{
	assert(NULL != pCallBack);

	if(IsLeafNode(*pCurrent))
	{
		pCallBack(pCurrent, this, pUserData);
	}
	else
	{
		for(UINT nCtr = 0; nCtr<4; nCtr++)
		{
			ForEachNonConstRecursive(pCallBack, pUserData, pCurrent->rgpChildren[nCtr]);
		}
	}
}

void CD3DFXSquareList::DeleteTreeRecursive(CD3DFXSquareList::SSquare* pCurrent)
{
	if(NULL != pCurrent)
	{
		if(IsLeafNode(*pCurrent))
		{
			delete pCurrent;
		}
		else
		{
			for(UINT nCtr = 0; nCtr<4; nCtr++)
			{
				DeleteTreeRecursive(pCurrent->rgpChildren[nCtr]);
			}
			delete pCurrent;
		}
	}
}



//square functions
struct SNeighborList
{
	UINT			 rgnInternalNeighborIndices[2];
	CD3DFXSquareList::NEIGHBOR_INDICES rgnInternalNeighbors[2];
	CD3DFXSquareList::NEIGHBOR_INDICES rgnExternalNeighors[2];
	UINT			 rgnExternalNeighborOffsets[2];
};

bool CD3DFXSquareList::TesselateSquare(CD3DFXSquareList::SSquare* pTarget)
{
	assert(NULL != pTarget);

	if(pTarget->nSideSize < 3)
	{
		return false;//too small to be subdivided 
	}

	UINT nLevel = GetSquareLevel(*pTarget);


	UINT nCtr;
	
	for(nCtr = 0; nCtr < 4; nCtr++)
	{
		pTarget->rgpChildren[nCtr] = new CD3DFXSquareList::SSquare;
		if(NULL == pTarget->rgpChildren[nCtr])
		{
			return false;
		}

		memset((pTarget->rgpChildren[nCtr]), 0, sizeof(CD3DFXSquareList::SSquare));

		pTarget->rgpChildren[nCtr]->nUpperLeftX = pTarget->nUpperLeftX + 
													((pTarget->nSideSize/2) * (nCtr % 2));
		pTarget->rgpChildren[nCtr]->nUpperLeftY = pTarget->nUpperLeftY + 
													((pTarget->nSideSize/2) * (nCtr / 2));

		pTarget->rgpChildren[nCtr]->nSideSize = (pTarget->nSideSize/2) + 1;

		pTarget->rgpChildren[nCtr]->fScore = ScoreSquare(*(pTarget->rgpChildren[nCtr]));
	}


	// 0 1
	// 2 3

	SNeighborList rgNeighborDependencies[4];


	rgNeighborDependencies[0].rgnInternalNeighbors[0] = NEIGHBOR_RIGHT;
	rgNeighborDependencies[0].rgnInternalNeighbors[1] = NEIGHBOR_DOWN;
	rgNeighborDependencies[0].rgnInternalNeighborIndices[0] = 1;
	rgNeighborDependencies[0].rgnInternalNeighborIndices[1] = 2;

	rgNeighborDependencies[1].rgnInternalNeighbors[0] = NEIGHBOR_LEFT;
	rgNeighborDependencies[1].rgnInternalNeighbors[1] = NEIGHBOR_DOWN;
	rgNeighborDependencies[1].rgnInternalNeighborIndices[0] = 0;
	rgNeighborDependencies[1].rgnInternalNeighborIndices[1] = 3;

	rgNeighborDependencies[2].rgnInternalNeighbors[0] = NEIGHBOR_RIGHT;
	rgNeighborDependencies[2].rgnInternalNeighbors[1] = NEIGHBOR_UP;
	rgNeighborDependencies[2].rgnInternalNeighborIndices[0] = 3;
	rgNeighborDependencies[2].rgnInternalNeighborIndices[1] = 0;

	rgNeighborDependencies[3].rgnInternalNeighbors[0] = NEIGHBOR_LEFT;
	rgNeighborDependencies[3].rgnInternalNeighbors[1] = NEIGHBOR_UP;
	rgNeighborDependencies[3].rgnInternalNeighborIndices[0] = 2;
	rgNeighborDependencies[3].rgnInternalNeighborIndices[1] = 1;


	for(nCtr = 0; nCtr<2; nCtr++)
	{
		rgNeighborDependencies[0].rgnExternalNeighors[nCtr] = rgNeighborDependencies[3].rgnInternalNeighbors[nCtr];
		rgNeighborDependencies[3].rgnExternalNeighors[nCtr] = rgNeighborDependencies[0].rgnInternalNeighbors[nCtr];

		rgNeighborDependencies[1].rgnExternalNeighors[nCtr] = rgNeighborDependencies[2].rgnInternalNeighbors[nCtr];
		rgNeighborDependencies[2].rgnExternalNeighors[nCtr] = rgNeighborDependencies[1].rgnInternalNeighbors[nCtr];
	}

	rgNeighborDependencies[0].rgnExternalNeighborOffsets[0] = 0;
	rgNeighborDependencies[0].rgnExternalNeighborOffsets[1] = 0;

	rgNeighborDependencies[1].rgnExternalNeighborOffsets[0] = 0;
	rgNeighborDependencies[1].rgnExternalNeighborOffsets[1] = 1;

	rgNeighborDependencies[2].rgnExternalNeighborOffsets[0] = 1;
	rgNeighborDependencies[2].rgnExternalNeighborOffsets[1] = 0;

	rgNeighborDependencies[3].rgnExternalNeighborOffsets[0] = 1;
	rgNeighborDependencies[3].rgnExternalNeighborOffsets[1] = 1;



	NEIGHBOR_INDICES rgOppositeTable[8];
	rgOppositeTable[NEIGHBOR_RIGHT] = NEIGHBOR_LEFT;
	rgOppositeTable[NEIGHBOR_RIGHT+1] = NEIGHBOR_LEFT;
	rgOppositeTable[NEIGHBOR_UP] = NEIGHBOR_DOWN;
	rgOppositeTable[NEIGHBOR_UP+1] = NEIGHBOR_DOWN;
	rgOppositeTable[NEIGHBOR_DOWN] = NEIGHBOR_UP;
	rgOppositeTable[NEIGHBOR_DOWN+1] = NEIGHBOR_UP;
	rgOppositeTable[NEIGHBOR_LEFT] = NEIGHBOR_RIGHT;
	rgOppositeTable[NEIGHBOR_LEFT+1] = NEIGHBOR_RIGHT;


	for(UINT nChild = 0; nChild<4; nChild++)
	{
		UINT nNeighbor;

		//set internal neighbor dependencies
		for(nNeighbor = 0; nNeighbor<2; nNeighbor++)
		{
			UINT nIndex = rgNeighborDependencies[nChild].rgnInternalNeighbors[nNeighbor];
			UINT nNeighborIndex = rgNeighborDependencies[nChild].rgnInternalNeighborIndices[nNeighbor];
			pTarget->rgpChildren[nChild]->rgpNeighbors[nIndex] = pTarget->rgpChildren[nNeighborIndex];
			pTarget->rgpChildren[nChild]->rgpNeighbors[nIndex+1] = NULL;
		}

		//set external neighbor dependencies
		for(nNeighbor = 0; nNeighbor<2; nNeighbor++)
		{
			UINT nIndex = rgNeighborDependencies[nChild].rgnExternalNeighors[nNeighbor];

			//check if the neighbor is the same size or bigger
			if(NULL == pTarget->rgpNeighbors[nIndex+1])
			{
				pTarget->rgpChildren[nChild]->rgpNeighbors[nIndex] = pTarget->rgpNeighbors[nIndex];
				/*if(NULL != pTarget->rgpNeighbors[nIndex])
				{
					pTarget->rgpNeighbors[nIndex]->rgpNeighbors[ rgOppositeTable[nIndex] ] = pTarget->rgpChildren[nChild];
					pTarget->rgpNeighbors[nIndex]->rgpNeighbors[ rgOppositeTable[nIndex] ] = pTarget->rgpChildren[nChild];
				}*/
			}
			else
			{
				UINT nParentIndex = nIndex + rgNeighborDependencies[nChild].rgnExternalNeighborOffsets[nNeighbor];

				pTarget->rgpChildren[nChild]->rgpNeighbors[nIndex] = pTarget->rgpNeighbors[nParentIndex];

				/*if(NULL != pTarget->rgpNeighbors[nIndex])
				{
					pTarget->rgpNeighbors[nIndex]->rgpNeighbors[ rgOppositeTable[nIndex] ]->rgpNeighbors[nIndex] =  pTarget->rgpChildren[nChild];
				}*/
			}
		}
	}

	

	UINT rgNeighborChildMap[8];
	rgNeighborChildMap[NEIGHBOR_LEFT] = 0;
	rgNeighborChildMap[NEIGHBOR_LEFT+1] = 2;

	rgNeighborChildMap[NEIGHBOR_UP] = 0;
	rgNeighborChildMap[NEIGHBOR_UP+1] = 1;

	rgNeighborChildMap[NEIGHBOR_RIGHT] = 1;
	rgNeighborChildMap[NEIGHBOR_RIGHT+1] = 3;

	rgNeighborChildMap[NEIGHBOR_DOWN] = 2;
	rgNeighborChildMap[NEIGHBOR_DOWN+1] = 3;


	//make neighbors point to the new children
	for(UINT nNeighbor = 0; nNeighbor<8; )
	{
		if(NULL != pTarget->rgpNeighbors[nNeighbor])
		{
			CD3DFXSquareList::SSquare& Neighbor = *(pTarget->rgpNeighbors[nNeighbor]);
		
			//neighbor is bigger than the new children
			if(GetSquareLevel(Neighbor) <= GetSquareLevel(*pTarget))
			{
				Neighbor.rgpNeighbors[ rgOppositeTable[nNeighbor] ] = pTarget->rgpChildren[ rgNeighborChildMap[nNeighbor] ];
				Neighbor.rgpNeighbors[ rgOppositeTable[nNeighbor]+1 ] = pTarget->rgpChildren[ rgNeighborChildMap[nNeighbor +1] ];

				nNeighbor += 2;
			}
			else
			{
				//neighbor is the same size as the new children
				Neighbor.rgpNeighbors[ rgOppositeTable[nNeighbor] ] = pTarget->rgpChildren[ rgNeighborChildMap[nNeighbor] ];
				nNeighbor++;
			}
		}
		else
		{
			nNeighbor++;
		}
	}
				
			


	return true;
}

bool CD3DFXSquareList::IsLeafNode(const CD3DFXSquareList::SSquare& Square) const
{
	return (NULL == Square.rgpChildren[0]);
}

void CD3DFXSquareList::GetNeighborLevelMinMax(const CD3DFXSquareList::SSquare& Start, 
											UINT* pMinOut, 
											UINT* pMaxOut) const
{
	assert(NULL != pMinOut);
	assert(NULL != pMaxOut);


	UINT nMin = 0xFFFFFFFF;
	UINT nMax = 0;

	//test if this is the root node, it is the only node with 0 neighbors
	if(Start.nSideSize == GetSideSize())
	{
		*pMinOut = GetSquareLevel(Start);
		*pMaxOut = GetSquareLevel(Start);
		return;
	}

	for(UINT nNeighbor = 0; nNeighbor < 8; nNeighbor++)
	{
		if(NULL != Start.rgpNeighbors[nNeighbor])
		{
			UINT nLevel = GetSquareLevel(*(Start.rgpNeighbors[nNeighbor]));
			if(nLevel < nMin)
			{
				nMin = nLevel;
			}

			if(nLevel > nMax)
			{
				nMax = nLevel;
			}
		}
	}

	*pMinOut = nMin;
	*pMaxOut = nMax;

}

bool CD3DFXSquareList::CanTesselateSquare(const CD3DFXSquareList::SSquare& Square) const
{
	if(Square.nSideSize < 5)
	{
		return false;
	}

	UINT nMaxNeighborLevel = 0;
	UINT nMinNeighborLevel = 0;

	UINT nLevel = GetSquareLevel(Square);

	GetNeighborLevelMinMax(Square, &nMinNeighborLevel, &nMaxNeighborLevel);

	assert(3 > abs((int)nMinNeighborLevel - (int)nMaxNeighborLevel));

	if(nMinNeighborLevel < nLevel)
	{
		return false;
	}

	return true;
}

bool CD3DFXSquareList::AreSquaresNeighbors(const CD3DFXSquareList::SSquare& SquareOne,
											const CD3DFXSquareList::SSquare& SquareTwo) const

{
	UINT nLevelOne = GetSquareLevel(SquareOne);
	UINT nLevelTwo = GetSquareLevel(SquareTwo);

	if(nLevelOne == nLevelTwo)
	{
		//one component must be the same, on must be off by nSideSize

		bool bXOffByOne = ((SquareOne.nSideSize-1) == abs((int)SquareOne.nUpperLeftX - (int)SquareTwo.nUpperLeftX));
		bool bYOffByOne = ((SquareOne.nSideSize-1) == abs((int)SquareOne.nUpperLeftY - (int)SquareTwo.nUpperLeftY));

		bool bXSame = (SquareOne.nUpperLeftX == SquareTwo.nUpperLeftX);
		bool bYSame = (SquareOne.nUpperLeftY == SquareTwo.nUpperLeftY);

		if(bXOffByOne)
		{
			return bYSame;
		}
		if(bYOffByOne)
		{
			return bXSame;
		}

		return false;

	}
	else
	{
		const CD3DFXSquareList::SSquare& Bigger = (nLevelOne < nLevelTwo) ? SquareOne : SquareTwo;
		const CD3DFXSquareList::SSquare& Smaller = (nLevelOne > nLevelTwo) ? SquareOne : SquareTwo;

		//simulate tessleating the bigger square and recurse
		CD3DFXSquareList::SSquare Child;



		for(UINT nChild = 0; nChild < 4; nChild++)
		{
			memset((&Child), 0, sizeof(CD3DFXSquareList::SSquare));

			Child.nUpperLeftX = Bigger.nUpperLeftX + 
										((Bigger.nSideSize/2) * (nChild % 2));

			Child.nUpperLeftY = Bigger.nUpperLeftY + 
										((Bigger.nSideSize/2) * (nChild / 2));

			Child.nSideSize = (Bigger.nSideSize/2) + 1;

			Child.fScore = 0.0;

			if(AreSquaresNeighbors(Child, Smaller))
			{
				return true;
			}
		}

		return false;
	}


		

		/*UINT nLittleSideSize = MIN(SquareOne.nSideSize, SquareTwo.nSideSize);
		UINT nBigSideSize = MAX(SquareOne.nSideSize, SquareTwo.nSideSize);

		//one component must be off by the bigger side size while another must be off by 0, or 1
		bool bXOffByTwo = ((nBigSideSize-1) == abs(((int)SquareOne.nUpperLeftX - (int)SquareTwo.nUpperLeftX)));
		bool bYOffByTwo = ((nBigSideSize-1) == abs(((int)SquareOne.nUpperLeftY - (int)SquareTwo.nUpperLeftY)));

		bool bXSame = ((2 * nLittleSideSize) > abs(((int)SquareOne.nUpperLeftX - (int)SquareTwo.nUpperLeftX)));
		bool bYSame = ((2 * nLittleSideSize) > abs(((int)SquareOne.nUpperLeftY - (int)SquareTwo.nUpperLeftY)));

		if(bXOffByTwo)
		{
			return bYSame;
		}
		if(bYOffByTwo)
		{
			return bXSame;
		}*/

		//return false;
	//}
}



	
bool CD3DFXSquareList::IsSquareInside(const CD3DFXSquareList::SSquare& SquareInner,
										const CD3DFXSquareList::SSquare& SquareOuter) const
{
	if(SquareInner.nSideSize >= SquareOuter.nSideSize)
	{
		return false;
	}


	bool bBigEnough = (SquareInner.nUpperLeftX >= SquareOuter.nUpperLeftX) && 
						(SquareInner.nUpperLeftY >= SquareOuter.nUpperLeftY);

	bool bSmallEnough = (SquareInner.nUpperLeftX + SquareInner.nSideSize) <= (SquareOuter.nUpperLeftX + SquareOuter.nSideSize);
	bSmallEnough = bSmallEnough &&((SquareInner.nUpperLeftY + SquareInner.nSideSize) <= (SquareOuter.nUpperLeftY + SquareOuter.nSideSize));

	return bBigEnough && bSmallEnough;
}

double CD3DFXSquareList::ScoreSquare(const CD3DFXSquareList::SSquare& Square) const
{
	assert(NULL != m_pScoreFunction);

	return m_pScoreFunction(Square, *this);
}

UINT CD3DFXSquareList::GetSquareLevel(const CD3DFXSquareList::SSquare& Square) const
{
	assert(NULL != m_pRoot);

	UINT nRootBits = GetHighBitSet(m_pRoot->nSideSize);
	UINT nTestBits = GetHighBitSet(Square.nSideSize);

	return (nRootBits - nTestBits);
}


UINT CD3DFXSquareList::GetHighBitSet(UINT nTest) const
{
	UINT nRet = 0;

	for(UINT nBit = 0; nBit < 32; nBit++)
	{
		if(1 == (nTest & 1))
		{
			nRet = nBit;
		}

		nTest = (nTest>>1);
	}
	
	return nRet;
}


double DefaultScoreFunction(const CD3DFXSquareList::SSquare& Square, 
							const CD3DFXSquareList& SquareList)
{
	const CD3DFXSquareList::SSquare* pRoot = SquareList.GetRootNode();
	
	assert(NULL != pRoot);

	//assume the root upper left is (0,0)
	float fCenterX = (pRoot->nSideSize-1)/2.0f;
	float fCenterY = (pRoot->nSideSize-1)/2.0f;

	float fSquareCenterX = (Square.nUpperLeftX + ((Square.nSideSize-1) / 2.0f));
	float fSquareCenterY = (Square.nUpperLeftY + ((Square.nSideSize-1) / 2.0f));
#warning "Need to implmentnted vector"
	/*D3DXVECTOR2 vRootCenter(fCenterX, fCenterY);
	D3DXVECTOR2 vTargetCenter(fSquareCenterX, fSquareCenterY);

	D3DXVECTOR2 vDifference = vRootCenter - vTargetCenter;

	float fLen = D3DXVec2Length(&vDifference);

	double fRet = 1.0 / (double)fLen;
	
	return fRet; */
	return 0.0;
}

	

void ToTriangleIterator(const CD3DFXSquareList::SSquare* pTarget,
						const CD3DFXSquareList* pList,
						void* pUserData)
{
	assert(NULL != pTarget);
	assert(NULL != pList);
	assert(NULL != pUserData);

	UINT** ppIndices = (UINT**) pUserData;

	UINT* pIndices = *ppIndices;

	
	//if a bit is true, then that side has a lower level(bigger) neighbor
	bool rgbHigherLevels[4];
	memset(rgbHigherLevels, 0, sizeof(rgbHigherLevels));

	const UINT nLevel = pList->GetSquareLevel(*pTarget);

	for(UINT nSide = 0; nSide < 4; nSide++)
	{
		UINT nBaseNeighborIndex = nSide * 2;

		for(UINT nNeighbor = 0; nNeighbor<2; nNeighbor++)
		{
			if(NULL != pTarget->rgpNeighbors[nBaseNeighborIndex + nNeighbor])
			{
				if(nLevel > pList->GetSquareLevel(*(pTarget->rgpNeighbors[nBaseNeighborIndex + nNeighbor])))
				{
					rgbHigherLevels[nSide] = true;
				}
			}
		}
	}



	UINT nGridSideSize = pList->GetRootNode()->nSideSize;

	UINT rgnIndices[3][3];

	for(UINT nRow = 0; nRow < 3; nRow++)
	{
		for(UINT nColumn = 0; nColumn < 3; nColumn++)
		{
			rgnIndices[nColumn][nRow] =  (pTarget->nUpperLeftY + (nColumn * (pTarget->nSideSize/2))) + 
										 ((pTarget->nUpperLeftX + (nRow * (pTarget->nSideSize/2))) * nGridSideSize);
		}
	}

	UINT nIndicesGenerated = 0;

	//generate the indices
	
	#define ADD_INDEX(x, y) {pIndices[nIndicesGenerated] = rgnIndices[x][y]; nIndicesGenerated++;}

	//left side
	if(rgbHigherLevels[CD3DFXSquareList::NEIGHBOR_UP/2])//NEIGHBOR_LEFT
	{
		ADD_INDEX(0, 2);
		ADD_INDEX(1, 1);
		ADD_INDEX(0, 0);
	}
	else
	{
		ADD_INDEX(0, 2);
		ADD_INDEX(1, 1);
		ADD_INDEX(0, 1);

		ADD_INDEX(0, 1);
		ADD_INDEX(1, 1);
		ADD_INDEX(0, 0);
	}

	//top side
	if(rgbHigherLevels[CD3DFXSquareList::NEIGHBOR_LEFT/2])//NEIGHBOR_UP
	{
		ADD_INDEX(0, 0);
		ADD_INDEX(1, 1);
		ADD_INDEX(2, 0);
	}
	else
	{
		ADD_INDEX(0, 0);
		ADD_INDEX(1, 1);
		ADD_INDEX(1, 0);

		ADD_INDEX(1, 0);
		ADD_INDEX(1, 1);
		ADD_INDEX(2, 0);
	}

	//right side
	if(rgbHigherLevels[CD3DFXSquareList::NEIGHBOR_DOWN/2])//NEIGHBOR_RIGHT
	{
		ADD_INDEX(2, 0);
		ADD_INDEX(1, 1);
		ADD_INDEX(2, 2);
	}
	else
	{
		ADD_INDEX(2, 0);
		ADD_INDEX(1, 1);
		ADD_INDEX(2, 1);

		ADD_INDEX(2, 1);
		ADD_INDEX(1, 1);
		ADD_INDEX(2, 2);
	}

	//bottom side
	if(rgbHigherLevels[CD3DFXSquareList::NEIGHBOR_RIGHT/2])//NEIGHBOR_DOWN
	{
		ADD_INDEX(2, 2);
		ADD_INDEX(1, 1);
		ADD_INDEX(0, 2);
	}
	else
	{
		ADD_INDEX(2, 2);
		ADD_INDEX(1, 1);
		ADD_INDEX(1, 2);

		ADD_INDEX(1, 2);
		ADD_INDEX(1, 1);
		ADD_INDEX(0, 2);
	}


	*ppIndices = &(pIndices[nIndicesGenerated]);
}

	






/*void Scorer(CD3DFXSquareList::SSquare* pCurrentSquare, 
			const CD3DFXSquareList* pList, 
			void* pSortedList)
{
	assert(NULL != pCurrentSquare);
	assert(NULL != pList);
	assert(NULL != pSortedList);
	
	SquareList* pSquareList = (SquareList*) pSortedList;

	SquareList NewList;

	SSquareWrapper NewNode;
	NewNode.pData = pCurrentSquare;

	NewList.insert(NewList.begin(), NewNode);

	pSquareList->merge(NewList);
}*/






