/*

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish and distribute.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
  AUTHOR BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
  AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  Copyright 1999 -   Laurent Balmelli
                     Laboratory For Audio-Visual Communications 
		     Ecole Polytechnique Federale de Lausanne, Switzerland

		     also:
		     Mathematics for Communication, Bell Laboratories
		     Lucent Technologies, USA

  contact, bugs:     Laurent.Balmelli@epfl.ch
                     balmelli@acm.org
		     
  PLEASE DO NOT REMOVE THIS COPYRIGHT NOTICE. 
		     
 */

#include "SpatialQuadtree.h"
#include "QTElement.h"
#include <math.h>

void SpatialQuadtree::setNode(int i, QTElement qtelement)
    {
        if(i > 0)
        {
	  //	  if(!Element out of bound){
                QTElement qtelement1 = (QTElement)Nodes[i];
                if(qtelement1.isValid())
                {
		  cout <<"SpatialQuadtree Error: node already exists at index " << i << ".\n";
                    return;
                }
                int j = (int)floor((float)(i - 1) / 4.0);
                QTElement qtelement2 = (QTElement)Nodes[j];
                if(!qtelement2.isValid() )
                {
		  printf("SpatialQuadtree Error: nodes has no father at index %d\n",j);
                    return;
                } else
                {
		  //Nodes.ensureCapacity(i);
		  Nodes[i]=qtelement;///.setElementAt(qtelement, i);
                    qtelement2.NumSons++;
                    int k = i % 4;
                    qtelement2.Sons[k] = true;
                    return;
                }
	
	//}
       } else
        {
	  Nodes[0]=qtelement;
            return;
        }
    }
void SpatialQuadtree::init(void)
{
  DistancesH = new int *[20];
  DistancesV = new int *[20];
  for(int i = 0; i < 20; i++)
    {
            DistancesH[i] = NULL;
            DistancesV[i] = NULL;
        

    }
}

   vector<int> SpatialQuadtree::base4(int i)
    {
        int j = (int)floor(log((float)i) / log(4.0)) + 1;
        if(i == 0)
            j = 1;
        vector<int> ai;
        if(i > 0)
        {
	  ai.resize(j);
            int k = (int)floor(log((float)i) / log(4.0));
            ai[k] = (int)floor((double)(float)i / pow(4.0, (float)k));
            int i1 = i - (int)((double)(float)ai[k] * pow(4.0, (float)k));
            for(int l = k; i1 > 0; l = k)
            {
                k = (int)floor(log((float)i1) / log(4.0));
                int j1 = l - k;
                if(j1 > 1)
                {
                    for(int k1 = 1; k1 <= j1 - 1; k1++)
                        ai[l - k1] = 0;

                }
                ai[k] = (int)floor((double)(float)i1 / pow(4.0, (float)k));
                i1 -= ai[k] * (int)pow(4.0, k);
            }

        } else
        {
	  ai.resize(1);
            ai[0] = 0;
        }
        return ai;
    }

vector<int> SpatialQuadtree::lbase4(int i)
    {
        int j = local(i);
        vector<int> ai = base4(j);
        int k = nodeLevel(i);
        int j1 = k - ai.size();
        vector<int> ai1(ai.size() + j1);
        for(unsigned int l = 0; l < ai.size(); l++)
            ai1[l] = ai[l];

        if((int)ai.size() < k)
        {
            for(unsigned int i1 = ai.size(); i1 < ai.size() + j1; i1++)
                ai1[i1] = 0;

        }
        return ai1;
    }

     vector<int>  SpatialQuadtree::slbase4(int i)
    {
      vector<int> ai= lbase4(i);
        ai = fliplr(ai);
        for(unsigned int j = 0; j < ai.size(); j++)
            if(j % 2 == 0)
                ai[j] = switchbit(ai[j]);

        return fliplr(ai);
    }

vector<int> SpatialQuadtree::locate(int i)
    {
      vector<int> ai(2);
        if(i > 0)
        {
	  vector<int> ai1 = slbase4(i);
            for(unsigned int j = 0; j < ai1.size(); j++)
            {
                if(ai1[j] == 1)
                    ai[0] += (int)pow(2.0, j);
                if(ai1[j] == 2)
                    ai[1] += (int)pow(2.0, j);
                if(ai1[j] == 3)
                {
                    ai[0] += (int)pow(2.0, j);
                    ai[1] += (int)pow(2.0, j);
                }
            }

        } else
        {
            ai[0] = 0;
            ai[1] = 0;
        }
        return ai;
    }

   int SpatialQuadtree::dh(int i)
    {
        double d = 6.0 * (pow(4.0, i) / 5.0) + pow(-1.0, i + 1) / 5.0;
        return (int)d;
    }

    int SpatialQuadtree::dht(int i)
    {
        double d = pow(4.0, i + 1) / 5.0 + pow(-1.0, i) / 5.0;
        return (int)d;
    }

    int SpatialQuadtree::dv(int i)
    {
        double d = 4.0 * (pow(4.0, i) / 3.0) + 0.66666666666666663;
        return (int)d;
    }

    int SpatialQuadtree::dvt(int i)
    {
        double d = 2.0 * (pow(4.0, i + 1) / 3.0) - 0.66666666666666663;
        return (int)d;
    }

    int SpatialQuadtree::switchbit(int i)
    {
        int j = (i + 1) % 2;
        if(i > 1)
            j += 2;
        return j;
    }

vector<int> SpatialQuadtree::fliplr(vector<int>ai)
    {
      vector<int> ai1(ai.size());
        int j = ai.size() - 1;
        for(unsigned int i = 0; i < ai.size(); i++)
            ai1[i] = ai[j - i];

        return ai1;
    }

    void SpatialQuadtree::computeDistanceVector(int i)
    {
        int j = i;
        vector<int> ai(3);
        vector<int> ai1(1);
        vector<int> ai2(3);
        if(i > 0)
        {
            if(DistancesH[i] == NULL)
            {
                DistancesH[i] = new int[(int)(pow(2.0, i) + 1.0)];
                j--;
                ai[0] = dht(0);
                ai[1] = -dh(0);
                ai[2] = dht(0);
                ai1[0] = -dh(0);
                for(int k3 = 1; k3 <= j; k3++)
                {
		  ai = vector<int>((int)(pow(2.0, k3 + 1) + 1.0));
                    if(k3 % 2 == 1)
                    {
		      ai2 = vector<int>((int)(pow(2.0, k3 + 1) - 1.0));
                        for(unsigned int k = 0; k < ai1.size(); k++)
                            ai2[k] = -ai1[k];

                        ai2[ai1.size()] = -dh(k3);
                        for(unsigned int l = ai1.size() + 1; l < 2 * ai1.size() + 1; l++)
                            ai2[l] = -ai1[l - (ai1.size() + 1)];

                        ai[0] = dht(k3);
                        for(unsigned int i1 = 1; i1 <= ai2.size(); i1++)
                            ai[i1] = ai2[i1 - 1];

                        ai[ai2.size() + 1] = dht(k3);
                    } else
                    {
		      ai1 = vector< int>((int)(pow(2.0, k3 + 1) - 1.0));
                        for(unsigned int j1 = 0; j1 < ai2.size(); j1++)
                            ai1[j1] = -ai2[j1];

                        ai1[ai2.size()] = -dh(k3);
                        for(unsigned int k1 = ai2.size() + 1; k1 < 2 * ai2.size() + 1; k1++)
                            ai1[k1] = -ai2[k1 - (ai2.size() + 1)];

                        ai[0] = dht(k3);
                        for(unsigned int l1 = 1; l1 <= ai1.size(); l1++)
                            ai[l1] = ai1[l1 - 1];

                        ai[ai1.size() + 1] = dht(k3);
                    }
                }
		for(unsigned int n=0; n < ai.size(); n++)
		  DistancesH[i][n]=ai[n];
            }
            ai = vector< int>(3);
            ai1 = vector<int>(1);
            ai2 = vector<int>(3);
            j = i;
            if(DistancesV[i] == NULL)
            {
                DistancesV[i] = new int[(int)(pow(2.0, i) + 1.0)];
                j--;
                ai[0] = -dvt(0);
                ai[1] = dv(0);
                ai[2] = -dvt(0);
                ai1[0] = dv(0);
                for(int l3 = 1; l3 <= j; l3++)
                {
                    ai = vector<int> ((int)(pow(2.0, l3 + 1) + 1.0));
                    if(l3 % 2 == 1)
                    {
                        ai2 = vector<int> ((int)(pow(2.0, l3 + 1) - 1.0));
                        for(unsigned int i2 = 0; i2 < ai1.size(); i2++)
                            ai2[i2] = ai1[i2];

                        ai2[ai1.size()] = dv(l3);
                        for(unsigned int j2 = ai1.size() + 1; j2 < 2 * ai1.size() + 1; j2++)
                            ai2[j2] = ai1[j2 - (ai1.size() + 1)];

                        ai[0] = -dvt(l3);
                        for(unsigned int k2 = 1; k2 <= ai2.size(); k2++)
                            ai[k2] = ai2[k2 - 1];

                        ai[ai2.size() + 1] = -dvt(l3);
                    } else
                    {
                        ai1 = vector<int> ((int)(pow(2.0, l3 + 1) - 1.0));
                        for(unsigned int l2 = 0; l2 < ai2.size(); l2++)
                            ai1[l2] = ai2[l2];

                        ai1[ai2.size()] = dv(l3);
                        for(unsigned int i3 = ai2.size() + 1; i3 < 2 * ai2.size() + 1; i3++)
                            ai1[i3] = ai2[i3 - (ai2.size() + 1)];

                        ai[0] = -dvt(l3);
                        for(unsigned int j3 = 1; j3 <= ai1.size(); j3++)
                            ai[j3] = ai1[j3 - 1];

                        ai[ai1.size() + 1] = -dvt(l3);
                    }
                }
		for(unsigned int n=0; n < ai.size(); n++)
		  DistancesV[i][n]=ai[n];

            }
        }
    }

int SpatialQuadtree::getNeighbor(int i, int j)
    {
      vector<int> ai = locate(i);
        int k = nodeLevel(i);
        computeDistanceVector(k);
        switch(j)
        {
        case 0: // '\0'
            return i - DistancesV[k][ai[1]];

        case 1: // '\001'
            return i + DistancesV[k][ai[1] + 1];

        case 2: // '\002'
            return i + DistancesH[k][ai[0] + 1];

        case 3: // '\003'
            return i - DistancesH[k][ai[0]];

        case 4: // '\004'
            return i - DistancesH[k][ai[0]] - DistancesV[k][ai[1]];

        case 5: // '\005'
            return (i + DistancesH[k][ai[0] + 1]) - DistancesV[k][ai[1]];

        case 6: // '\006'
            return i + DistancesH[k][ai[0] + 1] + DistancesV[k][ai[1] + 1];

        case 7: // '\007'
            return (i - DistancesH[k][ai[0]]) + DistancesV[k][ai[1] + 1];

        case 8: // '\b'
            computeDistanceVector(k + 1);
            if(k % 2 == 0)
                return (4 * i + 2) - DistancesH[k + 1][2 * ai[0]] - DistancesV[k + 1][2 * ai[1]];
            else
                return (4 * i + 1) - DistancesH[k + 1][2 * ai[0]] - DistancesV[k + 1][2 * ai[1]];

        case 9: // '\t'
            computeDistanceVector(k + 1);
            if(k % 2 == 0)
                return (4 * i + 1 + DistancesH[k + 1][2 * ai[0] + 2]) - DistancesV[k + 1][2 * ai[1]];
            else
                return (4 * i + 2 + DistancesH[k + 1][2 * ai[0] + 2]) - DistancesV[k + 1][2 * ai[1]];

        case 10: // '\n'
            computeDistanceVector(k + 1);
            if(k % 2 == 0)
                return 4 * i + 3 + DistancesH[k + 1][2 * ai[0] + 2] + DistancesV[k + 1][2 * ai[1] + 2];
            else
                return 4 * i + 4 + DistancesH[k + 1][2 * ai[0] + 2] + DistancesV[k + 1][2 * ai[1] + 2];

        case 11: // '\013'
            computeDistanceVector(k + 1);
            if(k % 2 == 0)
                return ((4 * i + 4) - DistancesH[k + 1][2 * ai[0]]) + DistancesV[k + 1][2 * ai[1] + 2];
            else
                return ((4 * i + 3) - DistancesH[k + 1][2 * ai[0]]) + DistancesV[k + 1][2 * ai[1] + 2];

        case 12: // '\f'
            computeDistanceVector(k + 1);
            if(k % 2 == 0)
                return (4 * i + 4) - DistancesH[k + 1][2 * ai[0]];
            else
                return (4 * i + 3) - DistancesH[k + 1][2 * ai[0]];

        case 13: // '\r'
            computeDistanceVector(k + 1);
            if(k % 2 == 0)
                return (4 * i + 2) - DistancesH[k + 1][2 * ai[0]];
            else
                return (4 * i + 1) - DistancesH[k + 1][2 * ai[0]];

        case 14: // '\016'
            computeDistanceVector(k + 1);
            if(k % 2 == 0)
                return 4 * i + 3 + DistancesH[k + 1][2 * ai[0] + 2];
            else
                return 4 * i + 4 + DistancesH[k + 1][2 * ai[0] + 2];

        case 15: // '\017'
            computeDistanceVector(k + 1);
            if(k % 2 == 0)
                return 4 * i + 1 + DistancesH[k + 1][2 * ai[0] + 2];
            else
                return 4 * i + 2 + DistancesH[k + 1][2 * ai[0] + 2];
        }
        return 0;
    }
