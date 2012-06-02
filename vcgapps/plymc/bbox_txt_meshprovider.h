#ifndef BBOXTXTMESHPROVIDER_H
#define BBOXTXTPROVIDER_H
#include "alnParser.h"
#include <list>
#include <vector>
#include <vcg/space/box3.h>
#include <wrap/ply/plystuff.h>
#include <wrap/io_trimesh/import.h>

using namespace std;
using namespace vcg;



template<class TriMeshType>
        class Bbox_Txt_MeshProvider
{
    std::vector< std::string > meshnames;
    std::vector<vcg::Matrix44f> TrV;
    std::vector<float> WV;		    // vettore con i pesi da applicare alla mesh.
    std::vector<vcg::Box3f> BBV;	    // vettore con i bbox trasformati delle mesh da scannare.
    vcg::Box3f fullBBox;
    MeshCache<TriMeshType> MC;
        public:
    bool loadedRealBBox;

    int size() {return meshnames.size();}

    int getCacheSize() {return MC.MaxSize;}
    int setCacheSize(size_t newsize)
    {
        if(newsize == MC.MaxSize)
            return MC.MaxSize;
        if(newsize <= 0)
            return MC.MaxSize;

        MC.MaxSize = newsize;
        return newsize;
    }

    bool openTXT (const char* txtName)
    {
        vector<RangeMap> rmaps;
        std::ifstream f(txtName);
        if(!f.is_open())
            return false;
        std::string fname;
        vcg::Point3f minP,maxP;
        Matrix44f identity; identity.SetIdentity();
        float meshWeight=1.0;

        while(f.good()){
            f>> fname >> minP[0]>>
                minP[1]>>
                minP[2]>>
                maxP[0]>>
                maxP[1]>>
                maxP[2];
            vcg::Box3f bbox(minP,maxP);

            TrV.push_back(identity);
            meshnames.push_back(fname);
            WV.push_back(meshWeight);
            BBV.push_back(bbox);

        }
        loadedRealBBox=true;

        return true;
    }

    bool AddSingleMesh(const char* meshName, Matrix44f &tr, float meshWeight=1)
    {
        assert(WV.size()==meshnames.size() && TrV.size() == WV.size());
        TrV.push_back(tr);
        meshnames.push_back(meshName);
        WV.push_back(meshWeight);
        BBV.push_back(Box3f());
        return true;
    }

    bool AddSingleMesh(const char* meshName)
    {
        Matrix44f identity; identity.SetIdentity();
        return AddSingleMesh(meshName, identity);
    }

    vcg::Box3f bb(int i) {return BBV[i];}
    vcg::Box3f fullBB(){ return fullBBox;}
    vcg::Matrix44f Tr(int i) const  {return TrV[i];}
    std::string MeshName(int i) const {return meshnames[i];}
    float W(int i) const {return WV[i];}

    void Clear()
    {
      meshnames.clear();
      TrV.clear();
      WV.clear();
      BBV.clear();
      fullBBox.SetNull();
      MC.clear();
    }

    bool Find(int i, TriMeshType * &sm)
    {
        return MC.Find(meshnames[i],sm);
    }

    bool InitBBox()
    {
	fullBBox.SetNull();

        for(int i=0;i<int(meshnames.size());++i)
        {
            if(!loadedRealBBox){
                Box3d b;
                bool ret;
                Matrix44f mt;
                Matrix44f Id; Id.SetIdentity();
                mt.Import(TrV[i]);
                printf("bbox scanning %4i/%i [%16s]      \r",i+1,(int)meshnames.size(), meshnames[i].c_str());
                if(tri::io::Importer<TriMeshType>::FileExtension(meshnames[i],"PLY")||tri::io::Importer<TriMeshType>::FileExtension(meshnames[i],"ply"))
                {
                    if(!(TrV[i]==Id))
                        ret=ply::ScanBBox(meshnames[i].c_str(),BBV[i],mt,true,0);
                    else
                        ret=vcg::ply::ScanBBox(meshnames[i].c_str(),BBV[i]);

                }
                else
                {   printf("Trying to import a non-ply file %s\n",meshnames[i].c_str());fflush(stdout);
                    TriMeshType m;
                    int retVal=tri::io::Importer<TriMeshType>::Open(m,meshnames[i].c_str());
                    ret = (retVal==0);
                    tri::UpdateBounding<TriMeshType>::Box(m);
                    BBV[i].Import(m.bbox);
                }
                if( ! ret)
                {
                    printf("\n\nwarning:\n file '%s' not found\n",meshnames[i].c_str());fflush(stdout);
                    continue;
                }

            }
            fullBBox.Add(BBV[i]);
        }
        return true;
    }

        };


#endif // Bbox_Txt_MeshProvider_H
