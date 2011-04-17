#include "Extents.h"
#include "TexturingQuery.h"
#include <osg/Matrix>
#include <osg/Geometry>
#include <osg/PrimitiveSet>
#include <map>
#include <osg/Texture2DArray>
#include <osgDB/FileUtils>
#include <osg/io_utils>
#include <osg/TriangleIndexFunctor>
#include "calcTexCoord.h"
using namespace SpatialIndex;
using namespace std;
bool writeCached(const std::string &outfilename,const std::string  sha2hash,osg::Vec4Array *ids,osg::Vec2Array *texCoords){
    FILE *fp=fopen(outfilename.c_str(),"w");
    if(!fp){
        fprintf(stderr, "Can't write file %s\n",outfilename.c_str());
        return false;
    }
    if(texCoords->size() < 1){
        fprintf(stderr, "Didn't store any tex coords\n");
        return false;
    }
    fprintf(fp,"%s\n",sha2hash.c_str());
    fprintf(fp,"%d\n",(int)ids->size());
    for(int i=0; i< (int)ids->size(); i++){
        for(int j=0; j <4; j++){
            int a=(int)ids->at(i)[j];
            fwrite((char*)&a,1,sizeof(int),fp);
        }
        for(int j=0; j <2; j++){
            float b=texCoords->at(i)[j];
            fwrite((char*)&b,1,sizeof(float),fp);
        }
        // printf("%f %f\n",texCoords[0]->at(i)[0],texCoords[0]->at(i)[1]);

    }
    return true;
}

bool loadCached(const std::string &file,osg::Vec4Array *ids,osg::Vec2Array *texCoords){
    FILE *fp=fopen(file.c_str(),"r");
    if(!fp){
        fprintf(stderr, "Can't read file %s\n",file.c_str());
        return -1;
    }
    char meshHash[1024];
    int numPts;
    int a;
    float b;
    fscanf(fp,"%s\n",meshHash);
    fscanf(fp,"%d\n",&numPts);
    printf("Num pts %d\n",numPts);
    for(int i=0; i< numPts; i++){
        osg::Vec4 id;
        osg::Vec2 tex;
        for(int j=0; j <4; j++){
            fread((char*)&a,1,sizeof(int),fp);
            id[j]=a;
        }
        for(int j=0; j <2; j++){
            fread((char*)&b,1,sizeof(float),fp);
            tex[j]=b;
        }
        ids->push_back(id);
        texCoords->push_back(tex);
        // printf("%f %f %f %f\n",id[0],id[1],id[2],id[3]);

        //printf("%f %f\n",tex[0],tex[1]);

    }
    return true;
}
std::string getHash(std::string mf){
    sha2 mySha2;
    mySha2.Init(sha2::enuSHA256);
    unsigned char	buf[BUFLEN];

    FILE *fp=fopen(mf.c_str(),"rb");
    if(!fp){
        std::cerr << "MEsh file " << mf<<" doesn't exist\n";
        return string();
    }
    int l;
    while ((l = fread(buf,1,BUFLEN,fp)) > 0) {

        mySha2.Update(buf,l);
    }
    fclose(fp);
    mySha2.End();
    return mySha2.StringHash();
}

int checkCached(std::string mf,std::string cachedloc,std::string &sha2hash){
    int npos=mf.find("/");
    std::string bbox_name=std::string(mf.substr(0,npos)+"/bbox-"+mf.substr(npos+1,mf.size()-9-npos-1)+".ply.txt");
    if(!osgDB::fileExists(bbox_name)){
        std::cerr << "Bbox file " << bbox_name << mf<<" doesn't exist\n";
        return -1;
    }
    char buffer1[2048];

    sha2hash=getHash(mf);
    if(osgDB::fileExists(cachedloc)){
        //Check hash
        std::ifstream fin(cachedloc.c_str());
        fin.getline(buffer1,2048);

        if(std::string(buffer1) == sha2hash)
        {
            std::cout << "Valid existing hash of texcoord file skipping "<< osgDB::getSimpleFileName(cachedloc)<<"\n";
            return 1;
        }else{
            std::cout << "Differing hashes "<< buffer1<< " != "<<sha2hash<<std::endl;
            return 0;
        }

    }else
        return 0;
}

TexturingQuery::TexturingQuery(TexturedSource *source,const Camera_Calib &calib,TexPyrAtlas &atlasGen,bool useTextureArray) :
        _calib(calib),
        _useTextureArray(useTextureArray),
        _atlasGen(atlasGen),
        _useAtlas(true),
        _source(source)
{
    _vertexAlias = AttributeAlias(0, "osg_Vertex");




}
TexturingQuery::~TexturingQuery(){

}
const CamDists TexturingQuery::getClosest(std::vector<int> tri_v,const osg::Vec3Array &verts){
    CamDists orderedProj;
    if(tri_v.size() != 3){
        fprintf(stderr,"Error not 3 verts\n");
        return orderedProj;
    }
    osg::Vec3 avgV;
    std::multimap<SpatialIndex::id_type,int> cam_counter;
    for(unsigned int i=0; i<tri_v.size(); i++){
        avgV += verts[tri_v[i]];
        pair<multimap<int, SpatialIndex::id_type>::iterator, multimap<int, SpatialIndex::id_type>::iterator> ppp;
        ppp=reproj.equal_range(tri_v[i]);
        for (multimap<int, SpatialIndex::id_type>::iterator it2 = ppp.first;
             it2 != ppp.second;
             ++it2){
            //   printf("here %d %d\n",(*it2).second.id,i);
            cam_counter.insert(make_pair<long,int>((*it2).second,i));
        }
    }
    avgV.x()/=3.0;
    avgV.y()/=3.0;
    avgV.z()/=3.0;

    // printf("Size of original %d %d\n",cam_counter.size(),tri_v.size());
    set<SpatialIndex::id_type> seeninalltri;
    for (multimap<SpatialIndex::id_type, int>::iterator it = cam_counter.begin();
    it != cam_counter.end();
    ++it){
        if(cam_counter.count((*it).first) == 3){
            seeninalltri.insert((*it).first);
        }
    }

    for(set<SpatialIndex::id_type>::iterator it=seeninalltri.begin(); it != seeninalltri.end(); ++it){
        CamProjAndDist tmp;
        findCamProjAndDist(tmp,avgV,(*it));
        orderedProj.push_back(tmp);
    }
    std::sort(orderedProj.begin(), orderedProj.end(), sort_pred());
    /* printf("New Tri %d\n",orderedProj.size());
    for(int i=0; i < orderedProj.size(); i++)
        printf("ID: %d Dist: %f\n",orderedProj[i].first,orderedProj[i].second);
   // printf("Size of reproject %d\n",seeninalltri.size());
*/
    return orderedProj;
}
osg::Vec2 TexturingQuery::convertToUV(const osg::Vec2 &pix){
    osg::Vec2 ratioPow2;
    osg::Vec2 pow2size(osg::Image::computeNearestPowerOfTwo(_calib.width),osg::Image::computeNearestPowerOfTwo(_calib.height));
    ratioPow2.x()=(pow2size.x()/_calib.width);
    ratioPow2.y()=(pow2size.y()/_calib.height);
    ratioPow2.x()*=pix.x();
    ratioPow2.y()*=pix.y();
    osg::Vec2 uv;
    uv.x()=ratioPow2.x()/pow2size.x();
    uv.y()=1.0-(ratioPow2.y()/pow2size.y());
    return uv;
}

void TexturingQuery::findCamProjAndDist(CamProjAndDist &cpad,osg::Vec3 v,SpatialIndex::id_type id){
    cpad.id=id;
    osg::Vec2 texC=reprojectPt(_source->_cameras[id].m,v);
    if(texC.x() < 0.0 || texC.x() > _calib.width || texC.y() < 0.0 || texC.y() > _calib.height )
        cpad.dist=DBL_MAX;
    else
        cpad.dist=(texC-osg::Vec2(_calib.width/2,_calib.height/2)).length2();
    cpad.uv=convertToUV(texC);

}
double TexturingQuery::getDistToCenter(osg::Vec3 v, TexturedSource::ProjectionCamera cam){
    osg::Vec2 texC=reprojectPt(cam.m,v);
    if(texC.x() < 0.0 || texC.x() > _calib.width || texC.y() < 0.0 || texC.y() > _calib.height )
        return DBL_MAX;
    else
        return (texC-osg::Vec2(_calib.width/2.0,_calib.height/2.0)).length2();
}

bool checkInBounds(osg::Vec3 tc){
    if(tc[0] >= 1.0 || tc[0] <= 0.0 || tc[1] >= 1.0 || tc[1] <=0.0){
      //  cout << tc<<endl;
        return false;
    }
    return true;
}
bool TexturingQuery::projectAllTriangles(osg::Vec4Array* camIdxArr,TexBlendCoord &texCoordsArray,
                                         const osg::PrimitiveSet& prset, const osg::Vec3Array &verts){
    int numIdx=prset.getNumIndices();
    osg::Timer_t before_computeMax = osg::Timer::instance()->tick();



    if(numIdx < 3 || !camIdxArr)
        return false;

    //Reproject all verticies
    for(unsigned int i=0; i<verts.size(); i++){
        double pt[3];
        pt[0]=verts[i].x();
        pt[1]=verts[i].y();
        pt[2]=verts[i].z();
        Point p = Point(pt, 3);
        ObjVisitor vis;
        // cout << p <<endl;
        _source->intersectsWithQuery(p,vis);
        if(vis.GetResultCount()){
            for(unsigned int r=0; r < vis.GetResults().size(); r++){
                if(_source->_cameras.count(vis.GetResults()[r]))
                    reproj.insert(std::pair<int,SpatialIndex::id_type>(i,vis.GetResults()[r]));
            }
        }
    }
    camIdxArr->resize(verts.size(),osg::Vec4(-1,-1,-1,-1));
    for(int f=0; f < (int)texCoordsArray.size() && f < maxNumTC; f++){
        if(texCoordsArray[f])
            texCoordsArray[f]->resize(verts.size());
    }
    //multimap<unsigned int, int> vert_reproj;

    for(int i=0; i<numIdx-2; i+=3){

        vector<int> tri_v;
        for(int k=0; k <3; k++){
            tri_v.push_back(prset.index(i+k));
        }
        const CamDists d=getClosest(tri_v,verts);
        osg::Vec4 camIdx(-1,-1,-1,-1);
        for(int v=0; v<(int)d.size() && v <4; v++){
            if(d[v].id >= 0)
                camIdx[v]=d[v].id;
        }
        for(int k=0; k <3; k++){
            /*
            if(d[0].id <0){
                printf("REmap\n");
                camIdxArr->at(prset.index(i+k))=osg::Vec4(-1,-1,-1,-1);
                if(texCoordsArray && d.size()) {
                    texCoordsArray->at(prset.index(i+k))=osg::Vec2(-1,-1);
                }

            }else*/{
                //   vert_reproj.insert(make_pair<unsigned int, int >(prset.index(i+k),1));
                camIdxArr->at(prset.index(i+k))=camIdx;
                for(int f=0; f <(int)d.size() && f<maxNumTC; f++){
                    if(texCoordsArray[f]) {
                        osg::Vec2 tc=convertToUV(reprojectPt(_source->_cameras[d[f].id].m,verts[tri_v[k]]));
                        texCoordsArray[f]->at(prset.index(i+k))=osg::Vec3(tc[0],tc[1],-1);
                    }
                }
            }
        }
        //REally innecfficent needs fixing
        for(int k=0; k <3; k++){
            for(int f=0; f <(int)d.size() && f<maxNumTC; f++){
                if(!checkInBounds(texCoordsArray[f]->at(prset.index(i+k))))
                    for(int k2=0; k2<3; k2++)
                        texCoordsArray[f]->at(prset.index(i+k2))=osg::Vec3(-1,-1,-1);

            }
        }
    }

    map<SpatialIndex::id_type,int> allIds=calcAllIds(camIdxArr);
    addImagesToAtlasGen(allIds);
    //cout << "Number of prim" << prset.getNumIndices() << " " << prset.getNumPrimitives() << endl;

    osg::Timer_t after_computeMax = osg::Timer::instance()->tick();

    OSG_INFO << "Time for projectAllTriangles = " << osg::Timer::instance()->delta_s(before_computeMax, after_computeMax) <<endl;

    return true;
}
bool loadShaderSource(osg::Shader* obj, const std::string& fileName )
{
    std::string fqFileName = osgDB::findDataFile(fileName);
    if( fqFileName.length() == 0 )
    {
        std::cerr << "File \"" << fileName << "\" not found." << std::endl;
        assert(0);
        exit(-1);
        return false;
    }
    bool success = obj->loadShaderSourceFromFile( fqFileName.c_str());
    if ( !success  )
    {
        std::cerr << "Couldn't load file: " << fileName << std::endl;
        assert(0);
        exit(-1);
        return false;
    }
    else
    {
        return true;
    }
}
void TexturingQuery::addImagesToAtlasGen(map<SpatialIndex::id_type,int> allIds){
    //cout << "Adding " << allIds.size() <<  endl;/
    //printf("0x%x\n",(int)this);
    std::vector<std::pair<SpatialIndex::id_type,string> > files(allIds.size());
    map<SpatialIndex::id_type,int>::const_iterator end = allIds.end();
    for (map<SpatialIndex::id_type,int>::const_iterator it = allIds.begin(); it != end; ++it)
    {   if(_source->_cameras.count(it->first) && _source->_cameras[it->first].filename.size())
        files[it->second]=make_pair<SpatialIndex::id_type,string> (it->first,_source->_cameras[it->first].filename);
        else
            OSG_ALWAYS << " Failed " <<it->first << " : " << it->second << '\n';
    }

    _atlasGen.addSources(files);
    //_atlasGen.setAllID(allIds);
}


osg::Vec2 TexturingQuery::reprojectPt(const osg::Matrixf &mat,const osg::Vec3 &v){

    osg::Vec3 cam_frame=mat*v;
    osg::Vec3 und_n;
    und_n.x()=cam_frame.x()/cam_frame.z();
    und_n.y()=cam_frame.y()/cam_frame.z();
    double r_2 = pow(und_n.x(),2) + pow(und_n.y(),2);
    double k_radial = 1 + _calib.kc1*r_2 + _calib.kc2*pow(r_2,2) + _calib.kc5*pow(r_2,3);
    double delta_x = 2*_calib.kc3*und_n.x()*und_n.y() + _calib.kc4*(r_2 + 2*pow(und_n.x(),2));
    double delta_y = _calib.kc3*(r_2 + 2*pow(und_n.y(),2)) + 2*_calib.kc4*und_n.x()*und_n.y();
    osg::Matrix m1=osg::Matrix::scale(_calib.fcx,_calib.fcy,1);
    osg::Matrix m2=osg::Matrix::translate(_calib.ccx,_calib.ccy,0);
    osg::Matrix k1=osg::Matrix::scale(k_radial,k_radial,1);
    osg::Matrix delta=osg::Matrix::translate(delta_x,delta_y,0);
    osg::Vec3 norm_c=k1*und_n*delta;
    osg::Vec3 pixel_c=(m1*norm_c*m2);
    return osg::Vec2(pixel_c.x(),pixel_c.y());
}
map<SpatialIndex::id_type,int> calcAllIds(osg::Vec4Array *v){

    map<SpatialIndex::id_type,int> allIds;
    unsigned int uniqueIdCount=0;
    for(int i=0; i< (int)v->size(); i++){
        for(int j=0; j< 4; j++){
            int id=(int)((*v)[i][j]);
            if(id < 0)
                continue;
            if(allIds.count(id) == 0){
                allIds[id]=uniqueIdCount++;
            }
        }
    }
    return allIds;
}

void calcAllIdsBack(osg::Vec4Array *v,map<SpatialIndex::id_type,int> &allIds,map<int,SpatialIndex::id_type>  &backMap){

    unsigned int uniqueIdCount=0;
    for(int i=0; i< (int)v->size(); i++){
        for(int j=0; j< 4; j++){
            int id=(int)((*v)[i][j]);
            if(id < 0)
                continue;
            if(allIds.count(id) == 0){
                allIds[id]=uniqueIdCount;
                backMap[uniqueIdCount]=id;
                uniqueIdCount++;
            }
        }
    }
}




class TriangleIndexVisitor
{
public:
    TriangleIndexVisitor(){newVertCounter=0;}
    set<int> indices_counted;
    map<int,int> indices_double_counted;
    vector<int> new_list;
    int newVertCounter;

    void operator()(const int v1, const int v2, const int v3)
    {
        new_list.push_back(v1);
        new_list.push_back(v2);
        new_list.push_back(v3);

        // toss the computed indices into the indices array
        if(indices_counted.count(v1) == 0)
            indices_counted.insert( v1 );
        else{
            indices_double_counted[v1]=newVertCounter++;
        }
        if(indices_counted.count(v2) == 0)
            indices_counted.insert( v2 );
        else
            indices_double_counted[v2]=newVertCounter++;
        if(indices_counted.count(v3) == 0)
            indices_counted.insert( v3 );
        else
            indices_double_counted[v3]=newVertCounter++;

    }
};
/*void TexturingQuery::remapSharedVert(osg::PrimitiveSet& prset, osg::Vec3Array &verts,std::map<int,int> remap){
    int origVertSize=verts.size();
    osg::DrawElementsUInt* elements
            = new osg::DrawElementsUInt(GL_TRIANGLES, taf._in_indices.begin(),
                               taf._in_indices.end());
    new_primitives.push_back(elements);
    for(int i=0; i<prset.getNumIndices()-2; i+=3){
        for(int k=0; k< 3; k++){
            if(remap.count(prset.index(i+k)))
                prset.index(i+k)=(origVertSize+remap[prset.index(i+k)]);
                verts->push_back(verts.at(prset.index(i+k)));
        }
    }
}*/
void addDups(osg::Geode *geode){
    osg::Drawable *drawable = geode->getDrawable(0);
    osg::TriangleIndexFunctor<TriangleIndexVisitor> tif;
    drawable->accept(tif);


    //  printf("Size %d",(int)tif.indices_double_counted.size());
    osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
    //geom->setUseDisplayList(false);
    osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());
    int origSize=tif.new_list.size();
    //  verts->resize(origSize+tif.indices_double_counted.size());
    for(int i=0; i<origSize; i++){
        if(tif.indices_double_counted.count(tif.new_list[i])){
            verts->push_back(verts->at(tif.new_list[i]));
            tif.new_list[i]=verts->size()-1;
        }
    }
    osg::DrawElementsUInt* elements
            = new osg::DrawElementsUInt(GL_TRIANGLES, tif.new_list.begin(),
                                        tif.new_list.end());
    geom->setPrimitiveSet(0,elements);
    printf("Dups to be added %d\n",(int)tif.indices_double_counted.size());
}

bool TexturingQuery::checkAndLoadCache(osg::Vec4Array *ids,osg::Vec2Array *texCoords){
    assert(ids != NULL && texCoords != NULL);
    string hash;
    string cachedfile=_source->tex_cache_dir+osgDB::getNameLessExtension(osgDB::getSimpleFileName(_source->getFileName()))+".txt";
    if(checkCached(_source->getFileName(),cachedfile,hash) == 1){



        loadCached(cachedfile,ids,texCoords);
        map<SpatialIndex::id_type,int> allIds=calcAllIds(ids);
        addImagesToAtlasGen(allIds);
        // OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_tile->_texCoordMutex);

        //  _tile->texCoordIDIndexPerModel.push_back(ids);
        //_tile->texCoordsPerModel.push_back(texCoords);
        return true;
    }
    ids=NULL;
    texCoords=NULL;
    return false;
}
bool TexturingQuery::projectModel(osg::Geode *geode){

    //No cached
    if(!geode){
        OSG_ALWAYS << "Not valid geode\n";
        return false;
    }
    for (unsigned int i=0; i<geode->getNumDrawables(); i++){
        reproj.clear();

        osg::Drawable *drawable = geode->getDrawable(i);

        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        //geom->setUseDisplayList(false);
        osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());

        //setVertexAttrib(*geom, _vertexAlias, geom->getVertexArray(), false, osg::Geometry::BIND_PER_VERTEX);
        // geom->setVertexArray(0);
        if(!verts || !verts->size()){
            OSG_INFO<< "Empty mesh continuing!" <<endl;
            continue;
        }
        //OSG_INFO
        std::cout<< "\tModel Size: "<< verts->size()<<endl;
        /*   if(checkCached && checkAndLoadCache())
            return true;
*/

        osg::Geometry::PrimitiveSetList& primitiveSets = geom->getPrimitiveSetList();
        osg::Geometry::PrimitiveSetList::iterator itr;
        osg::ref_ptr<osg::Vec4Array> v= new osg::Vec4Array;

        TexBlendCoord texCoords(4);
        texCoords[0]=new osg::Vec3Array;
        texCoords[1]=new osg::Vec3Array;
        texCoords[2]=new osg::Vec3Array;
        texCoords[3]=new osg::Vec3Array;

        osg::ref_ptr<osg::StateSet> stateset;
        bool projectValid=false;
        for(itr=primitiveSets.begin(); itr!=primitiveSets.end(); ++itr){
            int mode=(*itr)->getMode();
            switch(mode){
            case(osg::PrimitiveSet::TRIANGLES):
                //remapSharedVert(*(*itr), *verts,tif.indices_double_counted);
                projectValid=projectAllTriangles(v,texCoords,*(*itr), *verts);
                if(!v||!projectValid){
                    OSG_ALWAYS << "Failed to create reprojection array" <<endl;
                    continue;
                }

                {
                    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_tile->_texCoordMutex);

                    _tile->texCoordIDIndexPerModel[geode]=v;
                    //printf("v %d %d\n",v->size(), *itr->getNumIndices());

                    _tile->texCoordsPerModel[geode]=texCoords;
                }
                return true;
                break;
                default:
                OSG_ALWAYS << "Freakout shouldn't be anything but triangles\n";
            }
        }
    }
    return false;
}
