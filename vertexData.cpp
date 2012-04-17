/*  
    vertexData.cpp
    Copyright (c) 2007, Tobias Wolf <twolf@access.unizh.ch>
    All rights reserved.  
    
    Implementation of the VertexData class.
*/

/** note, derived from Equalizer LGPL source.*/

#include "typedefs.h"
#include "vertexData.h"
#include "ply.h"

#include <cstdlib>
#include <algorithm>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/io_utils>

using namespace std;
using namespace ply;
//int tmpaa=0;

struct Normal{
    osg::Vec3 triNormal;
    void normal(osg::Vec3 v1, osg::Vec3 v2, osg::Vec3 v3)
    {
        osg::Vec3 u,v;

        // right hand system, CCW triangle
        u = v2 - v1;
        v = v3 - v1;
        triNormal = u^v;
        triNormal.normalize();
    }
};

/*  Contructor.  */
VertexData::VertexData()
    : _invertFaces( false )
{
    // Initialize the members
    _vertices = NULL;
    _colors = NULL;
    _normals = NULL;
    _triangles = NULL;
    for(int f=0; f< (int)_texCoord.size(); f++)
        _texCoord[f]=NULL;
    _texIds=NULL;
    _bbox=NULL;
}


/*  Read the vertex and (if available/wanted) color data from the open file.  */
void VertexData::readVertices( PlyFile* file, const int nVertices, 
                               const bool readColors )
{
    // temporary vertex structure for ply loading
    struct _Vertex
    {
        float           x;
        float           y;
        float           z;
        unsigned char   r;
        unsigned char   g;
        unsigned char   b;
    } vertex;

    PlyProperty vertexProps[] = 
    {
        { "x", PLY_FLOAT, PLY_FLOAT, offsetof( _Vertex, x ), 0, 0, 0, 0 },
        { "y", PLY_FLOAT, PLY_FLOAT, offsetof( _Vertex, y ), 0, 0, 0, 0 },
        { "z", PLY_FLOAT, PLY_FLOAT, offsetof( _Vertex, z ), 0, 0, 0, 0 },
        { "red", PLY_UCHAR, PLY_UCHAR, offsetof( _Vertex, r ), 0, 0, 0, 0 },
        { "green", PLY_UCHAR, PLY_UCHAR, offsetof( _Vertex, g ), 0, 0, 0, 0 },
        { "blue", PLY_UCHAR, PLY_UCHAR, offsetof( _Vertex, b ), 0, 0, 0, 0 }
    };
    
    // use all 6 properties when reading colors, only the first 3 otherwise
    int limit = readColors ? 6 : 3;
    for( int i = 0; i < limit; ++i ) 
        ply_get_property( file, "vertex", &vertexProps[i] );
    
    // check whether array is valid otherwise allocate the space
    if(!_vertices.valid())
        _vertices = new osg::Vec3Array; 
    
    // If read colors allocate space for color array
    if( readColors )
    {
        if(!_colors.valid())
            _colors = new osg::Vec4Array;
    }

    // read in the vertices
    for( int i = 0; i < nVertices; ++i )
    {
        ply_get_element( file, static_cast< void* >( &vertex ) );
        osg::Vec3 tmp( vertex.x, vertex.y, vertex.z );
        //cout << tmp << " "<< _bbox->_min<<":"<<_bbox->contains(tmp)<<endl;
        /*if(_bbox){
            //cout << tmp << " "<< _bbox->_min<<endl;
            if(!_bbox->contains(tmp)){
                continue;
            }
           // tmpaa++;
            outbboxVert[i]=_vertices->size();
        }*/
        _tmp_verts.push_back(tmp);
        //_vertices->push_back( tmp );
        //  cout << tmp <<" " << _vertices->size()<<endl;
        if( readColors )
            _tmp_colors.push_back( osg::Vec4( (unsigned int) vertex.r / 256.0, (unsigned int) vertex.g / 256.0 , (unsigned int) vertex.b/ 256.0, 0.0 ) );
    }
}


/*  Read the index data from the open file.  */
void VertexData::readTriangles( PlyFile* file, const int nFaces,bool multTex,bool tex )
{
    // temporary face structure for ply loading
    struct _Face
    {
        unsigned char   nVertices;
        int*            vertices;
        unsigned char   nTex;
        float *texcoord;
        int id;
        unsigned char   nIds;
        float*            ids;
    } face;

    PlyProperty faceProps[] = 
    {
        { "vertex_indices", PLY_INT, PLY_INT, offsetof( _Face, vertices ), 
          1, PLY_UCHAR, PLY_UCHAR, offsetof( _Face, nVertices ) },
        { "texcoord", PLY_FLOAT, PLY_FLOAT, offsetof( _Face, texcoord ),
          1, PLY_UCHAR, PLY_UCHAR, offsetof( _Face, nTex ) },
        { "texnumber", PLY_INT, PLY_INT, offsetof( _Face, id ), 0, 0, 0, 0 },
        { "color", PLY_FLOAT, PLY_FLOAT, offsetof( _Face, ids ),
          1, PLY_UCHAR, PLY_UCHAR, offsetof( _Face, nIds ) }

    };
    
    ply_get_property( file, "face", &faceProps[0] );
    if(tex)
        ply_get_property( file, "face", &faceProps[1] );
    if(multTex){
        ply_get_property( file, "face", &faceProps[2] );
        ply_get_property( file, "face", &faceProps[3] );
    }

    
    // If read colors allocate space for color array
    if( multTex)
    {
        _texCoord.resize(4);
        for(int f=0; f<(int)_texCoord.size(); f++){
            if(!_texCoord[f].valid())
                _texCoord[f] = new osg::Vec3Array;
        }

        if(!_texIds.valid())
            _texIds = new osg::Vec4Array;
    }else if(tex){
        _texCoord.resize(1);
        for(int f=0; f<(int)_texCoord.size(); f++){
            if(!_texCoord[f].valid())
                _texCoord[f] = new osg::Vec3Array;
        }
    }

    //triangles.clear();
    //triangles.reserve( nFaces );
    if(!_triangles.valid())
        _triangles = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);

    
    // read in the faces, asserting that they are only triangles
    int ind1 = _invertFaces ? 2 : 0;
    int ind3 = _invertFaces ? 0 : 2;
    for( int i = 0; i < nFaces; ++i )
    {
        ply_get_element( file, static_cast< void* >( &face ) );
        MESHASSERT( face.vertices != 0 );
        if( (unsigned int)(face.nVertices) != 3 )
        {
            free( face.vertices );
            throw MeshException( "Error reading PLY file. Encountered a "
                                 "face which does not have three vertices." );
        }
        //remap
        bool add=true;
        if(_bbox){
            /* if(outbboxVert.count(face.vertices[ind1]) == 0 || outbboxVert.count(face.vertices[1]) == 0  || outbboxVert.count(face.vertices[ind3]) == 0 ){
                add=false;
            }else{
                face.vertices[ind1]= outbboxVert[face.vertices[ind1]];
                face.vertices[1]= outbboxVert[face.vertices[1]];
                face.vertices[ind3]= outbboxVert[face.vertices[ind3]];
            }*/
            if(!_bbox->contains(_tmp_verts[face.vertices[ind1]]) && !_bbox->contains(_tmp_verts[face.vertices[1]]) && !_bbox->contains(_tmp_verts[face.vertices[ind3]])){
                add=false;

            }
        }
        if(add){
            if(_colors){
                _colors->push_back(_tmp_colors[face.vertices[ind1]]);
            }
            _vertices->push_back(_tmp_verts[face.vertices[ind1]]);
            face.vertices[ind1]=_vertices->size()-1;

            if(_colors){
                _colors->push_back(_tmp_colors[face.vertices[1]]);
            }
            _vertices->push_back(_tmp_verts[face.vertices[1]]);
            face.vertices[1]=_vertices->size()-1;

            if(_colors){
                _colors->push_back(_tmp_colors[face.vertices[ind3]]);
            }
            _vertices->push_back(_tmp_verts[face.vertices[ind3]]);
            face.vertices[ind3]=_vertices->size()-1;

            // Add the face indices in the premitive set
            _triangles->push_back( face.vertices[ind1]);
            _triangles->push_back( face.vertices[1]);
            _triangles->push_back( face.vertices[ind3] );
            if(multTex){
                MESHASSERT( face.ids != 0 );
                if( (unsigned int)(face.nIds) != 12 )
                {
                    free( face.vertices );
                    printf("%d \n",face.nIds);

                    throw MeshException( "Error reading PLY file. Encountered a "
                                         "face which does not have 9 ids." );
                }
            }
            if(tex){

                MESHASSERT( face.texcoord != 0 );
                if( (unsigned int)(face.nTex) < 2  )
                {
                    free( face.texcoord );
                    printf("%d \n",face.nTex);

                    throw MeshException( "Error reading PLY file. Encountered a "
                                         "face which does not have tex coord." );
                }
                for(int k=0,f=0; k <face.nTex; k+=6){
                    _texCoord[f]->push_back(osg::Vec3(face.texcoord[k],face.texcoord[k+1],-1));
                    _texCoord[f]->push_back(osg::Vec3(face.texcoord[k+2],face.texcoord[k+3],-1));
                    _texCoord[f]->push_back(osg::Vec3(face.texcoord[k+4],face.texcoord[k+5],-1));
                    f++;
                }
            }
            if(multTex){
                tri_t t;
                t.idx[0]=face.vertices[ind1];
                t.idx[1]=face.vertices[1];
                t.idx[2]=face.vertices[ind3];
                t.tri_idx=_triangles->size()/3;
                for(int k=0; k<face.nIds; k+=4){
                    _texIds->push_back(osg::Vec4(face.ids[k+0],face.ids[k+1],face.ids[k+2],face.ids[k+3]));
                    //cout << (int)face.nIds << " "<<osg::Vec4(face.ids[k+0],face.ids[k+1],face.ids[k+2],face.ids[k+3]) << endl;
                }
                if(_texIds->at(_texIds->size()-1) != _texIds->at(_texIds->size()-2) || _texIds->at(_texIds->size()-1) != _texIds->at(_texIds->size()-3)
                        || _texIds->at(_texIds->size()-2) != _texIds->at(_texIds->size()-3)){
                    cerr<< _texIds->at(_texIds->size()-1) << " " <<_texIds->at(_texIds->size()-2) << " "<<_texIds->at(_texIds->size()-3)<<endl;
                    fprintf(stderr,"All the texIDs not aligned\n");
                    //exit(-1);
                }
                for(int l=0; l<4; l++){
                    int id=(int)_texIds->back()[l];
                    t.pos=l;
                    if(id >=0)
                        _img2tri[id].push_back(t);
                }
            }
        }

        // free the memory that was allocated by ply_get_element
        free( face.vertices );
        if(tex)
            free( face.texcoord );
        if(multTex)
            free( face.ids );

    }
}


/*  Open a PLY file and read vertex, color and index data. and returns the node  */
osg::Node* VertexData::readPlyFile( const char* filename, const bool ignoreColors,osg::BoundingBox *bbox )
{
    int     nPlyElems;
    char**  elemNames;
    int     fileType;
    float   version;
    bool    result = false;
    int     nComments;
    char**  comments;

    PlyFile* file = NULL;
    _bbox=bbox;
    outbboxVert.clear();
    _tmp_verts.clear();
    _tmp_colors.clear();
    printf("%s\n",filename);
    // Try to open ply file as for reading
    try{
        file  = ply_open_for_reading( const_cast< char* >( filename ),
                                      &nPlyElems, &elemNames,
                                      &fileType, &version );
    }
    // Catch the if any exception thrown
    catch( exception& e )
    {
        MESHERROR << "Unable to read PLY file, an exception occured:  " 
                  << e.what() << endl;
    }

    if( !file )
    {
        MESHERROR << "Unable to open PLY file " << filename 
                  << " for reading." << endl;
        return NULL;
    }

    MESHASSERT( elemNames != 0 );
    

    nComments = file->num_comments;
    comments = file->comments;
    
    
#ifndef NDEBUG
    MESHINFO << filename << ": " << nPlyElems << " elements, file type = " 
             << fileType << ", version = " << version << endl;
#endif

    for( int i = 0; i < nComments; i++ )
    {
        if( equal_strings( comments[i], "modified by flipply" ) )
        {
            _invertFaces = true;
        }

    }
    for( int i = 0; i < nPlyElems; ++i )
    {
        int nElems;
        int nProps;
        
        PlyProperty** props = NULL;
        try{
            props = ply_get_element_description( file, elemNames[i],
                                                 &nElems, &nProps );
        }
        catch( exception& e )
        {
            MESHERROR << "Unable to get PLY file description, an exception occured:  " 
                      << e.what() << endl;
        }
        MESHASSERT( props != 0 );
        
#ifndef NDEBUG
        MESHINFO << "element " << i << ": name = " << elemNames[i] << ", "
                 << nProps << " properties, " << nElems << " elements" << endl;
        for( int j = 0; j < nProps; ++j )
        {
            MESHINFO << "element " << i << ", property " << j << ": "
                     << "name = " << props[j]->name << endl;
        }
#endif
        
        // if the string is vertex means vertex data is started
        if( equal_strings( elemNames[i], "vertex" ) )
        {
            bool hasColors = false;
            // determine if the file stores vertex colors
            for( int j = 0; j < nProps; ++j )
                // if the string have the red means color info is there
                if( equal_strings( props[j]->name, "red" ) )
                    hasColors = true;
            
            if( ignoreColors )
                MESHINFO << "Colors in PLY file ignored per request." << endl;

            try {   
                int startcount=_vertices.valid() ? _vertices->size():0;
                // Read vertices and store in a std::vector array
                readVertices( file, nElems, hasColors && !ignoreColors );
                // Check whether all vertices are loaded or not
                //printf("%d %d\n",startcount,_vertices->size());
                /* if(!bbox){
                    MESHASSERT( _vertices->size()-startcount == static_cast< size_t >( nElems ) );
                }else{
                    MESHASSERT( _vertices->size()-startcount == outbboxVert.size() );
                }*/
                // Check all color elements read or not
                /* if( hasColors && !ignoreColors )
                {
                    if(!bbox){
                    MESHASSERT( _colors->size()-startcount == static_cast< size_t >( nElems ) );
                }else{
                    MESHASSERT( _colors->size()-startcount == outbboxVert.size() );

                }
                }*/

                result = true;
            }
            catch( exception& e )
            {
                MESHERROR << "Unable to read vertex in PLY file, an exception occured:  " 
                          << e.what() << endl;
                // stop for loop by setting the loop variable to break condition
                // this way resources still get released even on error cases
                i = nPlyElems;
                
            }
        }
        // If the string is face means triangle info started
        else if( equal_strings( elemNames[i], "face" ) ){
            bool multTex=false;
            bool tex=false;
            for( int j = 0; j < nProps; ++j ){
                // if the string have the red means color info is there
                if( equal_strings( props[j]->name, "texnumber" ) )
                    multTex = true;
                if( equal_strings( props[j]->name, "texcoord" ) )
                    tex = true;
            }
            try
            {
                int startcount= _triangles.valid()? _triangles->size()/3 : 0 ;
                // Read Triangles
                readTriangles( file, nElems ,multTex,tex);
                // Check whether all face elements read or not
                if(!_bbox){
                    MESHASSERT( (_triangles->size()/3) -startcount  == static_cast< size_t >( nElems ) );
                }
                result = true;
            }
            catch( exception& e )
            {
                MESHERROR << "Unable to read PLY file, an exception occured:  "
                          << e.what() << endl;
                // stop for loop by setting the loop variable to break condition
                // this way resources still get released even on error cases
                i = nPlyElems;
            }
        }
        
        // free the memory that was allocated by ply_get_element_description
        for( int j = 0; j < nProps; ++j )
            free( props[j] );
        free( props );
    }
    
    ply_close( file );
    
    // free the memory that was allocated by ply_open_for_reading
    for( int i = 0; i < nPlyElems; ++i )
        free( elemNames[i] );
    free( elemNames );

    // If the result is true means the ply file is successfully read
    if(result)
    {
        // Create geometry node
        if(!_geom.valid())
            _geom  =  new osg::Geometry;

        // set the vertex array
        _geom->setVertexArray(_vertices.get());

        // If the normals are not calculated calculate the normals for faces
        if(_triangles.valid())
        {
            if(!_normals.valid())
                _calculateNormals();

            // set the normals
            _geom->setNormalArray(_normals.get());
            _geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
        }
        printf("tri %d\n",_triangles->size());
        
        // Add the premetive set
        if(_geom->getPrimitiveSetList().size() ==0){

            if (_triangles.valid() && _triangles->size() > 0 )
                _geom->addPrimitiveSet(_triangles.get());
            //else
            //  _geom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, _vertices->size()));
        }
        // if color info is given set the color array
        if(_colors.valid())
        {
            _geom->setColorArray(_colors.get());
            _geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
            
        }


        // if color info is given set the color array
        /* if(_texCoord.valid())
        {
            geom->setTexCoordArray(0,_texCoord.get());
            //  geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

        }*/
        
        // set flage true to activate the vertex buffer object of drawable
        _geom->setUseVertexBufferObjects(true);
        
        if(!_geode.valid()){
            _geode = new osg::Geode;
        }
        if(_geode->getDrawableList().size() == 0 )
            _geode->addDrawable(_geom);
        // printf("Verts %d\n",tmpaa);
        return _geode;
    }
    
    return NULL;
}


/*  Calculate the face or vertex normals of the current vertex data.  */
void VertexData::_calculateNormals( const bool vertexNormals )
{

    if(_normals.valid())
        return;

#ifndef NDEBUG
    int wrongNormals = 0;
#endif

    if(!_normals.valid())
    {
        _normals = new osg::Vec3Array; 
    }
    
    //normals.clear();
    if( vertexNormals )
    {
        // initialize all normals to zero
        for( size_t i = 0; i < _vertices->size(); ++i )
        {
            _normals->push_back( osg::Vec3( 0, 0, 0 ) );
        }
    }

    
    for( size_t i = 0; i < ((_triangles->size()));  i += 3 )
    {
        // iterate over all triangles and add their normals to adjacent vertices
        Normal  triangleNormal;
        unsigned int i0, i1, i2;
        i0 = (*_triangles)[i+0];
        i1 = (*_triangles)[i+1];
        i2 = (*_triangles)[i+2];
        triangleNormal.normal((*_vertices)[i0],
                              (*_vertices)[i1],
                              (*_vertices)[i2] );
        
        // count emtpy normals in debug mode
#ifndef NDEBUG
        if( triangleNormal.triNormal.length() == 0.0f )
            ++wrongNormals;
#endif

        if( vertexNormals )
        {
            (*_normals)[i0] += triangleNormal.triNormal; 
            (*_normals)[i1] += triangleNormal.triNormal; 
            (*_normals)[i2] += triangleNormal.triNormal;
        }
        else
            _normals->push_back( triangleNormal.triNormal ); 
    }
    
    // normalize all the normals
    if( vertexNormals )
        for( size_t i = 0; i < _normals->size(); ++i )
            (*_normals)[i].normalize();
    
#ifndef NDEBUG
    if( wrongNormals > 0 )
        MESHINFO << wrongNormals << " faces had no valid normal." << endl;
#endif
}

bool checkIsEmptyPly(const char *filename){
    int     nPlyElems;
    char**  elemNames;
    int     fileType;
    float   version;

    int     nComments;
    char**  comments;

    PlyFile* file = NULL;

    // Try to open ply file as for reading
    try{
        file  = ply_open_for_reading( const_cast< char* >( filename ),
                                      &nPlyElems, &elemNames,
                                      &fileType, &version );
    }
    // Catch the if any exception thrown
    catch( exception& e )
    {
        MESHERROR << "Unable to read PLY file, an exception occured:  "
                  << e.what() << endl;
    }

    if( !file )
    {
        MESHERROR << "Unable to open PLY file " << filename
                  << " for reading." << endl;
        return NULL;
    }

    MESHASSERT( elemNames != 0 );


    nComments = file->num_comments;
    comments = file->comments;


#ifndef NDEBUG
    MESHINFO << filename << ": " << nPlyElems << " elements, file type = "
             << fileType << ", version = " << version << endl;
#endif


    for( int i = 0; i < nPlyElems; ++i )
    {
        int nElems;
        int nProps;

        PlyProperty** props = NULL;
        try{
            props = ply_get_element_description( file, elemNames[i],
                                                 &nElems, &nProps );
        }
        catch( exception& e )
        {
            MESHERROR << "Unable to get PLY file description, an exception occured:  "
                      << e.what() << endl;
        }
        MESHASSERT( props != 0 );

#ifndef NDEBUG
        MESHINFO << "element " << i << ": name = " << elemNames[i] << ", "
                 << nProps << " properties, " << nElems << " elements" << endl;
        for( int j = 0; j < nProps; ++j )
        {
            MESHINFO << "element " << i << ", property " << j << ": "
                     << "name = " << props[j]->name << endl;
        }
#endif

        // if the string is vertex means vertex data is started
        if( equal_strings( elemNames[i], "vertex" ) )
        {


            if(nElems > 0)
                return false;
            else
                return true;
        }

        // free the memory that was allocated by ply_get_element_description
        for( int j = 0; j < nProps; ++j )
            free( props[j] );
        free( props );
    }

    ply_close( file );

    // free the memory that was allocated by ply_open_for_reading
    for( int i = 0; i < nPlyElems; ++i )
        free( elemNames[i] );
    free( elemNames );

    return true;
}
