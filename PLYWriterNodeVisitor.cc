// -*-c++-*-

/*
 * Wavefront PLY loader for Open Scene Graph
 *
 * Copyright (C) 2001 Ulrich Hertlein <u.hertlein@web.de>
 *
 * Modified by Robert Osfield to support per Drawable coord, normal and
 * texture coord arrays, bug fixes, and support for texture mapping.
 *
 * Writing support added 2007 by Stephan Huber, http://digitalmind.de,
 * some ideas taken from the dae-plugin
 *
 * The Open Scene Graph (OSG) is a cross platform C++/OpenGL library for
 * real-time rendering of large 3D photo-realistic models.
 * The OSG homepage is http://www.openscenegraph.org/
 */

#include <osg/io_utils>
#include "PLYWriterNodeVisitor.h"



/** writes all values of an array out to a stream, applies a matrix beforehand if necessary */
class ValueVisitor : public osg::ValueVisitor {
public:
    ValueVisitor(std::ostream& fout, const osg::Matrix& m = osg::Matrix::identity(), bool isNormal = false) :
        osg::ValueVisitor(),
        _fout(fout),
        _m(m),
        _isNormal(isNormal)
    {
        _applyMatrix = (_m != osg::Matrix::identity());
        if (_isNormal) _origin = osg::Vec3(0,0,0) * _m;
    }

    virtual void apply (osg::Vec2 & inv)
    {
        _fout << inv[0] << ' ' << inv[1];
    }

    virtual void apply (osg::Vec3 & inv)
    {
        osg::Vec3 v(inv);
        float vf[3];

        if (_applyMatrix)  v = (_isNormal) ? (v * _m) - _origin : v * _m;
        vf[0]=v[0];
        vf[1]=v[1];
        vf[2]=v[2];
        _fout.write((char *)vf,3*sizeof(float));
        // _fout << v[0] << ' ' << v[1] << ' ' << v[2];
    }
    virtual void apply (osg::Vec4 & inv)
    {
        //FOR COLOR
        osg::Vec4 v(inv);
        unsigned char vf[3];

        vf[0]=(unsigned char)(255.0*v[0]);
        vf[1]=(unsigned char)(255.0*v[1]);
        vf[2]=(unsigned char)(255.0*v[2]);
        _fout.write((char *)vf,3*sizeof(unsigned char));
        // _fout << v[0] << ' ' << v[1] << ' ' << v[2];
    }

    virtual void apply (osg::Vec2b & inv)
    {
        _fout << inv[0] << ' ' << inv[1];
    }

    virtual void apply (osg::Vec3b & inv)
    {
        osg::Vec3 v(inv[0], inv[1], inv[2]);
        if (_applyMatrix)  v = (_isNormal) ? (v * _m) - _origin : v * _m;
        _fout << v[0] << ' ' << v[1] << ' ' << v[2];
    }

    virtual void apply (osg::Vec2s & inv)
    {
        _fout << inv[0] << ' ' << inv[1];
    }

    virtual void apply (osg::Vec3s & inv)
    {
        osg::Vec3 v(inv[0], inv[1], inv[2]);
        if (_applyMatrix)  v = (_isNormal) ? (v * _m) - _origin : v * _m;
        _fout << v[0] << ' ' << v[1] << ' ' << v[2];
    }
private:
    std::ostream&    _fout;
    osg::Matrix        _m;
    bool            _applyMatrix, _isNormal;
    osg::Vec3        _origin;
};

/** writes all primitives of a primitive-set out to a stream, decomposes quads to triangles, line-strips to lines etc */
class PrimitiveIndexWriter : public osg::PrimitiveIndexFunctor {
    
public:
    PrimitiveIndexWriter(std::ostream& fout,osg::Geometry* geo, unsigned int normalIndex, unsigned int lastVertexIndex, unsigned int  lastNormalIndex, unsigned int lastTexIndex,bool textured,osg::Vec4Array *textureID,std::vector<osg::ref_ptr<osg::Vec3Array> > *textureCoord,    std::vector<bool> *marginFace,osg::Vec4Array *colorArr) :
        osg::PrimitiveIndexFunctor(),
        _fout(fout),
        _lastVertexIndex(lastVertexIndex),
        _lastNormalIndex(lastNormalIndex),
        _lastTexIndex(lastTexIndex),
        _hasNormalCoords(geo->getNormalArray() != NULL),
        _hasTexCoords(geo->getTexCoordArray(0) != NULL),
        _geo(geo),
        _normalIndex(normalIndex),
        _textured(textured),
        _textureID(textureID),
        _textureCoord(textureCoord),
        _marginFace(marginFace),
        _colorArr(colorArr)

    {
    }

    virtual void setVertexArray(unsigned int,const osg::Vec2*) {}

    virtual void setVertexArray(unsigned int ,const osg::Vec3* ) {}

    virtual void setVertexArray(unsigned int,const osg::Vec4* ) {}

    virtual void setVertexArray(unsigned int,const osg::Vec2d*) {}

    virtual void setVertexArray(unsigned int ,const osg::Vec3d* ) {}

    virtual void setVertexArray(unsigned int,const osg::Vec4d* ) {}

    void write(unsigned int i)
    {
        _fout << (i + _lastVertexIndex) << "/";

        if (_hasTexCoords || _hasNormalCoords)
        {
            if (_hasTexCoords)
                _fout << (i + _lastTexIndex);
            _fout << "/";
            if (_hasNormalCoords)
            {
                if (_geo->getNormalBinding() == osg::Geometry::BIND_PER_VERTEX)
                    _fout << (i+_lastNormalIndex);
                else
                    _fout << (_normalIndex + _lastNormalIndex);
            }
        }
        _fout << " ";
    }

    // operator for triangles
    void writeTriangle(unsigned int i1, unsigned int i2, unsigned int i3)
    {
        int iout[3];
        unsigned char c=3;
        iout[0]=i1;
        iout[1]=i2;
        iout[2]=i3;

        _fout.write((char *)&c,sizeof(char));
        _fout.write((char*)iout,sizeof(int)*3);
        /*            _fout << "3 ";
            _fout << (i1) << " ";
            _fout << (i2) << " ";
            _fout << (i3);

            _fout << std::endl;*/
        if(_textured){
            unsigned char t=2*3*_textureCoord->size();
            float uv[2];
            _fout.write((char *)&t,sizeof(char));
            for(int f=0; f< (int)_textureCoord->size(); f++){
                osg::Vec3Array &tex=*(_textureCoord->at(f));

                uv[0]=tex[i1][0];
                uv[1]=tex[i1][1];
                _fout.write((char*)uv,sizeof(float)*2);

                uv[0]=tex[i2][0];
                uv[1]=tex[i2][1];
                _fout.write((char*)uv,sizeof(float)*2);

                uv[0]=tex[i3][0];
                uv[1]=tex[i3][1];
                _fout.write((char*)uv,sizeof(float)*2);
            }
            //_fout.write((char *)&t,sizeof(char));
            if(_textureCoord->size() > 1){
                int texnumber=(*_textureID)[i1][0];
                _fout.write((char *)&texnumber,sizeof(int));

                unsigned char p=4*3*1;
                _fout.write((char *)&p,sizeof(char));

                for(int f=0; f< 12; f++){
                    float id=(float)(*_textureID)[i1][f];
                    _fout.write((char *)&id,sizeof(float));
                }
            }
        }
        if(_marginFace){
            float qual;
            if(!((*_marginFace)[i1] == (*_marginFace)[i2] && (*_marginFace)[i2] == (*_marginFace)[i3])){
                printf("MArgin not the saem %d %d %d %d %d %d\n",i1,i2,i3,(*_marginFace)[i1] ,(*_marginFace)[i2] ,(*_marginFace)[i3] );
                exit(-1);
            }
            qual =(*_marginFace)[i1] ? -999.0 : 1.0;
            _fout.write((char *)&qual,sizeof(float));

        }

        // not sure if this is correct?
        if(_geo->getNormalBinding() && _geo->getNormalBinding() == osg::Geometry::BIND_PER_PRIMITIVE) ++_normalIndex;
    }

    // operator for lines
    void writeLine(unsigned int i1, unsigned int i2)
    {
        _fout << "l ";
        write(i1);
        write(i2);
        _fout << std::endl;
        // not sure if this is correct?
        if(_geo->getNormalBinding() && _geo->getNormalBinding() == osg::Geometry::BIND_PER_PRIMITIVE) ++_normalIndex;
    }

    // operator for points
    void writePoint(unsigned int i1)
    {
        _fout << "p ";
        write(i1);
        _fout << std::endl;
        // not sure if this is correct?
        if(_geo->getNormalBinding() && _geo->getNormalBinding() == osg::Geometry::BIND_PER_PRIMITIVE) ++_normalIndex;
    }

    virtual void begin(GLenum mode)
    {
        _modeCache = mode;
        _indexCache.clear();
    }

    virtual void vertex(unsigned int vert)
    {
        _indexCache.push_back(vert);
    }

    virtual void end()
    {
        if (!_indexCache.empty())
        {
            drawElements(_modeCache,_indexCache.size(),&_indexCache.front());
        }
    }

    virtual void drawArrays(GLenum mode,GLint first,GLsizei count);

    virtual void drawElements(GLenum mode,GLsizei count,const GLubyte* indices)
    {
        drawElementsImplementation<GLubyte>(mode, count, indices);
    }
    virtual void drawElements(GLenum mode,GLsizei count,const GLushort* indices)
    {
        drawElementsImplementation<GLushort>(mode, count, indices);
    }

    virtual void drawElements(GLenum mode,GLsizei count,const GLuint* indices)
    {
        drawElementsImplementation<GLuint>(mode, count, indices);
    }

protected:

    template<typename T>void drawElementsImplementation(GLenum mode, GLsizei count, const T* indices)
    {
        if (indices==0 || count==0) return;

        typedef const T* IndexPointer;

        switch(mode)
        {
        case(GL_TRIANGLES):
        {
            IndexPointer ilast = &indices[count];
            for(IndexPointer  iptr=indices;iptr<ilast;iptr+=3)
                writeTriangle(*iptr,*(iptr+1),*(iptr+2));

            break;
        }
        case(GL_TRIANGLE_STRIP):
        {
            IndexPointer iptr = indices;
            for(GLsizei i=2;i<count;++i,++iptr)
            {
                if ((i%2)) writeTriangle(*(iptr),*(iptr+2),*(iptr+1));
                else       writeTriangle(*(iptr),*(iptr+1),*(iptr+2));
            }
            break;
        }
        case(GL_QUADS):
        {
            IndexPointer iptr = indices;
            for(GLsizei i=3;i<count;i+=4,iptr+=4)
            {
                writeTriangle(*(iptr),*(iptr+1),*(iptr+2));
                writeTriangle(*(iptr),*(iptr+2),*(iptr+3));
            }
            break;
        }
        case(GL_QUAD_STRIP):
        {
            IndexPointer iptr = indices;
            for(GLsizei i=3;i<count;i+=2,iptr+=2)
            {
                writeTriangle(*(iptr),*(iptr+1),*(iptr+2));
                writeTriangle(*(iptr+1),*(iptr+3),*(iptr+2));
            }
            break;
        }
        case(GL_POLYGON): // treat polygons as GL_TRIANGLE_FAN
        case(GL_TRIANGLE_FAN):
        {
            IndexPointer iptr = indices;
            unsigned int first = *iptr;
            ++iptr;
            for(GLsizei i=2;i<count;++i,++iptr)
            {
                writeTriangle(first,*(iptr),*(iptr+1));
            }
            break;
        }
        case(GL_POINTS):
        {
            IndexPointer ilast = &indices[count];
            for(IndexPointer  iptr=indices;iptr<ilast;++iptr)

            {
                writePoint(*iptr);
            }
            break;
        }

        case(GL_LINES):
        {
            IndexPointer ilast = &indices[count];
            for(IndexPointer  iptr=indices;iptr<ilast;iptr+=2)
            {
                writeLine(*iptr, *(iptr+1));
            }
            break;
        }
        case(GL_LINE_STRIP):
        {

            IndexPointer ilast = &indices[count];
            for(IndexPointer  iptr=indices+1;iptr<ilast;iptr+=2)

            {
                writeLine(*(iptr-1), *iptr);
            }
            break;
        }
        case(GL_LINE_LOOP):
        {
            IndexPointer ilast = &indices[count];
            for(IndexPointer  iptr=indices+1;iptr<ilast;iptr+=2)
            {
                writeLine(*(iptr-1), *iptr);
            }
            writeLine(*ilast, *indices);
            break;
        }

        default:
            // uhm should never come to this point :)
            break;
        }
    }
    
private:
    std::ostream&         _fout;
    GLenum               _modeCache;
    std::vector<GLuint>  _indexCache;
    unsigned int         _lastVertexIndex, _lastNormalIndex, _lastTexIndex;
    bool                 _hasNormalCoords, _hasTexCoords;
    osg::Geometry*         _geo;
    unsigned int         _normalIndex;
    bool _textured;

    osg::Vec4Array *_textureID;
    std::vector<osg::ref_ptr<osg::Vec3Array> > *_textureCoord;

    std::vector<bool> *_marginFace;
    osg::Vec4Array *_colorArr;


};


void PrimitiveIndexWriter::drawArrays(GLenum mode,GLint first,GLsizei count)
{
    switch(mode)
    {
    case(GL_TRIANGLES):
    {
        unsigned int pos=first;
        for(GLsizei i=2;i<count;i+=3,pos+=3)
        {
            writeTriangle(pos,pos+1,pos+2);
        }
        break;
    }
    case(GL_TRIANGLE_STRIP):
    {
        unsigned int pos=first;
        for(GLsizei i=2;i<count;++i,++pos)
        {
            if ((i%2)) writeTriangle(pos,pos+2,pos+1);
            else       writeTriangle(pos,pos+1,pos+2);
        }
        break;
    }
    case(GL_QUADS):
    {
        unsigned int pos=first;
        for(GLsizei i=3;i<count;i+=4,pos+=4)
        {
            writeTriangle(pos,pos+1,pos+2);
            writeTriangle(pos,pos+2,pos+3);
        }
        break;
    }
    case(GL_QUAD_STRIP):
    {
        unsigned int pos=first;
        for(GLsizei i=3;i<count;i+=2,pos+=2)
        {
            writeTriangle(pos,pos+1,pos+2);
            writeTriangle(pos+1,pos+3,pos+2);
        }
        break;
    }
    case(GL_POLYGON): // treat polygons as GL_TRIANGLE_FAN
    case(GL_TRIANGLE_FAN):
    {
        unsigned int pos=first+1;
        for(GLsizei i=2;i<count;++i,++pos)
        {
            writeTriangle(first,pos,pos+1);
        }
        break;
    }
    case(GL_POINTS):
    {

        for(GLsizei i=0;i<count;++i)
        {
            writePoint(i);
        }
        break;
    }

    case(GL_LINES):
    {
        for(GLsizei i=0;i<count;i+=2)
        {
            writeLine(i, i+1);
        }
        break;
    }
    case(GL_LINE_STRIP):
    {
        for(GLsizei i=1;i<count;++i)
        {
            writeLine(i-1, i);
        }
        break;
    }
    case(GL_LINE_LOOP):
    {
        for(GLsizei i=1;i<count;++i)
        {
            writeLine(i-1, i);
        }
        writeLine(count-1, 0);
        break;
    }
    default:
        osg::notify(osg::WARN) << "PLYWriterNodeVisitor :: can't handle mode " << mode << std::endl;
        break;
    }
}


PLYWriterNodeVisitor::PLYMaterial::PLYMaterial(osg::Material* mat, osg::Texture* tex) :
    diffuse(1,1,1,1),
    ambient(0.2,0.2,0.2,1),
    specular(0,0,0,1),
    image("")
{
    static unsigned int s_objmaterial_id = 0;
    ++s_objmaterial_id;
    std::stringstream ss;
    ss << "material_" << s_objmaterial_id;
    name = ss.str();
    
    if (mat) {
        diffuse = mat->getDiffuse(osg::Material::FRONT);
        ambient = mat->getAmbient(osg::Material::FRONT);
        specular = mat->getSpecular(osg::Material::FRONT);
    }
    
    if (tex) {
        osg::Image* img = tex->getImage(0);
        if ((img) && (!img->getFileName().empty()))
            image = img->getFileName();
        
    }

}

std::ostream& operator<<(std::ostream& fout, const PLYWriterNodeVisitor::PLYMaterial& mat) {
    
    fout << "newmtl " << mat.name << std::endl;
    fout << "       " << "Ka " << mat.ambient << std::endl;
    fout << "       " << "Kd " << mat.diffuse << std::endl;
    fout << "       " << "Ks " << mat.specular << std::endl;
    
    if(!mat.image.empty())
        fout << "       " << "map_Kd " << mat.image << std::endl;
    
    return fout;
    
}

void PLYWriterNodeVisitor::writeMaterials(std::ostream& fout) 
{
    for(MaterialMap::iterator i = _materialMap.begin(); i != _materialMap.end(); ++i)
    {
        fout << (*i).second << std::endl;
    }
}


std::string PLYWriterNodeVisitor::getUniqueName(const std::string& defaultvalue) {
    
    std::string name = "";
    for(std::list<std::string>::iterator i = _nameStack.begin(); i != _nameStack.end(); ++i) {
        if (!name.empty()) name+="_";
        name += (*i);
    }
    
    if (!defaultvalue.empty())
        name += "_" +defaultvalue;
    
    if (_nameMap.find(name) == _nameMap.end())
        _nameMap.insert(std::make_pair(name, 0u));

    std::stringstream ss;
    ss << name << "_" << _nameMap[name];
    ++(_nameMap[name]);
    return ss.str();

}

void PLYWriterNodeVisitor::processArray(const std::string& key, osg::Array* array, osg::Array* array2,const osg::Matrix& m, bool isNormal)
{
    if (array == NULL)
        return;
    
    ValueVisitor vv(_fout, m, isNormal);
    //    _fout << std::endl;
    for(unsigned int i = 0; i < array->getNumElements(); ++i) {
        //        _fout << key << ' ';
        array->accept(i, vv);
        array2->accept(i, vv);

        //  _fout << std::endl;
    }
    
    //    _fout << "# " << array->getNumElements() << " elements written" << std::endl;
    
}
void PLYWriterNodeVisitor::processArray(const std::string& key, osg::Array* array,const osg::Matrix& m, bool isNormal)
{
    if (array == NULL)
        return;

    ValueVisitor vv(_fout, m, isNormal);
    //    _fout << std::endl;
    for(unsigned int i = 0; i < array->getNumElements(); ++i) {
        //        _fout << key << ' ';
        array->accept(i, vv);

        //  _fout << std::endl;
    }

    //    _fout << "# " << array->getNumElements() << " elements written" << std::endl;

}
void PLYWriterNodeVisitor::processStateSet(osg::StateSet* ss) 
{
    if (_materialMap.find(ss) != _materialMap.end()) {
        _fout << "usemtl " << _materialMap[ss].name << std::endl;
        return;
    }
    
    osg::Material* mat = dynamic_cast<osg::Material*>(ss->getAttribute(osg::StateAttribute::MATERIAL));
    osg::Texture* tex = dynamic_cast<osg::Texture*>(ss->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
    
    if (mat || tex)
    {
        _materialMap.insert(std::make_pair(osg::ref_ptr<osg::StateSet>(ss), PLYMaterial(mat, tex)));
        _fout << "usemtl " << _materialMap[ss].name << std::endl;
    }
    
}


void PLYWriterNodeVisitor::processGeometry(osg::Geometry* geo, osg::Matrix& m) {
    //_fout << std::endl;
    //_fout << "o " << getUniqueName( geo->getName().empty() ? geo->className() : geo->getName() ) << std::endl;
    
    //    processStateSet(_currentStateSet.get());
    if(_colorArr != NULL)
        processArray("v", geo->getVertexArray(), _colorArr,m, false);
    else
        processArray("v", geo->getVertexArray(),m, false);

    //processArray("vn", geo->getNormalArray(), m, true);
    //  processArray("vt", geo->getTexCoordArray(0)); // we support only tex-unit 0
    unsigned int normalIndex = 0;
    for(unsigned int i = 0; i < geo->getNumPrimitiveSets(); ++i)
    {
        osg::PrimitiveSet* ps = geo->getPrimitiveSet(i);
        
        PrimitiveIndexWriter pif(_fout, geo, normalIndex, _lastVertexIndex, _lastNormalIndex, _lastTexIndex,_textured,_textureID,_textureCoord,_marginFace,_colorArr);
        ps->accept(pif);
        
        if(geo->getNormalArray() && geo->getNormalBinding() == osg::Geometry::BIND_PER_PRIMITIVE_SET)
            ++normalIndex;
    }
    if (geo->getVertexArray())
        _lastVertexIndex += geo->getVertexArray()->getNumElements();
    if (geo->getNormalArray())
        _lastNormalIndex += geo->getNormalArray()->getNumElements();
    if(geo->getTexCoordArray(0))
        _lastTexIndex += geo->getTexCoordArray(0)->getNumElements();
    
}

void PLYWriterNodeVisitor::count_geo(osg::Geometry* geo) {
    for(unsigned int i = 0; i < geo->getNumPrimitiveSets(); ++i)
    {
        osg::PrimitiveSet* ps = geo->getPrimitiveSet(i);
        _total_face_count+= ps->getNumPrimitives ();
    }
    if (geo->getVertexArray())
        _total_vertex_count += geo->getVertexArray()->getNumElements();
    //    if (geo->getVertexArray())
    //  _total_vertex_count += geo->getVertexArray()->getNumElements();



    
}

void PLYWriterNodeVisitor::apply( osg::Geode &node )
{

    //    pushStateSet(node.getStateSet());
    // _nameStack.push_back(node.getName());
    osg::Matrix m = osg::computeLocalToWorld(getNodePath());
    unsigned int count = node.getNumDrawables();
    for ( unsigned int i = 0; i < count; i++ )
    {
        osg::Geometry *g = node.getDrawable( i )->asGeometry();
        if ( g != NULL )
        {
            //            pushStateSet(g->getStateSet());
            
            count_geo(g);
            
            // popStateSet(g->getStateSet());
        }
    }
    write_header();
    //  fprintf(stderr,"Faces %d Vert %d\n",_total_face_count,_total_vertex_count);
    for ( unsigned int i = 0; i < count; i++ )
    {
        osg::Geometry *g = node.getDrawable( i )->asGeometry();
        if ( g != NULL )
        {
            //            pushStateSet(g->getStateSet());
            
            processGeometry(g,m);
            
            // popStateSet(g->getStateSet());
        }
    }


    //    popStateSet(node.getStateSet());
    // _nameStack.pop_back();
}


void PLYWriterNodeVisitor::write_header(void){
    _fout <<"ply\n";
    _fout <<"format binary_little_endian 1.0\n";
    //_fout <<"comment PLY exporter written by Paul Adams\n";
    _fout <<"element vertex "<<_total_vertex_count <<std::endl;
    _fout <<"property float x\n";
    _fout <<"property float y\n";
    _fout <<"property float z\n";
    if(_colorArr != NULL){
        _fout <<"property uchar red\n";
        _fout <<"property uchar green\n";
        _fout <<"property uchar blue\n";
    }
    _fout <<"element face " <<_total_face_count<<std::endl;
    _fout <<"property list uchar int vertex_indices\n";
    if(_textured){
        _fout <<"property list uchar float texcoord\n";
        if(_mult_tex){
            _fout <<"property int texnumber\n";
            _fout <<"property list uchar float color\n";
        }
    }
    if(_marginFace){
        _fout <<"property float quality\n";

    }
    if(_comment.size())
        _fout <<"comment "<<_comment<<"\n";

    _fout <<"end_header\n";
}
