/* -*-c++-*- OpenSceneGraph example, osgposter.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#ifndef OSGPOSTER_POSTERPRINTER
#define OSGPOSTER_POSTERPRINTER

#include <osg/Camera>
#include <osg/PagedLOD>
#include <osgDB/FileUtils>
#include <osgUtil/IntersectionVisitor>
#include <string.h>
#include <osg/io_utils>
#include <opencv/cv.h>
#include <opencv/highgui.h>
/** PosterVisitor: A visitor for adding culling callbacks to newly allocated paged nodes */
class PosterVisitor : public osg::NodeVisitor
{
public:
    typedef std::set<std::string> PagedNodeNameSet;
    
    PosterVisitor();
    META_NodeVisitor( osgPoster, PosterVisitor );
    
    void insertName( const std::string& name )
    { if ( _pagedNodeNames.insert(name).second ) _needToApplyCount++; }
    
    void eraseName( const std::string& name )
    { if ( _pagedNodeNames.erase(name)>0 ) _needToApplyCount--; }
    
    void clearNames() { _pagedNodeNames.clear(); _needToApplyCount = 0; _appliedCount = 0; }
    unsigned int getNumNames() const { return _pagedNodeNames.size(); }
    
    PagedNodeNameSet& getPagedNodeNames() { return _pagedNodeNames; }
    const PagedNodeNameSet& getPagedNodeNames() const { return _pagedNodeNames; }
    
    unsigned int getNeedToApplyCount() const { return _needToApplyCount; }
    unsigned int getAppliedCount() const { return _appliedCount; }
    unsigned int inQueue() const
    { return _needToApplyCount>_appliedCount ? _needToApplyCount-_appliedCount : 0; }
    
    void setAddingCallbacks( bool b ) { _addingCallbacks = b; }
    bool getAddingCallbacks() const { return _addingCallbacks; }
    
    virtual void apply( osg::LOD& node );
    virtual void apply( osg::PagedLOD& node );
protected:
    bool hasCullCallback( osg::NodeCallback* nc, osg::NodeCallback* target )
    {
        if ( nc==target ) return true;
        else if ( !nc ) return false;
        return hasCullCallback( nc->getNestedCallback(), target );
    }
    
    void insertCullCallback( osg::Node& node, osg::NodeCallback* nc )
    {
        if ( node.getCullCallback() )
            nc->setNestedCallback( node.getCullCallback() );
        node.setCullCallback(nc);
    }
    
    PagedNodeNameSet _pagedNodeNames;
    unsigned int _needToApplyCount;
    unsigned int _appliedCount;
    bool _addingCallbacks;
};

/** PosterIntersector: A simple polytope intersector for updating pagedLODs in each image-tile */
class PosterIntersector : public osgUtil::Intersector
{
public:
    typedef std::set<std::string> PagedNodeNameSet;
    
    PosterIntersector( const osg::Polytope& polytope,osg::Camera * camera,osg::Matrix &modelMatrix);
    PosterIntersector( double xMin, double yMin, double xMax, double yMax ,osg::Camera *camera,osg::Matrix &modelMatrix);
    
    void setPosterVisitor( PosterVisitor* pcv ) { _visitor = pcv; }
    PosterVisitor* getPosterVisitor() { return _visitor.get(); }
    const PosterVisitor* getPosterVisitor() const { return _visitor.get(); }
    
    virtual Intersector* clone( osgUtil::IntersectionVisitor& iv );
    
    virtual bool containsIntersections()
    { return _visitor.valid()&&_visitor->getNumNames()>0; }
    
    virtual bool enter( const osg::Node& node )
    { return !node.isCullingActive() || _polytope.contains(node.getBound()); }
    
    virtual void leave() {}
    virtual void reset();
    virtual void intersect( osgUtil::IntersectionVisitor& iv, osg::Drawable* drawable );
protected:
    osg::Matrix &_modelMatrix;
    osg::ref_ptr<PosterVisitor> _visitor;
    PosterIntersector* _parent;
    osg::Polytope _polytope;
    osg::Camera *_camera;
};

/** PosterPrinter: The implementation class of high-res rendering */
class PosterPrinter : public osg::Referenced
{
public:
    typedef std::pair<unsigned int, unsigned int> TilePosition;
    typedef std::map< TilePosition, osg::ref_ptr<osg::Image> > TileImages;
    
    int getColumns(){return _tileColumns;}
    int getRows(){return _tileRows;}
    PosterPrinter();
    /** Set to output each sub-image-tile to disk */
    void setOutputTiles( bool b ) { _outputTiles = b; }
    bool getOutputTiles() const { return _outputTiles; }
    void setBaseName(std::string baseName){_baseName=baseName;}
    void writeMats(void);
    void setOverlap(int overlap){_tileOverlap=overlap;}
    /** Set the output sub-image-tile extension, e.g. bmp */
    void setOutputTileExtension( const std::string& ext ) { _outputTileExt = ext; }
    const std::string& getOutputTileExtension() const { return _outputTileExt; }
    
    /** Set the output poster name, e.g. output.bmp */
    void setOutputPosterName( const std::string& name ) { _outputPosterName = name; }
    const std::string& getOutputPosterName() const { return _outputPosterName; }
    
    /** Set the size of each sub-image-tile, e.g. 640x480 */
    void setTileSize( int w, int h ) { _tileSize.set(w, h); }
    const osg::Vec2& getTileSize() const { return _tileSize; }
    
    /** Set the final size of the high-res poster, e.g. 6400x4800 */
    void setPosterSize( int w, int h ) {_posterSize.set(w, h); }
    const osg::Vec2& getPosterSize() const { return _posterSize; }
    
    /** Set the capturing camera */
    void setCamera( osg::Camera* camera ) { _camera = camera; }
    const osg::Camera* getCamera() const { return _camera.get(); }
    
    /** Set the final poster image, should be already allocated */
    void setFinalPoster( IplImage* image ) { _finalPoster = image; }
    //const osg::Image* getFinalPoster() const { return _finalPoster.get(); }
    
    PosterVisitor* getPosterVisitor() { return _visitor.get(); }
    const PosterVisitor* getPosterVisitor() const { return _visitor.get(); }
    
    bool done() const { return !_isRunning && !_isFinishing; }
    void setOutputEmpty(bool b){_outputEmpty=b;}
    void init( const osg::Camera* camera ,std::vector<TilePosition> &valid,osg::BoundingBox bbox,std::pair<unsigned int,unsigned int> empty);
    void frame( const osg::FrameStamp* fs, osg::Node* node );
    void setTileOutputDir(const std::string &dir){_dir=dir; osgDB::makeDirectory(_dir);}
    void copyNeigborPixels(IplImage *img_overlap,int level, int col, int row,CvRect &src,CvRect &dst);

protected:
    virtual ~PosterPrinter() {}
    void doDeepZoom(void);
    void RecordXML(void);
    bool addCullCallbacks( const osg::FrameStamp* fs, osg::Node* node );
    void removeCullCallbacks( osg::Node* node );
    void bindCameraToImage( osg::Camera* camera, int row, int col,int idx );
    void recordImages();
    void addOverlap(int level,int row, int col,double width,double height);
    bool _outputTiles;
    bool _outputEmpty;
    int _maxLevel;
    std::pair<unsigned int,unsigned int> _emptyPair;
    std::string empty_name;
    bool _haveSavedFirstEmpty;
    std::string _outputTileExt;
    std::string _outputPosterName;
    osg::Vec2 _tileSize;
    osg::Vec2 _posterSize;
    std::vector<osg::Matrix> _validMats;
    int _tileOverlap;
    bool _isRunning;
    std::vector<osg::BoundingBox>_validBbox;
    bool _isFinishing;
    int _lastBindingFrame;
    int _tileRows;
    int _tileColumns;
    int _currentRow;
    int _currentColumn;
    int _validCurrent;
    osg::BoundingBox _bbox;
    std::string _baseName;
    std::string _tmpbase;
    osg::ref_ptr<PosterIntersector> _intersector;
    osg::ref_ptr<PosterVisitor> _visitor;
    std::string _dir;
    osg::Matrixd _currentViewMatrix;
    osg::Matrixd _currentProjectionMatrix;
    osg::ref_ptr<osg::Camera> _camera;
    IplImage *_finalPoster;
    osg::ref_ptr<osg::Image> _emptyImage;
    std::vector<TilePosition>   _validTiles;
    std::string _tmpTileExt;
osg::Matrix _model;
    TileImages _images;
};

#endif
