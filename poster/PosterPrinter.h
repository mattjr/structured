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
#include <osgUtil/IntersectionVisitor>
#include <auv_map_projection.hpp>

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
    unsigned int _appliedCount;
    unsigned int _needToApplyCount;

    bool _addingCallbacks;
};

/** PosterIntersector: A simple polytope intersector for updating pagedLODs in each image-tile */
class PosterIntersector : public osgUtil::Intersector
{
public:
    typedef std::set<std::string> PagedNodeNameSet;
    
    PosterIntersector( const osg::Polytope& polytope );
    PosterIntersector( double xMin, double yMin, double xMax, double yMax );
    
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
    osg::ref_ptr<PosterVisitor> _visitor;
    PosterIntersector* _parent;
    osg::Polytope _polytope;
};

/** PosterPrinter: The implementation class of high-res rendering */
class PosterPrinter : public osg::Referenced
{
public:
    typedef std::pair<unsigned int, unsigned int> TilePosition;
    typedef std::map< TilePosition, osg::ref_ptr<osg::Image> > TileImages;
    typedef std::map< TilePosition, osg::Matrix> TileMats;

    PosterPrinter();
    
    /** Set to output each sub-image-tile to disk */
    void setOutputTiles( bool b ) { _outputTiles = b; }
    bool getOutputTiles() const { return _outputTiles; }
    
    void setUseDepth( bool b ) { _useDepth = b; }
    bool getUseDepth() const { return _useDepth; }

    /** Set the output sub-image-tile extension, e.g. bmp */
    void setOutputTileExtension( const std::string& ext ) { _outputTileExt = ext; }
    const std::string& getOutputTileExtension() const { return _outputTileExt; }
    
    /** Set the output poster name, e.g. output.bmp */
    void setOutputPosterName( const std::string& name ) { _outputPosterName = name; }
    const std::string& getOutputPosterName() const { return _outputPosterName; }


    /** Set the output poster name, e.g. output.bmp */
    void setOutputHeightMapName( const std::string& name ) { _outputHeightMapName = name; }
    const std::string& getOutputHeightMapName() const { return _outputHeightMapName; }

    
    /** Set the size of each sub-image-tile, e.g. 640x480 */
    void setTileSize( int w, int h ) { _tileSize.set(w, h); }
    const osg::Vec2& getTileSize() const { return _tileSize; }
    
    /** Set the final size of the high-res poster, e.g. 6400x4800 */
    void setPosterSize( int w, int h ) { _posterSize.set(w, h); }
    const osg::Vec2& getPosterSize() const { return _posterSize; }
    
    /** Set the capturing camera */
    void setCamera( osg::Camera* camera ) { _camera = camera; }
    const osg::Camera* getCamera() const { return _camera.get(); }
    
    /** Set the final poster image, should be already allocated */
    void setFinalPoster( osg::Image* image ) { _finalPoster = image; }
    const osg::Image* getFinalPoster() const { return _finalPoster.get(); }

    /** Set the final poster image, should be already allocated */
    void setFinalHeightMap( osg::Image* image ) { _finalHeightMap = image; }
    const osg::Image* getFinalHeightMap() const { return _finalHeightMap.get(); }

    void applyGeoTags(const char* filename,osg::Matrix viewMatrix,osg::Matrix projMatrix,osg::Camera *camera);

    PosterVisitor* getPosterVisitor() { return _visitor.get(); }
    const PosterVisitor* getPosterVisitor() const { return _visitor.get(); }
    
    bool done() const { return !_isRunning && !_isFinishing; }
    
    void init( const osg::Camera* camera );
    void frame( const osg::FrameStamp* fs, osg::Node* node );
    osg::Vec3 unprojectFromScreen( const osg::Vec3 screen, osg::ref_ptr< osg::Camera > camera );
    void setGeoOrigin(osg::Vec2d ori){_geoOrigin=ori;}
    osg::Vec2d getGeoOrigin(void){return _geoOrigin;}
protected:
    virtual ~PosterPrinter() {}
    bool addCullCallbacks( const osg::FrameStamp* fs, osg::Node* node );
    void removeCullCallbacks( osg::Node* node );
    void bindCameraToImage( osg::Camera* camera, int row, int col );
    void recordImages();
    osg::Matrix getFromScreenMatrix( osg::ref_ptr< osg::Camera > camera );

    bool getClosestZValue(int x,int y,float &z);
    std::string _outputTileExt;
    std::string _outputPosterName;
    std::string _outputHeightMapName;

    osg::Vec2 _tileSize;
    osg::Vec2 _posterSize;
    osg::Vec2d _geoOrigin;
    bool _isRunning;
    bool _isFinishing;
    bool _useDepth;
    bool _outputTiles;

    int _lastBindingFrame;
    int _tileRows;
    int _tileColumns;
    int _currentRow;
    int _currentColumn;
    osg::ref_ptr<PosterIntersector> _intersector;
    osg::ref_ptr<PosterVisitor> _visitor;
    
    osg::Matrixd _currentViewMatrix;
    osg::Matrixd _currentProjectionMatrix;
    osg::ref_ptr<osg::Camera> _camera;
    osg::ref_ptr<osg::Image> _finalPoster;
    osg::ref_ptr<osg::Image> _finalHeightMap;

    TileImages _images;
    TileImages _depthImages;
    TileMats _depthMats;
};

#endif
