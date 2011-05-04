#include <osgDB/ReadFile>
#include <osg/NodeVisitor>
#include <osgDB/WriteFile>

unsigned int countGeometryVertices( osg::Geometry* geom )
{
    if (!geom->getVertexArray())
        return 0;

    // TBD This will eventually iterate over the PrimitiveSets and total the
    //   number of vertices actually used. But for now, it just returns the
    //   size of the vertex array.

    return geom->getVertexArray()->getNumElements();
}

class VertexCounter : public osg::NodeVisitor
{
public:
    VertexCounter( int limit )
        : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
        _limit( limit ),
        _total( 0 ) {}
    ~VertexCounter() {}

    int getTotal() { return _total; }
    bool exceeded() const { return _total > _limit; }
    void reset() { _total = 0; }

    virtual void apply( osg::Node& node )
    {
        // Check for early abort. If out total already exceeds the
        //   max number of vertices, no need to traverse further.
        if (exceeded())
            return;
        traverse( node );
    }

    virtual void apply( osg::Geode& geode )
    {
        // Possible early abort.
        if (exceeded())
            return;

        unsigned int i;
        for( i = 0; i < geode.getNumDrawables(); i++ )
        {
            osg::Geometry* geom = dynamic_cast<osg::Geometry *>(geode.getDrawable(i));
            if( !geom )
                continue;

            _total += countGeometryVertices( geom );

            if (_total > _limit)
                break;
        }
    }

protected:
    int _limit;
    int _total;
};
int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(argv[1]);
    VertexCounter vc( 100 );
    model->accept( vc );
    if(vc.getTotal()>0)
        osgDB::writeNodeFile(*model,argv[2]);

}

