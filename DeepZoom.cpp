#include <string>
#include <osgDB/FileUtils>

#include <osgDB/FileNameUtils>
#include <vips/vips>
#include <sstream>
#include <assert.h>
using namespace std;
using namespace vips;
class DeepZoom {
public:
    std::string xmlHeader;
    std::string schemaName;

    enum CmdParseState { DEFAULT, OUTPUTDIR, TILESIZE, OVERLAP, INPUTFILE };
    bool deleteExisting;
    std::string tileFormatExt;

    // The following can be overriden/set by the indicated command line arguments
    int tileSize;            // -tilesize
    int tileOverlap;           // -overlap
    std::string tileFormat;
    std::string outputDir;         // -outputdir or -o
    bool verboseMode;   // -verbose or -v
    bool debugMode;     // -debug
    std::vector<std::string> inputFile;  // must follow all other args
    DeepZoom(){
        tileSize=256;
        tileOverlap=1;
        deleteExisting=false;
        outputDir="dz";
        verboseMode=false;
        tileFormatExt="jpg";
        debugMode=false;
        xmlHeader= "<?xml version=\"1.0\" encoding=\"utf-8\"?>";
        schemaName = "http://schemas.microsoft.com/deepzoom/2009";
    }
    VImage getTile(vips::VImage img, int row, int col);
    void processImageFile(std::string inFile, std::string outputDir);
    void saveImageDescriptor(int width, int height, std::string file) ;
    void saveText(vector<string> lines, string file);
};

/**
     * @param args the command line arguments
     */
int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    DeepZoom dz;
    arguments.read("-tilesize",dz.tileSize);
    arguments.read("-overlap",dz.tileOverlap);
    dz.verboseMode=arguments.read("-v");
    dz.debugMode=arguments.read("-debug");
    arguments.read("-output",dz.outputDir);
    if (dz.debugMode) {
        printf("tileSize=%d ", dz.tileSize);
        printf("tileOverlap=%d ", dz.tileOverlap);
        printf("outputDir=%s\n", dz.outputDir.c_str());
    }


    if(!osgDB::fileExists(dz.outputDir))
        osgDB::makeDirectory(dz.outputDir);

    dz.processImageFile(argv[1], dz.outputDir);
}
/**
     * Process the given image file, producing its Deep Zoom output files
     * in a subdirectory of the given output directory.
     * @param inFile the file containing the image
     * @param outputDir the output directory
     */
void DeepZoom::processImageFile(string inFile, string outputDir) {
    if (verboseMode)
        printf("Processing image file: %s\n", inFile.c_str());

    string nameWithoutExtension = osgDB::getNameLessExtension(inFile);
    string pathWithoutExtension = outputDir + "/" + nameWithoutExtension;

    vips::VImage image(inFile.c_str(),"rb");

    int originalWidth = image.Xsize();
    int originalHeight = image.Ysize();

    double maxDim = max(originalWidth, originalHeight);

    int nLevels = (int)ceil(log(maxDim) / log(2));

    if (debugMode)
        printf("nLevels=%d\n", nLevels);

    // Delete any existing output files and folders for this image

    std::string descriptor((pathWithoutExtension + ".xml"));


    std::string imgDir = pathWithoutExtension;
    if (osgDB::fileExists(imgDir))  {
        cerr << ("Image directory already exists in output dir: " + imgDir);
    }

    if(!osgDB::makeDirectory(imgDir)){
        cerr<< "Failed to create " << imgDir <<endl;
        exit(-1);
    }
    double width = originalWidth;
    double height = originalHeight;

    for (int level = nLevels; level >= 0; level--) {
        int nCols = (int)ceil(width / tileSize);
        int nRows = (int)ceil(height / tileSize);
        if (debugMode)
            printf("level=%d w/h=%f/%f cols/rows=%d/%d\n",
                   level, width, height, nCols, nRows);

        std::ostringstream dir;
        dir<< imgDir << "/" << level;
        if(!osgDB::makeDirectory(dir.str())){
            cerr<< "Failed to create " << dir.str() <<endl;
            exit(-1);
        }

        for (int col = 0; col < nCols; col++) {
            for (int row = 0; row < nRows; row++) {
                VImage tile = getTile(image, row, col);
                //saveImage(tile, dir + File.separator + col + '_' + row);
                std::ostringstream imgName;
                imgName << dir << "/" << col << "_" << row;
                tile.write(imgName.str().c_str());
            }
        }

        // Scale down image for next level
        width = ceil(width / 2);
        height = ceil(height / 2);
        /*  if (width > 10 && height > 10) {
            // resize in stages to improve quality
            image = resizeImage(image, width * 1.66, height * 1.66);
            image = resizeImage(image, width * 1.33, height * 1.33);
        }*/
        double ratioWidth=width/originalWidth;
        double ratioHeight=height/originalHeight;

        image = image.affine(ratioWidth,0,0,ratioHeight,0,0,0,0,width,height);
    }

    saveImageDescriptor(originalWidth, originalHeight, descriptor);
}




/**
     * Gets an image containing the tile at the given row and column
     * for the given image.
     * @param img - the input image from whihc the tile is taken
     * @param row - the tile's row (i.e. y) index
     * @param col - the tile's column (i.e. x) index
     */
VImage DeepZoom::getTile(VImage img, int row, int col) {
    int x = col * tileSize - (col == 0 ? 0 : tileOverlap);
    int y = row * tileSize - (row == 0 ? 0 : tileOverlap);
    int w = tileSize + (col == 0 ? 1 : 2) * tileOverlap;
    int h = tileSize + (row == 0 ? 1 : 2) * tileOverlap;

    if (x + w > img.Xsize())
        w = img.Xsize() - x;
    if (y + h > img.Ysize())
        h = img.Ysize() - y;

    if (debugMode)
        printf("getTile: row=%d, col=%d, x=%d, y=%d, w=%d, h=%d\n",
               row, col, x, y, w, h);

    assert(w > 0);
    assert(h > 0);



    return img.extract_area(x,y,w,h);
}




/**
     * Write image descriptor XML file
     * @param width image width
     * @param height image height
     * @param file the file to which it is saved
     */
void DeepZoom::saveImageDescriptor(int width, int height, std::string file) {
    vector<string> lines;
    lines.push_back(xmlHeader);
    ostringstream s;
    s<<"<Image TileSize=\"" << tileSize << "\" Overlap=\"" << tileOverlap << "\" Format=\"" << tileFormat << "\" ServerFormat=\"Default\" xmnls=\"" << schemaName << "\">";
    lines.push_back(s.str());
    ostringstream s2;
    s2<<"<Size Width=\"" << width << "\" Height=\"" << height << "\" />";
    lines.push_back(s2.str());
    lines.push_back("</Image>");
    saveText(lines, file);
}

/**
     * Saves strings as text to the given file
     * @param lines the image to be saved
     * @param file the file to which it is saved
     */
void DeepZoom::saveText(vector<string> lines, string file) {
    ofstream of(file.c_str());
    if(of.good()){
        for (int i = 0; i < (int)lines.size(); i++)
            of <<lines[i];
    }else{
        cerr<<"Can't open " << file.c_str() <<endl;
        exit(-1);
    }
}


