#include <string>
#include <osgDB/FileUtils>
using namespace std;

class DeepZoom {

    static string xmlHeader = "<?xml version=\"1.0\" encoding=\"utf-8\"?>";
    static string schemaName = "http://schemas.microsoft.com/deepzoom/2009";

    enum CmdParseState { DEFAULT, OUTPUTDIR, TILESIZE, OVERLAP, INPUTFILE };
    bool deleteExisting;
    string tileFormatExt;

    // The following can be overriden/set by the indicated command line arguments
    int tileSize;            // -tilesize
    int tileOverlap;           // -overlap
    string outputDir;         // -outputdir or -o
    bool verboseMode;   // -verbose or -v
    bool debugMode;     // -debug
    std::vector<string> inputFile;  // must follow all other args
    DeepZoom(){
        tileSize=256;
        tileOverlap=1;
        outputDir=".";
        verboseMode=false;
        debugMode=false;
    }
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
    arguments.read("-output",dz.outputDir)
    if (debugMode) {
        printf("tileSize=%d ", dz.tileSize);
        printf("tileOverlap=%d ", dz.tileOverlap);
        printf("outputDir=%s\n", dz.outputDir.c_str());
    }


    if(!osgDB::FileExists(dz.outputDir))
        osgDB::makedir(dz.outputDir);

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
    //string pathWithoutExtension = outputDir + File.separator + nameWithoutExtension;

    vips::VImage image(inFile.c_str(),"rb");

    int originalWidth = image.getWidth();
    int originalHeight = image.getHeight();

    double maxDim = Math.max(originalWidth, originalHeight);

    int nLevels = (int)Math.ceil(Math.log(maxDim) / Math.log(2));

    if (debugMode)
        System.out.printf("nLevels=%d\n", nLevels);

    // Delete any existing output files and folders for this image

    File descriptor = new File(pathWithoutExtension + ".xml");
    if (descriptor.exists()) {
        if (deleteExisting)
            deleteFile(descriptor);
        else
            throw new IOException("File already exists in output dir: " + descriptor);
    }

    File imgDir = new File(pathWithoutExtension);
    if (imgDir.exists()) {
        if (deleteExisting) {
            if (debugMode)
                System.out.printf("Deleting directory: %s\n", imgDir);
            deleteDir(imgDir);
        } else
            throw new IOException("Image directory already exists in output dir: " + imgDir);
    }

    imgDir = createDir(outputDir, nameWithoutExtension);

    double width = originalWidth;
    double height = originalHeight;

    for (int level = nLevels; level >= 0; level--) {
        int nCols = (int)Math.ceil(width / tileSize);
        int nRows = (int)Math.ceil(height / tileSize);
        if (debugMode)
            System.out.printf("level=%d w/h=%f/%f cols/rows=%d/%d\n",
                              level, width, height, nCols, nRows);

        File dir = createDir(imgDir, Integer.toString(level));
        for (int col = 0; col < nCols; col++) {
            for (int row = 0; row < nRows; row++) {
                BufferedImage tile = getTile(image, row, col);
                saveImage(tile, dir + File.separator + col + '_' + row);
            }
        }

        // Scale down image for next level
        width = Math.ceil(width / 2);
        height = Math.ceil(height / 2);
        if (width > 10 && height > 10) {
            // resize in stages to improve quality
            image = resizeImage(image, width * 1.66, height * 1.66);
            image = resizeImage(image, width * 1.33, height * 1.33);
        }
        image = resizeImage(image, width, height);
    }

    saveImageDescriptor(originalWidth, originalHeight, descriptor);
}


/**
     * Delete a file
     * @param path the path of the directory to be deleted
     */
private static void deleteFile(File file) throws IOException {
    if (!file.delete())
        throw new IOException("Failed to delete file: " + file);
}

/**
     * Recursively deletes a directory
     * @param path the path of the directory to be deleted
     */
private static void deleteDir(File dir) throws IOException {
    if (!dir.isDirectory())
        deleteFile(dir);
    else {
        for (File file : dir.listFiles()) {
            if (file.isDirectory())
                deleteDir(file);
            else
                deleteFile(file);
        }
        if (!dir.delete())
            throw new IOException("Failed to delete directory: " + dir);
    }
}

/**
     * Creates a directory
     * @param parent the parent directory for the new directory
     * @param name the new directory name
     */
private static File createDir(File parent, String name) throws IOException {
    assert(parent.isDirectory());
    File result = new File(parent + File.separator + name);
    if (!result.mkdir())
        throw new IOException("Unable to create directory: " + result);
    return result;
}

/**
     * Loads image from file
     * @param file the file containing the image
     */
private static BufferedImage loadImage(File file) throws IOException {
    BufferedImage result = null;
    try {
        result = ImageIO.read(file);
    } catch (Exception e) {
        throw new IOException("Cannot read image file: " + file);
    }
    return result;
}

/**
     * Gets an image containing the tile at the given row and column
     * for the given image.
     * @param img - the input image from whihc the tile is taken
     * @param row - the tile's row (i.e. y) index
     * @param col - the tile's column (i.e. x) index
     */
private static BufferedImage getTile(BufferedImage img, int row, int col) {
    int x = col * tileSize - (col == 0 ? 0 : tileOverlap);
    int y = row * tileSize - (row == 0 ? 0 : tileOverlap);
    int w = tileSize + (col == 0 ? 1 : 2) * tileOverlap;
    int h = tileSize + (row == 0 ? 1 : 2) * tileOverlap;

    if (x + w > img.getWidth())
        w = img.getWidth() - x;
    if (y + h > img.getHeight())
        h = img.getHeight() - y;

    if (debugMode)
        System.out.printf("getTile: row=%d, col=%d, x=%d, y=%d, w=%d, h=%d\n",
                          row, col, x, y, w, h);

    assert(w > 0);
    assert(h > 0);

    BufferedImage result = new BufferedImage(w, h, img.getType());
    Graphics2D g = result.createGraphics();
    g.drawImage(img, 0, 0, w, h, x, y, x+w, y+h, null);

    return result;
}

/**
     * Returns resized image
     * NB - useful reference on high quality image resizing can be found here:
     *   http://today.java.net/pub/a/today/2007/04/03/perils-of-image-getscaledinstance.html
     * @param width the required width
     * @param height the frequired height
     * @param img the image to be resized
     */
private static BufferedImage resizeImage(BufferedImage img, double width, double height) {
    int w = (int)width;
    int h = (int)height;
    BufferedImage result = new BufferedImage(w, h, img.getType());
    Graphics2D g = result.createGraphics();
    g.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                       RenderingHints.VALUE_INTERPOLATION_BICUBIC);
    g.drawImage(img, 0, 0, w, h, 0, 0, img.getWidth(), img.getHeight(), null);
    return result;
}

/**
     * Saves image to the given file
     * @param img the image to be saved
     * @param path the path of the file to which it is saved (less the extension)
     */
private static void saveImage(BufferedImage img, String path) throws IOException {
    File outputFile = new File(path + "." + tileFormat);
    try {
        ImageIO.write(img, tileFormat, outputFile);
    } catch (IOException e) {
        throw new IOException("Unable to save image file: " + outputFile);
    }
}

/**
     * Write image descriptor XML file
     * @param width image width
     * @param height image height
     * @param file the file to which it is saved
     */
private static void saveImageDescriptor(int width, int height, File file) throws IOException {
    Vector lines = new Vector();
    lines.add(xmlHeader);
    lines.add("<Image TileSize=\"" + tileSize + "\" Overlap=\"" + tileOverlap +
              "\" Format=\"" + tileFormat + "\" ServerFormat=\"Default\" xmnls=\"" +
              schemaName + "\">");
    lines.add("<Size Width=\"" + width + "\" Height=\"" + height + "\" />");
    lines.add("</Image>");
    saveText(lines, file);
}

/**
     * Saves strings as text to the given file
     * @param lines the image to be saved
     * @param file the file to which it is saved
     */
private static void saveText(Vector lines, File file) throws IOException {
    try {
        FileOutputStream fos = new FileOutputStream(file);
        PrintStream ps = new PrintStream(fos);
        for (int i = 0; i < lines.size(); i++)
            ps.println((String)lines.elementAt(i));
    } catch (IOException e) {
        throw new IOException("Unable to write to text file: " + file);
    }
}

}
