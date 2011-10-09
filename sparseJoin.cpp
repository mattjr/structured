#include "GLImaging.h"
#include "MemUtils.h"
/* Hold our state in this.
 */
std::string format_elapsed(double d)
{
    char buf[256] = {0};

    if( d < 0.00000001 )
    {
        // show in ps with 4 digits
        sprintf(buf, "%0.4f ps", d * 1000000000000.0);
    }
    else if( d < 0.00001 )
    {
        // show in ns
        sprintf(buf, "%0.0f ns", d * 1000000000.0);
    }
    else if( d < 0.001 )
    {
        // show in us
        sprintf(buf, "%0.0f us", d * 1000000.0);
    }
    else if( d < 0.1 )
    {
        // show in ms
        sprintf(buf, "%0.0f ms", d * 1000.0);
    }
    else if( d <= 60.0 )
    {
        // show in seconds
        sprintf(buf, "%0.2f s", d);
    }
    else if( d < 3600.0 )
    {
        // show in min:sec
        sprintf(buf, "%01.0f:%02.2f", floor(d/60.0), fmod(d,60.0));
    }
    // show in h:min:sec
    else
        sprintf(buf, "%01.0f:%02.0f:%02.2f", floor(d/3600.0), floor(fmod(d,3600.0)/60.0), fmod(d,60.0));

    return buf;
}
typedef struct {
    /* Args.
         */
    IMAGE ** subs;		/* Sub image */
    IMAGE *out;		/* Output image */

    /* Geometry.
         */
    Rect rout;		/* Output space */
    int count;
    Rect ** rsubs;		/* Positions of sub in output */
} InsertVecState;

/* Trivial case: we just need pels from one of the inputs.
 */
static int
        just_one( REGION *outr, REGION *ir, int x, int y )
{
    Rect need;
    /* Find the part of pos we need.
         */
    need = outr->valid;
    need.left -= x;
    need.top -= y;
    if( im_prepare( ir, &need ) )
        return( -1 );

    /* Attach our output to it.
         */
    if( im_region_region( outr, ir, &outr->valid, need.left, need.top ) )
        return( -1 );

    return( 0 );
}
/* Paste in parts of ir that fall within or --- ir is an input REGION for an
 * image positioned at pos within or.
 */
static int
        paste_region( REGION *outr, REGION *ir, Rect *pos,Rect *irec )
{
    Rect ovl;

    /* Does any of the sub-image appear in the area we have been asked
         * to make?
         */
    im_rect_intersectrect( &outr->valid, irec, &ovl );
    if( !im_rect_isempty( &ovl ) ) {
        /* Find the part of in we need.
                 */
        ovl.left -= irec->left;
        ovl.top -= irec->top;
        /* Paint this area of pixels into or.
                 */
        if( im_prepare_to( ir, outr, &ovl,
                           pos->left,  pos->top ) )
            return( -1 );
    }

    return( 0 );
}

/* Insert generate function.
 */
static int
        insert_gen( REGION *outr, void *seq, void *a, void *b )
{
    REGION **ir = (REGION **) seq;
    InsertVecState *ins = (InsertVecState *) b;

    /* Does the rect we have been asked for fall entirely inside the
         * sub-image?
            !!!!!Assuming nonoverlapping!!!!!
         */
    for(int i=0; i < ins->count; i++ ){
        if( im_rect_includesrect( ins->rsubs[i], &outr->valid ) )
            return( just_one( outr, ir[i],
                              ins->rsubs[i]->left, ins->rsubs[i]->top ) );
    }
    /* Does it fall entirely inside the main, and not at all inside the
         * sub?
         */
    for(int i=0; i < ins->count; i++ ){
        Rect ovl;
        im_rect_intersectrect( &outr->valid, ins->rsubs[i], &ovl );
        if( !im_rect_isempty( &ovl ))

            if( paste_region( outr, ir[i], &ovl,ins->rsubs[i]) ){
            fprintf(stderr,"Region %d Fail!\n",i);
            return( -1 );
        }
    }


    return( 0 );
}
/**
 * im_insert:
 * @subs: small images to insert
 * @out: output image
 * @x: left position of @sub
 * @y: top position of @sub
 *
 * Insert one image into another. @sub is inserted into image @main at
 * position @x, @y relative to the top LH corner of @main. @out is made large
 * enough to hold both @main and @sub. Any areas of @out not coming from
 * either @main or @sub are set to black (binary 0). If @sub overlaps @main,
 * @sub will appear on top of @main.
 *
 * If the number of bands differs, one of the images
 * must have one band. In this case, an n-band image is formed from the
 * one-band image by joining n copies of the one-band image together, and then
 * the two n-band images are operated upon.
 *
 * The two input images are cast up to the smallest common type (see table
 * Smallest common format in
 * <link linkend="VIPS-arithmetic">arithmetic</link>).
 *
 * See also: im_insert_noexpand(), im_lrjoin().
 *
 * Returns: 0 on success, -1 on error
 */
int
        im_insert_many(std::vector<IMAGE *> &subs, IMAGE *out,Rect &rout, std::vector<Rect > rects ,int bands)
{
    InsertVecState *ins;
    if(subs.size() ==0){
        fprintf(stderr,"No input images\n");
        return -1;
    }
    IMAGE **vec = new IMAGE*[subs.size()+1];
    int t=0;
    for(t=0; t< (int) subs.size(); t++){
        vec[t]=subs[t];
    }
    vec[t]=NULL;

    if( !(ins = IM_NEW( out, InsertVecState )
        )){
        fprintf(stderr,"No input images\n");

        return( -1 );
    }

    /* Save args.
         */
    ins->subs = vec;
    ins->out = out;
    ins->count=rects.size();
    printf("%d\n",rects.size());
    ins->rsubs=new Rect*[rects.size()];
    for(int i=0; i<  rects.size(); i++){
        ins->rsubs[i]=  &(rects[i]);

    }



    ins->rout= rout;

    /* Set up the output header.
         */
    im_cp_desc(out,subs[0]);
    im_demand_hint_array( out, IM_SMALLTILE, vec );
    out->Xsize = ins->rout.width;
    out->Ysize = ins->rout.height;
    out->generate =NULL;
    out->start = NULL;
    out->stop =NULL;
    /* Make output image.
         */
    if( im_generate( out,
                     im_start_many, insert_gen, im_stop_many, vec, ins ) ){
        fprintf(stderr,"generate fail\n");
        return( -1 );
    }

    // out->Xoffset = ins->rmain.left;
    //out->Yoffset = ins->rmain.top;

    return( 0 );
}

int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");


    unsigned int width=512;
    unsigned int height=512;
    arguments.read("--pbuffer-only",width,height);
    int _tileColumns=0;
    int _tileRows=0;

    osg::BoundingBox totalbb;

    double lat=0,lon=0;
    arguments.read("-lat",lat);
    arguments.read("-lon",lon);
    bool untex= arguments.read("-untex");
    string ext;
    arguments.read("-nogfx",ext);

    std::vector<picture_cell> cells;
    FILE *fp=fopen(argv[1],"r");
    if(!fp){
        fprintf(stderr,"Can't open %s\n",argv[1]);
        exit(-1);
    }
    int cnt=0;
    while(!feof(fp)){
        char fname[1024];
        float minx,maxx,miny,maxy,minz,maxz;
        int row,col;
        int res=fscanf(fp,"%f %f %f %f %f %f %d %d %s\n",&minx,&maxx,&miny,&maxy,&minz,&maxz,&col,&row,fname);
        if(res != 9){
            fprintf(stderr,"Bad parse\n");
            exit(-1);
        }
        if(cnt==0){
            totalbb=osg::BoundingBox(minx,miny,minz,maxx,maxy,maxz);
            _tileColumns=col;
            _tileRows=row;

        }else{
            picture_cell cell;
            cell.bbox=osg::BoundingBox(minx,miny,minz,maxx,maxy,maxz);
            cell.col=col;
            cell.row=row;
            if(std::string(fname) != "null")
                cell.name=std::string(argv[2])+"/"+std::string(fname);
            else
                cell.name=std::string(fname);
            cells.push_back(cell);

        }
        cnt++;

    }





    osg::Matrixd view,proj;


    std::stringstream os2;
    os2<< "view.mat";

    std::fstream _file(os2.str().c_str(),std::ios::binary|std::ios::in);
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            _file.read(reinterpret_cast<char*>(&(view(i,j))),sizeof(double));
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            _file.read(reinterpret_cast<char*>(&(proj(i,j))),sizeof(double));
    _file.close();

    // std::ostringstream os;
    // os <<"subtile.ppm";//<<":deflate";
    IMAGE *rawI;
    int Xsize,Ysize;
    Xsize=width*_tileColumns;
    Ysize=height*_tileRows;
    Rect rout;
    rout.height=Ysize;
    rout.width=Xsize;
    rout.left=0;
    rout.top=0;
    double vm, rss;
    process_mem_usage(vm, rss);
    cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;
    printf("Creating %d %d\n",Xsize,Ysize);



    rawI=im_open("subtile.v","w");//tif:deflate,tile:256x256","w");

    process_mem_usage(vm, rss);
    cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;

    unsigned int validCount=0;
    for(int i=0; i < (int)cells.size(); i++){
        if(cells[i].name != "null")
            validCount++;
    }
    osg::Timer_t startTick = osg::Timer::instance()->tick();
  //  formatBar("Img",startTick,0,validCount);
    std::vector<IMAGE *> subs;		/* Sub image */
    std::vector<Rect> rsubs;		/* Positions of sub i */

    //int count=0;
    for(int i=0; i < (int)cells.size(); i++)
    {
        if(cells[i].name == "null" )
            continue;

        char tmp[1024];

        sprintf(tmp,"mesh-diced/image_r%04d_c%04d_rs%04d_cs%04d.%s",cells[i].row,cells[i].col,_tileRows,_tileColumns,ext.c_str());
        if(osgDB::fileExists(tmp)){
            IMAGE *tmpI=im_open(tmp,"r");
            if(!tmpI){
                fprintf(stderr,"Can't load tmp\n");
                return -1;
            }
            Rect r;
            r.left=width*cells[i].col;
            r.top=height*(_tileRows-cells[i].row-1);
            r.width=tmpI->Xsize;
            r.height=tmpI->Ysize;

            subs.push_back(tmpI);
            rsubs.push_back(r);
        }
    }




    if(im_insert_many(subs, rawI,rout,rsubs,4 )){
        fprintf(stderr,"Can't load insert many\n");
        return -1;
    }


    // formatBar("Img",startTick,validCount,validCount);
    /* osg::Timer_t writeStart = osg::Timer::instance()->tick();

    //raw.write("subtile.v");
    double writeTime = osg::Timer::instance()->delta_s(writeStart, osg::Timer::instance()->tick());
    fprintf(logfp,"Write Time %.1fs\n",writeTime);

    if(untex){
        osg::Timer_t writeStart = osg::Timer::instance()->tick();

        //raw_untex.write("subtile_untex.v");
        double writeTime = osg::Timer::instance()->delta_s(writeStart, osg::Timer::instance()->tick());
        fprintf(logfp,"Write Time 2 %.1fs\n",writeTime);

    }*/
    process_mem_usage(vm, rss);
    cout << "VM: " << get_size_string(vm) << "; RSS: " << get_size_string(rss) << endl;
    printf("Done\n");
    im_close(rawI);

    double totalTime = osg::Timer::instance()->delta_s(startTick, osg::Timer::instance()->tick());
    printf("Total Time %s\n",format_elapsed(totalTime).c_str());
    applyGeoTags(osg::Vec2(lat,lon),view,proj,Xsize,Ysize);

}
