#include "VipsSampler.h"
#include <stdlib.h>
/*static*/
void VipsSampler::sampleTriCallback(void * param, int x, int y, const osg::Vec3& bar, const osg::Vec3& dx, const osg::Vec3& dy, float coverage)
{
    ((VipsSampler *)param)->sampleTri(x, y, bar, dx, dy, coverage);
}

/*static*/
void VipsSampler::sampleQuadCallback(void * param, int x, int y, const osg::Vec3& bar, const osg::Vec3& dx, const osg::Vec3& dy, float coverage)
{
    ((VipsSampler *)param)->sampleQuad(x, y, bar, dx, dy, coverage);
}


void VipsSampler::setCurrentFace(uint vertexCount, const osg::Vec3 * positions, const osg::Vec3 * normals)
{
    //nvDebugCheck(vertexCount<=4);

    m_positions = positions;
    m_normals = normals;

    for (uint k=0; k<vertexCount; k++)
    {
        m_midedgenormals[k] = normalizeSafe(normals[k] + normals[(k+1)%vertexCount], osg::Vec3(0,0,0), 0);
    }

    if (vertexCount==4) {
        m_midedgenormals[4] = normalizeSafe(m_normals[0]+normals[1]+normals[2]+normals[3], osg::Vec3(0,0,0), 0);
    }
}

inline int clamp(int x, int a, int b){    return x < a ? a : (x > b ? b : x);}


void VipsSampler::sampleTri(int x, int y, const osg::Vec3& bar, const osg::Vec3& dx, const osg::Vec3& dy, float coverage)
{
    //nvDebugCheck(isFinite(coverage));

    //Vector3 position = bar.x() * m_positions[0] + bar.y() * m_positions[1] + bar.z() * m_positions[2];

    //m_image.pixel( x, y)=Color32(255,0,0);//, m_image.positionChannel());
    gRect.left=x;
    gRect.top=y;

    if(im_prepare(regOutput,&gRect)){
        fprintf(stderr,"Prepare fail\n");
        exit(-1);
    }

    unsigned int *ptr=(unsigned int*)IM_REGION_ADDR( regOutput, x, y );
    unsigned int c=0;
    unsigned char r,g,b;
    r=255;
    g=0;
    b=0;
    c= clamp(b,0,255) | clamp(g,0,255) << 8 | clamp(r,0,255) << 16 | 255 << 24;
    *ptr = c;

#if 0
    //Vector3 normal = normalizeSafe(cross(m_positions[1] - m_positions[0], m_positions[2] - m_positions[0]), Vector3(zero), 0.0f);
    Vector3 linearNormal = normalizeSafe(bar.x() * m_normals[0] + bar.y() * m_normals[1] + bar.z() * m_normals[2], Vector3(zero), 0.0f);
    float u=bar.x(), v=bar.y(), w=bar.z();

    Vector3 normal = normalizeSafe(	u*u*m_normals[0] + v*v*m_normals[1] + w*w*m_normals[2] +
                                    2*(u*v*m_midedgenormals[0] + v*w*m_midedgenormals[1] + w*u*m_midedgenormals[2]), Vector3(zero), 0);

    float mx = max(max(u,v),w);

    m_image.addPixel(coverage * normal, x, y, m_image.normalChannel());

    /*if (m_image.occlusionChannel() != -1)
    {
        float occlusion = sampleOcclusion(position, normal, x, y);
        nvCheck(occlusion >= 0.0f && occlusion <= 1.0f);

        m_image.addPixel(coverage * occlusion, x, y, m_image.occlusionChannel());
    }*/

    m_image.addPixel(coverage, x, y, m_image.coverageChannel());
#endif
    //m_imageMask.setBitAt(x, y);
}


static inline float triangleArea(const osg::Vec2& a, const osg::Vec2& b, const osg::Vec2& c)
{
    osg::Vec2 v0 = a - c;
    osg::Vec2 v1 = b - c;

    return (v0.x() * v1.y() - v0.y() * v1.x());
}

void VipsSampler::sampleQuad(int x, int y, const osg::Vec3& bar, const osg::Vec3& dx, const osg::Vec3& dy, float coverage)
{
#if 0
    nvDebugCheck(isFinite(coverage));

    const float u = bar.x(), U = 1-u;
    const float v = bar.y(), V = 1-v;

    // bilinear position interpolation
    Vector3 position = (U * m_positions[0] + u * m_positions[1]) * V +
            (U * m_positions[3] + u * m_positions[2]) * v;

    m_image.addPixel(coverage * position, x, y, m_image.positionChannel());


    // biquadratic normal interpolation
    Vector3 normal = normalizeSafe(
                (U*U * m_normals[0] + 2*U*u*m_midedgenormals[0] + u*u * m_normals[1]) * V*V +
                (U*U*m_midedgenormals[3] + 2*U*u*m_midedgenormals[4] + u*u*m_midedgenormals[1]) * 2*V*v +
                (U*U * m_normals[3] + 2*U*u*m_midedgenormals[2] + u*u * m_normals[2]) * v*v, Vector3(zero), 0);

    m_image.addPixel(coverage * normal, x, y, m_image.normalChannel());

    /*
    if (m_image.occlusionChannel() != -1)
    {
        // piecewise linear position interpolation
        #if 0
        Vector3 tripos;
        if (u < v)
        {
            float barx = triangleArea(Vector2(1,1), Vector2(0,1), Vector2(u,v));
            float bary = triangleArea(Vector2(0,0), Vector2(1,1), Vector2(u,v));
            float barz = triangleArea(Vector2(0,1), Vector2(0,0), Vector2(u,v));
            nvCheck(equal(1, barx+bary+barz));

            tripos = barx * m_positions[0] + bary * m_positions[1] + barz * m_positions[2];
        }
        else
        {
            float barx = triangleArea(Vector2(1,0), Vector2(1,1), Vector2(u,v));
            float bary = triangleArea(Vector2(1,1), Vector2(0,0), Vector2(u,v));
            float barz = triangleArea(Vector2(0,0), Vector2(1,0), Vector2(u,v));
            nvCheck(equal(1, barx+bary+barz));

            tripos = barx * m_positions[0]  + bary * m_positions[3] + barz * m_positions[2];
        }
        #endif

        float occlusion = sampleOcclusion(position, normal, x, y);
        nvCheck(occlusion >= 0.0f && occlusion <= 1.0f);

        m_image.addPixel(coverage * occlusion, x, y, m_image.occlusionChannel());
    }
    */

    m_image.addPixel(coverage, x, y, m_image.coverageChannel());

    m_imageMask.setBitAt(x, y);
#endif
}
