#ifndef VIPSSAMPLER_H
#define VIPSSAMPLER_H
#include <vips/vips>
#include <vips/vips.h>
#include <osg/Vec3>
#include <osg/Vec2>
/// Sampler that captures positions and normals.
class VipsSampler
{
public:
    typedef unsigned int uint;

    VipsSampler(vips::VImage &img) :m_img(img){};

    static void sampleTriCallback(void * param, int x, int y, const osg::Vec3& bar, const osg::Vec3& dx, const osg::Vec3& dy, float coverage);
    static void sampleQuadCallback(void * param, int x, int y, const osg::Vec3& bar, const osg::Vec3& dx, const osg::Vec3& dy, float coverage);

    void setCurrentFace(uint vertexCount, const osg::Vec3 * positions, const osg::Vec3 * normals);

    void sampleTri(int x, int y, const osg::Vec3& bar, const osg::Vec3& dx, const osg::Vec3& dy, float coverage);
    void sampleQuad(int x, int y, const osg::Vec3& bar, const osg::Vec3& dx, const osg::Vec3& dy, float coverage);

private:

    vips::VImage & m_img;
    REGION *regOutput;

    uint m_currentVertexCount;
    const osg::Vec3 * m_normals;
    const osg::Vec3 * m_positions;
    osg::Vec3 m_midedgenormals[5];
};
extern Rect gRect;

#define NV_EPSILON			(0.0001f)
inline bool isZero(const float f, const float epsilon = NV_EPSILON)
{
    return fabs(f) <= epsilon;
}
inline osg::Vec3 normalizeSafe(const osg::Vec3& v, const osg::Vec3& fallback, float epsilon = NV_EPSILON)
{
    float l = v.length();
    if (isZero(l, epsilon)) {
        return fallback;
    }
    return (v* (1.0f / l));
}
#endif // VIPSSAMPLER_H
