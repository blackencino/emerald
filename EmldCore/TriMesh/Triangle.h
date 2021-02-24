#ifndef _EmldCore_TriMesh_Triangle_h_
#define _EmldCore_TriMesh_Triangle_h_

#include "Foundation.h"
#include "Vertex.h"

#include <iostream>
#include <memory>
#include <vector>

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
class Triangle {
protected:
    void init();

public:
    Triangle(Vertex* a, Vertex* b, Vertex* c, int i_id) {
        m_vertices[0] = a;
        m_vertices[1] = b;
        m_vertices[2] = c;
        m_triangleId = i_id;
        init();
    }

    // id
    int triangleId() const {
        return m_triangleId;
    }

    //-*************************************************************************
    // 2D stuff.
    const Box2f& bounds2D() const {
        return m_bounds2D;
    }
    const V2f& center2D() const {
        return m_center2D;
    }

    // 2D hit.
    bool hit2D(const V2f& p, float d) const;

    // 2D hit depth
    bool hitDepth2D(const V2f& p, float& hitD) const;

    //-*************************************************************************
    // 3D stuff.
    const Box3f& bounds() const {
        return m_bounds;
    }
    const V3f& center() const {
        return m_center;
    }

    const V3f& planeNormal() const {
        return m_planeN;
    }

    float area() const {
        return m_area;
    }

    // Intersection with a line segment.
    bool intersectSegment(const V3f& sA,
                          const V3f& sB,
                          V3f& pFound,
                          float tol) const;

    // Closest point on the triangle
    V3f closestPoint(const V3f& p, float& d2) const;

    // Distance to point squared
    float squaredDistanceToPoint(const V3f& p) const;

    // Barycentric coordinate of a point, along with elevation.
    void computeBarycentric(const V3f& o_point,
                            V2f& o_baryAlphaBeta,
                            float& o_elevation) const;

    // Velocity at closest point
    V3f velAtClosestPoint(const V3f& p) const;

    // indices
    void indices(int& a, int& b, int& c) const {
        a = m_vertices[0]->index();
        b = m_vertices[1]->index();
        c = m_vertices[2]->index();
    }

    const Vertex* vertex(int i) const {
        return m_vertices[i];
    }

    const V3f barycentricEval(const V2f& i_baryAlphaBeta,
                              float i_elevation) const {
        float gamma = 1.0f - (i_baryAlphaBeta.x + i_baryAlphaBeta.y);
        V3f base((i_baryAlphaBeta.x * m_vertices[0]->position()) +
                 (i_baryAlphaBeta.y * m_vertices[1]->position()) +
                 (gamma * m_vertices[2]->position()));
        return base + (i_elevation * m_planeN);
    }

    const V3f velBarycentric(const V2f& i_baryAlphaBeta) const {
        float gamma = 1.0f - (i_baryAlphaBeta.x + i_baryAlphaBeta.y);
        return (i_baryAlphaBeta.x * m_vertices[0]->velocity()) +
               (i_baryAlphaBeta.y * m_vertices[1]->velocity()) +
               (gamma * m_vertices[2]->velocity());
    }

    void print(std::ostream& ostr) const {
        ostr << "Triangle[" << this << "]: " << std::endl
             << "\tbounds: " << m_bounds.min << " to " << m_bounds.max
             << std::endl
             << "\tcenter: " << m_center << std::endl
             << "\tnormal: " << m_planeN << std::endl
             << "\tvertex 0: " << m_vertices[0]->position()
             << " with index: " << m_vertices[0]->index() << std::endl
             << "\tvertex 1: " << m_vertices[1]->position()
             << " with index: " << m_vertices[1]->index() << std::endl
             << "\tvertex 2: " << m_vertices[2]->position()
             << " with index: " << m_vertices[2]->index() << std::endl;
    }

    // Rebuilds all the triangle information from the vertices.
    // Keeps only the vertex array itself.
    void rebuild() {
        init();
    }

protected:
    // Id
    int m_triangleId;

    // 2D stuff
    V2f m_points2D[3];
    float m_depths2D[3];
    float m_projection[2][2];
    Box2f m_bounds2D;
    V2f m_center2D;

    // 3D stuff
    Vertex* m_vertices[3];

    // Normalized Edges and Edge Lengths, for closest point finding.
    // ( ( v1 - v0 ) . ( p - v0 ) ) / len(v1-v0)^2
    V3f m_edgeOrigins[3];
    V3f m_edgeUnitDirs[3];
    float m_edgeLengths[3];

    // Plane information
    V3f m_planeN;
    float m_planeD;

    // Primary axis we should project along when going into 2D
    // This is different than the 2D points because it's unique
    // to the triangle, whereas the 2D points are always on the YZ plane.
    unsigned char m_projectionAxis;
    V2f m_p2[3];

    // Barycentric matrix for 2D projection points.
    M33f m_barycentricMatrix;

    // Bounds and center
    Box3f m_bounds;
    V3f m_center;

    // Area
    float m_area;
};

//-*****************************************************************************
typedef std::shared_ptr<Triangle> TrianglePtr;
typedef std::vector<TrianglePtr> TrianglePtrVector;

//-*****************************************************************************
inline std::ostream& operator<<(std::ostream& ostr, const Triangle& t) {
    t.print(ostr);
    return ostr;
}

}  // End namespace TriMesh
}  // End namespace EmldCore

#endif
