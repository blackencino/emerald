#ifndef _EmldCore_TriMesh_TriMesh_h_
#define _EmldCore_TriMesh_TriMesh_h_

#include "Foundation.h"
#include "TriKdTree2.h"
#include "TriKdTree3.h"
#include "Triangle.h"
#include "Vertex.h"

#include <boost/core/noncopyable.hpp>

#include <memory>
#include <string>
#include <vector>

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
// The triangle mesh is made from the components that comprise a mesh -
// the V3f points, the indices, and the face counts.
// the data is copied before being used.
class TriMesh : private boost::noncopyable {
public:
    TriMesh(const std::string& iName,
            const ConstV3fRange& iPoints,
            const ConstIntRange& iIndices,
            const ConstIntRange& iCounts,
            const M44d& iObjectToWorld = Imath::identity44d,
            float iTime = 0.0);

    const std::string& name() const {
        return m_name;
    }

    // Rebuild the mesh with new vertex positions,
    // but otherwise the same previous connectivity and topology.
    void rebuild(const ConstV3fRange& iPoints,
                 const M44d& iObjectToWorld = Imath::identity44d,
                 float iNewTime = 0.0);

    void rebuild(const ConstV3fRange& iPointsMin,
                 const ConstV3fRange& iPointsMax,
                 float iInterp,
                 const M44d& iObjectToWorldMin,
                 const M44d& iObjectToWorldMax,
                 float iNewTime = 0.0);

    void rebuild(const ConstV3fRange& iPointsMin,
                 const ConstV3fRange& iPointsMax,
                 float iInterp,
                 const M44d& iObjectToWorld = Imath::identity44d,
                 float iNewTime = 0.0) {
        rebuild(iPointsMin,
                iPointsMax,
                iInterp,
                iObjectToWorld,
                iObjectToWorld,
                iNewTime);
    }

    bool isClosed() const {
        return m_isClosed;
    }

    bool isInside(const V3f& p) const {
        int hitCount = m_tree2->hitCount(V2f(p.y, p.z), p.x);
        // This is saying hitCount is odd.
        return (bool)(hitCount & 1);
    }

    // Check all of the hits for a point's x-ray
    void xRayIntersections(const V2f& yz, FloatVector& xInts) const {
        m_tree2->xRayIntersections(yz, xInts);
    }

    bool intersects(const V3f& p) const {
        return isInside(p);
    }

    bool intersects(const Box3f& b) const {
        return m_tree3->intersects(b);
    }

    // This is intersecting a line segment.
    bool intersects(const V3f& start, const V3f& end) const {
        return m_tree3->intersects(start, end);
    }

    // This is returning the first intersection along a ray.
    bool firstIntersection(const V3f& i_start,
                           const V3f& i_end,
                           V3f& o_hitPoint,
                           float& o_hitDist) const {
        return m_tree3->firstIntersection(
          i_start, i_end, o_hitPoint, o_hitDist);
    }

    // Returns NULL if no point was found.
    // Otherwise returns the triangle that
    ClosestTriangle closestPoint(const V3f& p) const {
        return m_tree3->closestPoint(p);
    }

    // Return the closest point within a radius.
    // Returns NULL if nothing.
    ClosestTriangle closestPointWithinSquaredDistance(const V3f& p,
                                                      float r2) const {
        return m_tree3->closestPointWithinSquaredDistance(p, r2);
    }

    // Returns NULL if no point was found.
    // Otherwise returns the triangle that
    ClosestTriangle closestPointVerbose(const V3f& p,
                                        std::ostream& ostr) const {
        return m_tree3->closestPointVerbose(p, ostr);
    }

    // Return the closest point within a radius.
    // Returns NULL if nothing.
    ClosestTriangle closestPointWithinSquaredDistanceVerbose(
      const V3f& p, float r2, std::ostream& ostr) const {
        return m_tree3->closestPointWithinSquaredDistanceVerbose(p, r2, ostr);
    }

    // Return the closest point within a radius.
    // Returns NULL if nothing.
    AnyTriangle anyPointWithinSquaredDistance(const V3f& p, float r2) const {
        return m_tree3->anyPointWithinSquaredDistance(p, r2);
    }

    const Box3f& bounds() const {
        return m_bounds;
    }

    ConstV3fRange positions() const {
        return CreateV3fRange(m_positions);
    }
    ConstV3fRange normals() const {
        return CreateV3fRange(m_normals);
    }
    ConstV3fRange velocities() const {
        return CreateV3fRange(m_velocities);
    }
    ConstIntRange indices() const {
        return CreateIntRange(m_indices);
    }
    ConstIntRange counts() const {
        return CreateIntRange(m_counts);
    }

    void setVelocities(ConstV3fRange i_vel) {
        TRIMESH_ASSERT(i_vel.size() == m_velocities.size(),
                       "Can't set velocities out of range.");
        std::copy(i_vel.begin(), i_vel.end(), m_velocities.begin());
    }

    const TriKdTree3& getTree3() const {
        return *m_tree3;
    }

    float time() const {
        return m_time;
    }

    ConstUintRange triIndices() const {
        return CreateUintRange(m_triIndices);
    }

    size_t numTriangles() const {
        return m_triangles.size();
    }
    TrianglePtr triangle(size_t i) {
        return m_triangles[i];
    }
    const Triangle* triangle(size_t i) const {
        return m_triangles[i].get();
    }

    // Max velocity, by magnitude
    V3f maxVelocity() const {
        return m_maxVelocity;
    }

protected:
    // Name of the tri mesh.
    std::string m_name;

    // Copied from the inputs.
    V3fVector m_positions;
    V3fVector m_normals;
    IntVector m_indices;
    IntVector m_counts;

    // Created by forward differencing
    V3fVector m_velocities;

    // The bounds.
    Box3f m_bounds;

    // Whether or not the mesh is closed.
    bool m_isClosed;

    // Built only once, on construction, from the connectivity
    // information. Non-triangular faces are dumbly triangulated.
    VertexVector m_vertices;
    TrianglePtrVector m_triangles;

    // Triangular indices, for drawing.
    UintVector m_triIndices;

    // Rebuilt each time rebuild is called.
    std::unique_ptr<TriKdTree2> m_tree2;
    std::unique_ptr<TriKdTree3> m_tree3;

    // Time represented by this mesh.
    float m_time;

    // Max velocity of this mesh.
    V3f m_maxVelocity;
};

}  // End namespace TriMesh
}  // End namespace EmldCore

#endif
