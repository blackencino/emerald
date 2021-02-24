#include "TriMesh.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <map>
#include <vector>

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
// These helpers are for sorting edges.
namespace {

static inline V2i makeEdge(int ai, int bi) {
    return ai < bi ? V2i(ai, bi) : V2i(bi, ai);
}

struct EdgeOrder {
    EdgeOrder() {
    }
    inline bool operator()(const V2i& a, const V2i& b) const {
        if (a.x < b.x) {
            return true;
        } else if (a.x > b.x) {
            return false;
        } else {
            return (a.y < b.y);
        }
    }
};

typedef std::map<V2i, int, EdgeOrder> EdgeCountMap;

}  // End anonymous namespace

//-*****************************************************************************
TriMesh::TriMesh(const std::string& iName,
                 const ConstV3fRange& iPoints,
                 const ConstIntRange& iIndices,
                 const ConstIntRange& iCounts,
                 const M44d& iObjectToWorld,
                 float iTime)
  : m_name(iName)
  , m_time(iTime)
  , m_maxVelocity(0.0f) {
    // Validate inputs
    TRIMESH_ASSERT(!iPoints.empty(), "Empty Points Range in TriMesh ctor");
    TRIMESH_ASSERT(!iIndices.empty(), "Empty Indices Range in TriMesh ctor");
    TRIMESH_ASSERT(!iCounts.empty(), "Empty Counts Range in TriMesh ctor");

    // Copy the points, indices and counts.
    size_t numPoints = iPoints.size();
    size_t numIndices = iIndices.size();
    size_t numCounts = iCounts.size();
    assert(numPoints > 0 && numIndices > 0 && numCounts > 0);

    // Points and Vertices.
    m_bounds.makeEmpty();
    m_positions.resize(numPoints);
    m_normals.resize(numPoints);
    m_velocities.resize(numPoints);
    m_vertices.resize(numPoints);
    for (size_t i = 0; i < numPoints; ++i) {
        V3f p(V3d(iPoints[i]) * iObjectToWorld);
        // std::cout << "iPoints[" << i << "] = "
        //          << p << std::endl;
        m_positions[i] = p;
        m_normals[i] = V3f(0.0f, 1.0e-5f, 0.0f);
        m_velocities[i] = V3f(0.0f);
        m_bounds.extendBy(p);
        m_vertices[i] = Vertex(m_positions, m_velocities, i);
    }

    // Indices
    m_indices.resize(numIndices);
    for (size_t i = 0; i < numIndices; ++i) { m_indices[i] = iIndices[i]; }

    // Counts
    m_counts.resize(numCounts);
    for (size_t i = 0; i < numCounts; ++i) { m_counts[i] = iCounts[i]; }

    // Build triangles.
    m_triangles.reserve(numCounts);
    std::vector<V2i> edges;
    EdgeCountMap edgeCountMap;
    edges.reserve(numCounts * 3);
    int firstIndex = 0;
    for (size_t elem = 0; elem < numCounts; ++elem) {
        int numVerts = m_counts[elem];
        if (numVerts < 3) {
            // Skip it.
            firstIndex += numVerts;
            continue;
        }

        if (firstIndex + numVerts > (int)numIndices) {
            // This means we've gone beyond the edge of the
            // vertex array. Just get out.
            break;
        }

        // Polygon winding order 0, 1, 2, .... N-1
        // First triangle is 0, 1, 2.
        // Second triangle is 0, 2, 3
        // Last triangle is 0, N-2, N-1
        for (int lastIndex = 2; lastIndex < numVerts; ++lastIndex) {
            int Ai = m_indices[firstIndex];
            int Bi = m_indices[firstIndex + lastIndex - 1];
            int Ci = m_indices[firstIndex + lastIndex];

            if (Ai >= (int)numPoints || Bi >= (int)numPoints ||
                Ci >= (int)numPoints) {
                // Just skip this triangle.
                continue;
            }

            V2i edge0 = makeEdge(Ai, Bi);
            V2i edge1 = makeEdge(Bi, Ci);
            V2i edge2 = makeEdge(Ci, Ai);

#if 1
            EdgeCountMap::iterator fiter0 = edgeCountMap.find(edge0);
            if (fiter0 != edgeCountMap.end()) {
                if ((*fiter0).second > 1) { continue; }
                (*fiter0).second++;
            } else {
                edgeCountMap[edge0] = 1;
            }

            EdgeCountMap::iterator fiter1 = edgeCountMap.find(edge1);
            if (fiter1 != edgeCountMap.end()) {
                if ((*fiter1).second > 1) { continue; }
                (*fiter1).second++;
            } else {
                edgeCountMap[edge1] = 1;
            }

            EdgeCountMap::iterator fiter2 = edgeCountMap.find(edge2);
            if (fiter2 != edgeCountMap.end()) {
                if ((*fiter2).second > 1) { continue; }
                (*fiter2).second++;
            } else {
                edgeCountMap[edge2] = 1;
            }
#endif

            edges.push_back(edge0);
            edges.push_back(edge1);
            edges.push_back(edge2);

            int triId = m_triangles.size();
            TrianglePtr newTri(new Triangle(
              &(m_vertices[Ai]), &(m_vertices[Bi]), &(m_vertices[Ci]), triId));
            m_triangles.push_back(newTri);
            m_triIndices.push_back(Ai);
            m_triIndices.push_back(Bi);
            m_triIndices.push_back(Ci);

            V3f nAB = m_positions[Bi] - m_positions[Ai];
            nAB.normalize();
            V3f nAC = m_positions[Ci] - m_positions[Ai];
            nAC.normalize();
            V3f nBC = m_positions[Ci] - m_positions[Bi];
            nBC.normalize();

            float angleAtA = acosf(std::clamp(nAB.dot(nAC), -1.0f, 1.0f));
            float angleAtB = acosf(std::clamp(-(nAB.dot(nBC)), -1.0f, 1.0f));
            float angleAtC = acosf(std::clamp(nAC.dot(nBC), -1.0f, 1.0f));

            float na = newTri->area();
            V3f n = newTri->planeNormal().normalized();
            m_normals[Ai] += n * angleAtA;
            m_normals[Bi] += n * angleAtB;
            m_normals[Ci] += n * angleAtC;
        }

        firstIndex += numVerts;
    }

    // Normalize normals
    for (size_t i = 0; i < numPoints; ++i) { m_normals[i].normalize(); }

    // Triangles are built. Woo Hoo!
    // Figure out if we're open or closed.
    // We're closed if and only if every edge exists exactly twice.
    // So, we sort the edge list and loop through it counting repeats.
    // If we ever have only one repeat, we're open.
    // If we ever have more than two repeats, we're non-manifold. Eek.
    EdgeOrder eo;
    std::sort(edges.begin(), edges.end(), eo);

    // std::cout << "EDGES: " << std::endl;
    // std::copy( edges.begin(), edges.end(),
    //           std::ostream_iterator<V2i>( std::cout, ", " ) );

    int numFound = 0;
    V2i current(-987654321, -987654321);
    m_isClosed = true;
    for (std::vector<V2i>::iterator eiter = edges.begin(); eiter != edges.end();
         ++eiter) {
        const V2i& e = (*eiter);
        if (e != current) {
            if (numFound == 1) {
                // Mesh is open.
                m_isClosed = false;
                break;
            } else if (numFound > 2) {
                // Mesh is non-manifold. Eek!
                TRIMESH_THROW("TriMesh: " << m_name << " is non-manifold");
            }

            numFound = 1;
            current = e;
        } else {
            ++numFound;
        }
    }

    // Now we know if we're closed.
    // Lastly, build the trees.
    m_tree2.reset(new TriKdTree2(m_triangles));
    m_tree3.reset(new TriKdTree3(m_triangles));
}

//-*****************************************************************************
void TriMesh::rebuild(const ConstV3fRange& iPoints,
                      const M44d& iObjectToWorld,
                      float iTime) {
    // Get new time and delta time.
    float DT = iTime - m_time;
    m_time = iTime;

    std::cout << "TriMesh rebuild DT: " << DT << ", which is 1.0/" << 1.0 / DT
              << std::endl;

    // Clean up.
    m_tree2.reset();
    m_tree3.reset();

    // Reset max velocity
    m_maxVelocity = V3f(0.0f);
    float maxVel2 = 0.0f;

    // Validate inputs
    TRIMESH_ASSERT(!iPoints.empty(), "Empty Points Range in TriMesh ctor");

    // Copy the points, indices and counts.
    size_t numPoints = iPoints.size();
    assert(numPoints > 0);
    TRIMESH_ASSERT(numPoints == m_positions.size(),
                   "TriMesh::rebuild() called with changed topology. "
                     << "Name: " << m_name << std::endl
                     << "Old num points: " << m_positions.size()
                     << ", and new: " << numPoints);

    // Valid timestep.
    bool validTimestep = (bool)(std::abs(DT) > 0.0001);

    // Points.
    m_bounds.makeEmpty();
    for (size_t i = 0; i < numPoints; ++i) {
        V3f p(V3d(iPoints[i]) * iObjectToWorld);
        V3f dp = p - m_positions[i];
        m_positions[i] = p;
        m_normals[i] = V3f(0.0f, 1.0e-5f, 0.0f);  // Reset, accumulate later.

        V3f vel = validTimestep ? (dp / DT) : V3f(0.0f);
        m_velocities[i] = vel;
        float vr2 = vel.length2();
        if (vr2 > maxVel2) {
            maxVel2 = vr2;
            m_maxVelocity = vel;
        }

        m_bounds.extendBy(p);
    }

    // Rebuild triangles.
    for (TrianglePtrVector::iterator titer = m_triangles.begin();
         titer != m_triangles.end();
         ++titer) {
        (*titer)->rebuild();
        int ai, bi, ci;
        (*titer)->indices(ai, bi, ci);
        V3f n = (*titer)->planeNormal().normalized() * (*titer)->area();
        m_normals[ai] += n;
        m_normals[bi] += n;
        m_normals[ci] += n;
    }

    // Normalize normals
    for (size_t i = 0; i < numPoints; ++i) { m_normals[i].normalize(); }

    // Rebuild trees
    m_tree2.reset(new TriKdTree2(m_triangles));
    m_tree3.reset(new TriKdTree3(m_triangles));
}

//-*****************************************************************************
void TriMesh::rebuild(const ConstV3fRange& iPointsMin,
                      const ConstV3fRange& iPointsMax,
                      float iInterp,
                      const M44d& iObjectToWorldMin,
                      const M44d& iObjectToWorldMax,
                      float iTime) {
    // Get new time and delta time.
    float DT = iTime - m_time;
    m_time = iTime;

    // Clean up.
    m_tree2.reset();
    m_tree3.reset();

    // Validate inputs
    TRIMESH_ASSERT(!iPointsMin.empty(), "Empty Points Range in TriMesh ctor");
    TRIMESH_ASSERT(iPointsMin.size() == iPointsMax.size(),
                   "Invalid number of points");

    // Copy the points, indices and counts.
    size_t numPoints = iPointsMin.size();
    assert(numPoints > 0);
    TRIMESH_ASSERT(numPoints == m_positions.size(),
                   "TriMesh::rebuild() called with changed topology. "
                     << "Name: " << m_name << std::endl
                     << "Old num points: " << m_positions.size()
                     << ", and new: " << numPoints);

    // Valid timestep.
    bool validTimestep = (bool)(std::abs(DT) > 0.0001);

    // Points.
    m_bounds.makeEmpty();
    for (size_t i = 0; i < numPoints; ++i) {
        V3d pMin = V3d(iPointsMin[i]) * iObjectToWorldMin;
        V3d pMax = V3d(iPointsMax[i]) * iObjectToWorldMax;

        V3f p(lerp(pMin, pMax, iInterp));
        V3f dp = p - m_positions[i];
        m_positions[i] = p;
        m_normals[i] = V3f(0.0f, 1.0e-5f, 0.0f);  // Reset, accumulate later.
        m_velocities[i] = validTimestep ? (dp / DT) : V3f(0.0f);
        m_bounds.extendBy(p);
    }

    // Rebuild triangles.
    for (TrianglePtrVector::iterator titer = m_triangles.begin();
         titer != m_triangles.end();
         ++titer) {
        (*titer)->rebuild();
        int ai, bi, ci;
        (*titer)->indices(ai, bi, ci);
        V3f n = (*titer)->planeNormal().normalized() * (*titer)->area();
        m_normals[ai] += n;
        m_normals[bi] += n;
        m_normals[ci] += n;
    }

    // Normalize normals
    for (size_t i = 0; i < numPoints; ++i) { m_normals[i].normalize(); }

    // Rebuild trees
    m_tree2.reset(new TriKdTree2(m_triangles));
    m_tree3.reset(new TriKdTree3(m_triangles));
}

}  // End namespace TriMesh
}  // End namespace EmldCore
