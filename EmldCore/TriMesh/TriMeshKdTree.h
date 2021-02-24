#ifndef _EmldCore_TriMesh_TriMeshKdTree_h_
#define _EmldCore_TriMesh_TriMeshKdTree_h_

#include "Foundation.h"
#include "TriKdTree3.h"
#include "TriMesh.h"

#include <EmldCore/SpatialSubd/Inclusion.h>
#include <EmldCore/SpatialSubd/KdTree3.h>
#include <OpenEXR/ImathBoxAlgo.h>
#include <OpenEXR/ImathLineAlgo.h>

#include <memory>
#include <vector>

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
// An "any inside at all" traverser
struct PointInsideAnyMesh {
    PointInsideAnyMesh(const V3f& p)
      : testPoint(p)
      , foundMesh(NULL) {
    }

    bool validBounds(const Box3f& bnds) const {
        return bnds.intersects(testPoint);
    }

    void leaf(const TriMesh* mesh) {
        if (mesh->isInside(testPoint)) { foundMesh = mesh; }
    }

    bool unfinished() const {
        return !((bool)foundMesh);
    }

    bool found() const {
        return (bool)foundMesh;
    }

    V3f testPoint;
    const TriMesh* foundMesh;
};

//-*****************************************************************************
// An "any bounds intersection" traverser.
struct BoundsIntersectAnyMesh {
    BoundsIntersectAnyMesh(const Box3f& i_bounds)
      : testBounds(i_bounds)
      , foundMesh(NULL) {
    }

    bool validBounds(const Box3f& i_bounds) const {
        return i_bounds.intersects(testBounds);
    }

    void leaf(const TriMesh* i_mesh) {
        if (!foundMesh) {
            if (i_mesh->intersects(testBounds)) { foundMesh = i_mesh; }
        }
    }

    bool unfinished() const {
        return !foundMesh;
    }

    bool foundAny() const {
        return (bool)foundMesh;
    }

    Box3f testBounds;
    const TriMesh* foundMesh;
};

//-*****************************************************************************
// A Kd Tree traversal tool.
struct ClosestHit : public ClosestTriangle {
    ClosestHit(const V3f& p)
      : ClosestTriangle(p)
      , bestMesh(NULL) {
    }

    ClosestHit(const V3f& p, float maxD2)
      : ClosestTriangle(p, maxD2)
      , bestMesh(NULL) {
    }

    bool validBounds(const Box3f& bnds) const {
        return SpatialSubd::PointBoxMinimumSquaredDistanceLessThan(
          testPoint, bnds, bestSquaredDist);
    }

    void leaf(const TriMesh* mesh) {
        ClosestTriangle ct =
          (*mesh).closestPointWithinSquaredDistance(testPoint, bestSquaredDist);

        if (ct.bestSquaredDist < bestSquaredDist) {
            bestSquaredDist = ct.bestSquaredDist;
            bestPoint = ct.bestPoint;
            bestTriangle = ct.bestTriangle;
            bestMesh = mesh;
        }
    }

    // Unfinished and foundAny both work as-is.
    const TriMesh* bestMesh;
};

//-*****************************************************************************
struct SegmentFirstIntersectMeshTraversal {
    SegmentFirstIntersectMeshTraversal(const V3f& i_start, const V3f& i_end)
      : testStart(i_start)
      , testEnd(i_end)
      , testRay(i_start, i_end)
      , testRayLength((i_end - i_start).length())
      , bestHit(0.0f)
      , bestDist(testRayLength * 2.0f)
      , found(false) {
        // Nothing
    }

    bool validBounds(const Box3f& i_bnds) const {
        V3f hitPoint;

        // This returns the first intersection along the ray.
        if (Imath::intersects(i_bnds, testRay, hitPoint)) {
            const float proj = (hitPoint - testRay.pos).dot(testRay.dir);
            if (found) {
                // If we've already found something, a bounds is only valid
                // if it is potentially closer that the existing hit.
                // if the first hit in the bounds is further away than the
                // best dist, don't bother!
                return (proj >= 0.0f && proj <= bestDist);
            } else {
                return (proj >= 0.0f && proj <= testRayLength);
            }
        } else {
            return false;
        }
    }

    void leaf(const TriMesh* i_mesh) {
        V3f hitPoint;
        float hitDist;
        bool f =
          i_mesh->firstIntersection(testStart, testEnd, hitPoint, hitDist);
        if (f) {
            const float proj = (hitPoint - testRay.pos).dot(testRay.dir);
            if (found) {
                if (proj >= 0.0f && proj < bestDist) {
                    bestHit = hitPoint;
                    bestDist = proj;
                    found = true;
                }
            } else {
                if (proj >= 0.0f && proj < testRayLength) {
                    bestHit = hitPoint;
                    bestDist = proj;
                    found = true;
                }
            }
        }
    }

    bool unfinished() const {
        return !found;
    }

    V3f testStart;
    V3f testEnd;
    Imath::Line3f testRay;
    float testRayLength;
    V3f bestHit;
    float bestDist;
    bool found;
};

//-*****************************************************************************
struct SegmentAnyIntersectMeshTraversal {
    SegmentAnyIntersectMeshTraversal(const V3f& start, const V3f& end)
      : testStart(start)
      , testEnd(end)
      , testRay(start, end)
      , found(false) {
        // Nothing
    }

    bool validBounds(const Box3f& bnds) const {
        return Imath::intersects(bnds, testRay);
    }

    void leaf(const TriMesh* mesh) {
        if (found) { return; }

        if (mesh->intersects(testStart, testEnd)) { found = true; }
    }

    bool unfinished() const {
        return !found;
    }

    V3f testStart;
    V3f testEnd;
    Imath::Line3f testRay;
    bool found;
};

//-*****************************************************************************
// Finds closest points and does inside/outside tests.
// We want this to be as versatile as possible, so we assume no
// container semantics for the TriMeshes.
class TriMeshKdTree {
public:
    struct MeshBounds {
        MeshBounds() {
        }
        inline const Box3f& operator()(const TriMesh* m) const {
            return m->bounds();
        }
    };

    struct MeshSortPt {
        MeshSortPt() {
        }
        inline V3f operator()(const TriMesh* m) const {
            return m->bounds().center();
        }
    };

    typedef SpatialSubd::KdTree3<float, const TriMesh*, MeshBounds, MeshSortPt>
      Tree;

    template <class MESH_POINTER_ITERATOR>
    TriMeshKdTree(MESH_POINTER_ITERATOR begin, MESH_POINTER_ITERATOR end) {
        // This lets me trick boost pointers.
        for (MESH_POINTER_ITERATOR iter = begin; iter != end; ++iter) {
            const TriMesh* tm = &(*(*iter));
            m_meshes.push_back(tm);
        }

        MeshBounds mb;
        MeshSortPt ms;
        m_tree.reset(new Tree(m_meshes, mb, ms));
    }

    // Don't need a destructor.

    //-*************************************************************************
    // Finders.
    bool isInside(const V3f& p) const {
        PointInsideAnyMesh pia(p);
        m_tree->constTraverse(pia);
        return pia.found();
    }

    //-*************************************************************************
    bool intersects(const Box3f& bnds) const {
        BoundsIntersectAnyMesh bi(bnds);
        m_tree->constTraverse(bi);
        return bi.foundAny();
    }

    //-*************************************************************************
    bool intersects(const V3f& i_rayBegin, const V3f& i_rayEnd) const {
        SegmentAnyIntersectMeshTraversal s(i_rayBegin, i_rayEnd);
        m_tree->constTraverse(s);
        return s.found;
    }

    //-*************************************************************************
    bool firstIntersection(const V3f& i_rayBegin,
                           const V3f& i_rayEnd,
                           V3f& o_hitPoint,
                           float& o_hitDist) const {
        SegmentFirstIntersectMeshTraversal s(i_rayBegin, i_rayEnd);
        m_tree->constTraverse(s);
        if (s.found) {
            o_hitPoint = s.bestHit;
            o_hitDist = s.bestDist;
            return true;
        } else {
            return false;
        }
    }

    //-*************************************************************************
    ClosestHit closestPoint(const V3f& p) const {
        ClosestHit hit(p);
        m_tree->constTraverse(hit);
        return hit;
    }

    //-*************************************************************************
    ClosestHit closestPointWithinSquaredDistance(const V3f& p, float r2) const {
        ClosestHit hit(p, r2);
        m_tree->constTraverse(hit);
        return hit;
    }

    Tree& tree() {
        return *m_tree;
    }
    const Tree& tree() const {
        return *m_tree;
    }

protected:
    std::vector<const TriMesh*> m_meshes;
    std::unique_ptr<Tree> m_tree;
};

}  // End namespace TriMesh
}  // End namespace EmldCore

#endif
