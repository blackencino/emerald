#ifndef _EmldCore_TriMesh_TriKdTree3_h_
#define _EmldCore_TriMesh_TriKdTree3_h_

#include "Foundation.h"
#include "Triangle.h"

#include <EmldCore/SpatialSubd/Inclusion.h>
#include <EmldCore/SpatialSubd/KdTree3.h>
#include <OpenEXR/ImathBoxAlgo.h>
#include <OpenEXR/ImathLine.h>
#include <OpenEXR/ImathLineAlgo.h>

#include <iostream>
#include <memory>
#include <vector>

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
// This is a KdTree traversal tool.
struct ClosestTriangle {
    ClosestTriangle(const V3f& p)
      : testPoint(p)
      , bestSquaredDist(FLT_MAX)
      , maxSquaredDist(FLT_MAX)
      , bestPoint(p)
      , bestTriangle(NULL) {
    }

    ClosestTriangle(const V3f& p, float maxD2)
      : testPoint(p)
      , bestSquaredDist(maxD2)
      , maxSquaredDist(maxD2)
      , bestPoint(p)
      , bestTriangle(NULL) {
    }

    bool validBounds(const Box3f& bnds) const {
        // Expand the bounds _slightly_, to make sure that roundoff
        // error doesn't nuke it.
        return SpatialSubd::PointBoxMinimumSquaredDistanceLessThan(
          testPoint, bnds, 1.01f * bestSquaredDist);
    }

    void leaf(Triangle* tri) {
        float d2 = FLT_MAX;
        V3f point = tri->closestPoint(testPoint, d2);
        if (d2 < bestSquaredDist) {
            bestSquaredDist = d2;
            bestPoint = point;
            bestTriangle = tri;
        }
    }

    // Always need to keep checking for closest point stuff.
    bool unfinished() const {
        return true;
    }

    bool foundAny() const {
        return bestTriangle != NULL && (bestSquaredDist <= maxSquaredDist);
    }

    V3f testPoint;
    float bestSquaredDist;
    float maxSquaredDist;
    V3f bestPoint;
    Triangle* bestTriangle;
};

//-*****************************************************************************
struct NoisyClosestTriangle : public ClosestTriangle {
    NoisyClosestTriangle(const V3f& p, std::ostream& o)
      : ClosestTriangle(p)
      , ostr(o) {
        ostr << "Created noisy closest triangle for point: " << p << std::endl;
    }

    NoisyClosestTriangle(const V3f& p, float maxD2, std::ostream& o)
      : ClosestTriangle(p, maxD2)
      , ostr(o) {
        ostr << "Created noisy closest triangle for point: " << p
             << " with maxSquaredDistance: " << maxD2 << std::endl;
    }

    bool validBounds(const Box3f& bnds) const {
        bool ret = SpatialSubd::PointBoxMinimumSquaredDistanceLessThan(
          testPoint, bnds, 1.01f * bestSquaredDist);
        std::cout << "Testing point: " << testPoint
                  << " against bounds: " << bnds.min << " to " << bnds.max
                  << " and returning: " << ret << std::endl;
        return ret;
    }

    void leaf(Triangle* tri) {
        float d2 = FLT_MAX;
        V3f point = tri->closestPoint(testPoint, d2);

        std::cout << "Closest point for point: " << testPoint
                  << " against triangle: " << (*tri) << std::endl
                  << "closest: " << point << " with d2 = " << d2 << std::endl
                  << "Best Squared Dist so far: " << bestSquaredDist
                  << " for point: " << bestPoint << std::endl;

        if (d2 < bestSquaredDist) {
            std::cout << "Updating to this new point!" << std::endl;
            bestSquaredDist = d2;
            bestPoint = point;
            bestTriangle = tri;
        }
    }

    std::ostream& ostr;
};

//-*****************************************************************************
// This is a KdTree traversal tool.
struct AnyTriangle {
    AnyTriangle(const V3f& p, float maxD2)
      : testPoint(p)
      , bestSquaredDist(maxD2)
      , maxSquaredDist(maxD2)
      , bestPoint(p)
      , bestTriangle(NULL) {
    }

    bool validBounds(const Box3f& bnds) const {
        // Expand the bounds _slightly_, to make sure that roundoff
        // error doesn't nuke it.
        return SpatialSubd::PointBoxMinimumSquaredDistanceLessThan(
          testPoint, bnds, 1.01f * bestSquaredDist);
    }

    void leaf(Triangle* tri) {
        float d2 = FLT_MAX;
        V3f point = tri->closestPoint(testPoint, d2);
        if (d2 < bestSquaredDist) {
            bestSquaredDist = d2;
            bestPoint = point;
            bestTriangle = tri;
        }
    }

    // Always need to keep checking for closest point stuff.
    bool unfinished() const {
        return bestTriangle == NULL;
    }

    bool foundAny() const {
        return bestTriangle != NULL;
    }

    V3f testPoint;
    float bestSquaredDist;
    float maxSquaredDist;
    V3f bestPoint;
    Triangle* bestTriangle;
};

//-*****************************************************************************
struct BoundsIntersect {
    BoundsIntersect(const Box3f& b)
      : testBounds(b)
      , found(false) {
    }

    bool validBounds(const Box3f& bnds) const {
        return testBounds.intersects(bnds);
    }

    void leaf(const Triangle* tri) {
        if (!found) {
            if (testBounds.intersects(tri->bounds())) { found = true; }
        }
    }

    bool unfinished() const {
        return !found;
    }

    bool foundAny() const {
        return found;
    }

    Box3f testBounds;
    bool found;
};

//-*****************************************************************************
struct SegmentAnyIntersectTraversal {
    SegmentAnyIntersectTraversal(const V3f& start, const V3f& end)
      : testStart(start)
      , testEnd(end)
      , testRay(start, end)
      , testRaySquaredDist((end - start).length2())
      , found(false) {
        // Nothing
    }

    bool validBounds(const Box3f& bnds) const {
        return Imath::intersects(bnds, testRay);
    }

    void leaf(const Triangle* tri) {
        if (found) { return; }
        V3f hitPoint;
        bool f = tri->intersectSegment(testStart, testEnd, hitPoint, 1.0e-6f);
        if (f) { found = true; }
    }

    bool unfinished() const {
        return !found;
    }

    V3f testStart;
    V3f testEnd;
    Imath::Line3f testRay;
    float testRaySquaredDist;
    bool found;
};

//-*****************************************************************************
struct SegmentFirstIntersectTraversal {
    SegmentFirstIntersectTraversal(const V3f& i_start, const V3f& i_end)
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

    void leaf(const Triangle* tri) {
        V3f hitPoint;
        bool f = tri->intersectSegment(testStart, testEnd, hitPoint, 1.0e-6f);
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
// Because the KdTree will be used to gather points for potential modification,
// we need to have a modifiable list of triangles, which in turn should
// have a modifiable list of vertices.
// We will make our own copy of the triangles, so we can sort the list of
// them in place.
class TriKdTree3 {
public:
    struct TriBounds3 {
        TriBounds3() {
        }
        inline const Box3f& operator()(const Triangle* a) const {
            return a->bounds();
        }
    };

    struct TriSortPt3 {
        TriSortPt3() {
        }
        inline const V3f& operator()(const Triangle* a) const {
            return a->center();
        }
    };

    typedef SpatialSubd::KdTree3<float, Triangle*, TriBounds3, TriSortPt3> Tree;
    TriKdTree3(TrianglePtrVector& tris);

    const Tree& tree() const {
        return *m_tree;
    }

    ClosestTriangle closestPoint(const V3f& p) const {
        ClosestTriangle cct(p);
        m_tree->constTraverse(cct);
        return cct;
    }

    ClosestTriangle closestPointWithinSquaredDistance(const V3f& p,
                                                      float r2) const {
        ClosestTriangle cct(p, r2);
        m_tree->constTraverse(cct);
        return cct;
    }

    ClosestTriangle closestPointVerbose(const V3f& p,
                                        std::ostream& ostr) const {
        NoisyClosestTriangle cct(p, ostr);
        m_tree->constTraverse(cct);
        return cct;
    }

    ClosestTriangle closestPointWithinSquaredDistanceVerbose(
      const V3f& p, float r2, std::ostream& ostr) const {
        NoisyClosestTriangle cct(p, r2, ostr);
        m_tree->constTraverse(cct);
        return cct;
    }

    AnyTriangle anyPointWithinSquaredDistance(const V3f& p, float r2) const {
        AnyTriangle cct(p, r2);
        m_tree->constTraverse(cct);
        return cct;
    }

    bool intersects(const Box3f& bnds) const {
        BoundsIntersect bi(bnds);
        m_tree->constTraverse(bi);
        return bi.foundAny();
    }

    bool intersects(const V3f& i_start, const V3f& i_end) const {
        SegmentAnyIntersectTraversal st(i_start, i_end);
        m_tree->constTraverse(st);
        return st.found;
    }

    bool firstIntersection(const V3f& i_start,
                           const V3f& i_end,
                           V3f& o_hitPoint,
                           float& o_hitDist) const {
        SegmentFirstIntersectTraversal st(i_start, i_end);
        m_tree->constTraverse(st);
        if (st.found) {
            o_hitPoint = st.bestHit;
            o_hitDist = st.bestDist;
            return true;
        } else {
            return false;
        }
    }

    const Box3f& bounds() const {
        return m_tree->bounds();
    }

protected:
    std::vector<Triangle*> m_triangles;
    std::unique_ptr<Tree> m_tree;
};

}  // End namespace TriMesh
}  // End namespace EmldCore

#endif
