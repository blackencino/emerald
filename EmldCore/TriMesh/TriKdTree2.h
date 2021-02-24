#ifndef _EmldCore_TriMesh_TriKdTree2_h_
#define _EmldCore_TriMesh_TriKdTree2_h_

#include "Foundation.h"
#include "Triangle.h"
#include "Vertex.h"

#include <EmldCore/SpatialSubd/KdTree2.h>

#include <memory>
#include <vector>

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
// Because the KdTree will be used to gather points for potential modification,
// we need to have a modifiable list of triangles, which in turn should
// have a modifiable list of vertices.
// We will make our own copy of the triangles, so we can sort the list of
// them in place.
class TriKdTree2 {
public:
    // We know we're constructing this from what TriMesh stores.
    TriKdTree2(TrianglePtrVector& tris);

protected:
    struct HitCountTraverser {
        HitCountTraverser(const V2f& p2, float d)
          : point(p2)
          , depth(d)
          , count(0) {
        }

        bool validBounds(const Box2f& bnds) const {
            return bnds.intersects(point);
        }

        void leaf(const Triangle* tri) {
            count += (int)(tri->hit2D(point, depth));
        }

        // Need to check all.
        bool unfinished() const {
            return true;
        }

        V2f point;
        float depth;
        int count;
    };

    struct XRayIsectTraverser {
        XRayIsectTraverser(const V2f& p2, FloatVector& xInts)
          : point(p2)
          , xIntersections(xInts) {
        }

        bool validBounds(const Box2f& bnds) const {
            return bnds.intersects(point);
        }

        void leaf(const Triangle* tri) {
            float tmpD;
            if (tri->hitDepth2D(point, tmpD)) {
                xIntersections.push_back(tmpD);
            }
        }

        bool unfinished() const {
            return true;
        }

        V2f point;
        FloatVector& xIntersections;
    };

public:
    int hitCount(const V2f& p2, float d) const {
        HitCountTraverser hct(p2, d);
        m_tree->constTraverse(hct);
        return hct.count;
    }

    void xRayIntersections(const V2f& p2, FloatVector& xInts) const {
        XRayIsectTraverser xrt(p2, xInts);
        m_tree->constTraverse(xrt);
    }

    const Box2f& bounds() const {
        return m_tree->bounds();
    }

protected:
    struct TriBounds2 {
        TriBounds2() {
        }
        inline const Box2f& operator()(const Triangle* a) const {
            return a->bounds2D();
        }
    };

    struct TriSortPt2 {
        TriSortPt2() {
        }
        inline const V2f& operator()(const Triangle* a) const {
            return a->center2D();
        }
    };

    typedef SpatialSubd::KdTree2<float, Triangle*, TriBounds2, TriSortPt2> Tree;

    std::vector<Triangle*> m_triangles;
    std::unique_ptr<Tree> m_tree;
};

}  // End namespace TriMesh
}  // End namespace EmldCore

#endif
