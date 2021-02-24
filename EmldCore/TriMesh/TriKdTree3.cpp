#include "TriKdTree3.h"

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
TriKdTree3::TriKdTree3(TrianglePtrVector& tris) {
    m_triangles.resize(tris.size());
    for (int i = 0; i < tris.size(); ++i) { m_triangles[i] = tris[i].get(); }

    TriBounds3 tb3;
    TriSortPt3 tsp3;
    m_tree.reset(new Tree(m_triangles, tb3, tsp3));
}

}  // End namespace TriMesh
}  // End namespace EmldCore
