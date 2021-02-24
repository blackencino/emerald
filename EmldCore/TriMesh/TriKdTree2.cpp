#include "TriKdTree2.h"

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
TriKdTree2::TriKdTree2(TrianglePtrVector& tris) {
    m_triangles.resize(tris.size());
    for (int i = 0; i < tris.size(); ++i) { m_triangles[i] = tris[i].get(); }

    TriBounds2 tb2;
    TriSortPt2 tsp2;
    m_tree.reset(new Tree(m_triangles, tb2, tsp2));
}

}  // End namespace TriMesh
}  // End namespace EmldCore
