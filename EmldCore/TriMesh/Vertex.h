#ifndef _EmldCore_TriMesh_Vertex_h_
#define _EmldCore_TriMesh_Vertex_h_

#include "Foundation.h"

#include <vector>

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
class Vertex {
public:
    Vertex()
      : m_points()
      , m_velocities()
      , m_index(-1) {
    }

    Vertex(V3fVector& pnts, V3fVector& vels, int idx)
      : m_points(&pnts)
      , m_velocities(&vels)
      , m_index(idx) {
    }

    // Default copy & assignment operator
    // ...

    // Don't allow direct modification of points vector
    const V3fVector& points() const {
        return *m_points;
    }
    const V3fVector& velocities() const {
        return *m_velocities;
    }

    // Const reference to position
    const V3f& position() const {
        return (*m_points)[m_index];
    }

    // Const reference to velocity
    const V3f& velocity() const {
        return (*m_velocities)[m_index];
    }

    // Editable position should be done explicitly.
    void setPosition(const V3f& np) {
        (*m_points)[m_index] = np;
    }

    // Editable velocity should be done explicitly.
    void setVelocity(const V3f& nv) {
        (*m_velocities)[m_index] = nv;
    }

    int index() const {
        return m_index;
    }

protected:
    V3fVector* m_points;
    V3fVector* m_velocities;
    int m_index;
};

//-*****************************************************************************
typedef std::vector<Vertex> VertexVector;

}  // End namespace TriMesh
}  // End namespace EmldCore

#endif
