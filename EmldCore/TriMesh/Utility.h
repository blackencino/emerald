#ifndef _EmldCore_TriMesh_Utility_h_
#define _EmldCore_TriMesh_Utility_h_

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
// Handy convenience function used in multiple places
//
// A function which takes three values and rotates them so that the
// smallest is in the zero spot, but does not change their winding order.
template <typename T>
void rotateMinToZero(T v[]) {
    T tmp;
    if (v[0] < v[1]) {
        if (v[0] < v[2]) {
            // Nothing
        } else {
            // V2 is the lowest. rotate.
            tmp = v[0];
            v[0] = v[2];
            v[2] = v[1];
            v[1] = tmp;
        }
    } else {
        if (v[1] < v[2]) {
            // V1 is the lowest. rotate.
            tmp = v[0];
            v[0] = v[1];
            v[1] = v[2];
            v[2] = tmp;
        } else {
            // V2 is the lowest. rotate.
            tmp = v[0];
            v[0] = v[2];
            v[2] = v[1];
            v[1] = tmp;
        }
    }
}

}  // End namespace TriMesh
}  // End namespace EmldCore

#endif
