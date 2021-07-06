#ifndef _EmldCore_TriMesh_Foundation_h_
#define _EmldCore_TriMesh_Foundation_h_


#pragma warning(push)
#pragma warning( disable : 4244 4100 4456 )

#include <EmldCore/Util/Exception.h>

#include <OpenEXR/ImathBox.h>
#include <OpenEXR/ImathMatrix.h>
#include <OpenEXR/ImathVec.h>
#include <boost/range/iterator_range.hpp>

#include <vector>

namespace EmldCore {
namespace TriMesh {

//-*****************************************************************************
#define TRIMESH_THROW(TEXT) EMLD_FAIL(TEXT)
#define TRIMESH_ASSERT(COND, TEXT) EMLD_ASSERT(COND, TEXT)

//-*****************************************************************************
// Bring in Imath types!
using Imath::Box2d;
using Imath::Box2f;
using Imath::Box3d;
using Imath::Box3f;
using Imath::M33f;
using Imath::M44d;
using Imath::M44f;
using Imath::V2f;
using Imath::V2i;
using Imath::V3d;
using Imath::V3f;

//-*****************************************************************************
// Make some standard vectors
typedef std::vector<V3f> V3fVector;
typedef std::vector<int> IntVector;
typedef std::vector<unsigned int> UintVector;
typedef std::vector<float> FloatVector;

//-*****************************************************************************
// We use ranges as a way of passing arrays of data around, without explicitly
// declaring them as vectors or any other storage.
typedef boost::iterator_range<const V3f*> ConstV3fRange;
typedef boost::iterator_range<const int*> ConstIntRange;
typedef boost::iterator_range<const unsigned int*> ConstUintRange;

//-*****************************************************************************
template <class T>
inline boost::iterator_range<const T*> CreateRange(const std::vector<T>& iVec) {
    if (iVec.empty()) {
        return boost::iterator_range<const T*>((const T*)NULL, (const T*)NULL);
    } else {
        return boost::iterator_range<const T*>(&iVec.front(),
                                               iVec.size() + &iVec.front());
    }
}

//-*****************************************************************************
inline ConstV3fRange CreateV3fRange(const V3fVector& iVec) {
    return CreateRange<V3f>(iVec);
}

//-*****************************************************************************
inline ConstIntRange CreateIntRange(const IntVector& iVec) {
    return CreateRange<int>(iVec);
}

//-*****************************************************************************
inline ConstUintRange CreateUintRange(const UintVector& iVec) {
    return CreateRange<unsigned int>(iVec);
}

//-*****************************************************************************
template <class T, class RANGE>
inline const T* GetBeginPointer(const RANGE& iRng) {
    if (iRng.empty()) {
        return (const T*)NULL;
    } else {
        return &(iRng.front());
    }
}

//-*****************************************************************************
template <class RANGE>
inline const V3f* GetBeginV3fPointer(const RANGE& iRng) {
    return GetBeginPointer<V3f, RANGE>(iRng);
}

//-*****************************************************************************
template <class RANGE>
inline const int* GetBeginIntPointer(const RANGE& iRng) {
    return GetBeginPointer<int, RANGE>(iRng);
}

//-*****************************************************************************
template <class RANGE>
inline const unsigned int* GetBeginUintPointer(const RANGE& iRng) {
    return GetBeginPointer<unsigned int, RANGE>(iRng);
}

}  // End namespace TriMesh
}  // End namespace EmldCore

#pragma warning(pop)

#endif

