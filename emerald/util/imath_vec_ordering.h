#pragma once

#include <emerald/util/foundation.h>

#ifdef IMATH_INTERNAL_NAMESPACE_HEADER_ENTER
IMATH_INTERNAL_NAMESPACE_HEADER_ENTER
#else
namespace Imath {
#endif

//-*****************************************************************************
//-*****************************************************************************
// VEC2 COMPARISONS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
bool operator<(Vec2<T> const& A, Vec2<T> const& B) {
    if (A.y < B.y) {
        return true;
    } else if (A.y > B.y) {
        return false;
    } else {
        return (A.x < B.x);
    }
}

//-*****************************************************************************
template <typename T>
bool operator<=(Vec2<T> const& A, Vec2<T> const& B) {
    if (A.y < B.y) {
        return true;
    } else if (A.y > B.y) {
        return false;
    } else {
        return (A.x <= B.x);
    }
}

//-*****************************************************************************
template <typename T>
bool operator>(Vec2<T> const& A, Vec2<T> const& B) {
    if (A.y > B.y) {
        return true;
    } else if (A.y < B.y) {
        return false;
    } else {
        return (A.x > B.x);
    }
}

//-*****************************************************************************
template <typename T>
bool operator>=(Vec2<T> const& A, Vec2<T> const& B) {
    if (A.y > B.y) {
        return true;
    } else if (A.y < B.y) {
        return false;
    } else {
        return (A.x >= B.x);
    }
}

//-*****************************************************************************
//-*****************************************************************************
// VEC3 COMPARISONS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
bool operator<(Vec3<T> const& A, Vec3<T> const& B) {
    if (A.z < B.z) {
        return true;
    } else if (A.z > B.z) {
        return false;
    } else if (A.y < B.y) {
        return true;
    } else if (A.y > B.y) {
        return false;
    } else {
        return (A.x < B.x);
    }
}

//-*****************************************************************************
template <typename T>
bool operator<=(Vec3<T> const& A, Vec3<T> const& B) {
    if (A.z < B.z) {
        return true;
    } else if (A.z > B.z) {
        return false;
    } else if (A.y < B.y) {
        return true;
    } else if (A.y > B.y) {
        return false;
    } else {
        return (A.x <= B.x);
    }
}

//-*****************************************************************************
template <typename T>
bool operator>(Vec3<T> const& A, Vec3<T> const& B) {
    if (A.z > B.z) {
        return true;
    } else if (A.z < B.z) {
        return false;
    } else if (A.y > B.y) {
        return true;
    } else if (A.y < B.y) {
        return false;
    } else {
        return (A.x > B.x);
    }
}

//-*****************************************************************************
template <typename T>
bool operator>=(Vec3<T> const& A, Vec3<T> const& B) {
    if (A.z > B.z) {
        return true;
    } else if (A.z < B.z) {
        return false;
    } else if (A.y > B.y) {
        return true;
    } else if (A.y < B.y) {
        return false;
    } else {
        return (A.x >= B.x);
    }
}

//-*****************************************************************************
//-*****************************************************************************
// VEC4 COMPARISONS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
bool operator<(Vec4<T> const& A, Vec4<T> const& B) {
    if (A[3] < B[3]) {
        return true;
    } else if (A[3] > B[3]) {
        return false;
    } else if (A.z < B.z) {
        return true;
    } else if (A.z > B.z) {
        return false;
    } else if (A.y < B.y) {
        return true;
    } else if (A.y > B.y) {
        return false;
    } else {
        return (A.x < B.x);
    }
}

//-*****************************************************************************
template <typename T>
bool operator<=(Vec4<T> const& A, Vec4<T> const& B) {
    if (A[3] < B[3]) {
        return true;
    } else if (A[3] > B[3]) {
        return false;
    } else if (A.z < B.z) {
        return true;
    } else if (A.z > B.z) {
        return false;
    } else if (A.y < B.y) {
        return true;
    } else if (A.y > B.y) {
        return false;
    } else {
        return (A.x <= B.x);
    }
}

//-*****************************************************************************
template <typename T>
bool operator>(Vec4<T> const& A, Vec4<T> const& B) {
    if (A[3] > B[3]) {
        return true;
    } else if (A[3] < B[3]) {
        return false;
    } else if (A.z > B.z) {
        return true;
    } else if (A.z < B.z) {
        return false;
    } else if (A.y > B.y) {
        return true;
    } else if (A.y < B.y) {
        return false;
    } else {
        return (A.x > B.x);
    }
}

//-*****************************************************************************
template <typename T>
bool operator>=(Vec4<T> const& A, Vec4<T> const& B) {
    if (A[3] > B[3]) {
        return true;
    } else if (A[3] < B[3]) {
        return false;
    } else if (A.z > B.z) {
        return true;
    } else if (A.z < B.z) {
        return false;
    } else if (A.y > B.y) {
        return true;
    } else if (A.y < B.y) {
        return false;
    } else {
        return (A.x >= B.x);
    }
}

#ifdef IMATH_INTERNAL_NAMESPACE_HEADER_EXIT
IMATH_INTERNAL_NAMESPACE_HEADER_EXIT
#else
}  // End namespace Imath
#endif
