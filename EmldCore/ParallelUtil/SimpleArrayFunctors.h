//-*****************************************************************************
// Copyright (c) 2001-2013, Christopher Jon Horvath. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of Christopher Jon Horvath nor the names of his
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//-*****************************************************************************

#ifndef _EmldCore_ParallelUtil_SimpleArrayFunctors_h_
#define _EmldCore_ParallelUtil_SimpleArrayFunctors_h_

#include "Foundation.h"

#include <EmldCore/Util/Foundation.h>

#include <cstddef>
#include <cstdint>
#include <utility>

namespace EmldCore {
namespace ParallelUtil {

//-*****************************************************************************
template <typename VALUE_TYPE, typename INDEX_TYPE = std::ptrdiff_t>
struct ValuesFunctor {
    typedef VALUE_TYPE value_type;
    typedef INDEX_TYPE index_type;
    typedef INDEX_TYPE difference_type;
    typedef INDEX_TYPE size_type;
};

//-*****************************************************************************
template <typename DERIVED,
          typename VALUE_TYPE,
          typename INDEX_TYPE = std::ptrdiff_t>
struct ZeroValuesFunctor : public EmldCore::Util::AllZeroConstructor<DERIVED>,
                           public ValuesFunctor<VALUE_TYPE, INDEX_TYPE> {
    // Nothing
};

//-*****************************************************************************
template <typename T, typename INDEX_TYPE = std::ptrdiff_t>
struct DirectValuesFunctor
  : public AllZeroConstructor<DirectValuesFunctor<T, INDEX_TYPE> > {
    typedef T value_type;
    typedef INDEX_TYPE index_type;
    value_type* Values;

    value_type& operator[](index_type i) {
        return Values[i];
    }

    const value_type& operator[](index_type i) const {
        return Values[i];
    }
};

//-*****************************************************************************
template <typename T, typename INDEX_TYPE = std::ptrdiff_t>
struct DirectConstValuesFunctor
  : public AllZeroConstructor<DirectConstValuesFunctor<T, INDEX_TYPE> > {
    typedef T value_type;
    typedef INDEX_TYPE index_type;
    const value_type* Values;

    const value_type& operator[](index_type i) const {
        return Values[i];
    }
};

//-*****************************************************************************
template <typename INDEX_TYPE = std::ptrdiff_t>
struct IndexValuesFunctor {
    typedef INDEX_TYPE value_type;
    typedef INDEX_TYPE index_type;

    value_type operator[](index_type i) const {
        return i;
    }
};

//-*****************************************************************************
template <typename VEC, typename INDEX_TYPE = std::ptrdiff_t>
struct VecLength2ValuesFunctor
  : public AllZeroConstructor<VecLength2ValuesFunctor<VEC, INDEX_TYPE> > {
    typedef VEC vec_type;
    typedef typename VEC::BaseType value_type;
    typedef INDEX_TYPE index_type;
    const vec_type* Values;

    value_type operator[](index_type i) const {
        return Values[i].length2();
    }
};

//-*****************************************************************************
template <typename VEC, typename INDEX_TYPE = std::ptrdiff_t>
struct VecAndLength2ValuesFunctor
  : public AllZeroConstructor<VecAndLength2ValuesFunctor<VEC, INDEX_TYPE> > {
    typedef VEC vec_type;
    typedef typename VEC::BaseType vec_base_type;
    typedef std::pair<vec_type, vec_base_type> value_type;
    typedef INDEX_TYPE index_type;
    const vec_type* Values;

    value_type operator[](index_type i) const {
        const vec_type v = Values[i];
        return value_type(v, v.length2());
    }
};

//-*****************************************************************************
template <typename VEC, typename INDEX_TYPE = std::ptrdiff_t>
struct VecLengthValuesFunctor
  : public AllZeroConstructor<VecLengthValuesFunctor<VEC, INDEX_TYPE> > {
    typedef VEC vec_type;
    typedef typename VEC::BaseType value_type;
    typedef INDEX_TYPE index_type;
    const vec_type* Values;

    value_type operator[](index_type i) const {
        return Values[i].length();
    }
};

//-*****************************************************************************
template <typename VEC, typename INDEX_TYPE = std::ptrdiff_t>
struct VecAndLengthValuesFunctor
  : public AllZeroConstructor<VecAndLengthValuesFunctor<VEC, INDEX_TYPE> > {
    typedef VEC vec_type;
    typedef typename VEC::BaseType vec_base_type;
    typedef std::pair<vec_type, vec_base_type> value_type;
    typedef INDEX_TYPE index_type;
    const vec_type* Values;

    value_type operator[](index_type i) const {
        const vec_type v = Values[i];
        return value_type(v, v.length());
    }
};

}  // End namespace ParallelUtil
}  // End namespace EmldCore

#endif
