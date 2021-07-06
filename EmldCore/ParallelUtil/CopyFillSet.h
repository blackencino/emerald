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

#ifndef _EmldCore_ParallelUtil_CopyFillSet_h_
#define _EmldCore_ParallelUtil_CopyFillSet_h_

#include "For.h"
#include "Foundation.h"
#include "SimpleArrayFunctors.h"

#include <EmldCore/Util/Foundation.h>
#include <EmldCore/Util/VectorUtil.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace EmldCore {
namespace ParallelUtil {

//-*****************************************************************************
//-*****************************************************************************
// PARALLEL VECTOR COPYING.
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename SRC_T, typename DST_T>
struct PtrCopyFunctor : public ForEachFunctor<PtrCopyFunctor<SRC_T, DST_T> > {
    const SRC_T* SrcBegin;
    DST_T* DstBegin;

    typedef PtrCopyFunctor<SRC_T, DST_T> this_type;

    void operator()(typename this_type::index_type i_begin,
                    typename this_type::index_type i_end) const {
        std::copy(SrcBegin + i_begin,
                  SrcBegin + i_end,
                  const_cast<DST_T*>(DstBegin) + i_begin);
    }
};

//-*****************************************************************************
template <typename SRC_ITER, typename DST_ITER>
struct IterCopyFunctor
  : public ForEachFunctor<IterCopyFunctor<SRC_ITER, DST_ITER> > {
    SRC_ITER SrcBegin;
    DST_ITER DstBegin;

    typedef IterCopyFunctor<SRC_ITER, DST_ITER> this_type;

    void operator()(typename this_type::index_type i_begin,
                    typename this_type::index_type i_end) const {
        std::copy(SrcBegin + i_begin, SrcBegin + i_end, DstBegin + i_begin);
    }
};

//-*****************************************************************************
template <typename SRCVEC, typename DSTVEC>
void VectorCopy(const SRCVEC& i_src, DSTVEC& o_dst) {
    typedef typename std::ptrdiff_t index_type;
    typedef typename SRCVEC::const_iterator src_iter_type;
    typedef typename DSTVEC::iterator dst_iter_type;

    const index_type N = i_src.size();
    o_dst.resize(N);
    if (N < 1) { return; }
#if 1
    IterCopyFunctor<src_iter_type, dst_iter_type> F;
    F.SrcBegin = i_src.begin();
    F.DstBegin = o_dst.begin();
    F.execute(N);
#else
    std::copy(i_src.begin(), i_src.end(), o_dst.begin());
#endif
}

//-*****************************************************************************
//-*****************************************************************************
// PARALLEL VECTOR FILLING.
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename SRC_VALUE, typename DST_T>
struct PtrFillFunctor
  : public ForEachFunctor<PtrFillFunctor<SRC_VALUE, DST_T> > {
    SRC_VALUE SrcValue;
    DST_T* DstBegin;

    typedef PtrFillFunctor<SRC_VALUE, DST_T> this_type;

    void operator()(typename this_type::index_type i_begin,
                    typename this_type::index_type i_end) const {
        std::fill(const_cast<DST_T*>(DstBegin) + i_begin,
                  const_cast<DST_T*>(DstBegin) + i_end,
                  SrcValue);
    }
};

//-*****************************************************************************
template <typename SRC_VALUE, typename DST_ITER>
struct IterFillFunctor
  : public ForEachFunctor<IterFillFunctor<SRC_VALUE, DST_ITER> > {
    SRC_VALUE SrcValue;
    DST_ITER DstBegin;

    typedef IterFillFunctor<SRC_VALUE, DST_ITER> this_type;

    void operator()(typename this_type::index_type i_begin,
                    typename this_type::index_type i_end) const {
        std::fill(DstBegin + i_begin, DstBegin + i_end, SrcValue);
    }
};

//-*****************************************************************************
template <typename SRCVAL, typename DSTVEC>
void VectorFill(DSTVEC& o_dst, const SRCVAL& i_src) {
    typedef typename std::ptrdiff_t index_type;
    typedef typename DSTVEC::iterator dst_iter_type;

    const index_type N = o_dst.size();
    if (N < 1) { return; }
#if 1
    IterFillFunctor<SRCVAL, dst_iter_type> F;
    F.SrcValue = i_src;
    F.DstBegin = o_dst.begin();
    F.execute(N);
#else
    std::fill(o_dst.begin(), o_dst.end(), i_src);
#endif
}

//-*****************************************************************************
// Setting the contents of a vector to bitwise zero. For any other form of
// zero, just use VectorFill above.
template <typename DSTVEC>
void VectorZeroBits(DSTVEC& o_dst) {
    typedef typename DSTVEC::value_type value_type;

    static const zero_bits<value_type> zero;

    VectorFill(o_dst, zero.bits);
}

//-*****************************************************************************
//-*****************************************************************************
// PARALLEL SET ORDERED INDICES
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename INDEX_TYPE = std::ptrdiff_t>
struct SetOrderedIndicesFunctor
  : public ZeroForEachFunctorI<SetOrderedIndicesFunctor<INDEX_TYPE>,
                               INDEX_TYPE> {
    INDEX_TYPE* Indices;

    void operator()(INDEX_TYPE i) const {
        Indices[i] = i;
    }
};

//-*****************************************************************************
template <typename VECTOR>
void VectorSetOrderedIndices(VECTOR& o_indices) {
    typedef typename VECTOR::value_type index_type;

    const std::size_t N = o_indices.size();
    if (N < 1) { return; }

    SetOrderedIndicesFunctor<index_type> F;
    F.Indices = vector_data(o_indices);
    F.execute(static_cast<index_type>(N));
}

}  // End namespace ParallelUtil
}  // End namespace EmldCore

#endif
