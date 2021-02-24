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

#ifndef _EmldCore_ParallelUtil_Reduce_h_
#define _EmldCore_ParallelUtil_Reduce_h_

#include "Foundation.h"
#include "SimpleArrayFunctors.h"

#include <EmldCore/Util/VectorUtil.h>

#include <OpenEXR/ImathBox.h>
#include <tbb/parallel_reduce.h>

#include <cstddef>
#include <cstdint>
#include <limits>

namespace EmldCore {
namespace ParallelUtil {

//-*****************************************************************************
//-*****************************************************************************
// Reduction is primarily accessed through the wrapper functions
// at the end,
// ParallelVectorMinValue, etc.
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename VALUE_FUNCTOR, typename REDUCE_FUNCTOR>
struct ValueReduceAdaptor {
    typedef VALUE_FUNCTOR value_functor_type;
    typedef typename VALUE_FUNCTOR::value_type value_type;
    typedef typename VALUE_FUNCTOR::index_type index_type;

    typedef REDUCE_FUNCTOR reduce_functor_type;
    typedef typename REDUCE_FUNCTOR::reduce_value_type reduce_value_type;

    typedef tbb::blocked_range<index_type> range_type;

    typedef ValueReduceAdaptor<VALUE_FUNCTOR, REDUCE_FUNCTOR> this_type;

    const VALUE_FUNCTOR& ValueFunctor;
    const REDUCE_FUNCTOR& ReduceFunctor;
    reduce_value_type ReducedValue;

    ValueReduceAdaptor(const VALUE_FUNCTOR& i_valueF,
                       const REDUCE_FUNCTOR& i_reduceF)
      : ValueFunctor(i_valueF)
      , ReduceFunctor(i_reduceF) {
        ReduceFunctor(ReducedValue);
    }

    ValueReduceAdaptor(this_type& i_copy, tbb::split)
      : ValueFunctor(i_copy.ValueFunctor)
      , ReduceFunctor(i_copy.ReduceFunctor) {
        ReduceFunctor(ReducedValue);
    }

    void operator()(const range_type& i_rng) {
        for (index_type i = i_rng.begin(); i < i_rng.end(); ++i) {
            ReduceFunctor(ReducedValue, ValueFunctor[i]);
        }
    }

    void join(const this_type& i_other) {
        ReduceFunctor(ReducedValue, i_other.ReducedValue);
    }

    void execute(index_type i_N) {
        tbb::parallel_reduce(range_type(0, i_N), *this);
    }
};

//-*****************************************************************************
//-*****************************************************************************
// REDUCE FUNCTORS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename VEC_TYPE>
struct BoundsReduceFunctor {
    typedef VEC_TYPE vec_type;
    typedef Imath::Box<VEC_TYPE> box_type;

    typedef box_type reduce_value_type;
    typedef vec_type value_type;

    void operator()(reduce_value_type& o_box) const {
        o_box.makeEmpty();
    }

    void operator()(reduce_value_type& io_box, const value_type& i_vec) const {
        io_box.extendBy(i_vec);
    }

    void operator()(reduce_value_type& io_box,
                    const reduce_value_type& i_box) const {
        io_box.extendBy(i_box);
    }
};

//-*****************************************************************************
template <typename BOX_TYPE>
struct BoundsBoundsReduceFunctor {
    typedef BOX_TYPE box_type;

    typedef box_type reduce_value_type;
    typedef box_type value_type;

    void operator()(reduce_value_type& o_box) const {
        o_box.makeEmpty();
    }

    void operator()(box_type& io_box, const box_type& i_box) const {
        io_box.extendBy(i_box);
    }
};

//-*****************************************************************************
template <typename T>
struct MinReduceFunctor {
    typedef T reduce_value_type;
    typedef T value_type;

    void operator()(reduce_value_type& o_val) const {
        o_val = std::numeric_limits<T>::max();
    }

    void operator()(reduce_value_type& io_val,
                    const value_type& i_other) const {
        io_val = std::min(io_val, i_other);
    }
};

//-*****************************************************************************
template <typename T>
struct MaxReduceFunctor {
    typedef T reduce_value_type;
    typedef T value_type;

    void operator()(reduce_value_type& o_val) const {
#if EMLD_USE_CXX11
        o_val = std::numeric_limits<T>::lowest();
#else
        o_val = -std::numeric_limits<T>::max();
#endif
    }

    void operator()(reduce_value_type& io_val,
                    const value_type& i_other) const {
        io_val = std::max(io_val, i_other);
    }
};

//-*****************************************************************************
template <typename T>
struct MinMaxReduceFunctor {
    typedef std::pair<T, T> reduce_value_type;
    typedef T value_type;

    void operator()(reduce_value_type& o_val) const {
        o_val = reduce_value_type(std::numeric_limits<T>::max(),
                                  std::numeric_limits<T>::lowest());
    }

    void operator()(reduce_value_type& io_val, const value_type& i_val) const {
        io_val.first = std::min(io_val.first, i_val);
        io_val.second = std::max(io_val.second, i_val);
    }

    void operator()(reduce_value_type& io_val,
                    const reduce_value_type& i_other) const {
        io_val.first = std::min(io_val.first, i_other.first);
        io_val.second = std::max(io_val.second, i_other.second);
    }
};

//-*****************************************************************************
template <typename T>
struct AddReduceFunctor {
    typedef T reduce_value_type;
    typedef T value_type;

    void operator()(reduce_value_type& o_val) const {
        set_zero<T>(o_val);
    }

    void operator()(reduce_value_type& io_val,
                    const value_type& i_other) const {
        io_val += i_other;
    }
};

//-*****************************************************************************
//-*****************************************************************************
// FUNCTIONS WHICH USE THESE TO REDUCE
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename VECTOR, typename VALUE_FUNCTOR, typename REDUCE_FUNCTOR>
typename REDUCE_FUNCTOR::reduce_value_type VectorReduce(
  const VECTOR& i_vector) {
    typedef typename std::ptrdiff_t index_type;
    typedef typename VALUE_FUNCTOR::value_type value_type;
    typedef typename REDUCE_FUNCTOR::reduce_value_type reduce_value_type;

    typedef ValueReduceAdaptor<VALUE_FUNCTOR, REDUCE_FUNCTOR> vra_type;
    {
        VALUE_FUNCTOR VF;
        VF.Values = vector_cdata(i_vector);

        REDUCE_FUNCTOR RF;

        vra_type VRA(VF, RF);
        VRA.execute(i_vector.size());

        return VRA.ReducedValue;
    }
};

//-*****************************************************************************
template <typename VALUE_FUNCTOR, typename REDUCE_FUNCTOR>
typename REDUCE_FUNCTOR::reduce_value_type FunctorReduce(
  const VALUE_FUNCTOR& i_VF, std::ptrdiff_t N) {
    typedef typename std::ptrdiff_t index_type;
    typedef typename REDUCE_FUNCTOR::reduce_value_type reduce_value_type;

    typedef ValueReduceAdaptor<VALUE_FUNCTOR, REDUCE_FUNCTOR> vra_type;
    {
        REDUCE_FUNCTOR RF;

        vra_type VRA(i_VF, RF);
        VRA.execute(N);

        return VRA.ReducedValue;
    }
};

//-*****************************************************************************
template <typename VEC_VECTOR>
Imath::Box<typename VEC_VECTOR::value_type> VectorVecBounds(
  const VEC_VECTOR& i_vector) {
    typedef typename VEC_VECTOR::value_type vec_type;
    typedef typename std::ptrdiff_t index_type;

    typedef DirectConstValuesFunctor<vec_type, index_type> VF_type;
    typedef BoundsReduceFunctor<vec_type> RF_type;

    return VectorReduce<VEC_VECTOR, VF_type, RF_type>(i_vector);
}

//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename VEC_VECTOR>
typename VEC_VECTOR::value_type::BaseType VectorMinVecLength2(
  const VEC_VECTOR& i_vector) {
    typedef typename VEC_VECTOR::value_type vec_type;
    typedef typename vec_type::BaseType value_type;
    typedef typename std::ptrdiff_t index_type;

    typedef VecLength2ValuesFunctor<vec_type, index_type> VF_type;
    typedef MinReduceFunctor<value_type> RF_type;

    return VectorReduce<VEC_VECTOR, VF_type, RF_type>(i_vector);
}

//-*****************************************************************************
template <typename VEC_VECTOR>
typename VEC_VECTOR::value_type::BaseType VectorMaxVecLength2(
  const VEC_VECTOR& i_vector) {
    typedef typename VEC_VECTOR::value_type vec_type;
    typedef typename vec_type::BaseType value_type;
    typedef typename std::ptrdiff_t index_type;

    typedef VecLength2ValuesFunctor<vec_type, index_type> VF_type;
    typedef MaxReduceFunctor<value_type> RF_type;

    return VectorReduce<VEC_VECTOR, VF_type, RF_type>(i_vector);
}

//-*****************************************************************************
template <typename VEC_VECTOR>
typename VEC_VECTOR::value_type::BaseType VectorAvgVecLength2(
  const VEC_VECTOR& i_vector) {
    typedef typename VEC_VECTOR::value_type vec_type;
    typedef typename vec_type::BaseType value_type;
    typedef typename std::ptrdiff_t index_type;

    typedef VecLength2ValuesFunctor<vec_type, index_type> VF_type;
    typedef AddReduceFunctor<value_type> RF_type;

    const index_type N = i_vector.size();
    const value_type numer =
      VectorReduce<VEC_VECTOR, VF_type, RF_type>(i_vector);
    if (N > 0) {
        return numer / static_cast<value_type>(N);
    } else {
        return numer;
    }
}

//-*****************************************************************************
template <typename VEC_VECTOR>
typename VEC_VECTOR::value_type::BaseType VectorAvgVecLength(
  const VEC_VECTOR& i_vector) {
    typedef typename VEC_VECTOR::value_type vec_type;
    typedef typename vec_type::BaseType value_type;
    typedef typename std::ptrdiff_t index_type;

    typedef VecLengthValuesFunctor<vec_type, index_type> VF_type;
    typedef AddReduceFunctor<value_type> RF_type;

    const index_type N = i_vector.size();
    const value_type numer =
      VectorReduce<VEC_VECTOR, VF_type, RF_type>(i_vector);
    if (N > 0) {
        return numer / static_cast<value_type>(N);
    } else {
        return numer;
    }
}

//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename FUNCTOR>
typename FUNCTOR::value_type FunctorMin(const FUNCTOR& i_F, std::ptrdiff_t N) {
    typedef typename FUNCTOR::value_type value_type;

    typedef MinReduceFunctor<value_type> RF_type;
    return FunctorReduce<FUNCTOR, RF_type>(i_F, N);
}

//-*****************************************************************************
template <typename FUNCTOR>
typename FUNCTOR::value_type FunctorMax(const FUNCTOR& i_F, std::ptrdiff_t N) {
    typedef typename FUNCTOR::value_type value_type;

    typedef MaxReduceFunctor<value_type> RF_type;
    return FunctorReduce<FUNCTOR, RF_type>(i_F, N);
}

//-*****************************************************************************
template <typename VECTOR>
typename VECTOR::value_type VectorMin(const VECTOR& i_vector) {
    typedef typename VECTOR::value_type value_type;
    typedef typename std::ptrdiff_t index_type;

    typedef DirectConstValuesFunctor<value_type, index_type> VF_type;
    typedef MinReduceFunctor<value_type> RF_type;

    return VectorReduce<VECTOR, VF_type, RF_type>(i_vector);
}

//-*****************************************************************************
template <typename VECTOR>
typename VECTOR::value_type VectorMax(const VECTOR& i_vector) {
    typedef typename VECTOR::value_type value_type;
    typedef typename std::ptrdiff_t index_type;

    typedef DirectConstValuesFunctor<value_type, index_type> VF_type;
    typedef MaxReduceFunctor<value_type> RF_type;

    return VectorReduce<VECTOR, VF_type, RF_type>(i_vector);
}

//-*****************************************************************************
template <typename VECTOR, typename AVERAGE_TYPE>
AVERAGE_TYPE VectorAverage(const VECTOR& i_vector) {
    typedef typename VECTOR::value_type value_type;
    typedef typename std::ptrdiff_t index_type;
    typedef AVERAGE_TYPE reduce_value_type;

    typedef DirectConstValuesFunctor<value_type, index_type> VF_type;
    typedef AddReduceFunctor<reduce_value_type> RF_type;

    const index_type N = i_vector.size();
    const reduce_value_type numer =
      VectorReduce<VECTOR, VF_type, RF_type>(i_vector);
    if (N > 0) {
        return numer / static_cast<reduce_value_type>(N);
    } else {
        return numer;
    }
}

//-*****************************************************************************
template <typename VECTOR>
typename VECTOR::value_type VectorAverageValue(const VECTOR& i_vector) {
    typedef typename VECTOR::value_type value_type;
    return VectorAverage<VECTOR, value_type>(i_vector);
}

//-*****************************************************************************
template <typename VECTOR>
std::pair<typename VECTOR::value_type, typename VECTOR::value_type>
VectorMinMax(const VECTOR& i_vector) {
    typedef typename VECTOR::value_type value_type;
    typedef typename std::ptrdiff_t index_type;

    typedef DirectConstValuesFunctor<value_type, index_type> VF_type;
    typedef MinMaxReduceFunctor<value_type> RF_type;

    return VectorReduce<VECTOR, VF_type, RF_type>(i_vector);
}

}  // End namespace ParallelUtil
}  // End namespace EmldCore

#endif
