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

#ifndef _EmldCore_ParallelUtil_Scan_h_
#define _EmldCore_ParallelUtil_Scan_h_

#include "Foundation.h"
#include "SimpleArrayFunctors.h"

#include <EmldCore/Util/VectorUtil.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_scan.h>

namespace EmldCore {
namespace ParallelUtil {

//-*****************************************************************************
//-*****************************************************************************
// A scan of a sequence is a process whereby some accumulation occurs
// of value at the previous index in the list and the current index.
// The most common is the "prefix_sum", in which the values of the sequence
// are added.
// The prefix_sum value at index 0 is equal to the sequence value at index 0.
// The prefix_sum value at index i is the sum of the prefix_sum at index i-1
// and the sequence value at index i.
// Any accumulation can be applied here, and the accumulations can be
// ordered, like a subtraction
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
//-*****************************************************************************
// THE ADAPTOR - this adapts our preferred functor syntax to the TBB
// parallel_scan interface.
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename VALUE_FUNCTOR, typename SCAN_FUNCTOR>
struct ValueScanAdaptor {
    typedef VALUE_FUNCTOR value_functor_type;
    typedef typename VALUE_FUNCTOR::value_type value_type;
    typedef typename VALUE_FUNCTOR::index_type index_type;

    typedef SCAN_FUNCTOR scan_functor_type;
    typedef typename SCAN_FUNCTOR::scan_value_type scan_value_type;

    typedef tbb::blocked_range<index_type> range_type;

    typedef ValueScanAdaptor<VALUE_FUNCTOR, SCAN_FUNCTOR> this_type;

    VALUE_FUNCTOR& ValueFunctor;
    SCAN_FUNCTOR& ScanFunctor;
    scan_value_type* ScannedValues;
    scan_value_type ScannedValue;

    ValueScanAdaptor(VALUE_FUNCTOR& i_valueFunctor,
                     SCAN_FUNCTOR& i_scanFunctor,
                     scan_value_type* o_scannedValues)
      : ValueFunctor(i_valueFunctor)
      , ScanFunctor(i_scanFunctor)
      , ScannedValues(o_scannedValues) {
        ScanFunctor(ScannedValue);
    }

    ValueScanAdaptor(this_type& i_copy, tbb::split)
      : ValueFunctor(i_copy.ValueFunctor)
      , ScanFunctor(i_copy.ScanFunctor)
      , ScannedValues(i_copy.ScannedValues) {
        ScanFunctor(ScannedValue);
    }

    // Called by parallel scan.
    template <typename TAG>
    void operator()(const range_type& i_range, TAG i_tag) {
        scan_value_type temp = ScannedValue;
        for (index_type i = i_range.begin(); i < i_range.end(); ++i) {
            ScanFunctor(temp, ValueFunctor[i]);
            if (TAG::is_final_scan()) { ScannedValues[i] = temp; }
        }
        ScannedValue = temp;
    }

    void reverse_join(this_type& i_other) {
        ScanFunctor(ScannedValue, i_other.ScannedValue);
    }

    void assign(this_type& i_other) {
        ScannedValue = i_other.ScannedValue;
    }

    void execute(index_type i_N) {
        tbb::parallel_scan(range_type(0, i_N), *this);
    }
};

//-*****************************************************************************
//-*****************************************************************************
// SOME SCAN FUNCTORS
// Honestly, the only one I can think of is add.
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
struct AddScanFunctor {
    typedef T scan_value_type;
    typedef T value_type;

    void operator()(scan_value_type& o_val) const {
        set_zero<T>(o_val);
    }

    void operator()(scan_value_type& io_val, const value_type& i_other) const {
        io_val += i_other;
    }
};

//-*****************************************************************************
//-*****************************************************************************
// FUNCTIONS WHICH USE THESE TO REDUCE
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename VALUE_FUNCTOR, typename SCAN_VECTOR, typename SCAN_FUNCTOR>
typename SCAN_FUNCTOR::scan_value_type FunctorVectorScan(
  VALUE_FUNCTOR& i_vfunc, SCAN_VECTOR& o_scanVector) {
    typedef typename VALUE_FUNCTOR::index_type index_type;
    typedef typename VALUE_FUNCTOR::value_type value_type;
    typedef typename SCAN_FUNCTOR::scan_value_type scan_value_type;

    typedef ValueScanAdaptor<VALUE_FUNCTOR, SCAN_FUNCTOR> VSA_type;
    {
        SCAN_FUNCTOR SF;

        VSA_type VSA(i_vfunc, SF, vector_data(o_scanVector));
        VSA.execute(o_scanVector.size());

        return VSA.ScannedValue;
    }
};

//-*****************************************************************************
template <typename VECTOR,
          typename SCAN_VECTOR,
          typename VALUE_FUNCTOR,
          typename SCAN_FUNCTOR>
typename SCAN_FUNCTOR::scan_value_type VectorScan(const VECTOR& i_vector,
                                                  SCAN_VECTOR& o_scanVector) {
    typedef typename std::ptrdiff_t index_type;
    typedef typename VALUE_FUNCTOR::value_type value_type;
    typedef typename SCAN_FUNCTOR::scan_value_type scan_value_type;

    // Make sure output vector is the right size.
    const index_type N = i_vector.size();
    o_scanVector.resize(N);

    typedef ValueScanAdaptor<VALUE_FUNCTOR, SCAN_FUNCTOR> VSA_type;
    {
        VALUE_FUNCTOR VF;
        VF.Values = vector_cdata(i_vector);

        SCAN_FUNCTOR SF;

        VSA_type VSA(VF, SF, vector_data(o_scanVector));
        VSA.execute(N);

        return VSA.ScannedValue;
    }
};

//-*****************************************************************************
//-*****************************************************************************
// WRAPPER FUNCTIONS
//-*****************************************************************************
//-*****************************************************************************
template <typename VECTOR, typename SCAN_VECTOR>
typename SCAN_VECTOR::value_type VectorPrefixSum(const VECTOR& i_vector,
                                                 SCAN_VECTOR& o_prefixSums) {
    typedef typename VECTOR::value_type value_type;
    typedef typename std::ptrdiff_t index_type;
    typedef typename SCAN_VECTOR::value_type scan_value_type;

    typedef DirectConstValuesFunctor<value_type, index_type> VF_type;
    typedef AddScanFunctor<scan_value_type> SF_type;

    // This will resize prefixSums vector accordingly.
    return VectorScan<VECTOR, SCAN_VECTOR, VF_type, SF_type>(i_vector,
                                                             o_prefixSums);
}

//-*****************************************************************************
template <typename VALUE_FUNCTOR, typename SCAN_VECTOR>
typename SCAN_VECTOR::value_type FunctorVectorPrefixSum(
  VALUE_FUNCTOR& i_vfunc, SCAN_VECTOR& o_prefixSums) {
    typedef typename SCAN_VECTOR::value_type scan_value_type;

    typedef AddScanFunctor<scan_value_type> SF_type;

    // This will actually take its size from the output prefix sums.
    return FunctorVectorScan<VALUE_FUNCTOR, SCAN_VECTOR, SF_type>(i_vfunc,
                                                                  o_prefixSums);
}

}  // End namespace ParallelUtil
}  // End namespace EmldCore

#endif
