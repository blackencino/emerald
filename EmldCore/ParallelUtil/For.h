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

#ifndef _EmldCore_ParallelUtil_For_h_
#define _EmldCore_ParallelUtil_For_h_

#include "Foundation.h"

namespace EmldCore {
namespace ParallelUtil {

//-*****************************************************************************
//-*****************************************************************************
// GLOBAL_SERIAL SWITCH FOR PARALLEL FOR
//-*****************************************************************************
//-*****************************************************************************
template <typename RANGE, typename ADAPTOR>
void PFOR( const RANGE& i_range, ADAPTOR& i_functor )
{
#if 1
    tbb::parallel_for( i_range, i_functor );
#else
    i_functor( i_range );
#endif
}

//-*****************************************************************************
//-*****************************************************************************
// ADAPTORS. By sharing a reference to the core functor, they avoid
// copy construction if the core functor is big.
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename FUNCTOR, typename INDEX_TYPE = std::ptrdiff_t>
struct RangeAdaptor
{
    typedef INDEX_TYPE index_type;
    typedef tbb::blocked_range<index_type> range_type;
    FUNCTOR& functor;

    RangeAdaptor( FUNCTOR& i_functor ) : functor( i_functor ) {}

    void operator()( const range_type& i_range ) const
    {
        functor( i_range.begin(), i_range.end() );
    }
};

//-*****************************************************************************
template <typename FUNCTOR, typename INDEX_TYPE = std::ptrdiff_t>
struct IndexAdaptor
{
    typedef INDEX_TYPE index_type;
    typedef tbb::blocked_range<index_type> range_type;
    FUNCTOR& functor;

    IndexAdaptor( FUNCTOR& i_functor ) : functor( i_functor ) {}

    void operator()( const range_type& i_range ) const
    {
        for ( index_type i = i_range.begin(); i < i_range.end(); ++i )
        {
            functor( i );
        }
    }
};

//-*****************************************************************************
//-*****************************************************************************
// SOME FUNCTOR BASE CLASSES
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template < typename DERIVED, typename ADAPTOR >
struct ForEachFunctorAdapted
{
    typedef DERIVED derived_type;
    typedef ADAPTOR adaptor_type;
    typedef typename ADAPTOR::index_type index_type;
    typedef tbb::blocked_range<index_type> range_type;

    index_type N;

    ForEachFunctorAdapted() : N( -1 ) {}

    void preExecute() {}

    void postExecute() {}

    void execute( index_type i_N, index_type i_Grain = 0 )
    {
        N = i_N;

        DERIVED* dptr = static_cast<DERIVED*>( this );
        dptr->preExecute();
        {
            ADAPTOR A( *dptr );
            if ( i_Grain > 0 )
            {
                PFOR( range_type( 0, i_N, i_Grain ), A );
            }
            else
            {
                PFOR( range_type( 0, i_N ), A );
            }
        }
        dptr->postExecute();
    }

    void serialExecute( index_type i_N, index_type i_Grain = 0 )
    {
        N = i_N;

        DERIVED* dptr = static_cast<DERIVED*>( this );
        dptr->preExecute();

        {
            ADAPTOR A( *dptr );
            A( range_type( 0, i_N ) );
        }
        dptr->postExecute();
    }
};

//-*****************************************************************************
template <typename DERIVED, typename INDEX_TYPE = std::ptrdiff_t>
struct ForEachFunctor
        : public ForEachFunctorAdapted < DERIVED,
          RangeAdaptor<DERIVED, INDEX_TYPE> >
{
    ForEachFunctor()
        : ForEachFunctorAdapted<DERIVED, RangeAdaptor<DERIVED, INDEX_TYPE> > ()
    {}
};

//-*****************************************************************************
template <typename DERIVED, typename INDEX_TYPE = std::ptrdiff_t>
struct ZeroForEachFunctor
        : public EmldCore::Util::AllZeroConstructor<DERIVED>
        , public ForEachFunctor<DERIVED, INDEX_TYPE>

{
    ZeroForEachFunctor()
        : EmldCore::Util::AllZeroConstructor<DERIVED>()
        , ForEachFunctor<DERIVED, INDEX_TYPE>()
    {}
};


//-*****************************************************************************
template <typename DERIVED, typename INDEX_TYPE = std::ptrdiff_t>
struct ForEachFunctorI
        : public ForEachFunctorAdapted < DERIVED,
          IndexAdaptor<DERIVED, INDEX_TYPE> >
{
    ForEachFunctorI()
        : ForEachFunctorAdapted<DERIVED, IndexAdaptor<DERIVED, INDEX_TYPE> > ()
    {}
};

//-*****************************************************************************
template <typename DERIVED, typename INDEX_TYPE = std::ptrdiff_t>
struct ZeroForEachFunctorI
        : public EmldCore::Util::AllZeroConstructor<DERIVED>
        , public ForEachFunctorI<DERIVED, INDEX_TYPE>

{
    ZeroForEachFunctorI()
        : EmldCore::Util::AllZeroConstructor<DERIVED>()
        , ForEachFunctorI<DERIVED, INDEX_TYPE>()
    {}
};

} // End namespace ParallelUtil
} // End namespace EmldCore

#endif
