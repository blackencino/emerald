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

#ifndef _EmldCore_CompactHashMap_VectorManager_h_
#define _EmldCore_CompactHashMap_VectorManager_h_

#include "Foundation.h"

namespace EmldCore {
namespace CompactHashMap {

//-*****************************************************************************
template <typename T>
class TypedManagedCvectorPtrHandle
{
public:
    typedef T                                   value_type;
    typedef std::size_t                         size_type;
    typedef std::ptrdiff_t                      difference_type;

    typedef T*                                  iterator;
    typedef const T*                            const_iterator;
    typedef T&                                  reference;
    typedef const T&                            const_reference;

    typedef T*                                  pointer;
    typedef const T*                            const_pointer;

    typedef TypedManagedCvectorPtrHandle<T>     this_type;

    typedef typename storage_traits<T>::storage_type      storage_value_type;
    typedef BucketedConcurrentVector<storage_value_type>  storage_vector_type;
    typedef storage_vector_type*                          storage_vector_ptr;

private:
    storage_vector_ptr m_storageRawPtr;

protected:
    void resetRaw() { m_storageRawPtr = NULL; }

public:
    TypedManagedCvectorPtrHandle()
        : m_storageRawPtr( NULL ) {}

    explicit TypedManagedCvectorPtrHandle( storage_vector_ptr i_ptr )
        : m_storageRawPtr( i_ptr )
    {
        EMLD_ASSERT( sizeof( storage_value_type ) == sizeof( value_type ),
                        "Incompatible storage & vector types." );
    }

    pointer data()
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::data() "
                        << "NULL storage pointer" );
        if ( m_storageRawPtr->size() == 0 )
        {
            return NULL;
        }
        else
        {
            return reinterpret_cast<T*>( &( m_storageRawPtr->front() ) );
        }
    }

    const_pointer cdata() const
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::data() const "
                        << "NULL storage pointer" );
        if ( m_storageRawPtr->size() == 0 )
        {
            return NULL;
        }
        else
        {
            return reinterpret_cast<const T*>( &( m_storageRawPtr->front() ) );
        }
    }

    const_pointer data() const
    {
        return cdata();
    }

    size_type size() const
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::size() "
                        << "NULL storage pointer" );
        return m_storageRawPtr->size();
    }

    iterator begin() { return data(); }
    const_iterator begin() const { return data(); }
    const_iterator cbegin() const { return data(); }
    iterator end() { return data() + size(); }
    const_iterator end() const { return data() + size(); }
    const_iterator cend() const { return data() + size(); }

    reference operator[]( size_type i ) { return data()[i]; }
    const_reference operator[]( size_type i ) const { return data()[i]; }

    reference front() { return data()[0]; }
    const_reference front() const { return cdata()[0]; }

    reference back() { return data()[size()-1]; }
    const_reference back() const { return cdata()[size()-1]; }

    void reserve( size_type i_size )
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::reserve() "
                        << "NULL storage pointer" );
        m_storageRawPtr->reserve( i_size );
    }
    void resize( size_type i_size )
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::resize() "
                        << "NULL storage pointer" );
        m_storageRawPtr->resize( i_size );
    }
    void clear()
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::clear() "
                        << "NULL storage pointer" );
        m_storageRawPtr->clear();
    }

    void push_back( const_reference i_value )
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::push_back() "
                        << "NULL storage pointer" );
        m_storageRawPtr->push_back(
            reinterpret_cast<const storage_value_type&>( i_value ) );
    }

    template <typename ITER>
    void append( ITER i_begin, ITER i_end )
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::append( begin, end ) "
                        << "NULL storage pointer" );
        size_type n = i_end - i_begin;
        std::copy( i_begin, i_end, m_storageRawPtr->grow_by( n ) );
    }

    template <typename VEC>
    void append( const VEC& i_vec )
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::append( vec ) "
                        << "NULL storage pointer" );
        std::copy( i_vec.begin(), i_vec.end(),
                   m_storageRawPtr->grow_by( i_vec.size() ) );
    }

    // Swapping
    void swap( TypedManagedCvectorPtrHandle<T> i_other )
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::swap() "
                        << "NULL storage pointer" );
        EMLD_ASSERT( i_other.m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::swap() "
                        << "NULL other shared storage pointer" );
        m_storageRawPtr->swap( *( i_other.m_storageRawPtr ) );
    }

    // Copying.
    template <typename VEC>
    void copy_from( const VEC& i_vec )
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::copy_from() "
                        << "NULL storage pointer" );
        EmldCore::ParallelUtil::VectorCopy( i_vec, *this );
    }

    // Zero-fill.
    void fill_with_zero()
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::fill_with_zero() "
                        << "NULL storage pointer" );
        EmldCore::ParallelUtil::VectorZeroBits( *this );
    }

    // Value-fill.
    void fill_with_value( const_reference i_val )
    {
        EMLD_ASSERT( m_storageRawPtr,
                        "TypedManagedCvectorPtrHandle::fill_with_value() "
                        << "NULL storage pointer" );
        EmldCore::ParallelUtil::VectorFill( *this, i_val );
    }

    // Validity of the handle.
    bool valid() const { return ( m_storageRawPtr != NULL ); }
    operator bool() const { return valid(); }
    bool operator!() const { return !valid(); }
};

//-*****************************************************************************
//-*****************************************************************************
// MANAGER FOR A SINGLE STORAGE SINGLE TYPE
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename STORAGE_TYPE>
class SubVectorManager
{
public:
    typedef STORAGE_TYPE                                    storage_value_type;
    typedef BucketedConcurrentVector<storage_value_type>    container_type;
    typedef container_type*                                 container_ptr;
    typedef EMLD_SHARED_PTR<container_type>              container_sptr;
    typedef std::vector<container_sptr>                     container_sptr_vec;

    typedef typename container_type::size_type              size_type;
    typedef typename container_type::difference_type        difference_type;

protected:
    // The used and unused containers
    container_sptr_vec m_used;
    container_sptr_vec m_unused;

    // The bucket size.
    size_type m_bucketSize;


protected:
    SubVectorManager() 
    {
        m_bucketSize = container_type::DEFAULT_BUCKET_SIZE();
    }
    SubVectorManager( std::size_t i_bs ) 
    {
        m_bucketSize = std::max( container_type::MIN_BUCKET_SIZE(), i_bs );
    }

public:
    size_type bucketSize() const { return m_bucketSize; }
    void setBucketSize( size_type i_bs )
    {
        m_bucketSize = std::max( container_type::MIN_BUCKET_SIZE(), i_bs );
        for ( typename container_sptr_vec::iterator iter = m_used.begin();
              iter != m_used.end(); ++iter )
        {
            (*iter)->set_bucket_size( m_bucketSize );
        }
        for ( typename container_sptr_vec::iterator iter = m_unused.begin();
              iter != m_unused.end(); ++iter )
        {
            (*iter)->set_bucket_size( m_bucketSize );
        }
    }

    void protectedGet( container_ptr& o_c )
    {
        container_sptr c;
        if ( m_unused.size() > 0 )
        {
            c = m_unused.back();
            m_unused.pop_back();
#ifdef EMLD_TEST_MVEC_DEBUG_VERBOSE
            std::cout << "MVEC: UU.get popped "
                      << c.get()
                      << " off unused size[" << sizeof( storage_value_type )
                      << "]" << std::endl;
#endif
        }
        else
        {
            c.reset( new container_type() );
#ifdef EMLD_TEST_MVEC_DEBUG_VERBOSE
            std::cout << "MVEC: UU.get created "
                      << c.get()
                      << " size[" << sizeof( storage_value_type )
                      << "]" << std::endl;
#endif
        }
        m_used.push_back( c );
        o_c = c.get();
    }

    void protectedRelease( container_ptr i_c )
    {
        size_type n = m_used.size();
        for ( size_type i = 0; i < n; ++i )
        {
            container_sptr c = m_used[i];
            if ( c.get() == i_c )
            {
#ifdef EMLD_TEST_MVEC_DEBUG_VERBOSE
                std::cout << "MVEC: released "
                          << c.get()
                          << " off used[" << i
                          << "] size[" << sizeof( storage_value_type )
                          << "]" << std::endl;
#endif
                if ( i != n - 1 )
                {
                    m_used[i] = m_used[n - 1];
                }
                m_used.pop_back();
                m_unused.push_back( c );
                return;
            }
        }
        EMLD_THROW( "Attempting to release a vector that's not in "
                       "the used list." );
    }
};

//-*****************************************************************************
template <typename DERIVED>
class VectorManagerWrapper
{
public:
    typedef DERIVED derived_type;
    typedef VectorManagerWrapper<DERIVED> this_type;

    typedef std::size_t size_type;

    //-*************************************************************************
    VectorManagerWrapper() {}

    //-*************************************************************************
    template <typename CONTAINER>
    class StorageVectorDeleter
    {
    public:
        StorageVectorDeleter( VectorManagerWrapper<DERIVED>& i_manager )
        : m_manager( i_manager )
        {}

        void operator()( CONTAINER* i_ptr )
        {
            m_manager.storageVectorDeleterRelease<CONTAINER>( i_ptr );
        }

    protected:    
        VectorManagerWrapper<DERIVED>& m_manager;
    };

    //-*************************************************************************
    // A Handle around the Concurrent Vector which allows us to insert
    // a deleter.
    template <typename T>
    class TypedManagedCvectorHandle : public TypedManagedCvectorPtrHandle<T>
    {
    public:
        typedef TypedManagedCvectorPtrHandle<T>           super_type;
        typedef super_type ptr_handle_type;
        typedef typename super_type::storage_vector_type  storage_vector_type;
        typedef typename super_type::storage_vector_ptr   storage_vector_ptr;
        typedef EMLD_SHARED_PTR<storage_vector_type>   storage_vector_sptr;

        TypedManagedCvectorHandle()
            : TypedManagedCvectorPtrHandle<T>() {}

        // Parallel functors shouldn't have an object that has a shared ptr
        // in it, they may confuse the reference count and accidentally delete
        // something we need to keep. This function returns an internally
        // raw version of the handle for keeping in parallel functors.
        TypedManagedCvectorPtrHandle<T> ptrHandle()
        {
            return TypedManagedCvectorPtrHandle<T>( m_storageSharedPtr.get() );
        }

        void reset()
        {
            this->resetRaw();
            m_storageSharedPtr.reset();
        }

        // Not actually a public constructor.
        TypedManagedCvectorHandle( 
                storage_vector_ptr i_ptr,
                StorageVectorDeleter<storage_vector_type> i_svd )
            : TypedManagedCvectorPtrHandle<T>( i_ptr )
            , m_storageSharedPtr( i_ptr, i_svd )
        {}

    protected:
        storage_vector_sptr m_storageSharedPtr;
    };

    //-*************************************************************************
    template <typename T>
    TypedManagedCvectorHandle<T> get( size_type N )
    {
        typedef TypedManagedCvectorHandle<T> handle_type;
        typedef T value_type;
        typedef typename handle_type::storage_value_type storage_value_type;
        typedef typename handle_type::storage_vector_type storage_vector_type;
        typedef typename handle_type::storage_vector_ptr storage_vector_ptr;
        typedef typename handle_type::storage_vector_sptr storage_vector_sptr;
        typedef StorageVectorDeleter<storage_vector_type> 
            storage_vector_deleter;
        typedef SubVectorManager<storage_value_type> sub_type;

        storage_vector_ptr data = NULL;

        DERIVED* derivedThis = static_cast<DERIVED*>( this );
        sub_type* subThis = static_cast<sub_type*>( derivedThis );

        subThis->protectedGet( data );

        handle_type handle( data, storage_vector_deleter( *this ) );
        handle.resize( N );
        return handle;
    }

    //-*************************************************************************
    // Don't call this directly.
    template <typename storage_vector_type>
    void storageVectorDeleterRelease( storage_vector_type* i_ptr )
    {
        typedef typename storage_vector_type::value_type storage_value_type;
        typedef SubVectorManager<storage_value_type> sub_type;

        DERIVED* derivedThis = static_cast<DERIVED*>( this );
        sub_type* subThis = static_cast<sub_type*>( derivedThis );

        subThis->protectedRelease( i_ptr );
    }
};

//-*****************************************************************************
// EXAMPLE
// class MyVectorManager 
// : public VectorManagerWrapper<MyVectorManager>
// , public SubVectorManager<storage_bytes_0>
// , public SubVectorManager<storage_bytes_1>
// {};
//-*****************************************************************************


} // End namespace CompactHashMap
} // End namespace EmldCore

#endif
