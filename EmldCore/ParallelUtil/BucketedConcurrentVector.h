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

#ifndef _EmldCore_ParallelUtil_BucketedConcurrentVector_h_
#define _EmldCore_ParallelUtil_BucketedConcurrentVector_h_

//-*****************************************************************************
// THIS CLASS/FILE IS INTENTIONALLY KEPT MOSTLY SEPARATE.
// It's a very portable construction, and very useful, so it's made to
// not worry about Foundation.h
//-*****************************************************************************

//-*****************************************************************************
// Thread Building Blocks
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

// I greatly reluctantly add this dependency here, it would be great to lose
// it.
#include <boost/utility/value_init.hpp>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cfloat>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <shared_mutex>
#include <utility>
#include <vector>

//-*****************************************************************************
namespace EmldCore {
namespace ParallelUtil {

#ifdef EMLD_DEBUG_ASSERT
#define BCV_DEBUG_ASSERT(COND, TEXT) EMLD_DEBUG_ASSERT(COND, TEXT)
#else
#define BCV_DEBUG_ASSERT(COND, TEXT) assert(COND)
#endif

//-*****************************************************************************
// We need a better tool than tbb::concurrent_vector, so we'll create
// this bucketed_vector, which has threadsafe growth in bucket-sized chunks.
template <typename T, typename CONTAINER_TYPE = std::vector<T> >
class BucketedConcurrentVector {
public:
    typedef T value_type;
    typedef CONTAINER_TYPE container_type;
    typedef typename container_type::size_type size_type;
    typedef typename container_type::difference_type difference_type;
    typedef typename std::ptrdiff_t index_type;

    typedef typename container_type::iterator iterator;
    typedef typename container_type::const_iterator const_iterator;
    typedef typename container_type::reference reference;
    typedef typename container_type::const_reference const_reference;
    typedef typename container_type::pointer pointer;
    typedef typename container_type::const_pointer const_pointer;

    static const size_type DEFAULT_BUCKET_SIZE() {
        return 1024;
    }
    static const size_type MIN_BUCKET_SIZE() {
        return 1;
    }

    typedef BucketedConcurrentVector<T, CONTAINER_TYPE> this_type;

protected:
    std::atomic<size_type> m_size;
    size_type m_bucketSize;
    std::shared_mutex m_storageMutex;
    container_type m_storage;

public:
    // We always have at least one bucket of space.
    BucketedConcurrentVector()
      : m_bucketSize(DEFAULT_BUCKET_SIZE())
      , m_storage(DEFAULT_BUCKET_SIZE()) {
        // Assignment of an atomic is atomic.
        m_size = 0;
    }

    // Bucket sizing.
    size_type bucket_size() const {
        return m_bucketSize;
    }
    void set_bucket_size(size_type i_bs) {
        BCV_DEBUG_ASSERT(i_bs > 0, "Can't have zero bucket size.");
        m_bucketSize = std::max(MIN_BUCKET_SIZE(), i_bs);
    }

    // Reserve just allocates storage, it doesn't modify the
    // size variable at all.
    void reserve(size_type i_reservedSize) {
        // if the storage does not have sufficient capacity, resize it!
        std::shared_lock<std::shared_mutex> lock(m_storageMutex);
        if (i_reservedSize > m_storage.size()) {
            // Need more space. Upgrade to writer lock.
            lock.unlock();
            std::unique_lock<std::shared_mutex> ulock(m_storageMutex);

            // Need to re-check inside writer lock.
            if (i_reservedSize > m_storage.size()) {
                // Figure out how much size to actually create.
                size_type numBuckets = i_reservedSize / m_bucketSize;
                size_type newSize = numBuckets * m_bucketSize;
                if (newSize < i_reservedSize) { newSize += m_bucketSize; }
                assert(newSize >= i_reservedSize);

                // Resize storage. This invalidates iterators
                // and attempts to write data.
#ifdef EMLD_TEST_MVEC_DEBUG_VERBOSE
                std::cout << "BVEC: reserve() Resizing storage to: " << newSize
                          << " to accomodate reservation of: " << i_reservedSize
                          << std::endl;
#endif
                m_storage.resize(newSize);
            }
        }
    }

    // Don't call from writing threads.
    // It's safe, but answer could change as you are going.
    size_type size() const {
        return m_size;
    }

    // Don't call from writing threads.
    // It's safe, but answer could change as you are going.
    size_type capacity() const {
        // Get a read-only lock on the mutex.
        std::shared_lock<std::shared_mutex> lock(m_storageMutex);
        return m_storage.size();
    }

    // Can call from writing threads.
    void push_back(const T& i_value) {
        // Get the index of the back, for us to push to, and increase
        // the size. The fetch_and_add is an atomic operation, which
        // returns the value of m_size AND adds 1 to it in a single operation.
        // That means we can safely use nextVal as a location that is
        // "ours".
        size_type nextVal = m_size.fetch_add(1);

        // Add capacity
        size_type reservedSize = nextVal + 1;

        // if the storage does not have sufficient capacity, resize it!
        std::shared_lock<std::shared_mutex> lock(m_storageMutex);
        if (reservedSize > m_storage.size()) {
            // Need more space. Upgrade to writer lock.
            lock.unlock();
            std::unique_lock<std::shared_mutex> ulock(m_storageMutex);

            // Need to re-check inside writer lock.
            if (reservedSize > m_storage.size()) {
                // Figure out how much size to actually create.
                size_type numBuckets = reservedSize / m_bucketSize;
                size_type newSize = numBuckets * m_bucketSize;
                if (newSize < reservedSize) { newSize += m_bucketSize; }
                assert(newSize >= reservedSize);

                // Resize storage. This invalidates iterators
                // and attempts to write data.
#ifdef EMLD_TEST_MVEC_DEBUG_VERBOSE
                std::cout << "BVEC: push_back() Resizing storage to: "
                          << newSize
                          << " to accomodate reservation of: " << reservedSize
                          << std::endl;
#endif
                m_storage.resize(newSize);
            }
        }

        if (!lock.owns_lock()) {
            lock.lock();
        }

        // Write the value. The read/write status of the mutex only refers
        // to the memory structure, not the values in the memory.
        m_storage[nextVal] = i_value;
    }

    // Append. Can call from writing threads.
    template <typename INPUT_ITER>
    void append(INPUT_ITER i_begin, INPUT_ITER i_end) {
        size_type num = i_end - i_begin;

        // Get the index of the back, for us to append to, and increase
        // the size. Same as above, nextVal represents the beginning of a
        // range that is "ours" to write to, because the fetch_and_add
        // operation is atomic.
        size_type nextVal = m_size.fetch_add(num);

        // Add capacity
        size_type reservedSize = nextVal + num;

        // if the storage does not have sufficient capacity, resize it!
        std::shared_lock<std::shared_mutex> lock(m_storageMutex);
        if (reservedSize > m_storage.size()) {
            // Need more space. Upgrade to writer lock.
            lock.unlock();
            std::unique_lock<std::shared_mutex> ulock(m_storageMutex);

            // Need to re-check inside writer lock.
            if (reservedSize > m_storage.size()) {
                // Figure out how much size to actually create.
                size_type numBuckets = reservedSize / m_bucketSize;
                size_type newSize = numBuckets * m_bucketSize;
                if (newSize < reservedSize) { newSize += m_bucketSize; }
                assert(newSize >= reservedSize);

                // Resize storage. This invalidates iterators
                // and attempts to write data.
#ifdef EMLD_TEST_MVEC_DEBUG_VERBOSE
                std::cout << "BVEC: append() Resizing storage to: " << newSize
                          << " to accomodate reservation of: " << reservedSize
                          << std::endl;
#endif
                m_storage.resize(newSize);
            }
        }

        if (!lock.owns_lock()) {
            lock.lock();
        }

        // Write the values. The locations are still "ours".
        std::copy(i_begin, i_end, m_storage.begin() + nextVal);
    }

    // tiny functor for parallel copy
    struct __copier {
        T* src;
        T* dst;
        void operator()(tbb::blocked_range<size_type>& i_range) const {
            std::copy(src + i_range.begin(),
                      src + i_range.end(),
                      dst + i_range.begin());
        }
    };

    //-*************************************************************************
    // NOT THREAD SAFE.
    template <typename INPUT_ITER>
    void copy(INPUT_ITER i_begin, INPUT_ITER i_end, bool i_inParallel = true) {
        size_type num = i_end - i_begin;

        // Add capacity
        size_type reservedSize = num;

        // if the storage does not have sufficient capacity, resize it!
        std::shared_lock<std::shared_mutex> lock(m_storageMutex);
        if (reservedSize > m_storage.size()) {
            // Need more space. Upgrade to writer lock.
            lock.unlock();
            std::unique_lock<std::shared_mutex> ulock(m_storageMutex);

            // Need to re-check inside writer lock.
            if (reservedSize > m_storage.size()) {
                // Figure out how much size to actually create.
                size_type numBuckets = reservedSize / m_bucketSize;
                size_type newSize = numBuckets * m_bucketSize;
                if (newSize < reservedSize) { newSize += m_bucketSize; }
                assert(newSize >= reservedSize);

                // Resize storage. This invalidates iterators
                // and attempts to write data.
#ifdef MGSTEST_MVEC_DEBUG_VERBOSE
                std::cout << "BVEC: copy() Resizing storage to: " << newSize
                          << " to accomodate reservation of: " << reservedSize
                          << std::endl;
#endif
                m_storage.resize(newSize);
            }
        }

        if (!lock.owns_lock()) {
            lock.lock();
        }

        // Write the values, in parallel or in serial.
        if (i_inParallel) {
            __copier F;
            tbb::blocked_range<size_type> range(0, num);
            F.src = (T*)(*i_begin);
            F.dst = (T*)(&(m_storage.front()));
            tbb::parallel_for(range, F);
        } else {
            std::copy(i_begin, i_end, m_storage.begin());
        }

        // Set the size.
        m_size = num;
    }

    //-*************************************************************************
    // Parallel tool for fill

    // tiny functor for parallel fill
    struct __filler {
        T* dst;
        void operator()(tbb::blocked_range<size_type>& i_range) const {
            static const boost::value_initialized<T> val;
            std::fill(dst + i_range.begin(), dst + i_range.end(), val);
        }
    };

    //-*************************************************************************
    // NOT THREAD SAFE
    void resize(size_type i_newSize, bool i_initInParallel = true) {
        // reserve the space, and then just change the size.
        reserve(i_newSize);

        if (i_newSize > m_size) {
            if (i_initInParallel) {
                __filler F;
                tbb::blocked_range<size_type> range(m_size, i_newSize);
                // F.val takes default constructor value.
                F.dst = (T*)(&(m_storage.front()));
                tbb::parallel_for(range, F);
            } else {
                static const boost::value_initialized<T> val;
                std::fill(m_size + &(m_storage.front()),
                          i_newSize + &(m_storage.front()),
                          val);
            }
        }

        // Set size.
        m_size = i_newSize;
    }

    //-*************************************************************************
    // NOT THREAD SAFE
    iterator grow_by(size_type n) {
        size_type oldSize = m_size;
        resize(m_size + n);
        return m_storage.begin() + oldSize;
    }

    //-*************************************************************************
    // NOT THREAD SAFE
    void clear() {
        // We never shrink.
        m_size = 0;
    }

    //-*************************************************************************
    // NOT THREAD SAFE
    void swap(this_type& i_other) {
        // No locks here, so be careful.
        m_storage.swap(i_other.m_storage);
        size_type tmpSize = m_size;
        m_size = i_other.m_size;
        i_other.m_size = tmpSize;
    }

    //-*************************************************************************
    // NOT THREAD SAFE
    void swap(container_type& i_other) {
        // resize m_storage to m_size, essentially trimming off
        // up to a bucket's worth of currently unused elements.
        m_storage.resize(m_size);

        // swap m_storage with i_other, so that i_other now owns
        // all the elements from this.
        m_storage.swap(i_other);

        // update m_size to match the current number of elements
        // now held by this.
        m_size = m_storage.size();

        // now resize so that m_storage is properly bucketed.
        resize(m_size);
    }

    //-*************************************************************************
    // NON-DEFAULT CONSTRUCTORS
    //-*************************************************************************

    // Build a vector with a given size, and capacity to hold it.
    // Don't call from threads.
    explicit BucketedConcurrentVector(size_type i_size)
      : m_bucketSize(DEFAULT_BUCKET_SIZE())
      , m_storage(m_bucketSize) {
        m_size = 0;
        resize(i_size);
    }

    // Build a vector with a given size, and a given bucket size.
    BucketedConcurrentVector(size_type i_size, size_type i_bucketSize)
      : m_bucketSize(std::max(MIN_BUCKET_SIZE(), i_bucketSize))
      , m_storage(m_bucketSize) {
        m_size = 0;
        resize(i_size);
    }

    //-*************************************************************************
    // THREAD SAFE FOR READS, AND FOR NON-SIZE-CHANGING, NON-OVERLAPPING
    // THREADED WRITES
    //-*************************************************************************

    // None of these iterator calls are safe inside threads that
    // are potentially changing the size. They're safe if it's
    // read-only (or non-size-changing read-write) access.
    iterator begin() {
        return m_storage.begin();
    }
    const_iterator cbegin() const {
        return m_storage.begin();
    }
    const_iterator begin() const {
        return cbegin();
    }

    iterator end() {
        return m_storage.begin() + m_size;
    }
    const_iterator cend() const {
        return m_storage.begin() + m_size;
    }
    const_iterator end() const {
        return cend();
    }

    reference operator[](size_t i) {
        return m_storage[i];
    }
    const_reference operator[](size_t i) const {
        return m_storage[i];
    }

    reference front() {
        return m_storage.front();
    }
    const_reference front() const {
        return m_storage.front();
    }
    reference back() {
        return m_storage[m_size - 1];
    }
    const_reference back() const {
        return m_storage[m_size - 1];
    }

    pointer data() {
        if (m_size == 0) {
            return reinterpret_cast<pointer>(0);
        } else {
            return reinterpret_cast<pointer>(&(m_storage[0]));
        }
    }

    const_pointer data() const {
        if (m_size == 0) {
            return reinterpret_cast<const_pointer>(0);
        } else {
            return reinterpret_cast<const_pointer>(&(m_storage[0]));
        }
    }

    const_pointer cdata() const {
        if (m_size == 0) {
            return reinterpret_cast<const_pointer>(0);
        } else {
            return reinterpret_cast<const_pointer>(&(m_storage[0]));
        }
    }
};

}  // End namespace ParallelUtil
}  // End namespace EmldCore

#endif
