#include <EmldCore/ParallelUtil/LRUCache.h>
#include <EmldCore/Util/Exception.h>
#include <EmldCore/Util/Foundation.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace epu = EmldCore::ParallelUtil;
namespace eu = EmldCore::Util;

std::mutex g_printMutex;

struct Data {
    std::string buffer;

    std::size_t size() const {
        return buffer.length() + 1;
    }
};

typedef EMLD_SHARED_PTR<Data> DataSptr;

struct FirstBufferCompute {
    Data* operator()(void) {
        Data* d = new Data;
        std::stringstream sstr;

        for (int i = 0; i < 100; i++) { sstr << "First Buffer [" << i << "]."; }

        (*d).buffer = sstr.str();

        std::scoped_lock<std::mutex> lock(g_printMutex);
        std::cout << "Made first buffer" << std::endl;
        return d;
    }
};

struct SecondBufferCompute {
    Data* operator()(void) {
        Data* d = new Data;
        std::stringstream sstr;

        for (int i = 0; i < 63; i++) { sstr << "Second Buffer [" << i << "]."; }

        (*d).buffer = sstr.str();

        std::scoped_lock<std::mutex> lock(g_printMutex);
        std::cout << "Made second buffer" << std::endl;
        return d;
    }
};

struct ThirdBufferCompute {
    Data* operator()(void) {
        Data* d = new Data;
        std::stringstream sstr;

        for (int i = 0; i < 76; i++) { sstr << "Third Buffer [" << i << "]."; }

        (*d).buffer = sstr.str();

        std::scoped_lock<std::mutex> lock(g_printMutex);
        std::cout << "Made third buffer" << std::endl;
        return d;
    }
};

struct StringHash {
    std::size_t hash(const std::string& i_key) const {
        return EMLD_HASH<std::string>()(i_key);
    }

    bool equal(const std::string& a, const std::string& b) const {
        return a == b;
    }
};

typedef epu::LRUCache<std::string, Data, StringHash> Cache;

struct TestFunctor {
    Cache* cache;

    void test(int i) {
        int comp = i % 3;
        DataSptr d;
        if (comp == 0) {
            FirstBufferCompute c;
            d = cache->get("Buffer1", c);
        } else if (comp == 1) {
            SecondBufferCompute c;
            d = cache->get("Buffer2", c);
        } else if (comp == 2) {
            ThirdBufferCompute c;
            d = cache->get("Buffer3", c);
        }
    }

    void operator()(const tbb::blocked_range<int>& i_range) const {
        TestFunctor* nonConstThis = const_cast<TestFunctor*>(this);
        for (int i = i_range.begin(); i < i_range.end(); ++i) {
            nonConstThis->test(i);
        }
    }
};

int main(int argc, char* argv[]) {
    Cache::Parameters params;
    params.evictThreshold = 4096;
    params.evictMargin = 512;
    params.mapNumBuckets = 1024;
    params.minQueueCleanupSize = 32;
    params.maxQueueCleanupRatio = 2.0;
    params.maxQueueCleanupWaste = 32;
    Cache cache(params);
    TestFunctor F;
    F.cache = &cache;
    tbb::parallel_for(tbb::blocked_range<int>(1, 1024 * 1024), F);
    return 0;
}
