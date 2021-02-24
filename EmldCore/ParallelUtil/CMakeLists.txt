emldcore_legacy_define_library(ParallelUtil
  HEADERS
     All.h
     BucketedConcurrentVector.h
     ContiguousBlocks.h
     CopyFillSet.h
     For.h
     Foundation.h
     LRUCache.h
     Reduce.h
     RemoveDuplicates.h
     Scan.h
     SimpleArrayFunctors.h
     Sort.h
  SOURCES
    Foundation.cpp
  DEPENDENCIES
    emerald::EmldCore_Util
    Alembic::Alembic
    OpenEXR::OpenEXR
    Boost::Boost
    TBB::TBB
  )

# Recurse into test subdirectory
add_subdirectory(Tests)

