##-*****************************************************************************
## Copyright (c) 2001-2013, Christopher Jon Horvath. All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
## 1. Redistributions of source code must retain the above copyright notice,
## this list of conditions and the following disclaimer.
##
## 2. Redistributions in binary form must reproduce the above copyright notice,
## this list of conditions and the following disclaimer in the documentation
## and/or other materials provided with the distribution.
##
## 3. Neither the name of Christopher Jon Horvath nor the names of his
## contributors may be used to endorse or promote products derived from this
## software without specific prior written permission.
## 
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
## LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
## CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.
##-*****************************************************************************

SET( THIS_LIBS
  EmldCoreParallelUtil
  EmldCoreUtil
  ${EMLD_ALEMBIC_LIBS}
  ${EMLD_HDF5_LIBS}
  ${EMLD_PARTIO_LIBS}
  ${EMLD_ILMBASE_LIBS}
  ${EMLD_BOOST_LIBS}
  ${EMLD_TBB_LIBS}
  ${EMLD_GL_LIBS}
  ${EMLD_THREAD_LIBS}
  ${EMLD_Z_LIBS} m )


#-******************************************************************************
# Bucketed Concurrent Vector Test
ADD_EXECUTABLE( emld_test_BucketedConcurrentVector 
                test_BucketedConcurrentVector.cpp )
TARGET_LINK_LIBRARIES( emld_test_BucketedConcurrentVector ${THIS_LIBS} )
ADD_TEST( emld_TEST_BucketedConcurrentVector 
          emld_test_BucketedConcurrentVector )

#-******************************************************************************
# LRUCache Test
ADD_EXECUTABLE( emld_test_LRUCache test_LRUCache.cpp )
TARGET_LINK_LIBRARIES( emld_test_LRUCache ${THIS_LIBS} )
ADD_TEST( emld_TEST_LRUCache emld_test_LRUCache )

#-******************************************************************************
# Parallel For Test
ADD_EXECUTABLE( emld_test_ParallelFor test_ParallelFor.cpp )
TARGET_LINK_LIBRARIES( emld_test_ParallelFor ${THIS_LIBS} )
ADD_TEST( emld_TEST_ParallelFor emld_test_ParallelFor )

#-******************************************************************************
# Parallel Reduce Test
ADD_EXECUTABLE( emld_test_ParallelReduce test_ParallelReduce.cpp )
TARGET_LINK_LIBRARIES( emld_test_ParallelReduce ${THIS_LIBS} )
ADD_TEST( emld_TEST_ParallelReduce emld_test_ParallelReduce )

#-******************************************************************************
# Parallel Scan Test
ADD_EXECUTABLE( emld_test_ParallelScan test_ParallelScan.cpp )
TARGET_LINK_LIBRARIES( emld_test_ParallelScan ${THIS_LIBS} )
ADD_TEST( emld_TEST_ParallelScan emld_test_ParallelScan )

#-******************************************************************************
# Parallel Sort Test
ADD_EXECUTABLE( emld_test_ParallelSort test_ParallelSort.cpp )
TARGET_LINK_LIBRARIES( emld_test_ParallelSort ${THIS_LIBS} )
ADD_TEST( emld_TEST_ParallelSort emld_test_ParallelSort )

#-******************************************************************************
# Parallel Contiguous Blocks Test
ADD_EXECUTABLE( emld_test_ParallelContiguousBlocks 
                test_ParallelContiguousBlocks.cpp )
TARGET_LINK_LIBRARIES( emld_test_ParallelContiguousBlocks ${THIS_LIBS} )
ADD_TEST( emld_TEST_ParallelContiguousBlocks 
          emld_test_ParallelContiguousBlocks )

#-******************************************************************************
# Parallel CopyFillSet Test
ADD_EXECUTABLE( emld_test_ParallelCopyFillSet test_ParallelCopyFillSet.cpp )
TARGET_LINK_LIBRARIES( emld_test_ParallelCopyFillSet ${THIS_LIBS} )
ADD_TEST( emld_TEST_ParallelCopyFillSet emld_test_ParallelCopyFillSet )

#-******************************************************************************
# Remove Duplicates Test
ADD_EXECUTABLE( emld_test_RemoveDuplicates 
                test_RemoveDuplicates.cpp )
TARGET_LINK_LIBRARIES( emld_test_RemoveDuplicates ${THIS_LIBS} )
ADD_TEST( emld_TEST_RemoveDuplicates 
          emld_test_RemoveDuplicates )
