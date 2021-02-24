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
  EmldCoreCompactHashMap
  EmldCoreParallelUtil
  EmldCoreNoise
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

SET( SIMPLE_LIBS
  ${EMLD_ILMBASE_LIBS}
  ${EMLD_BOOST_LIBS}
  ${EMLD_TBB_LIBS}
  ${EMLD_GL_LIBS}
  ${EMLD_THREAD_LIBS}
  ${EMLD_Z_LIBS} m )

#-******************************************************************************
# Multi Inherit Test
ADD_EXECUTABLE( chm_test_MultiInherit test_MultiInherit.cpp )
TARGET_LINK_LIBRARIES( chm_test_MultiInherit ${THIS_LIBS} )
ADD_TEST( chm_TEST_MultiInherit chm_test_MultiInherit )


#-******************************************************************************
# Vector Manager Test
ADD_EXECUTABLE( chm_test_VectorManager test_VectorManager.cpp )
TARGET_LINK_LIBRARIES( chm_test_VectorManager ${THIS_LIBS} )
ADD_TEST( chm_TEST_VectorManager chm_test_VectorManager )


#-******************************************************************************
# IndexBlock Test
ADD_EXECUTABLE( chm_test_IndexBlock test_IndexBlock.cpp )
TARGET_LINK_LIBRARIES( chm_test_IndexBlock ${THIS_LIBS} )
ADD_TEST( chm_TEST_IndexBlock chm_test_IndexBlock )

#-******************************************************************************
# BaseCompactHashMap Test
ADD_EXECUTABLE( chm_test_BaseCompactHashMap test_BaseCompactHashMap.cpp )
TARGET_LINK_LIBRARIES( chm_test_BaseCompactHashMap ${THIS_LIBS} )
ADD_TEST( chm_TEST_BaseCompactHashMap chm_test_BaseCompactHashMap )

#-******************************************************************************
# Zindex Test
ADD_EXECUTABLE( chm_test_Zindex test_Zindex.cpp )
TARGET_LINK_LIBRARIES( chm_test_Zindex ${THIS_LIBS} )
ADD_TEST( chm_TEST_Zindex chm_test_Zindex )

#-******************************************************************************
# StateReorder Test
ADD_EXECUTABLE( chm_test_StateReorder test_StateReorder.cpp )
TARGET_LINK_LIBRARIES( chm_test_StateReorder ${THIS_LIBS} )
ADD_TEST( chm_TEST_StateReorder chm_test_StateReorder )


#-******************************************************************************
# StateCompactHashMap Test
ADD_EXECUTABLE( chm_test_StateCompactHashMap test_StateCompactHashMap.cpp )
TARGET_LINK_LIBRARIES( chm_test_StateCompactHashMap ${THIS_LIBS} )
ADD_TEST( chm_TEST_StateCompactHashMap chm_test_StateCompactHashMap )

##-*****************************************************************************
# Self-contained CHM Test
ADD_EXECUTABLE( chm_test_SimpleSelfContainedCHM test_SimpleSelfContainedCHM.cpp )
TARGET_LINK_LIBRARIES( chm_test_SimpleSelfContainedCHM ${SIMPLE_LIBS} )
ADD_TEST( chm_TEST_SimpleSelfContainedCHM chm_test_SimpleSelfContainedCHM )

