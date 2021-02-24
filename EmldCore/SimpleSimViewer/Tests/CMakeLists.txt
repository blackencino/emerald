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

SET( TEST_LIBS
    EmldCoreSimpleSimViewer
    EmldCoreGeepGLFW
    EmldCoreUtil
    ${EMLD_ALEMBIC_LIBS}
    ${EMLD_HDF5_LIBS}
    ${EMLD_ILMBASE_LIBS}
    ${EMLD_BOOST_LIBS}
    ${EMLD_GL_LIBS}
    ${EMLD_THREAD_LIBS}
    ${EMLD_Z_LIBS} m )

#-******************************************************************************
# Simple Mesh Sim
ADD_EXECUTABLE( emld_test_SimpleMeshSim test_SimpleMeshSim.cpp )
TARGET_LINK_LIBRARIES( emld_test_SimpleMeshSim ${TEST_LIBS} )

#-******************************************************************************
# Simple Points Sim
ADD_EXECUTABLE( emld_test_SimplePointsSim test_SimplePointsSim.cpp )
TARGET_LINK_LIBRARIES( emld_test_SimplePointsSim ${TEST_LIBS} )
