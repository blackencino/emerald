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

# C++ files for this project
SET( H_FILES 
     Exception.h
     Foundation.h 
     Functions.h
     ImathVecOrdering.h
     MemPool.h
     Murmur3.h
     Random.h
     StorageTraits.h
     Timer.h
     VectorUtil.h
     All.h )

SET( CXX_FILES 
     Exception.cpp 
     Functions.cpp 
     Murmur3.cpp
     Random.cpp )

SET( SOURCE_FILES ${CXX_FILES} ${H_FILES} )

ADD_LIBRARY( EmldCoreUtil ${SOURCE_FILES} )

INSTALL( TARGETS EmldCoreUtil
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib/static )

INSTALL( FILES ${H_FILES}
         DESTINATION include/EmldCore/Util
         PERMISSIONS OWNER_READ GROUP_READ WORLD_READ )

ADD_SUBDIRECTORY( Tests )
