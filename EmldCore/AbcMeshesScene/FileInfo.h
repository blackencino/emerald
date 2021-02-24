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

#ifndef _EmldCore_AbcMeshesScene_FileInfo_h_
#define _EmldCore_AbcMeshesScene_FileInfo_h_

#include "Foundation.h"

namespace EmldCore {
namespace AbcMeshesScene {

#define ABCM_REGEX std::regex
#define ABCM_REGEX_MATCH std::regex_match

//-*****************************************************************************
struct FileInfo
{
    FileInfo()
        : fileName( "" )
        , onRegEx( "" )
        , offRegEx( "" )
    {}

    FileInfo( const std::string& i_fileName )
        : fileName( i_fileName )
        , onRegEx( "" )
        , offRegEx( "" )
    {}

    FileInfo( const std::string& i_fileName,
              const std::string& i_onRegEx,
              const std::string& i_offRegEx )
        : fileName( i_fileName )
        , onRegEx( i_onRegEx )
        , offRegEx( i_offRegEx )
    {}

    std::string fileName;
    std::string onRegEx;
    std::string offRegEx;
};

//-*****************************************************************************
inline std::ostream& operator<<( std::ostream& ostr,
                                 const FileInfo& i_mfi )
{
    ostr << "{\"" << i_mfi.fileName << "\",\""
         << i_mfi.onRegEx << "\",\""
         << i_mfi.offRegEx << "\"}";
    return ostr;
}

//-*****************************************************************************
typedef std::vector<FileInfo> FileInfoVec;

//-*****************************************************************************
class OnOffSet
{
public:
    OnOffSet() {}

    explicit OnOffSet( const std::string& i_onRegExStr );

    OnOffSet( const std::string& i_onRegExStr,
              const std::string& i_offRegExStr );

    bool isFullNameIncluded( const std::string& i_fullName ) const;

protected:
    std::string m_onRegExStr;
    std::string m_offRegExStr;

    ABCM_UNIQUE_PTR<ABCM_REGEX> m_onRegEx;
    ABCM_UNIQUE_PTR<ABCM_REGEX> m_offRegEx;
};

} // End namespace AbcMeshesScene
} // End namespace EmldCore

#endif
