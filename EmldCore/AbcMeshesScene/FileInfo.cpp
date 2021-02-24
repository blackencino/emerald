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

#include "FileInfo.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
OnOffSet::OnOffSet( const std::string& i_onRegExStr )
    : m_onRegExStr( i_onRegExStr )
{
    if ( m_onRegExStr != "" )
    {
        m_onRegEx.reset( new ABCM_REGEX( m_onRegExStr,
                                         ABCM_REGEX::extended ) );
    }
}

//-*****************************************************************************
OnOffSet::OnOffSet( const std::string& i_onRegExStr,
                    const std::string& i_offRegExStr )
    : m_onRegExStr( i_onRegExStr )
    , m_offRegExStr( i_offRegExStr )
{
    if ( m_onRegExStr != "" )
    {
        m_onRegEx.reset( new ABCM_REGEX( m_onRegExStr,
                                         ABCM_REGEX::extended ) );
    }
    if ( m_offRegExStr != "" )
    {
        m_offRegEx.reset( new ABCM_REGEX( m_offRegExStr,
                                          ABCM_REGEX::extended ) );
    }
}

//-*****************************************************************************
bool OnOffSet::isFullNameIncluded( const std::string& i_fullName ) const
{
    bool matchedOnList = true;
    if ( m_onRegEx )
    {
        matchedOnList = ABCM_REGEX_MATCH( i_fullName, *m_onRegEx );
    }
    if ( !matchedOnList )
    {
        return false;
    }

    bool matchedOffList = false;
    if ( m_offRegEx )
    {
        matchedOffList = ABCM_REGEX_MATCH( i_fullName, *m_offRegEx );
    }
    if ( matchedOffList )
    {
        return false;
    }

    return true;
}

} // End namespace AbcMeshesScene
} // End namespace EmldCore

