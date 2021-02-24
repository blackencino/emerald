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

#include "Xform.h"
#include "Interpolation.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
Xform::Xform( Object& i_enclosingObject,
              AbcG::IXform& i_abcXform )
    : Object::Internal( i_enclosingObject )
    , m_abcXform( i_abcXform )
{
    //std::cout << "Creating Xform" << std::endl;

    AbcG::IXformSchema& schema = m_abcXform.getSchema();

    // The object has already set up the min time and max time of
    // all the children.
    // if we have a non-constant time sampling, we should get times
    // out of it.
    AbcG::TimeSamplingPtr iTsmp = schema.getTimeSampling();
    if ( !schema.isConstant() )
    {
        size_t numSamps = schema.getNumSamples();
        if ( numSamps > 0 )
        {
            m_internalMinTime = iTsmp->getSampleTime( 0 );
            m_internalMaxTime = iTsmp->getSampleTime( numSamps-1 );
        }
    }

    // Eval at init time.
    evalTime( m_enclosingObject.currentTime() );
}

//-*****************************************************************************
void Xform::setTime()
{
    evalTime( m_enclosingObject.currentTime() );
}

//-*****************************************************************************
void Xform::evalTime( chrono_t i_time )
{
    // Get xform.
    m_internalToLocal = Interpolate( m_abcXform.getSchema(), i_time );
    m_localToInternal = m_internalToLocal;
    m_localToInternal.invert();
    m_internalBounds.makeEmpty();
}

} // End namespace AbcMeshesScene
} // End namespace EmldCore
