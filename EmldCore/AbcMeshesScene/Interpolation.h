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

#ifndef _EmldCore_AbcMeshesScene_Interpolation_h_
#define _EmldCore_AbcMeshesScene_Interpolation_h_

#include "Foundation.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
bool Interpolate( AbcG::IBoolProperty& i_prop, chrono_t i_time );

//-*****************************************************************************
int32_t Interpolate( AbcG::IInt32Property& i_prop, chrono_t i_time );

//-*****************************************************************************
int64_t Interpolate( AbcG::IInt64Property& i_prop, chrono_t i_time );

//-*****************************************************************************
float Interpolate( AbcG::IFloatProperty &iProp, chrono_t iTime );

//-*****************************************************************************
double Interpolate( AbcG::IDoubleProperty &iProp, chrono_t iTime );

//-*****************************************************************************
V3f Interpolate( AbcG::IV3fProperty& i_prop, chrono_t i_time );

//-*****************************************************************************
V3d Interpolate( AbcG::IV3dProperty& i_prop, chrono_t i_time );

//-*****************************************************************************
V3f Interpolate( AbcG::IP3fProperty& i_prop, chrono_t i_time );

//-*****************************************************************************
V3d Interpolate( AbcG::IP3dProperty& i_prop, chrono_t i_time );

//-*****************************************************************************
C3f Interpolate( AbcG::IC3fProperty& i_prop, chrono_t i_time );

//-*****************************************************************************
N3f Interpolate( AbcG::IN3fProperty& i_prop, chrono_t i_time );

//-*****************************************************************************
N3d Interpolate( AbcG::IN3dProperty& i_prop, chrono_t i_time );

//-*****************************************************************************
std::string Interpolate( AbcG::IStringProperty& i_prop, chrono_t i_time );

//-*****************************************************************************
M44d Interpolate( AbcG::IXformSchema &iXform, chrono_t iTime );
    
} // End namespace AbcMeshesScene
} // End namespace EmldCore

#endif
