//-*****************************************************************************
// Copyright (c) 2001-2003 Tweak Inc. All rights reserved.
//-*****************************************************************************

//-*****************************************************************************
// Modified by Christopher Jon Horvath in 2010 to deal use Imath math classes
// instead of TwkMath.
//-*****************************************************************************

#ifndef _EmldCore_MeshEdit_FixOrientation_h_
#define _EmldCore_MeshEdit_FixOrientation_h_

#include "Foundation.h"
#include "PolyEdge.h"

namespace EmldCore {
namespace MeshEdit {

//-*****************************************************************************
void FixOrientation( PolyEdge &iPolyEdge );

} // End namespace MeshEdit
} // End namespace EmldCore

#endif
