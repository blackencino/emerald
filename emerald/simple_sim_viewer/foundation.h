#pragma once

#include <emerald/geep_glfw/foundation.h>
#include <emerald/geep_glfw/util_gl.h>
#include <emerald/util/assert.h>
#include <emerald/util/foundation.h>

#pragma warning(push)
#pragma warning( disable : 4244 4100 4456 4005 )

#include <Alembic/Util/All.h>

#include <OpenEXR/ImathBox.h>
#include <OpenEXR/ImathBoxAlgo.h>
#include <OpenEXR/ImathColor.h>
#include <OpenEXR/ImathFrustum.h>
#include <OpenEXR/ImathFun.h>
#include <OpenEXR/ImathLine.h>
#include <OpenEXR/ImathMath.h>
#include <OpenEXR/ImathMatrix.h>
#include <OpenEXR/ImathMatrixAlgo.h>
#include <OpenEXR/ImathQuat.h>
#include <OpenEXR/ImathVec.h>

namespace emerald {
namespace simple_sim_viewer {

using namespace emerald::util;
using namespace emerald::geep_glfw;

typedef Imath::Vec3<unsigned int> V3ui;

}  // End namespace simple_sim_viewer
}  // End namespace emerald


#pragma warning(pop)