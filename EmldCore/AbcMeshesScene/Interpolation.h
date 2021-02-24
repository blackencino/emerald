#ifndef _EmldCore_AbcMeshesScene_Interpolation_h_
#define _EmldCore_AbcMeshesScene_Interpolation_h_

#include "Foundation.h"

namespace EmldCore {
namespace AbcMeshesScene {

//-*****************************************************************************
bool Interpolate(AbcG::IBoolProperty& i_prop, chrono_t i_time);

//-*****************************************************************************
int32_t Interpolate(AbcG::IInt32Property& i_prop, chrono_t i_time);

//-*****************************************************************************
int64_t Interpolate(AbcG::IInt64Property& i_prop, chrono_t i_time);

//-*****************************************************************************
float Interpolate(AbcG::IFloatProperty& iProp, chrono_t iTime);

//-*****************************************************************************
double Interpolate(AbcG::IDoubleProperty& iProp, chrono_t iTime);

//-*****************************************************************************
V3f Interpolate(AbcG::IV3fProperty& i_prop, chrono_t i_time);

//-*****************************************************************************
V3d Interpolate(AbcG::IV3dProperty& i_prop, chrono_t i_time);

//-*****************************************************************************
V3f Interpolate(AbcG::IP3fProperty& i_prop, chrono_t i_time);

//-*****************************************************************************
V3d Interpolate(AbcG::IP3dProperty& i_prop, chrono_t i_time);

//-*****************************************************************************
C3f Interpolate(AbcG::IC3fProperty& i_prop, chrono_t i_time);

//-*****************************************************************************
N3f Interpolate(AbcG::IN3fProperty& i_prop, chrono_t i_time);

//-*****************************************************************************
N3d Interpolate(AbcG::IN3dProperty& i_prop, chrono_t i_time);

//-*****************************************************************************
std::string Interpolate(AbcG::IStringProperty& i_prop, chrono_t i_time);

//-*****************************************************************************
M44d Interpolate(AbcG::IXformSchema& iXform, chrono_t iTime);

}  // End namespace AbcMeshesScene
}  // End namespace EmldCore

#endif
