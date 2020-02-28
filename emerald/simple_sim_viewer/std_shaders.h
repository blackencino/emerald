#pragma once

#include <emerald/simple_sim_viewer/foundation.h>
#include <emerald/simple_sim_viewer/gl_camera.h>

#include <emerald/geep_glfw/program.h>

#include <string>

namespace emerald {
namespace simple_sim_viewer {

//-*****************************************************************************
std::string StdShaderHeader();
std::string StdMatrices();
std::string StdTransformFunctions();
std::string StdSpecDiffuseGammaFunctions();
std::string SimpleVertexShader();
std::string SimpleVertexShader_P_N();
std::string SimpleVertexShader_P_C();
std::string SimplePointsGeometryShader();
std::string SimpleTrianglesGeometryShader();
std::string SimpleTrianglesWireframeGeometryShader();
std::string KeyFillFragmentShader();
std::string ConstantRedFragmentShader();
std::string ConstantWhiteFragmentShader();
void SetStdMatrices(Program& o_program,
                    const GLCamera& i_cam,
                    const M44d& i_objectToWorld);
void SetKeyFillLights(Program& o_program,
                      const V3f& i_toKey,
                      const V3f& i_keyColor,
                      const V3f& i_toFill,
                      const V3f& i_fillColor);
void SetStdMaterial(Program& o_program,
                    const V3f& i_diffColor,
                    float const vertex_color_mix,
                    const V3f& i_specColor,
                    float i_specExponent);

}  // End namespace simple_sim_viewer
}  // End namespace emerald
