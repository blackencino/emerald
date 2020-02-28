#include <emerald/simple_sim_viewer/std_shaders.h>

namespace emerald {
namespace simple_sim_viewer {

//-*****************************************************************************
// Header.
static const char* g_shaderHeader =
    R"(
    #version 150
)";

//-*****************************************************************************
static const char* g_simpleVertexShaderBase_P_N_C =
    R"(
    in vec3 g_Pobj;
    in vec3 g_Nobj;
    in vec3 g_Color;

    out vec3 gv_Pobj;
    out vec3 gv_Nobj;
    out vec3 gv_Color;

    void main() {
       gv_Pobj = g_Pobj;
       gv_Nobj = g_Nobj;
       gv_Color = g_Color;
       gl_Position = vec4( g_Pobj, 1 );
    }
)";

//-*****************************************************************************
static const char* g_simpleVertexShaderBase_P_N =
    R"(
    in vec3 g_Pobj;
    in vec3 g_Nobj;

    out vec3 gv_Pobj;
    out vec3 gv_Nobj;
    out vec3 gv_Color;

    void main() {
       gv_Pobj = g_Pobj;
       gv_Nobj = g_Nobj;
       gv_Color = vec3(1, 1, 1);
       gl_Position = vec4( g_Pobj, 1 );
    }
)";

//-*****************************************************************************
static const char* g_simpleVertexShaderBase_P_C =
    R"(
    in vec3 g_Pobj;
    in vec3 g_Color;

    out vec3 gv_Pobj;
    out vec3 gv_Nobj;
    out vec3 gv_Color;

    void main() {
       gv_Pobj = g_Pobj;
       gv_Nobj = vec3(1, 1, 1);
       gv_Color = g_Color;
       gl_Position = vec4( g_Pobj, 1 );
    }
)";

//-*****************************************************************************
static const char* g_transformFunctions =
    R"(
    vec3 transform( vec3 p, mat4 m ) {
        return vec3( m * vec4( p, 1.0 ) );
    }
    vec3 vtransform( vec3 v, mat4 m ) {
        return vec3( m * vec4( v, 0.0 ) );
    }
    vec3 ntransform( vec3 n, mat4 m ) {
        return normalize( vtransform( n, m ) );
    }
)";

//-*****************************************************************************
static const char* g_stdMatrices =
    R"(
    uniform mat4 projection_matrix;

    uniform mat4 world_to_rhc_matrix;
    uniform mat4 world_to_rhc_nmatrix;

    uniform mat4 rhc_to_world_matrix;
    uniform mat4 rhc_to_world_nmatrix;

    uniform mat4 obj_to_world_matrix;
    uniform mat4 obj_to_world_nmatrix;

    uniform mat4 world_to_obj_matrix;
    uniform mat4 world_to_obj_nmatrix;

    uniform vec3 g_eyeWld;
)";

//-*****************************************************************************
static const char* g_simplePointsGeometryShaderBase =
    R"(
    layout(points) in;
    layout(points, max_vertices = 1) out;
    in vec3 gv_Pobj[1];
    in vec3 gv_Nobj[1];
    in vec3 gv_Color[1];
    out vec3 gg_Pwld;
    out vec3 gg_Nwld;
    out vec3 gg_Color;

    uniform float g_pointSize;

    void main( void )
    {
       mat4 modelview_matrix =
           world_to_rhc_matrix * obj_to_world_matrix;
       mat4 pmv = projection_matrix * modelview_matrix;

       gg_Pwld = transform( gv_Pobj[0], obj_to_world_matrix );
       gg_Nwld = ntransform( gv_Nobj[0], obj_to_world_nmatrix );
       gg_Color = gv_Color[0];
       gl_Position = pmv * vec4( gv_Pobj[0], 1 );
       gl_PointSize = g_pointSize;
       EmitVertex(); //EndPrimitive();
    }
)";

//-*****************************************************************************
static const char* g_simpleTrianglesGeometryShaderBase =
    R"(
    layout(triangles) in;
    layout(triangle_strip, max_vertices = 3) out;
    in vec3 gv_Pobj[3];
    in vec3 gv_Nobj[3];
    in vec3 gv_Color[3];
    out vec3 gg_Pwld;
    out vec3 gg_Nwld;
    out vec3 gg_Color;

    void main( void )
    {
       mat4 modelview_matrix =
           world_to_rhc_matrix * obj_to_world_matrix;
       mat4 pmv = projection_matrix * modelview_matrix;

       for ( int i = 0; i < 3; ++i )
       {
           gg_Pwld = transform( gv_Pobj[i], obj_to_world_matrix );
           gg_Nwld = ntransform( gv_Nobj[i], obj_to_world_nmatrix );
           gg_Color = gv_Color[i];
           gl_Position = pmv * vec4( gv_Pobj[i], 1 );
           EmitVertex();
       }
       EndPrimitive();
    }

)";

//-*****************************************************************************
static const char* g_simpleTrianglesWireframeGeometryShaderBase =
    R"(
    layout(triangles) in;
    layout(line_strip, max_vertices = 3) out;
    in vec3 gv_Pobj[3];
    in vec3 gv_Nobj[3];
    in vec3 gv_Color[3];
    out vec3 gg_Pwld;
    out vec3 gg_Nwld;
    out vec3 gg_Color;

    void main( void )
    {
       mat4 modelview_matrix =
           world_to_rhc_matrix * obj_to_world_matrix;
       mat4 pmv = projection_matrix * modelview_matrix;

       for ( int i = 0; i < 3; ++i )
       {
           gg_Pwld = transform( gv_Pobj[i], obj_to_world_matrix );
           gg_Nwld = ntransform( gv_Nobj[i], obj_to_world_nmatrix );
           gg_Color = gv_Color[i];
           gl_Position = pmv * vec4( gv_Pobj[i], 1 );
           EmitVertex();
       }
       EndPrimitive();
    }

    )";

//-*****************************************************************************
static const char* g_specDiffuseGamma =
    R"(
    //-***************************************************************
    vec3 gammaCorrect( in vec3 col, in float g )
    {
        return vec3( pow( clamp( col.r, 0.0, 1.0 ), 1.0/g ),
                     pow( clamp( col.g, 0.0, 1.0 ), 1.0/g ),
                     pow( clamp( col.b, 0.0, 1.0 ), 1.0/g ) );
    }


    //-***************************************************************
    float kSpecular( vec3 In, vec3 Nn, vec3 Ln, float m )
    {
        vec3 Vn = -In;
        vec3 H = normalize( Ln + Vn );
        float d = dot( Nn, H ); d *= d;
        d = max( d, 0.0 );
        return pow( d, m/2 );
    }

    //-***************************************************************
    float kDiffuse( vec3 Nn, vec3 Ln )
    {
        float d = dot( Nn, Ln );
        return clamp( d, 0, 1 );
    }

)";

//-*****************************************************************************
static const char* g_keyFillFragmentShaderBase =
    R"(
    in vec3 gg_Pwld;
    in vec3 gg_Nwld;
    in vec3 gg_Color;
    out vec4 g_fragmentColor;

    uniform vec3 g_toKey;
    uniform vec3 g_keyColor;
    uniform vec3 g_toFill;
    uniform vec3 g_fillColor;
    uniform vec3 g_diffColor;
    uniform vec3 g_specColor;
    uniform float g_specExponent;

    //-***************************************************************
    float sqr( float a ) { return a * a; }

    //-***************************************************************
    float linstep( float edge0, float edge1, float t )
    {
        return clamp( ( t - edge0 ) / ( edge1 - edge0 ),
                      0, 1 );
    }

    //-***************************************************************
    void main()
    {
        vec3 I = gg_Pwld - g_eyeWld;
        vec3 In = normalize( I );
        vec3 Nn = normalize( gg_Nwld );

        vec3 ToKey = normalize( g_toKey );
        vec3 ToFill = normalize( g_toFill );

        vec3 Cdiff = gg_Color * g_diffColor *
           ( ( g_keyColor * kDiffuse( Nn, ToKey ) ) +
             ( g_fillColor * kDiffuse( Nn, ToFill ) ) );
        vec3 Cspec = g_specColor *
           ( ( g_keyColor * kSpecular( In, Nn, ToKey, g_specExponent ) ) +

             ( g_fillColor * kSpecular( In, Nn, ToFill, g_specExponent ) ) );

        vec3 finalCol = gammaCorrect( Cdiff + Cspec, 2.2 );
        g_fragmentColor = vec4( finalCol, 1.0 );
    }

)";

//-*****************************************************************************
static const char* g_constantRedFragmentBase =
    R"(
    out vec4 g_fragmentColor;
    void main() { g_fragmentColor = vec4( 1, 0, 0, 1 ); }
)";

//-*****************************************************************************
static const char* g_constantWhiteFragmentBase =
    R"(
    out vec4 g_fragmentColor;
    void main() { g_fragmentColor = vec4( 1, 1, 1, 1 ); }
)";

//-*****************************************************************************
std::string StdShaderHeader() {
    return std::string{g_shaderHeader};
}

//-*****************************************************************************
std::string StdMatrices() {
    return std::string{g_stdMatrices};
}

//-*****************************************************************************
std::string StdTransformFunctions() {
    return std::string{g_transformFunctions};
}

//-*****************************************************************************
std::string StdSpecDiffuseGammaFunctions() {
    return std::string{g_specDiffuseGamma};
}

//-*****************************************************************************
std::string SimpleVertexShader() {
    return StdShaderHeader() + StdMatrices() + StdTransformFunctions() +
           std::string{g_simpleVertexShaderBase_P_N_C};
}

std::string SimpleVertexShader_P_N() {
    return StdShaderHeader() + StdMatrices() + StdTransformFunctions() +
           std::string{g_simpleVertexShaderBase_P_N};
}

std::string SimpleVertexShader_P_C() {
    return StdShaderHeader() + StdMatrices() + StdTransformFunctions() +
           std::string{g_simpleVertexShaderBase_P_C};
}

//-*****************************************************************************
std::string SimplePointsGeometryShader() {
    return StdShaderHeader() + StdMatrices() + StdTransformFunctions() +
           std::string(g_simplePointsGeometryShaderBase);
}

//-*****************************************************************************
std::string SimpleTrianglesGeometryShader() {
    return StdShaderHeader() + StdMatrices() + StdTransformFunctions() +
           std::string(g_simpleTrianglesGeometryShaderBase);
}

//-*****************************************************************************
std::string SimpleTrianglesWireframeGeometryShader() {
    return StdShaderHeader() + StdMatrices() + StdTransformFunctions() +
           std::string(g_simpleTrianglesWireframeGeometryShaderBase);
}

//-*****************************************************************************
std::string KeyFillFragmentShader() {
    return StdShaderHeader() + StdMatrices() + StdTransformFunctions() +
           StdSpecDiffuseGammaFunctions() +
           std::string(g_keyFillFragmentShaderBase);
}

//-*****************************************************************************
std::string ConstantRedFragmentShader() {
    return StdShaderHeader() + std::string(g_constantRedFragmentBase);
}

//-*****************************************************************************
std::string ConstantWhiteFragmentShader() {
    return StdShaderHeader() + std::string(g_constantWhiteFragmentBase);
}

//-*****************************************************************************
static M44d nmatrix(const M44d& m) {
    V3d s;
    V3d h;
    Imath::Eulerd r;
    V3d t;
    bool status = Imath::extractSHRT(m, s, h, r, t);
    EMLD_ASSERT(status, "Should have good matrix");

    M44d S;
    S.setScale(s);
    M44d H;
    H.setShear(h);
    M44d R = r.toMatrix44();

    M44d nm = S * H * R;
    nm.gjInvert();
    nm.transpose();
    return nm;
}

//-*****************************************************************************
//  "uniform mat4 projection_matrix;                            \n"
//  "uniform mat4 world_to_rhc_matrix;                          \n"
//  "uniform mat4 obj_to_world_matrix;                          \n"
//  "uniform mat4 world_to_obj_matrix;                          \n"
//  "uniform mat4 rhc_to_world_matrix;                          \n"
//  "uniform vec3 g_eyeWld;                                     \n";
void SetStdMatrices(Program& o_program,
                    const GLCamera& i_cam,
                    const M44d& i_objectToWorld) {
    M44d proj = projection_matrix(i_cam);
    M44d world_to_rhc = model_view_matrix(i_cam);
    M44d world_to_rhc_n = nmatrix(world_to_rhc);

    M44d rhc_to_world = world_to_rhc.gjInverse();
    M44d rhc_to_world_n = nmatrix(rhc_to_world);

    M44d world_to_obj = i_objectToWorld.gjInverse();
    M44d world_to_obj_n = nmatrix(world_to_obj);

    M44d obj_to_world_n = nmatrix(i_objectToWorld);

    o_program(Uniform("projection_matrix", proj));
    o_program(Uniform("world_to_rhc_matrix", world_to_rhc));
    o_program(Uniform("world_to_rhc_nmatrix", world_to_rhc_n));

    o_program(Uniform("rhc_to_world_matrix", rhc_to_world));
    o_program(Uniform("rhc_to_world_nmatrix", rhc_to_world_n));

    o_program(Uniform("world_to_obj_matrix", world_to_obj));
    o_program(Uniform("world_to_obj_nmatrix", world_to_obj_n));

    o_program(Uniform("obj_to_world_matrix", i_objectToWorld));
    o_program(Uniform("obj_to_world_nmatrix", obj_to_world_n));

    // Eye location.
    const V3d& eye = i_cam.translation;
    o_program(Uniform("g_eyeWld", eye /*, Uniform::kRequireWarning*/));
}

//-*****************************************************************************
//  "uniform mat4 projection_matrix;                            \n"
//  "uniform mat4 world_to_rhc_matrix;                          \n"
//  "uniform mat4 obj_to_world_matrix;                          \n"
//  "uniform mat4 world_to_obj_matrix;                          \n"
//  "uniform mat4 rhc_to_world_matrix;                          \n"
//  "uniform vec3 g_eyeWld;                                     \n";
void SetStdMatrices(Program& o_program,
                    const GLCamera2D& i_cam,
                    const M44d& i_objectToWorld) {
    M44d proj = projection_matrix(i_cam);
    M44d world_to_rhc = model_view_matrix(i_cam);
    M44d world_to_rhc_n = nmatrix(world_to_rhc);

    M44d rhc_to_world = world_to_rhc.gjInverse();
    M44d rhc_to_world_n = nmatrix(rhc_to_world);

    M44d world_to_obj = i_objectToWorld.gjInverse();
    M44d world_to_obj_n = nmatrix(world_to_obj);

    M44d obj_to_world_n = nmatrix(i_objectToWorld);

    o_program(Uniform("projection_matrix", proj));
    o_program(Uniform("world_to_rhc_matrix", world_to_rhc));
    o_program(Uniform("world_to_rhc_nmatrix", world_to_rhc_n));

    o_program(Uniform("rhc_to_world_matrix", rhc_to_world));
    o_program(Uniform("rhc_to_world_nmatrix", rhc_to_world_n));

    o_program(Uniform("world_to_obj_matrix", world_to_obj));
    o_program(Uniform("world_to_obj_nmatrix", world_to_obj_n));

    o_program(Uniform("obj_to_world_matrix", i_objectToWorld));
    o_program(Uniform("obj_to_world_nmatrix", obj_to_world_n));

    // Eye location.
    V3d const eye{i_cam.translation.x,
                  i_cam.translation.y,
                  100.0 * i_cam.world_view_height};
    o_program(Uniform("g_eyeWld", eye /*, Uniform::kRequireWarning*/));
}

//-*****************************************************************************
// "uniform vec3 g_toKey;                                             \n"
//    "uniform vec3 g_keyColor;                                          \n"
//    "uniform vec3 g_toFill;                                            \n"
//    "uniform vec3 g_fillColor;                                         \n"
void SetKeyFillLights(Program& o_program,
                      const V3f& i_toKey,
                      const V3f& i_keyColor,
                      const V3f& i_toFill,
                      const V3f& i_fillColor) {
    o_program(Uniform("g_toKey", i_toKey));
    o_program(Uniform("g_keyColor", i_keyColor));
    o_program(Uniform("g_toFill", i_toFill));
    o_program(Uniform("g_fillColor", i_fillColor));
}

//-*****************************************************************************
void SetStdMaterial(Program& o_program,
                    const V3f& i_diffColor,
                    float const vertex_color_mix,
                    const V3f& i_specColor,
                    float i_specExponent) {
    o_program(Uniform("g_diffColor", i_diffColor));
    // o_program(Uniform("g_vertex_color_mix", vertex_color_mix));
    o_program(Uniform("g_specColor", i_specColor));
    o_program(Uniform("g_specExponent", i_specExponent));
}

}  // End namespace simple_sim_viewer
}  // End namespace emerald
