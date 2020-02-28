#include <emerald/simple_sim_viewer/particle_sphere_shaders.h>

#include <emerald/simple_sim_viewer/std_shaders.h>

namespace emerald {
namespace simple_sim_viewer {

// //-*****************************************************************************
// static const char* g_keyFillFragmentShaderBase =
//     R"(
//     in vec3 gg_Pwld;
//     in vec3 gg_Nwld;
//     in vec3 gg_Color;
//     out vec4 g_fragmentColor;

//     uniform vec3 g_toKey;
//     uniform vec3 g_keyColor;
//     uniform vec3 g_toFill;
//     uniform vec3 g_fillColor;
//     uniform vec3 g_diffColor;
//     uniform vec3 g_specColor;
//     uniform float g_specExponent;

//     float sqr( float a ) { return a * a; }

//     float linstep( float edge0, float edge1, float t )
//     {
//         return clamp( ( t - edge0 ) / ( edge1 - edge0 ),
//                       0, 1 );
//     }

//     void main()
//     {
//         vec3 I = gg_Pwld - g_eyeWld;
//         vec3 In = normalize( I );
//         vec3 Nn = normalize( gg_Nwld );

//         vec3 ToKey = normalize( g_toKey );
//         vec3 ToFill = normalize( g_toFill );

//         vec3 Cdiff = gg_Color * g_diffColor *
//            ( ( g_keyColor * kDiffuse( Nn, ToKey ) ) +
//              ( g_fillColor * kDiffuse( Nn, ToFill ) ) );
//         vec3 Cspec = g_specColor *
//            ( ( g_keyColor * kSpecular( In, Nn, ToKey, g_specExponent ) ) +

//              ( g_fillColor * kSpecular( In, Nn, ToFill, g_specExponent ) ) );

//         vec3 finalCol = gammaCorrect( Cdiff + Cspec, 2.2 );
//         g_fragmentColor = vec4( finalCol, 1.0 );
//     }

// )";

static auto const* const g_particle_sphere_fragment_base =
    R"(
    in vec3 gg_Pwld;
    in vec3 gg_Color;
    out vec4 g_fragmentColor;

    //uniform vec3 g_toKey;
    //uniform vec3 g_keyColor;
    //uniform vec3 g_toFill;
    //uniform vec3 g_fillColor;
    //uniform vec3 g_diffColor;
    //uniform vec3 g_specColor;
    //uniform float g_specExponent;

    float sqr( float a ) { return a * a; }

    float linstep( float edge0, float edge1, float t )
    {
        return clamp( ( t - edge0 ) / ( edge1 - edge0 ),
                      0, 1 );
    }

    void main() {
        float cenS = ( 2.0 * gl_PointCoord[0] ) - 1.0;
        float cenT = 1.0 - ( 2.0 * gl_PointCoord[1] );
        float r2 = (cenS*cenS) + (cenT*cenT);
        if ( r2 > 1 ) { discard; }
        vec3 NnEye = normalize( vec3( cenS, cenT,
                            sqrt( 1 - r2 ) ) );
        const vec3 InEye = vec3( 0, 0, -1);
        const vec3 VnEye = vec3( 0, 0, 1 );

        vec3 ToSunEye = normalize( vec3( 100, 75, -30 ) );
        vec3 ToMoonEye = normalize( vec3( -100, -10, 25 ) );
        vec3 SunCol = vec3( 3.01, 3, 2.8 );
        vec3 MoonCol = vec3( 0.5, 0.5, 0.55 );

        vec3 Cdiff = gg_Color *
        ( SunCol * kDiffuse( NnEye, ToSunEye ) +
        MoonCol * kDiffuse( NnEye, ToMoonEye ) );

        vec3 Cspec = 0.5 *
        ( SunCol * kSpecular( InEye, NnEye, ToSunEye, 400 ) +
        MoonCol * kSpecular( InEye, NnEye, ToMoonEye, 30 ) );

        vec3 finalCol = gammaCorrect( Cspec + Cdiff, 2.2 );
        g_fragmentColor = vec4( finalCol, 1.0 );
    }
)";

static auto const* const g_particle_circle_fragment_base =
    R"(
    in vec3 gg_Pwld;
    in vec3 gg_Color;
    out vec4 g_fragmentColor;

    void main() {
        float cenS = ( 2.0 * gl_PointCoord[0] ) - 1.0;
        float cenT = 1.0 - ( 2.0 * gl_PointCoord[1] );
        float r2 = (cenS*cenS) + (cenT*cenT);
        if ( r2 > 1 ) { discard; }
        vec3 finalCol = gammaCorrect( gg_Color, 2.2 );
        g_fragmentColor = vec4( finalCol, 1.0 );
    }
)";

//-*****************************************************************************
std::string ParticleSphereFragmentShader() {
    return StdShaderHeader() + StdMatrices() + StdTransformFunctions() +
           StdSpecDiffuseGammaFunctions() +
           std::string{g_particle_sphere_fragment_base};
}

//-*****************************************************************************
std::string ParticleCircleFragmentShader() {
    return StdShaderHeader() + StdMatrices() + StdTransformFunctions() +
           std::string{g_particle_circle_fragment_base};
}

}  // End namespace simple_sim_viewer
}  // End namespace emerald
