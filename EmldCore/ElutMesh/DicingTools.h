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

#ifndef _EmldCore_ElutMesh_DicingTools_h_
#define _EmldCore_ElutMesh_DicingTools_h_

#include "Foundation.h"

namespace EmldCore {
namespace ElutMesh {

//-*****************************************************************************
inline bool IsPowerOf2( unsigned int x )
{
    return x && !( x & ( x - 1 ) );
}

//-*****************************************************************************
inline bool IsPositivePowerOf2( int x )
{
    return ( x > 0 ) && IsPowerOf2( ( unsigned int )x );
}

//-*****************************************************************************
// ((N-1)*(2^p)) + 1 = x
// (N-1)*(2^p) = (x-1)
// (2^p) = (x-1)/(N-1)
inline bool IsValidGridSize( int x, int N )
{
    int div = ( x - 1 ) / ( N - 1 );
    return IsPositivePowerOf2( div ) &&
           ( ( div * ( N - 1 ) ) == ( x - 1 ) );
}

//-*****************************************************************************
inline bool IsValidGridSize( const V3i& v, int N )
{
    return IsValidGridSize( v.x, N ) &&
           IsValidGridSize( v.y, N ) &&
           IsValidGridSize( v.z, N );
}

//-*****************************************************************************
inline bool IsValidGridBounds( const Box3i& b, int N )
{
    const V3i s = ( b.max - b.min ) + V3i( 1 );
    return IsValidGridSize( s, N );
}

//-*****************************************************************************
// Return a grid size that's valid and greater than or equal to x.
inline int ValidGridSize( int x, int N )
{
    int ret = ( N - 1 ) * 2;
    while ( ( ret + 1 ) < x ) { ret *= 2; }
    return ret + 1;
}

//-*****************************************************************************
// ((N-1)*(2^p)) + 1 = x
// (N-1)*(2^p) = (x-1)
// (2^p) = (x-1)/(N-1)
inline int NumGridSubdivs( int x, int N )
{
    int subdivs = 0;
    int g = x - 1;
    const int Nm1 = N - 1;
    while ( g > Nm1 )
    {
        g /= 2;
        ++subdivs;
    }

    EMLD_ASSERT( subdivs > 0, "Should have subdivs" );
    EMLD_ASSERT( ValidGridSize( x, N ), "invalid grid size" );
    EMLD_ASSERT( g == Nm1, "incorrect remainder." );

    return subdivs;
}


//-*****************************************************************************
template <typename T>
class DiceRectangle
{
public:
    typedef T value_type;
    typedef Imath::Euler<T> EulerT;
    typedef Imath::Vec3<T> V3T;
    typedef Imath::Box<V3T> B3T;
    typedef Imath::Matrix44<T> M44T;

    //
    struct Parameters
    {
        V3T center;
        V3T size;
        EulerT rotation;
        T detailSize;
        int microGridSize;

        Parameters()
            : center( 0.0f, 0.0f, 0.0f )
            , size( 10.0f, 10.0f, 10.0f )
            , rotation( V3T( T( 0 ) ) )
            , detailSize( 0.05 )
            , microGridSize( 8 )
        {}
    };

    DiceRectangle() {}
    DiceRectangle( const Parameters& i_params )
        : m_params( i_params )
    {
        // We define a coordinate system that is 0,0,0 at the center,
        // and steps in "detailSize" steps until it reaches the extent.

        // Compute matrix.
        {
            // Convert into translation, rotation, and center of interest.
            M44T rotM = m_params.rotation.toMatrix44();
            M44T transM;
            transM.setTranslation( m_params.center );

            m_rectToObject = rotM * transM;
        }
        //std::cout << "Rect To Object: " << m_rectToObject << std::endl;

        // Invert it to get objectToRect
        m_objectToRect = m_rectToObject.gjInverse();
        //std::cout << "Object to Rect: " << m_objectToRect << std::endl;

        m_cellBounds.max.x =
            std::ceil( m_params.size.x /
                                  ( T( 2 ) * m_params.detailSize ) );
        m_cellBounds.max.y =
            std::ceil( m_params.size.y /
                                  ( T( 2 ) * m_params.detailSize ) );
        m_cellBounds.max.z =
            std::ceil( m_params.size.z /
                                  ( T( 2 ) * m_params.detailSize ) );

        m_cellBounds.min = -m_cellBounds.max;
        //std::cout << "Cell bounds: " << m_cellBounds << std::endl;

        // Create initial grid bounds.
        int gridLargestXYZ =
            std::max( std::max( m_cellBounds.size().x,
                                m_cellBounds.size().y ),
                      m_cellBounds.size().z );

        // std::cout << "grid largest xyz: " << gridLargestXYZ << std::endl;
        int gridN = ValidGridSize( gridLargestXYZ, m_params.microGridSize );
        // std::cout << "grid n: " << gridN << std::endl;
        int gridMaxXYZ = ( gridN - 1 ) / 2;
        // std::cout << "grid max xyz: " << gridMaxXYZ << std::endl;
        int gridMinXYZ = -gridMaxXYZ;

        m_initGridBounds.min = V3i( gridMinXYZ, gridMinXYZ, gridMinXYZ );
        m_initGridBounds.max = V3i( gridMaxXYZ, gridMaxXYZ, gridMaxXYZ );
        EMLD_DEBUG_ASSERT( IsValidGridBounds( m_initGridBounds,
                                              m_params.microGridSize ),
                           "Bad Init Grid Bounds" );

        m_initGridSubdivs = NumGridSubdivs( gridN, m_params.microGridSize );
    }

    const M44T& rectObject() const { return m_rectToObject; }
    const M44T& objectToRect() const { return m_objectToRect; }

    V3T cellToRectangle( const V3i& i_cell ) const
    {
        return m_params.detailSize * V3T( i_cell );
    }

    V3T cellToObject( const V3i& i_cell ) const
    {
        return cellToRectangle( i_cell ) * m_rectToObject;
    }

    const Box3i& cellBounds() const { return m_cellBounds; }

    const Box3i& initGridBounds() const { return m_initGridBounds; }

    int initGridSubdivs() const { return m_initGridSubdivs; }

    B3T cellToRectangle( const V3i& i_cellMin,
                         const V3i& i_cellMax ) const
    {
        return B3T( cellToRectangle( i_cellMin ),
                    cellToRectangle( i_cellMax ) );
    }

    B3T cellToObject( const V3i& i_cellMin,
                      const V3i& i_cellMax ) const
    {
        return Imath::transform( cellToRectangle( i_cellMin, i_cellMax ),
                                 m_rectToObject );
    }

    B3T cellToObject( const Box3i& i_cellBounds ) const
    {
        return cellToObject( i_cellBounds.min, i_cellBounds.max );
    }

    const Parameters& params() const { return m_params; }

    V3T operator()( const V3i& i_cell ) const
    { return cellToObject( i_cell ); }

    B3T operator()( const Box3i& i_cellBounds ) const
    {
        return cellToObject( i_cellBounds.min, i_cellBounds.max );
    }

    int microGridSize() const { return m_params.microGridSize; }

    void cellToObjectFilterAxes( const V3i& i_corner,
                                 V3T& o_Pobj,
                                 V3T& o_dPdu,
                                 V3T& o_dPdv,
                                 V3T& o_dPdw ) const
    {
        o_Pobj = cellToObject( i_corner );
        const V3T pU =
            cellToObject( V3i( i_corner.x + 1, i_corner.y, i_corner.z ) );
        const V3T pV =
            cellToObject( V3i( i_corner.x, i_corner.y + 1, i_corner.z ) );
        const V3T pW =
            cellToObject( V3i( i_corner.x, i_corner.y, i_corner.z + 1 ) );

        o_dPdu = pU - o_Pobj;
        o_dPdv = pV - o_Pobj;
        o_dPdw = pW - o_Pobj;
    }

protected:
    Parameters m_params;

    M44T m_rectToObject;
    M44T m_objectToRect;
    Box3i m_cellBounds;

    Box3i m_initGridBounds;
    int m_initGridSubdivs;
};

//-*****************************************************************************
typedef DiceRectangle<float> DiceRectanglef;
typedef DiceRectangle<double> DiceRectangled;

//-*****************************************************************************
template <typename T>
class DiceFrustum
{
public:
    typedef T value_type;
    typedef Imath::Vec3<T> V3T;
    typedef Imath::Box<V3T> B3T;
    typedef Imath::Matrix44<T> M44T;

    // CJH HACK : This should eventually be a more robust camera model.
    struct Parameters
    {
        V3T eye;
        V3T at;
        V3T up;
        T near;
        T far;
        T fovy;
        T aspect;
        T shadingRate;
        int resolution;
        int microGridSize;

        Parameters()
            : eye( 0.0f, 0.0f, 25.0f )
            , at( 50.0f, 50.0f, 0.0f )
            , up( 0.0f, 0.0f, 1.0f )
            , near( 10.0f )
            , far( 300.0f )
            , fovy( 60.0f )
            , aspect( 4.0f / 3.0f )
            , shadingRate( 4.0f )
            , resolution( 2048 )
            , microGridSize( 8 )
        {}
    };

    DiceFrustum( const Parameters& i_params )
        : m_params( i_params )
    {
        // Compute model-view matrix.
        {
            // Convert into translation, rotation, and center of interest.
            V3T translation = m_params.eye;

            const V3T dt = m_params.at - m_params.eye;

            const T xyLen = std::sqrt( ( dt.x * dt.x ) +
                                                  ( dt.y * dt.y ) );

            V3T rotation( std::atan2( dt.z, xyLen ),
                          T( 0 ),
                          std::atan2( -dt.x, dt.y ) );

            // Now convert into matrices.
            // CJH HACK: This should use our actual input "up" vector,
            // rather than assuming Z-up.
            M44T ZupToYup;
            ZupToYup.setAxisAngle( V3T( T( 1 ), T( 0 ), T( 0 ) ),
                                   radians( T( -90 ) ) );

            M44T UnRotY;
            UnRotY.setAxisAngle( V3T( T( 0 ), T( 1 ), T( 0 ) ),
                                 -rotation[1] );

            M44T UnRotX;
            UnRotX.setAxisAngle( V3T( T( 1 ), T( 0 ), T( 0 ) ),
                                 -rotation[0] );

            M44T UnRotZ;
            UnRotZ.setAxisAngle( V3T( T( 0 ), T( 0 ), T( 1 ) ),
                                 -rotation[2] );

            M44T UnTranslate;
            UnTranslate.setTranslation( -translation );

            m_objectToRhCamera =
                UnTranslate * UnRotZ * UnRotX * UnRotY * ZupToYup;
        }
        //std::cout << "Object To Rh Camera: "
        // << m_objectToRhCamera << std::endl;

        // Invert it to get cameraToObject
        m_rhCameraToObject = m_objectToRhCamera.gjInverse();
        //std::cout << "Rh Camera to Object: "
        // << m_rhCameraToObject << std::endl;

        // Make rh camera to screen
        Imath::Frustum<T> projFrust;
        projFrust.set( m_params.near, m_params.far,
                       0.0,
                       radians( m_params.fovy ),
                       m_params.aspect );

        m_rhCameraToScreen = projFrust.projectionMatrix();
        //std::cout << "Rh Camera to Screen: "
        // << m_rhCameraToScreen << std::endl;

        // Invert to make screen to rh camera.
        m_screenToRhCamera = m_rhCameraToScreen.gjInverse();
        //std::cout << "Screen to Rh Camera: "
        // << m_screenToRhCamera << std::endl;

        // Multiply to get screen to object.
        m_screenToObject = m_screenToRhCamera * m_rhCameraToObject;

        // Invert to get object to screen.
        m_objectToScreen = m_screenToObject.gjInverse();

        // We want to define an integer coordinate space
        // that is 0,0 at the lens axis. On the near plane,
        {
            T sw = T( m_params.resolution ) / m_params.shadingRate;
            T sh = sw / m_params.aspect;

            T xSizeNear = projFrust.right() - projFrust.left();
            //std::cout << "x-size near: " << xSizeNear << std::endl;
            m_cellDZ0 = xSizeNear / sw;
            //std::cout << "cell dz0: " << m_cellDZ0 << std::endl;

            m_cellZ0 = projFrust.hither();
            //std::cout << "cell z0: " << m_cellZ0 << std::endl;
            m_cellBase = T( 1 ) + ( m_cellDZ0 / m_cellZ0 );
            //std::cout << "cell base: " << m_cellBase << std::endl;

            m_cellBounds.max.x = ( int )std::ceil( sw / T( 2 ) );
            m_cellBounds.max.y = ( int )std::ceil( sh / T( 2 ) );

            m_cellBounds.min.x = -m_cellBounds.max.x;
            m_cellBounds.min.y = -m_cellBounds.max.y;

            m_cellBounds.min.z = 0;

            // zi = z0 * pow( base, i )
            // log( zi / z0 ) = log( pow( base, i ) )
            // log( zi / z0 ) = log( base ) * i
            // i = log( zi / z0 ) / log( base )

            m_cellBounds.max.z =
                ( int )
                std::ceil(
                    std::log( projFrust.yon() / m_cellZ0 ) /
                    std::log( m_cellBase ) );
            //std::cout << "cell bounds: " << m_cellBounds << std::endl;
        }

        // Dice frustum contains a cell bounds that encompasses the dicing
        // frustum. We need to figure out what the right grid size is for our
        // maximum of x & y & z.
        int gridLargestXYZ =
            std::max( std::max( m_cellBounds.size().x,
                                m_cellBounds.size().y ),
                      m_cellBounds.size().z );

        //std::cout << "grid largest xyz: " << gridLargestXYZ << std::endl;
        int gridN = ValidGridSize( gridLargestXYZ, m_params.microGridSize );
        //std::cout << "grid n: " << gridN << std::endl;
        int gridMaxXY = ( gridN - 1 ) / 2;
        //std::cout << "grid max xy: " << gridMaxXY << std::endl;
        int gridMinXY = -gridMaxXY;
        int gridMinZ = 0;
        int gridMaxZ = gridN - 1;

        m_initGridBounds.min = V3i( gridMinXY, gridMinXY, gridMinZ );
        m_initGridBounds.max = V3i( gridMaxXY, gridMaxXY, gridMaxZ );
        EMLD_DEBUG_ASSERT( IsValidGridBounds( m_initGridBounds,
                                              m_params.microGridSize ),
                           "Bad Init Grid Bounds" );

        m_initGridSubdivs = NumGridSubdivs( gridN, m_params.microGridSize );
    }

    const M44T& rhCameraToObject() const { return m_rhCameraToObject; }
    const M44T& objectToRhCamera() const { return m_objectToRhCamera; }
    //const M44T& rhCameraToScreen() const { return m_rhCameraToScreen; }
    //const M44T& screenToRhCamera() const { return m_screenToRhCamera; }
    //const M44T& screenToObject() const { return m_screenToObject; }
    //const M44T& objectToScreen() const { return m_objectToScreen; }

    V3T cellToRhCamera( const V3i& i_cell ) const
    {
        T depth = m_cellZ0 *
                  std::pow( m_cellBase, T( i_cell.z ) );
        T camX = T( i_cell.x ) * m_cellDZ0 * ( depth / m_cellZ0 );
        T camY = T( i_cell.y ) * m_cellDZ0 * ( depth / m_cellZ0 );

        //std::cout << "cell: " << i_cell << ", rhcam: "
        //          << V3T( camX, camY, -depth ) << std::endl;

        return V3T( camX, camY, -depth );
    }

    V3T cellToObject( const V3i& i_cell ) const
    {
        V3T ret = cellToRhCamera( i_cell ) * m_rhCameraToObject;
        //std::cout << "cell: " << i_cell << ", object: "
        //          << ret << std::endl;
        return ret;
    }

    V3T operator()( const V3i& i_cell ) const
    {
        return cellToObject( i_cell );
    }

    const Box3i& cellBounds() const { return m_cellBounds; }

    const Box3i& initGridBounds() const { return m_initGridBounds; }

    int initGridSubdivs() const { return m_initGridSubdivs; }

    T cellDZ0() const { return m_cellDZ0; }
    T cellZ0() const { return m_cellZ0; }
    T cellBase() const { return m_cellBase; }

    B3T cellToObject( const V3i& i_cellMin,
                      const V3i& i_cellMax ) const
    {
        B3T wbnds;
        wbnds.makeEmpty();
        V3i p;

        p.x = i_cellMin.x;
        p.y = i_cellMin.y;
        p.z = i_cellMin.z;
        wbnds.extendBy( cellToObject( p ) );

        p.x = i_cellMax.x;
        p.y = i_cellMin.y;
        p.z = i_cellMin.z;
        wbnds.extendBy( cellToObject( p ) );

        p.x = i_cellMin.x;
        p.y = i_cellMax.y;
        p.z = i_cellMin.z;
        wbnds.extendBy( cellToObject( p ) );

        p.x = i_cellMax.x;
        p.y = i_cellMax.y;
        p.z = i_cellMin.z;
        wbnds.extendBy( cellToObject( p ) );

        p.x = i_cellMin.x;
        p.y = i_cellMin.y;
        p.z = i_cellMax.z;
        wbnds.extendBy( cellToObject( p ) );

        p.x = i_cellMax.x;
        p.y = i_cellMin.y;
        p.z = i_cellMax.z;
        wbnds.extendBy( cellToObject( p ) );

        p.x = i_cellMin.x;
        p.y = i_cellMax.y;
        p.z = i_cellMax.z;
        wbnds.extendBy( cellToObject( p ) );

        p.x = i_cellMax.x;
        p.y = i_cellMax.y;
        p.z = i_cellMax.z;
        wbnds.extendBy( cellToObject( p ) );

        return wbnds;
    }

    B3T cellToObject( const Box3i& i_cellBounds ) const
    {
        return cellToObject( i_cellBounds.min, i_cellBounds.max );
    }

    B3T operator()( const Box3i& i_cellBounds ) const
    {
        return cellToObject( i_cellBounds );
    }

    int microGridSize() const { return m_params.microGridSize; }

    void cellToObjectFilterAxes( const V3i& i_corner,
                                 V3T& o_Pobj,
                                 V3T& o_dPdu,
                                 V3T& o_dPdv,
                                 V3T& o_dPdw ) const
    {
        o_Pobj = cellToObject( i_corner );
        const V3T pU =
            cellToObject( V3i( i_corner.x + 1, i_corner.y, i_corner.z ) );
        const V3T pV =
            cellToObject( V3i( i_corner.x, i_corner.y + 1, i_corner.z ) );
        const V3T pW =
            cellToObject( V3i( i_corner.x, i_corner.y, i_corner.z + 1 ) );

        o_dPdu = pU - o_Pobj;
        o_dPdv = pV - o_Pobj;
        o_dPdw = pW - o_Pobj;
    }


protected:
    Parameters m_params;

    M44T m_rhCameraToObject;
    M44T m_objectToRhCamera;

    M44T m_rhCameraToScreen;
    M44T m_screenToRhCamera;

    M44T m_screenToObject;
    M44T m_objectToScreen;

    T m_cellDZ0;
    T m_cellZ0;
    T m_cellBase;
    Box3i m_cellBounds;

    Box3i m_initGridBounds;
    int m_initGridSubdivs;
};

//-*****************************************************************************
typedef DiceFrustum<float> DiceFrustumf;
typedef DiceFrustum<double> DiceFrustumd;

} // End namespace ElutMesh
} // End namespace EmldCore


#endif