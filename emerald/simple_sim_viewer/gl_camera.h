#pragma once

#include <emerald/simple_sim_viewer/foundation.h>
#include <emerald/util/functions.h>

#include <OpenEXR/ImathLine.h>

#include <algorithm>
#include <string>
#include <utility>

namespace emerald {
namespace simple_sim_viewer {

//------------------------------------------------------------------------------
// 3D Camera
struct GLCamera {
    // Euler angles are stupid, we should fix this.
    V3d rotation = V3d{0.0, 0.0, 0.0};
    V3d translation = V3d{0.0, 0.0, 0.0};

    double center_of_interest = 15.0;
    double fovy = radians(45.0);
    V2d clip = V2d{0.1, 1.0e30};
    V2i size = V2i{100, 100};
};

M44d model_view_matrix(GLCamera const& camera);
M44d projection_matrix(GLCamera const& camera);

V2d auto_clipping_planes(GLCamera const& camera, Box3d const& bounds);
GLCamera track(GLCamera const& camera, V2d const& point);
GLCamera dolly(GLCamera const& camera,
               V2d const& point,
               double const dollySpeed = 5.0);
GLCamera rotate(GLCamera const& camera,
                V2d const& point,
                double const rotateSpeed = radians(400.0));
GLCamera frame(GLCamera const& camera, Box3d const& bounds);
GLCamera look_at(GLCamera const& camera, V3d const& eye, V3d const& at);

// RAY STUFF - convert a screen space point, in raster space, to a
// ray from the eye.
Imath::Line3d get_ray_through_raster_point(GLCamera const& camera,
                                           V2d const& pt_raster);

//------------------------------------------------------------------------------
// 2D Camera. No rotation
struct GLCamera2D {
    V2d translation = V2d{0.0, 0.0};

    double world_view_height = 1.0;
    V2i size = V2i{100, 100};
};

M44d model_view_matrix(GLCamera2D const& camera);
M44d projection_matrix(GLCamera2D const& camera);

GLCamera2D track(GLCamera2D const& camera, V2d const& point);
GLCamera2D dolly(GLCamera2D const& camera,
                 V2d const& point,
                 double const dollySpeed = 5.0);
GLCamera2D frame(GLCamera2D const& camera, Box2d const& bounds);
GLCamera2D look_at(GLCamera2D const& camera, V2d const& at);

}  // End namespace simple_sim_viewer
}  // End namespace emerald
