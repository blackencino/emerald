#include <emerald/simple_sim_viewer/gl_camera.h>

#include <emerald/util/functions.h>

#include <algorithm>
#include <cmath>

namespace emerald::simple_sim_viewer {

//-*****************************************************************************
static V3d rotate_vector_around_x(double const rx, V3d const& v) {
    auto const sin_x = std::sin(rx);
    auto const cos_x = std::cos(rx);

    return V3d{
        v.x, (v.y * cos_x) - (v.z * sin_x), (v.y * sin_x) + (v.z * cos_x)};
}

static V3d rotate_vector_around_y(double const ry, V3d const& v) {
    auto const sin_y = std::sin(ry);
    auto const cos_y = std::cos(ry);

    return V3d{
        (v.x * cos_y) + (v.z * sin_y), v.y, (v.x * -sin_y) + (v.z * cos_y)};
}

static V3d rotate_vector(double const rx, double const ry, V3d const& v) {
    return rotate_vector_around_y(ry, rotate_vector_around_x(rx, v));
}

static M44d x_rotation_matrix(double const angle) {
    M44d m;
    m.setAxisAngle(V3d{1.0, 0.0, 0.0}, angle);
    return m;
}

static M44d y_rotation_matrix(double const angle) {
    M44d m;
    m.setAxisAngle(V3d{0.0, 1.0, 0.0}, angle);
    return m;
}

static M44d z_rotation_matrix(double const angle) {
    M44d m;
    m.setAxisAngle(V3d{0.0, 0.0, 1.0}, angle);
    return m;
}

static M44d translation_matrix(V3d const& translation) {
    M44d m;
    m.setTranslation(translation);
    return m;
}

static M44d scale_matrix(V3d const& scale) {
    M44d m;
    m.setScale(scale);
    return m;
}

//-*****************************************************************************
V2d auto_clipping_planes(GLCamera const& camera, Box3d const& bounds) {
    auto const bsize = bounds.size();
    auto const tiny = 0.0001 * std::min(std::min(bsize.x, bsize.y), bsize.z);

    auto clip_near = std::numeric_limits<decltype(bsize.x)>::max();
    auto clip_far = std::numeric_limits<decltype(bsize.x)>::min();

    auto const to_target =
        rotate_vector(camera.rotation.x,
                      camera.rotation.y,
                      V3d{0.0, 0.0, -camera.center_of_interest});
    auto const target = camera.translation + to_target;
    auto const to_target_dir = to_target.normalized();

    for (auto const bx : {bounds.min.x, bounds.max.x}) {
        for (auto const by : {bounds.min.y, bounds.max.y}) {
            for (auto const bz : {bounds.min.z, bounds.max.z}) {
                auto const dp = V3d{bx, by, bz} - camera.translation;
                auto const proj = dp.dot(to_target_dir);
                clip_near = std::min(proj, clip_near);
                clip_far = std::max(proj, clip_far);
            }
        }
    }

    clip_near -= tiny;
    clip_far += tiny;
    clip_near = std::clamp(clip_near, tiny, 1.0e30);
    clip_far = std::clamp(clip_far, tiny, 1.0e30);

    EMLD_ASSERT(clip_far > clip_near, "Bad clipping");

    return V2d{clip_near, clip_far};
}

//-*****************************************************************************
GLCamera frame(GLCamera const& camera, Box3d const& bounds) {
    auto const r = 0.5 * bounds.size().length();

    auto const g = (1.1 * r) / std::sin(camera.fovy * 0.5);
    auto const cen = bounds.center();
    return look_at(camera, cen + V3d{0.0, 0.0, g}, cen);
}

//-*****************************************************************************
GLCamera look_at(GLCamera const& camera, V3d const& eye, V3d const& at) {
    auto const to_target = at - eye;
    auto const xz_len = std::hypot(to_target.x, to_target.z);

    return {V3d{std::atan2(to_target.y, xz_len),
                std::atan2(to_target.x, -to_target.z),
                0.0},
            eye,
            to_target.length(),
            camera.fovy,
            camera.clip,
            camera.size};
}

//-*****************************************************************************
M44d model_view_matrix(GLCamera const& camera) {
    return translation_matrix(-camera.translation) *
           y_rotation_matrix(-camera.rotation.y) *
           x_rotation_matrix(-camera.rotation.x) *
           z_rotation_matrix(-camera.rotation.z)

        ;
}

//-*****************************************************************************
M44d projection_matrix(GLCamera const& camera) {
    Imath::Frustum<double> F;
    F.set(camera.clip[0],
          camera.clip[1],
          0.0,
          camera.fovy,
          double(camera.size.x) / double(camera.size.y));

    return F.projectionMatrix();
}

//-*****************************************************************************
GLCamera track(GLCamera const& camera, V2d const& point) {
    auto const ds =
        rotate_vector(camera.rotation.x, camera.rotation.y, V3d{1.0, 0.0, 0.0});
    auto const dt =
        rotate_vector(camera.rotation.x, camera.rotation.y, V3d{0.0, 1.0, 0.0});

    auto mult_s = 2.0 * camera.center_of_interest * std::tan(camera.fovy / 2.0);
    auto const mult_t = mult_s / double(camera.size.y);
    mult_s /= double(camera.size.x);

    auto const s = -mult_s * point.x;
    auto const t = mult_t * point.y;

    return {camera.rotation,
            camera.translation + (s * ds) + (t * dt),
            camera.center_of_interest,
            camera.fovy,
            camera.clip,
            camera.size};
}

//-*****************************************************************************
GLCamera dolly(GLCamera const& camera,
               V2d const& point,
               double const dolly_speed) {
    auto const to_target =
        rotate_vector(camera.rotation.x,
                      camera.rotation.y,
                      V3d{0.0, 0.0, -camera.center_of_interest});
    auto const target = camera.translation + to_target;
    auto const to_target_dir = to_target.normalized();

    auto const t = point.x / double(camera.size.x);
    double dolly_by = 1.0 - std::exp(-dolly_speed * t);

    EMLD_ASSERT(std::abs(dolly_by) < 1.0, "bad dolly");
    dolly_by *= camera.center_of_interest;
    auto const new_translation =
        camera.translation + (dolly_by * to_target_dir);

    return {camera.rotation,
            new_translation,
            (new_translation - target).length(),
            camera.fovy,
            camera.clip,
            camera.size};
}

//-*****************************************************************************
GLCamera rotate(GLCamera const& camera,
                V2d const& point,
                double const rotate_speed) {
    auto const to_target =
        rotate_vector(camera.rotation.x,
                      camera.rotation.y,
                      V3d{0.0, 0.0, -camera.center_of_interest});

    auto const target = camera.translation + to_target;

    auto const new_rotation =
        camera.rotation - rotate_speed * V3d{point.y / double(camera.size.y),
                                             point.x / double(camera.size.x),
                                             0.0};

    auto const new_to_target =
        rotate_vector(new_rotation.x,
                      new_rotation.y,
                      V3d{0.0, 0.0, -camera.center_of_interest});

    return {new_rotation,
            target - new_to_target,
            camera.center_of_interest,
            camera.fovy,
            camera.clip,
            camera.size};
}

//-*****************************************************************************
Imath::Line3d get_ray_through_raster_point(GLCamera const& camera,
                                           V2d const& pt_raster) {
    auto const rhc_to_world = model_view_matrix(camera).inverse();
    Imath::Frustum<double> F;
    F.set(camera.clip[0],
          camera.clip[1],
          0.0,
          camera.fovy,
          double(camera.size.x) / double(camera.size.y));

    V2d const pt_screen{
        2.0 * (pt_raster.x / double(camera.size.x)) - 1.0,
        2.0 * ((double(camera.size.y) - pt_raster.y) / double(camera.size.y)) -
            1.0};

    auto ray = F.projectScreenToRay(pt_screen);
    ray.pos = camera.translation;
    V3d new_dir;
    rhc_to_world.multDirMatrix(ray.dir, new_dir);
    new_dir.normalize();
    ray.dir = new_dir;
    return ray;
}

//------------------------------------------------------------------------------
// 2D Camera. No rotation
M44d model_view_matrix(GLCamera2D const& camera) {
    return translation_matrix(
        -V3d{camera.translation.x, camera.translation.y, 0.0});
}

M44d projection_matrix(GLCamera2D const& camera) {
    auto const scale = 2.0 / camera.world_view_height;
    return scale_matrix(V3d{scale, scale, 1.0});
}

GLCamera2D track(GLCamera2D const& camera, V2d const& point) {
    auto const scale =
        camera.world_view_height / static_cast<double>(camera.size.y);
    return {camera.translation + point * scale,
            camera.world_view_height,
            camera.size};
}

GLCamera2D dolly(GLCamera2D const& camera,
                 V2d const& point,
                 double const dolly_speed) {
    auto const t = point.x / double(camera.size.x);
    double dolly_by = 1.0 - std::exp(-dolly_speed * t);

    EMLD_ASSERT(std::abs(dolly_by) < 1.0, "bad dolly");

    return {
        camera.translation, camera.world_view_height * dolly_by, camera.size};
}

GLCamera2D frame(GLCamera2D const& camera, Box2d const& bounds) {
    return {bounds.center(), bounds.size().y, camera.size};
}

GLCamera2D look_at(GLCamera2D const& camera, V2d const& at) {
    return {at, camera.world_view_height, camera.size};
}

}  // namespace emerald::simple_sim_viewer
