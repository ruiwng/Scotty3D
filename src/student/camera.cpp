
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute the position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: Compute the ray direction in camera space and use
    // the camera transform to transform it back into world space.
    float half_height = std::tanf(Radians(get_fov()) * 0.5);
    float half_width = std::tanf(Radians(get_h_fov()) * 0.5);
    Vec3 view_pos(half_width * (screen_coord.x - 0.5) * 2.0, half_height * (screen_coord.y - 0.5) * 2.0, -1.f);
    view_pos *= focal_dist;
    Vec3 new_pos = position + (RNG::unit() - 0.5) * aperture * iview[0].xyz() + (RNG::unit() - 0.5f) * aperture * iview[1].xyz();
    return Ray(new_pos, iview * view_pos - new_pos);
}
