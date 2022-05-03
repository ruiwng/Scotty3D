
#include "../rays/env_light.h"

#include <limits>

namespace PT {

Vec3 Env_Map::sample() const {

    // TODO (PathTracer): Task 7

    // First, implement Samplers::Sphere::Uniform so the following line works.
    // Second, implement Samplers::Sphere::Image and swap to image_sampler

    return uniform_sampler.sample();
}

float Env_Map::pdf(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // First, return the pdf for a uniform spherical distribution.
    // Second, swap to image_sampler.pdf().

    return 1.0f / (4.0f * PI_F);
}

Spectrum Env_Map::evaluate(Vec3 dir) const {

    float theta = 1.0 - std::acos(dir.y) / PI_F;
    float phi = std::atan2(dir.z, dir.x);
    if(phi < 0.f) {
        phi += PI_F * 2.f;
    }
    phi /= PI_F * 2.f;
    auto dimension = image.dimension();
    float x = phi * (dimension.first - 1);
    float y = theta * (dimension.second - 1);
    
    size_t x_floor = floor(x);
    size_t y_floor = floor(y);
    size_t x_ceil = std::min(dimension.first - 1, x_floor + 1);
    size_t y_ceil = std::min(dimension.second - 1, y_floor + 1);
    float x_weight = x - x_floor;
    float y_weight = y - y_floor;

    Spectrum v1 = image.at(x_floor, y_floor) * (1.0 - y_weight) + image.at(x_floor, y_ceil) * y_weight;
    Spectrum v2 = image.at(x_ceil, y_floor) * (1.0 - y_weight) + image.at(x_ceil, y_ceil) * y_weight;

    // TODO (PathTracer): Task 7

    // Compute emitted radiance along a given direction by finding the corresponding
    // pixels in the enviornment image. You should bi-linearly interpolate the value
    // between the 4 nearest pixels.

    return v1 * (1.0f - x_weight) + v2 * x_weight;
}

Vec3 Env_Hemisphere::sample() const {
    return sampler.sample();
}

float Env_Hemisphere::pdf(Vec3 dir) const {
    return 1.0f / (2.0f * PI_F);
}

Spectrum Env_Hemisphere::evaluate(Vec3 dir) const {
    if(dir.y > 0.0f) return radiance;
    return {};
}

Vec3 Env_Sphere::sample() const {
    return sampler.sample();
}

float Env_Sphere::pdf(Vec3 dir) const {
    return 1.0f / (4.0f * PI_F);
}

Spectrum Env_Sphere::evaluate(Vec3) const {
    return radiance;
}

} // namespace PT
