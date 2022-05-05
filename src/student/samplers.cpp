
#include "../rays/samplers.h"
#include "../util/rand.h"

namespace Samplers {

Vec2 Rect::sample() const {

    // TODO (PathTracer): Task 1

    // Generate a uniformly random point on a rectangle of size size.x * size.y
    // Tip: RNG::unit()

    return Vec2{RNG::unit() * size.x, RNG::unit() * size.y};
}

Vec3 Sphere::Uniform::sample() const {

    // TODO (PathTracer): Task 7

    // Generate a uniformly random point on the unit sphere.
    // Tip: start with Hemisphere::Uniform
    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    if(RNG::coin_flip(0.5)) {
        ys = -ys;
    }
    return Vec3(xs, ys, zs);
}

Sphere::Image::Image(const HDR_Image& image) {

    // TODO (PathTracer): Task 7

    // Set up importance sampling data structures for a spherical environment map image.
    // You may make use of the _pdf, _cdf, and total members, or create your own.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;
    total = 0.0f;
    _pdf.reserve(w * h);
    _cdf.reserve(w * h);
    for(size_t i = 0; i < h; ++i) {
        for(size_t j = 0; j < w; ++j) {
            float luminance = image.at(j, i).luma();
            _pdf.push_back(luminance);
            total += luminance;
            _cdf.push_back(total);
        }
    }
}

Vec3 Sphere::Image::sample() const {

    // TODO (PathTracer): Task 7

    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound
    size_t offset = std::upper_bound(_cdf.begin(), _cdf.end(), RNG::unit() * total) - _cdf.begin();
    size_t row = offset / w;
    float theta = PI_F - row * PI_F / h;
    size_t column = offset - row * w;
    float phi = column * PI_F * 2.0f / w;
    return Vec3{std::sin(theta) * std::cos(phi), std::cos(theta), std::sin(theta) * std::sin(phi)};
}

float Sphere::Image::pdf(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // What is the PDF of this distribution at a particular direction?
    float theta = PI_F - std::acos(dir.y);
    float phi = std::atan2(dir.z, dir.x);
    if(phi < 0.f) {
        phi += PI_F * 2.f;
    }
    size_t y = theta * h / PI_F;
    size_t x = phi * w / (PI_F * 2.0f);
    size_t offset = y * w + x;
    return _pdf[offset] * w * h / (2.0 * total * PI_F * PI_F * std::sin(theta));
}

Vec3 Point::sample() const {
    return point;
}

Vec3 Triangle::sample() const {
    float u = std::sqrt(RNG::unit());
    float v = RNG::unit();
    float a = u * (1.0f - v);
    float b = u * v;
    return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

Vec3 Hemisphere::Uniform::sample() const {

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);
}

Vec3 Hemisphere::Cosine::sample() const {

    float phi = RNG::unit() * 2.0f * PI_F;
    float cos_t = std::sqrt(RNG::unit());

    float sin_t = std::sqrt(1 - cos_t * cos_t);
    float x = std::cos(phi) * sin_t;
    float z = std::sin(phi) * sin_t;
    float y = cos_t;

    return Vec3(x, y, z);
}

} // namespace Samplers
