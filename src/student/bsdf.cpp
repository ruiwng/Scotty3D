
#include "../rays/bsdf.h"
#include "../util/rand.h"

namespace PT {

static Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 5
    // Return reflection of dir about the surface normal (0,1,0).
    return Vec3(0.0f, 2.0f * dir.y, 0.0f) - dir;
}

static Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 5
    // Use Snell's Law to refract out_dir through the surface.
    // Return the refracted direction. Set was_internal to true if
    // refraction does not occur due to total internal reflection,
    // and false otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.
    float refraction_in_over_out;
    Vec3 y_axis;
    if(out_dir.y >= 0.0f) {
        refraction_in_over_out = 1.0f / index_of_refraction;
        y_axis = Vec3{0.0f, 1.0f, 0.0f};
    } else {
        refraction_in_over_out = index_of_refraction;
        y_axis = Vec3{0.0f, -1.0f, 0.0f};
    }
    Vec3 x_axis{out_dir.x, 0.0f, out_dir.z};
    float sin_theta = x_axis.norm();
    float sin_phi = refraction_in_over_out * sin_theta;
    if(sin_phi > 1.0f) {
        was_internal = true;
        return Vec3();
    }
    was_internal = false;
    float cos_phi = std::sqrt(1.0f - sin_phi * sin_phi);
    x_axis = -x_axis.unit();
    Vec3 refract_dir = sin_phi * x_axis + (-cos_phi) * y_axis;
    return refract_dir;
}

Scatter BSDF_Lambertian::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 4

    // Sample the BSDF distribution using the cosine-weighted hemisphere sampler.
    // You can use BSDF_Lambertian::evaluate() to compute attenuation.
    /*
    Scatter ret;
    ret.direction = Vec3{};
    ret.attenuation = Spectrum{};
    */
    Scatter ret;
    ret.direction = sampler.sample();
    // ret.direction = Vec3(RNG::unit() * 2.0f - 1.0f, RNG::unit(), RNG::unit() * 2.0f - 1.0f).unit();
    ret.attenuation = evaluate(out_dir, ret.direction);
    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {

    // TODO (PathTracer): Task 4

    // Compute the ratio of reflected/incoming radiance when light from in_dir
    // is reflected through out_dir: albedo * cos(theta).
    return albedo * std::max(0.0f, in_dir.unit().y);
    // return Spectrum{};
}

float BSDF_Lambertian::pdf(Vec3 out_dir, Vec3 in_dir) const {

    // TODO (PathTracer): Task 4

    // Compute the PDF for sampling in_dir from the cosine-weighted hemisphere distribution.
    return in_dir.unit().y / PI_F;
    // return 0.0f;
}

Scatter BSDF_Mirror::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5

    Scatter ret;
    ret.direction = reflect(out_dir);
    ret.attenuation = Spectrum{1.0f};
    return ret;
}

Scatter BSDF_Glass::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5

    // (1) Compute Fresnel coefficient. Tip: Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    // What happens upon total internal reflection?
    bool was_internal;
    Vec3 refract_dir = refract(out_dir, index_of_refraction, was_internal);
    Scatter ret;
    if(was_internal) {
        ret.direction = reflect(out_dir);
        ret.attenuation = Spectrum{1.0f};
    } else {
        float r0 = (1.0f - index_of_refraction) / (1.0f + index_of_refraction);
        r0 = r0 * r0;
        float fresnel = r0 + (1.0f - r0) * std::pow(1.0f - std::abs(out_dir.y), 5);
        if(RNG::coin_flip(fresnel)) {
            ret.direction = reflect(out_dir);
            ret.attenuation = Spectrum(fresnel);
        } else {
            ret.direction = refract_dir;
            ret.attenuation = Spectrum(1.0f - fresnel);
        }
    }
    return ret;
}

Scatter BSDF_Refract::scatter(Vec3 out_dir) const {

    // OPTIONAL (PathTracer): Task 5

    // When debugging BSDF_Glass, it may be useful to compare to a pure-refraction BSDF

    Scatter ret;
    ret.direction = Vec3();
    ret.attenuation = Spectrum{};
    return ret;
}

Spectrum BSDF_Diffuse::emissive() const {
    return radiance;
}

} // namespace PT
