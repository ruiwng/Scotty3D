
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!
    float a = ray.dir.norm_squared();
    float b = 2.0f * dot(ray.point, ray.dir);
    float c = ray.point.norm_squared() - radius * radius;
    float determinant = b * b - 4.0 * a * c;
    Trace ret;
    ret.hit = false;
    ret.origin = ray.point;
    if(determinant < 0.f) {
        return ret;
    }
    float sqrt_determinant = sqrt(determinant);
    float x = (-b - sqrt_determinant) / (2.f * a);
    if(x > ray.dist_bounds[1]) {
        return ret;
    } else if(x < ray.dist_bounds[0]) {
        x = (-b + sqrt_determinant) / (2.f * a);
        if(x < ray.dist_bounds[0] || x > ray.dist_bounds[1]) {
            return ret;
        }
    }
    ret.hit = true;       // was there an intersection?
    
    ret.distance = x;   // at what distance did the intersection occur?
    ret.position = ray.point + ray.dir * x; // where was the intersection?
    ret.normal = ret.position.unit();   // what was the surface normal at the intersection?
    return ret;
}

} // namespace PT
