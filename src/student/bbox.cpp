
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer): Task 3
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.
    float tmin, tmax;
    for(int i = 0; i < 3; ++i) {
        tmin = (min[i] - ray.point[i]) / ray.dir[i];
        tmax = (max[i] - ray.point[i]) / ray.dir[i];
        if(tmin > tmax) {
            std::swap(tmin, tmax);
        }
        if(tmin > times.x) {
            times.x = tmin;
        } 
        if(tmax < times.y) {
            times.y = tmax;
        }
        if(times.x > times.y) {
            return false;
        }
    }
    return true;
}
