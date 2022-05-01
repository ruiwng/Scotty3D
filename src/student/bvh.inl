
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>

#define PARTITION_COUNT  12
namespace PT {

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    // Replace these

    this->max_leaf_size = max_leaf_size;
    std::vector<BBoxInfo> bounding_boxes;
    for(size_t i = 0; i < primitives.size(); ++i) {
        auto bbox = primitives[i].bbox();
        bounding_boxes.push_back({bbox, bbox.center(), i});
    }
    root_idx = build_recursively(0, primitives.size(), bounding_boxes);

    std::vector<Primitive> sorted_primitives;
    sorted_primitives.reserve(primitives.size());
    for(size_t i = 0; i < bounding_boxes.size(); ++i) {
        sorted_primitives.push_back(std::move(primitives[bounding_boxes[i].index]));
    }
    std::swap(primitives, sorted_primitives);
}

template<typename Primitive>
size_t BVH<Primitive>::build_recursively(size_t start, size_t size, std::vector<BBoxInfo> &bounding_boxes) {
    BBox box;
    for(size_t i = start; i < start + size; ++i) {
        box.enclose(bounding_boxes[i].bbox);
    }
    if(size <= max_leaf_size) {
        return new_node(box, start, size, 0, 0);
    }
    float total_cost = std::numeric_limits<float>::max();
    size_t min_dim;
    size_t min_left_count;
    for(size_t dim = 0; dim < 3; ++dim) {
        std::sort(bounding_boxes.begin() + start, bounding_boxes.begin() + start + size, 
        [&](const BBoxInfo& lhs, const BBoxInfo& rhs) -> bool {
            return lhs.center[dim] < rhs.center[dim];
        });
        // compute bounding box from left to right
        std::vector<BBox> left_to_right_bbox;
        BBox left_bbox;
        left_to_right_bbox.push_back(left_bbox);
        for(size_t j = start; j < start + size; ++j) {
            left_bbox.enclose(bounding_boxes[j].bbox);
            left_to_right_bbox.push_back(left_bbox);
        }
        
        // compute bounding box from right to left
        std::vector<BBox> right_to_left_bbox;
        BBox right_bbox;
        right_to_left_bbox.push_back(right_bbox);
        for(size_t j = start + size - 1; ; --j) {
            right_bbox.enclose(bounding_boxes[j].bbox);
            right_to_left_bbox.push_back(right_bbox);
            if(j == start) {
                break;
            }
        }

        float step_size = (box.max[dim] - box.min[dim]) / PARTITION_COUNT;
        float partition = box.min[dim];
        size_t left_count = 0;
        for(int j = 0; j < PARTITION_COUNT - 1; ++j) {
            partition += step_size;
            while(left_count < size && bounding_boxes[start + left_count].center[dim] < partition) {
                left_count += 1;
            }
            auto right_count = size - left_count;
            auto& left_box = left_to_right_bbox[left_count];
            auto& right_box = right_to_left_bbox[right_count];
            float cost = .125f + (left_count * left_box.surface_area() + right_count * right_box.surface_area()) / box.surface_area(); 
            if(cost < total_cost) {
                total_cost = cost;
                min_dim = dim;
                min_left_count = cost;
            }
            if(left_count == size) {
                break;
            }
        }
    }
    if(min_left_count == 0 || min_left_count == size) {
        return new_node(box, start, size, 0, 0);
    }
    std::sort(bounding_boxes.begin() + start, bounding_boxes.begin() + start + size, 
    [&](const BBoxInfo& lhs, const BBoxInfo& rhs) -> bool {
        return lhs.center[min_dim] < rhs.center[min_dim];
    });
    int left_child = build_recursively(start, min_left_count, bounding_boxes);
    int right_child = build_recursively(start + min_left_count, size - min_left_count, bounding_boxes);
    log("start %d, size %d, left_child %d, right_child %d", start, size, left_child, right_child);
    return new_node(box, start, size, left_child, right_child);
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    /*
    Trace ret;
    for(const Primitive& prim : primitives) {
        Trace hit = prim.hit(ray);
        ret = Trace::min(ret, hit);
    }
    return ret;
    */

    bool is_hit = nodes[root_idx].bbox.hit(ray, ray.dist_bounds);
    if(!is_hit) {
        return Trace();
    }
    return find_first_hit(nodes[root_idx], ray);
}

template<typename Primitive>
Trace BVH<Primitive>::find_first_hit(const Node& node, const Ray& ray) const {
    if(node.is_leaf()) {
        Trace ret;
        for(auto i = node.start; i < node.start + node.size; ++i) {
            Trace hit = primitives[i].hit(ray);
            ret = Trace::min(ret, hit);
        }
        return ret;
    }
    const Node& left_node = nodes[node.l];
    Vec2 left_interval = ray.dist_bounds;
    bool is_left_hit = left_node.bbox.hit(ray, left_interval);
    const Node& right_node = nodes[node.r];
    Vec2 right_interval = ray.dist_bounds;
    bool is_right_hit = right_node.bbox.hit(ray, right_interval);
    if(is_left_hit && is_right_hit) {
        const Node& first_node = left_interval[0] < right_interval[0]?left_node: right_node;
        const Node& second_node = left_interval[0] < right_interval[0]?right_node: left_node;
        const Vec2& second_interval = left_interval[0] < right_interval[0]?right_interval: left_interval;
        Trace trace = find_first_hit(first_node, ray);
        if(!trace.hit || (trace.hit && trace.distance > second_interval[0])) {
            trace = Trace::min(trace, find_first_hit(second_node, ray));
        }
        return trace;
        
    } else if(is_left_hit) {
        return find_first_hit(left_node, ray);
    } else if(is_right_hit) {
        return find_first_hit(right_node, ray);
    } else {
        return Trace();
    }
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {

    // A node is a leaf if l == r, since all interior nodes must have distinct children
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(!node.is_leaf()) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c + lvl, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
