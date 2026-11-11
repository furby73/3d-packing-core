#pragma once

#include <memory>
#include <vector>
#include "Geometry.h"

struct BVHNode {
    AABB box;
    std::unique_ptr<BVHNode> left, right;
    std::vector<int> tri_indices;
    bool isLeaf() const { return !left && !right; }
};

struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<int> indices;
    std::unique_ptr<BVHNode> bvh_root;
    Vec3 centroid = {0, 0, 0};
    // One surface vertex per connected component, for full containment checks.
    std::vector<int> componentVertices;
};

void buildMeshAcceleration(Mesh &mesh);
AABB transformAABB(const AABB &local, const Mat3 &mat, const Vec3 &pos);
AABB computeTightAABB(const Mesh &mesh, const Mat3 &mat, const Vec3 &pos);

// Exact triangle geometry (double precision), BVH-pruned. The gap is a surface
// distance, not an AABB padding. Closed components are also tested for containment.
// Zero gap permits AABB face contact; other ambiguous surface contact is conservative.
bool meshesConflict(const Mesh &a, const Mat3 &ra, const Vec3 &pa, const AABB &ba, const Mesh &b, const Mat3 &rb,
                    const Vec3 &pb, const AABB &bb, double gap);
