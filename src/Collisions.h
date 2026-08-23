#pragma once

#include <memory>
#include <vector>

#include "Geometry.h"

struct BVHNode {
    AABB box;
    std::unique_ptr<BVHNode> left;
    std::unique_ptr<BVHNode> right;
    std::vector<int> tri_indices;

    bool isLeaf() const { return left == nullptr && right == nullptr; }
};

struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<int> indices;
    std::unique_ptr<BVHNode> bvh_root;
};

struct EnergyResult {
    float volume;
    long long collisions;
    float totalEnergy;
};

std::unique_ptr<BVHNode> buildBVH(const Mesh& mesh, std::vector<int>& tris);
AABB transformAABB(const AABB& local, const Mat3& mat, const Vec3& pos);
long long getMeshCollisions(const BVHNode* nodeA,
                            const Mat3& matA,
                            const Transform& ta,
                            const Mesh& meshA,
                            const BVHNode* nodeB,
                            const Mat3& matB,
                            const Transform& tb,
                            const Mesh& meshB);
EnergyResult calculateEnergy(const std::vector<Mesh>& meshes,
                             const std::vector<Transform>& state,
                             float penaltyWeight,
                             float progress);
