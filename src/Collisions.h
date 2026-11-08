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
    // Offset subtracted from the input vertices to center the mesh on its origin
    Vec3 centroid = {0.0f, 0.0f, 0.0f};
};

struct EnergyResult {
    float volume;
    long long collisions;
    float totalEnergy;
};

struct SolverState {
    std::vector<Transform> transforms;
    std::vector<Mat3> mats;
    std::vector<AABB> tightBoxes;
    std::vector<std::vector<long long>> pairCollisions;
    long long totalCollisions = 0;
    float volume = 0.0f;
    float totalEnergy = 0.0f;
};

std::unique_ptr<BVHNode> buildBVH(const Mesh& mesh, std::vector<int>& tris);
AABB transformAABB(const AABB& local, const Mat3& mat, const Vec3& pos);
AABB computeTightAABB(const Mesh& mesh, const Mat3& mat, const Vec3& pos);

long long getMeshCollisions(const BVHNode* nodeA,
                            const Mesh& meshA,
                            const BVHNode* nodeB,
                            const Mesh& meshB,
                            const Mat3& relMatBtoA,
                            const Vec3& relPosBtoA);

long long computePairCollisions(const Mesh& meshA,
                                const Mat3& matA,
                                const Vec3& posA,
                                const AABB& boxA,
                                const Mesh& meshB,
                                const Mat3& matB,
                                const Vec3& posB,
                                const AABB& boxB);

void recomputeTotalEnergy(SolverState& state, float penaltyWeight, float progress);
SolverState initializeSolverState(const std::vector<Mesh>& meshes,
                                  const std::vector<Transform>& initialTransforms,
                                  float penaltyWeight,
                                  float progress);
