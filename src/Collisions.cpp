#include "Collisions.h"

#include <algorithm>
#include <cmath>

namespace {

bool overlapOnAxis(const Vec3& axis, const Vec3 p[3], const Vec3 q[3], float epsSq) {
    if (axis.lengthSq() <= epsSq) {
        return true;
    }

    float minA = dot(axis, p[0]);
    float maxA = minA;
    float minB = dot(axis, q[0]);
    float maxB = minB;

    for (int i = 1; i < 3; ++i) {
        const float dA = dot(axis, p[i]);
        minA = std::min(minA, dA);
        maxA = std::max(maxA, dA);

        const float dB = dot(axis, q[i]);
        minB = std::min(minB, dB);
        maxB = std::max(maxB, dB);
    }

    return !(maxA < minB || maxB < minA);
}

bool trianglesIntersect(const Vec3 p[3], const Vec3 q[3]) {
    constexpr float kRelEpsSq = 1e-12f;
    const Vec3 eA[3] = {p[1] - p[0], p[2] - p[1], p[0] - p[2]};
    const Vec3 eB[3] = {q[1] - q[0], q[2] - q[1], q[0] - q[2]};

    const float lenSqA[3] = {eA[0].lengthSq(), eA[1].lengthSq(), eA[2].lengthSq()};
    const float lenSqB[3] = {eB[0].lengthSq(), eB[1].lengthSq(), eB[2].lengthSq()};

    if (!overlapOnAxis(cross(eA[0], eA[1]), p, q, kRelEpsSq * lenSqA[0] * lenSqA[1])) {
        return false;
    }
    if (!overlapOnAxis(cross(eB[0], eB[1]), p, q, kRelEpsSq * lenSqB[0] * lenSqB[1])) {
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!overlapOnAxis(cross(eA[i], eB[j]), p, q, kRelEpsSq * lenSqA[i] * lenSqB[j])) {
                return false;
            }
        }
    }

    return true;
}

bool rayIntersectsTriangle(const Vec3& ro, const Vec3& rd, const Vec3& v0, const Vec3& v1, const Vec3& v2) {
    constexpr float kEpsilon = 1e-7f;
    const Vec3 edge1 = v1 - v0;
    const Vec3 edge2 = v2 - v0;
    const Vec3 h = cross(rd, edge2);
    const float a = dot(edge1, h);
    if (a > -kEpsilon && a < kEpsilon) {
        return false;
    }

    const float f = 1.0f / a;
    const Vec3 s = ro - v0;
    const float u = f * dot(s, h);
    if (u < 0.0f || u > 1.0f) {
        return false;
    }

    const Vec3 q = cross(s, edge1);
    const float v = f * dot(rd, q);
    if (v < 0.0f || (u + v) > 1.0f) {
        return false;
    }

    const float t = f * dot(edge2, q);
    return t > kEpsilon;
}

bool rayIntersectsAABB(const Vec3& ro, const Vec3& invRd, const AABB& box) {
    const float tx1 = (box.min.x - ro.x) * invRd.x;
    const float tx2 = (box.max.x - ro.x) * invRd.x;
    float tmin = std::min(tx1, tx2);
    float tmax = std::max(tx1, tx2);

    const float ty1 = (box.min.y - ro.y) * invRd.y;
    const float ty2 = (box.max.y - ro.y) * invRd.y;
    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));

    const float tz1 = (box.min.z - ro.z) * invRd.z;
    const float tz2 = (box.max.z - ro.z) * invRd.z;
    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));

    return tmax >= std::max(0.0f, tmin);
}

Vec3 safeInverseDir(const Vec3& rd) {
    const auto inv = [](float v) {
        return std::abs(v) > 1e-8f ? (1.0f / v) : (v >= 0.0f ? 1e8f : -1e8f);
    };
    return {inv(rd.x), inv(rd.y), inv(rd.z)};
}

// Operates entirely in the target mesh's local space (zero matrix transforms during traversal)
int countRayHitsLocal(const BVHNode* node, const Mesh& mesh, const Vec3& localRo, const Vec3& localRd,
                      const Vec3& invLocalRd) {
    if (!rayIntersectsAABB(localRo, invLocalRd, node->box)) {
        return 0;
    }

    if (node->isLeaf()) {
        int hits = 0;
        for (int i : node->tri_indices) {
            const Vec3& v0 = mesh.vertices[mesh.indices[i * 3]];
            const Vec3& v1 = mesh.vertices[mesh.indices[i * 3 + 1]];
            const Vec3& v2 = mesh.vertices[mesh.indices[i * 3 + 2]];
            if (rayIntersectsTriangle(localRo, localRd, v0, v1, v2)) {
                ++hits;
            }
        }
        return hits;
    }

    int hits = 0;
    if (node->left) {
        hits += countRayHitsLocal(node->left.get(), mesh, localRo, localRd, invLocalRd);
    }
    if (node->right) {
        hits += countRayHitsLocal(node->right.get(), mesh, localRo, localRd, invLocalRd);
    }
    return hits;
}

bool isPointInsideMesh(const Mesh& mesh, const Mat3& mat, const Vec3& pos, const Vec3& worldPt) {
    const Vec3 localRo = mat.transposeTransform(worldPt - pos);
    if (!mesh.bvh_root->box.contains(localRo)) {
        return false;
    }

    constexpr Vec3 kRayDir1 = {0.7385489f, 0.4431293f, 0.5076391f};
    const Vec3 localRd1 = mat.transposeTransform(kRayDir1);
    const int hits1 = countRayHitsLocal(mesh.bvh_root.get(), mesh, localRo, localRd1, safeInverseDir(localRd1));
    if ((hits1 & 1) == 0) {
        return false;
    }

    // Confirm with a second independent ray to prevent edge/vertex grazing false positives
    constexpr Vec3 kRayDir2 = {-0.5144958f, 0.6859943f, -0.5144958f};
    const Vec3 localRd2 = mat.transposeTransform(kRayDir2);
    const int hits2 = countRayHitsLocal(mesh.bvh_root.get(), mesh, localRo, localRd2, safeInverseDir(localRd2));
    return (hits2 & 1) != 0;
}

std::unique_ptr<BVHNode> buildBVHRange(const Mesh& mesh, const std::vector<Vec3>& centroids, std::vector<int>& tris,
                                       size_t begin, size_t end) {
    auto node = std::make_unique<BVHNode>();

    for (size_t i = begin; i < end; ++i) {
        const int idx = tris[i];
        node->box.expand(mesh.vertices[mesh.indices[idx * 3]]);
        node->box.expand(mesh.vertices[mesh.indices[idx * 3 + 1]]);
        node->box.expand(mesh.vertices[mesh.indices[idx * 3 + 2]]);
    }

    const size_t count = end - begin;
    if (count <= 4) {
        node->tri_indices.assign(tris.begin() + static_cast<std::ptrdiff_t>(begin),
                                 tris.begin() + static_cast<std::ptrdiff_t>(end));
        return node;
    }

    const Vec3 extent = node->box.max - node->box.min;
    int axis = 0;
    if (extent.y >= extent.x && extent.y >= extent.z) {
        axis = 1;
    } else if (extent.z >= extent.x && extent.z >= extent.y) {
        axis = 2;
    }

    const size_t mid = begin + count / 2;
    std::nth_element(tris.begin() + static_cast<std::ptrdiff_t>(begin),
                     tris.begin() + static_cast<std::ptrdiff_t>(mid),
                     tris.begin() + static_cast<std::ptrdiff_t>(end),
                     [&](int a, int b) {
                         if (axis == 0) return centroids[a].x < centroids[b].x;
                         if (axis == 1) return centroids[a].y < centroids[b].y;
                         return centroids[a].z < centroids[b].z;
                     });

    node->left = buildBVHRange(mesh, centroids, tris, begin, mid);
    node->right = buildBVHRange(mesh, centroids, tris, mid, end);
    return node;
}

} // namespace

std::unique_ptr<BVHNode> buildBVH(const Mesh& mesh, std::vector<int>& tris) {
    const size_t numTris = mesh.indices.size() / 3;
    std::vector<Vec3> centroids(numTris);
    for (size_t i = 0; i < numTris; ++i) {
        centroids[i] = (mesh.vertices[mesh.indices[i * 3]] +
                        mesh.vertices[mesh.indices[i * 3 + 1]] +
                        mesh.vertices[mesh.indices[i * 3 + 2]]) * (1.0f / 3.0f);
    }
    return buildBVHRange(mesh, centroids, tris, 0, tris.size());
}

// Arvo's fast AABB transformation algorithm (9 multiplies instead of 8 corner matrix transforms)
AABB transformAABB(const AABB& local, const Mat3& mat, const Vec3& pos) {
    AABB res;
    res.min = pos;
    res.max = pos;

    const float minArr[3] = {local.min.x, local.min.y, local.min.z};
    const float maxArr[3] = {local.max.x, local.max.y, local.max.z};
    float outMin[3] = {pos.x, pos.y, pos.z};
    float outMax[3] = {pos.x, pos.y, pos.z};

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const float a = mat.m[i][j] * minArr[j];
            const float b = mat.m[i][j] * maxArr[j];
            outMin[i] += std::min(a, b);
            outMax[i] += std::max(a, b);
        }
    }

    res.min = {outMin[0], outMin[1], outMin[2]};
    res.max = {outMax[0], outMax[1], outMax[2]};
    return res;
}

AABB computeTightAABB(const Mesh& mesh, const Mat3& mat, const Vec3& pos) {
    AABB tight;
    for (const Vec3& v : mesh.vertices) {
        tight.expand(mat.transform(v) + pos);
    }
    return tight;
}

long long getMeshCollisions(const BVHNode* nodeA, const Mesh& meshA, const BVHNode* nodeB, const Mesh& meshB,
                            const Mat3& relMatBtoA, const Vec3& relPosBtoA) {
    const AABB& boxA = nodeA->box;
    const AABB boxB = transformAABB(nodeB->box, relMatBtoA, relPosBtoA);

    if (!boxA.intersects(boxB)) {
        return 0;
    }

    if (nodeA->isLeaf() && nodeB->isLeaf()) {
        long long hits = 0;
        for (int iA : nodeA->tri_indices) {
            const Vec3 p[3] = {
                meshA.vertices[meshA.indices[iA * 3]],
                meshA.vertices[meshA.indices[iA * 3 + 1]],
                meshA.vertices[meshA.indices[iA * 3 + 2]],
            };

            for (int iB : nodeB->tri_indices) {
                const Vec3 q[3] = {
                    relMatBtoA.transform(meshB.vertices[meshB.indices[iB * 3]]) + relPosBtoA,
                    relMatBtoA.transform(meshB.vertices[meshB.indices[iB * 3 + 1]]) + relPosBtoA,
                    relMatBtoA.transform(meshB.vertices[meshB.indices[iB * 3 + 2]]) + relPosBtoA,
                };

                if (trianglesIntersect(p, q)) {
                    ++hits;
                }
            }
        }
        return hits;
    }

    if (nodeA->isLeaf()) {
        return getMeshCollisions(nodeA, meshA, nodeB->left.get(), meshB, relMatBtoA, relPosBtoA) +
               getMeshCollisions(nodeA, meshA, nodeB->right.get(), meshB, relMatBtoA, relPosBtoA);
    }

    if (nodeB->isLeaf()) {
        return getMeshCollisions(nodeA->left.get(), meshA, nodeB, meshB, relMatBtoA, relPosBtoA) +
               getMeshCollisions(nodeA->right.get(), meshA, nodeB, meshB, relMatBtoA, relPosBtoA);
    }

    if (boxA.volume() > boxB.volume()) {
        return getMeshCollisions(nodeA->left.get(), meshA, nodeB, meshB, relMatBtoA, relPosBtoA) +
               getMeshCollisions(nodeA->right.get(), meshA, nodeB, meshB, relMatBtoA, relPosBtoA);
    }

    return getMeshCollisions(nodeA, meshA, nodeB->left.get(), meshB, relMatBtoA, relPosBtoA) +
           getMeshCollisions(nodeA, meshA, nodeB->right.get(), meshB, relMatBtoA, relPosBtoA);
}

long long computePairCollisions(const Mesh& meshA, const Mat3& matA, const Vec3& posA, const AABB& boxA,
                                const Mesh& meshB, const Mat3& matB, const Vec3& posB, const AABB& boxB) {
    if (!boxA.intersects(boxB)) {
        return 0;
    }

    const Mat3 relMatBtoA = matA.transposeMultiply(matB);
    const Vec3 relPosBtoA = matA.transposeTransform(posB - posA);

    long long collisions =
        getMeshCollisions(meshA.bvh_root.get(), meshA, meshB.bvh_root.get(), meshB, relMatBtoA, relPosBtoA);

    // Check full interior containment if one mesh's vertex lies inside the other's tight AABB
    if (!meshA.vertices.empty() && !meshB.vertices.empty()) {
        const Vec3 tpA = matA.transform(meshA.vertices[0]) + posA;
        if (boxB.contains(tpA) && isPointInsideMesh(meshB, matB, posB, tpA)) {
            collisions += 100000;
        }

        const Vec3 tpB = matB.transform(meshB.vertices[0]) + posB;
        if (boxA.contains(tpB) && isPointInsideMesh(meshA, matA, posA, tpB)) {
            collisions += 100000;
        }
    }

    return collisions;
}

void recomputeTotalEnergy(SolverState& state, float penaltyWeight, float progress) {
    AABB totalBox;
    Vec3 centroid = {0.0f, 0.0f, 0.0f};
    const size_t n = state.tightBoxes.size();

    for (size_t i = 0; i < n; ++i) {
        totalBox.expand(state.tightBoxes[i]);
        centroid = centroid + state.tightBoxes[i].center();
    }
    if (n > 0) {
        centroid = centroid * (1.0f / static_cast<float>(n));
    }

    // Small gravity tie-breaker pulls non-boundary interior objects tightly toward the center
    float compactness = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        compactness += (state.tightBoxes[i].center() - centroid).lengthSq();
    }

    state.volume = totalBox.volume();
    const float ramp = 0.15f + 0.85f * progress * progress;
    const float collisionCost = static_cast<float>(state.totalCollisions) * penaltyWeight * ramp;
    state.totalEnergy = state.volume + collisionCost + (1e-3f * compactness);
}

SolverState initializeSolverState(const std::vector<Mesh>& meshes, const std::vector<Transform>& initialTransforms,
                                  float penaltyWeight, float progress) {
    const size_t n = meshes.size();
    SolverState state;
    state.transforms = initialTransforms;
    state.mats.resize(n);
    state.tightBoxes.resize(n);
    state.pairCollisions.assign(n, std::vector<long long>(n, 0));

    for (size_t i = 0; i < n; ++i) {
        state.mats[i] = state.transforms[i].getMatrix();
        state.tightBoxes[i] = computeTightAABB(meshes[i], state.mats[i], state.transforms[i].pos);
    }

    state.totalCollisions = 0;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            const long long c = computePairCollisions(meshes[i], state.mats[i], state.transforms[i].pos,
                                                      state.tightBoxes[i], meshes[j], state.mats[j],
                                                      state.transforms[j].pos, state.tightBoxes[j]);
            state.pairCollisions[i][j] = c;
            state.pairCollisions[j][i] = c;
            state.totalCollisions += c;
        }
    }

    recomputeTotalEnergy(state, penaltyWeight, progress);
    return state;
}
