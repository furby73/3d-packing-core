#include "Collisions.h"

#include <algorithm>

namespace {

bool overlapOnAxis(const Vec3 &axis, const Vec3 p[3], const Vec3 q[3]) {
    if (axis.length() < 1e-6f) {
        return true;
    }

    float minA = dot(axis, p[0]);
    float maxA = minA;
    float minB = dot(axis, q[0]);
    float maxB = minB;

    for (int i = 1; i < 3; ++i) {
        float d = dot(axis, p[i]);
        minA = std::min(minA, d);
        maxA = std::max(maxA, d);

        d = dot(axis, q[i]);
        minB = std::min(minB, d);
        maxB = std::max(maxB, d);
    }

    return !(maxA < minB || maxB < minA);
}

bool trianglesIntersect(const Vec3 p[3], const Vec3 q[3]) {
    const Vec3 eA[3] = {p[1] - p[0], p[2] - p[1], p[0] - p[2]};
    const Vec3 eB[3] = {q[1] - q[0], q[2] - q[1], q[0] - q[2]};

    if (!overlapOnAxis(cross(eA[0], eA[1]), p, q)) {
        return false;
    }
    if (!overlapOnAxis(cross(eB[0], eB[1]), p, q)) {
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!overlapOnAxis(cross(eA[i], eB[j]), p, q)) {
                return false;
            }
        }
    }

    return true;
}

bool rayIntersectsTriangle(const Vec3 &ro, const Vec3 &rd, const Vec3 &v0, const Vec3 &v1, const Vec3 &v2) {
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

bool rayIntersectsAABB(const Vec3 &ro, const Vec3 &invRd, const AABB &box) {
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

int countRayHits(const BVHNode *node, const Mat3 &mat, const Transform &t, const Mesh &mesh, const Vec3 &ro,
                 const Vec3 &rd, const Vec3 &invRd) {
    const AABB globalBox = transformAABB(node->box, mat, t.pos);
    if (!rayIntersectsAABB(ro, invRd, globalBox)) {
        return 0;
    }

    if (node->isLeaf()) {
        int hits = 0;
        for (int i : node->tri_indices) {
            const Vec3 v0 = mat.transform(mesh.vertices[mesh.indices[i * 3]]) + t.pos;
            const Vec3 v1 = mat.transform(mesh.vertices[mesh.indices[i * 3 + 1]]) + t.pos;
            const Vec3 v2 = mat.transform(mesh.vertices[mesh.indices[i * 3 + 2]]) + t.pos;
            if (rayIntersectsTriangle(ro, rd, v0, v1, v2)) {
                ++hits;
            }
        }
        return hits;
    }

    int hits = 0;
    if (node->left) {
        hits += countRayHits(node->left.get(), mat, t, mesh, ro, rd, invRd);
    }
    if (node->right) {
        hits += countRayHits(node->right.get(), mat, t, mesh, ro, rd, invRd);
    }
    return hits;
}

} // namespace

std::unique_ptr<BVHNode> buildBVH(const Mesh &mesh, std::vector<int> &tris) {
    auto node = std::make_unique<BVHNode>();

    for (int idx : tris) {
        node->box.expand(mesh.vertices[mesh.indices[idx * 3]]);
        node->box.expand(mesh.vertices[mesh.indices[idx * 3 + 1]]);
        node->box.expand(mesh.vertices[mesh.indices[idx * 3 + 2]]);
    }

    if (tris.size() <= 4) {
        node->tri_indices = tris;
        return node;
    }

    const Vec3 extent = node->box.max - node->box.min;
    int axis = 0;
    if (extent.y >= extent.x && extent.y >= extent.z) {
        axis = 1;
    } else if (extent.z >= extent.x && extent.z >= extent.y) {
        axis = 2;
    }

    std::sort(tris.begin(), tris.end(), [&](int a, int b) {
        const Vec3 cA = (mesh.vertices[mesh.indices[a * 3]] + mesh.vertices[mesh.indices[a * 3 + 1]] +
                         mesh.vertices[mesh.indices[a * 3 + 2]]) *
                        0.3333f;

        const Vec3 cB = (mesh.vertices[mesh.indices[b * 3]] + mesh.vertices[mesh.indices[b * 3 + 1]] +
                         mesh.vertices[mesh.indices[b * 3 + 2]]) *
                        0.3333f;

        if (axis == 0) {
            return cA.x < cB.x;
        }
        if (axis == 1) {
            return cA.y < cB.y;
        }
        return cA.z < cB.z;
    });

    const size_t half = tris.size() / 2;
    std::vector<int> leftTris(tris.begin(), tris.begin() + static_cast<long long>(half));
    std::vector<int> rightTris(tris.begin() + static_cast<long long>(half), tris.end());

    node->left = buildBVH(mesh, leftTris);
    node->right = buildBVH(mesh, rightTris);
    return node;
}

AABB transformAABB(const AABB &local, const Mat3 &mat, const Vec3 &pos) {
    const Vec3 corners[8] = {
        {local.min.x, local.min.y, local.min.z}, {local.max.x, local.min.y, local.min.z},
        {local.min.x, local.max.y, local.min.z}, {local.max.x, local.max.y, local.min.z},
        {local.min.x, local.min.y, local.max.z}, {local.max.x, local.min.y, local.max.z},
        {local.min.x, local.max.y, local.max.z}, {local.max.x, local.max.y, local.max.z},
    };

    AABB global;
    for (const Vec3 &corner : corners) {
        global.expand(mat.transform(corner) + pos);
    }
    return global;
}

long long getMeshCollisions(const BVHNode *nodeA, const Mat3 &matA, const Transform &ta, const Mesh &meshA,
                            const BVHNode *nodeB, const Mat3 &matB, const Transform &tb, const Mesh &meshB) {
    const AABB boxA = transformAABB(nodeA->box, matA, ta.pos);
    const AABB boxB = transformAABB(nodeB->box, matB, tb.pos);

    if (!boxA.intersects(boxB)) {
        return 0;
    }

    if (nodeA->isLeaf() && nodeB->isLeaf()) {
        long long hits = 0;
        for (int iA : nodeA->tri_indices) {
            const Vec3 p[3] = {
                matA.transform(meshA.vertices[meshA.indices[iA * 3]]) + ta.pos,
                matA.transform(meshA.vertices[meshA.indices[iA * 3 + 1]]) + ta.pos,
                matA.transform(meshA.vertices[meshA.indices[iA * 3 + 2]]) + ta.pos,
            };

            for (int iB : nodeB->tri_indices) {
                const Vec3 q[3] = {
                    matB.transform(meshB.vertices[meshB.indices[iB * 3]]) + tb.pos,
                    matB.transform(meshB.vertices[meshB.indices[iB * 3 + 1]]) + tb.pos,
                    matB.transform(meshB.vertices[meshB.indices[iB * 3 + 2]]) + tb.pos,
                };

                if (trianglesIntersect(p, q)) {
                    ++hits;
                }
            }
        }
        return hits;
    }

    if (nodeA->isLeaf()) {
        return getMeshCollisions(nodeA, matA, ta, meshA, nodeB->left.get(), matB, tb, meshB) +
               getMeshCollisions(nodeA, matA, ta, meshA, nodeB->right.get(), matB, tb, meshB);
    }

    if (nodeB->isLeaf()) {
        return getMeshCollisions(nodeA->left.get(), matA, ta, meshA, nodeB, matB, tb, meshB) +
               getMeshCollisions(nodeA->right.get(), matA, ta, meshA, nodeB, matB, tb, meshB);
    }

    if (boxA.volume() > boxB.volume()) {
        return getMeshCollisions(nodeA->left.get(), matA, ta, meshA, nodeB, matB, tb, meshB) +
               getMeshCollisions(nodeA->right.get(), matA, ta, meshA, nodeB, matB, tb, meshB);
    }

    return getMeshCollisions(nodeA, matA, ta, meshA, nodeB->left.get(), matB, tb, meshB) +
           getMeshCollisions(nodeA, matA, ta, meshA, nodeB->right.get(), matB, tb, meshB);
}

EnergyResult calculateEnergy(const std::vector<Mesh> &meshes, const std::vector<Transform> &state, float penaltyWeight,
                             float progress) {
    std::vector<Mat3> mats(meshes.size());
    std::vector<AABB> globalBoxes(meshes.size());
    AABB totalBox;

    for (size_t i = 0; i < meshes.size(); ++i) {
        mats[i] = state[i].getMatrix();
        globalBoxes[i] = transformAABB(meshes[i].bvh_root->box, mats[i], state[i].pos);
        totalBox.expand(globalBoxes[i].min);
        totalBox.expand(globalBoxes[i].max);
    }

    long long totalCollisions = 0;
    const Vec3 rd = {0.7385f, 0.4431f, 0.5076f};
    const Vec3 invRd = {1.0f / rd.x, 1.0f / rd.y, 1.0f / rd.z};

    for (size_t i = 0; i < meshes.size(); ++i) {
        for (size_t j = i + 1; j < meshes.size(); ++j) {
            if (globalBoxes[i].intersects(globalBoxes[j])) {
                totalCollisions += getMeshCollisions(meshes[i].bvh_root.get(), mats[i], state[i], meshes[i],
                                                     meshes[j].bvh_root.get(), mats[j], state[j], meshes[j]);

                if (!meshes[i].vertices.empty() && !meshes[j].vertices.empty()) {
                    const Vec3 tpI = mats[i].transform(meshes[i].vertices[0]) + state[i].pos;
                    if ((countRayHits(meshes[j].bvh_root.get(), mats[j], state[j], meshes[j], tpI, rd, invRd) % 2) !=
                        0) {
                        totalCollisions += 100000;
                    }

                    const Vec3 tpJ = mats[j].transform(meshes[j].vertices[0]) + state[j].pos;
                    if ((countRayHits(meshes[i].bvh_root.get(), mats[i], state[i], meshes[i], tpJ, rd, invRd) % 2) !=
                        0) {
                        totalCollisions += 100000;
                    }
                }
            }
        }
    }

    const float vol = totalBox.volume();
    const float energy = vol + (static_cast<float>(totalCollisions) * penaltyWeight * progress * progress);
    return {vol, totalCollisions, energy};
}
