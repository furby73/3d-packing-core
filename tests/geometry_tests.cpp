#include <iostream>
#include <stdexcept>
#include "Collisions.h"
#include "Packing.h"

void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
void addBox(Mesh &mesh, Vec3 lo, Vec3 hi) {
    const int base = static_cast<int>(mesh.vertices.size());
    mesh.vertices.insert(mesh.vertices.end(), {{lo.x, lo.y, lo.z},
                                               {hi.x, lo.y, lo.z},
                                               {lo.x, hi.y, lo.z},
                                               {hi.x, hi.y, lo.z},
                                               {lo.x, lo.y, hi.z},
                                               {hi.x, lo.y, hi.z},
                                               {lo.x, hi.y, hi.z},
                                               {hi.x, hi.y, hi.z}});
    const int indices[] = {0, 3, 1, 0, 2, 3, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                           2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5};
    for (int i : indices)
        mesh.indices.push_back(base + i);
}
Mesh box(Vec3 lo, Vec3 hi) {
    Mesh m;
    addBox(m, lo, hi);
    buildMeshAcceleration(m);
    return m;
}
bool collision(const Mesh &a, const Mesh &b, Vec3 pos, double gap, Vec3 rotation = {0, 0, 0}) {
    Transform t;
    t.rot = rotation;
    const Mat3 r = t.getMatrix(), identity = Transform{}.getMatrix();
    return meshesConflict(a, identity, {0, 0, 0}, computeTightAABB(a, identity, {0, 0, 0}), b, r, pos,
                          computeTightAABB(b, r, pos), gap);
}
int main() {
    try {
        auto outer = box({-1, -1, -1}, {1, 1, 1}), inner = box({-.1, -.1, -.1}, {.1, .1, .1});
        require(collision(outer, inner, {0, 0, 0}, 0), "Centered containment must not be missed at ray/edge hits");
        Mesh disconnected;
        addBox(disconnected, {10, 0, 0}, {11, 1, 1});
        addBox(disconnected, {.03, .02, .05}, {.1, .1, .1});
        buildMeshAcceleration(disconnected);
        require(collision(outer, disconnected, {0, 0, 0}, 0), "Check every connected component for containment");
        Mesh a, b;
        a.vertices = {{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
        a.indices = {0, 1, 2};
        buildMeshAcceleration(a);
        b.vertices = {{2, 2, 0}, {2, 1.1, 0}, {1.1, 2, 0}};
        b.indices = {0, 1, 2};
        buildMeshAcceleration(b);
        require(!collision(a, b, {0, 0, 0}, 0), "Disjoint coplanar triangles must not collide");
        require(collision(a, a, {.1, .1, 0}, 0), "Overlapping coplanar triangles must collide");
        require(collision(a, a, {0, 0, .09}, .1), "Face-interior distance must enforce clearance");
        require(!collision(a, a, {0, 0, .11}, .1), "Sufficient parallel-face clearance must pass");
        for (double scale : {1e-4, 1.0, 1e4}) {
            auto cube = box({0, 0, 0}, {scale, scale, scale});
            require(!collision(cube, cube, {scale, 0, 0}, 0), "Zero gap allows box face contact");
            require(collision(cube, cube, {.9 * scale, 0, 0}, 0), "Overlapping cubes must collide");
            require(collision(cube, cube, {1.099 * scale, 0, 0}, .1 * scale), "Sub-threshold distance must collide");
            require(!collision(cube, cube, {1.1 * scale, 0, 0}, .1 * scale), "Exact requested gap must pass");
            require(!collision(cube, cube, {1.08 * scale, 1.08 * scale, 0}, .1 * scale),
                    "Gap is Euclidean, not expanded AABB overlap");
            require(collision(cube, cube, {1.05 * scale, 1.05 * scale, 0}, .1 * scale),
                    "Edge-to-edge distance must enforce gap");
        }
        Mesh concave;
        addBox(concave, {-2, 0, 0}, {-1, 1, 1});
        addBox(concave, {1, 0, 0}, {2, 1, 1});
        addBox(concave, {-2, 2, 0}, {2, 3, 1});
        buildMeshAcceleration(concave);
        auto middle = box({-.2, .3, .3}, {.2, .7, .7});
        require(!collision(concave, middle, {0, 0, 0}, .2),
                "Interlocking bounding boxes with clear surfaces must pass");
        std::vector<Mesh> cubes;
        for (int i = 0; i < 8; ++i)
            cubes.push_back(box({0, 0, 0}, {1, 1, 1}));
        PackingOptions options{{2, 2, 2}, 0, .1};
        auto result = packMeshes(cubes, options);
        require(result.binCount == 1, "Eight unit cubes must fit exactly in one 2x2x2 bin");
        require(result.attemptedBinCounts == std::vector<size_t>{1}, "One-bin result must try one first");
        validatePacking(cubes, options, result);
        cubes.push_back(box({0, 0, 0}, {1, 1, 1}));
        result = packMeshes(cubes, options);
        require(result.binCount == 2, "Nine unit cubes need two 2x2x2 bins");
        require(result.attemptedBinCounts == std::vector<size_t>({1, 2}), "Try one before two containers");
        validatePacking(cubes, options, result);
        std::vector<Mesh> four;
        for (int i = 0; i < 4; ++i)
            four.push_back(box({0, 0, 0}, {1, 1, 1}));
        options = {{2.1, 2.1, 1}, .1, .1};
        result = packMeshes(four, options);
        require(result.binCount == 1, "Four cubes with .1 gap must fit a 2.1x2.1x1 bin");
        validatePacking(four, options, result);
        options.gap = .11;
        result = packMeshes(four, options);
        require(result.binCount > 1, "Larger clearance must increase bin count when needed");
        validatePacking(four, options, result);
        std::vector<Mesh> three;
        for (int i = 0; i < 3; ++i)
            three.push_back(box({0, 0, 0}, {1, 1, 1}));
        options = {{1, 1, 1}, 0, .15};
        std::vector<size_t> reported;
        result = packMeshes(three, options, [&](int, size_t, size_t target) {
            if (target && (reported.empty() || reported.back() != target))
                reported.push_back(target);
        });
        require(result.binCount == 3, "Three full-size parts require three containers");
        require(result.attemptedBinCounts == std::vector<size_t>({1, 2, 3}),
                "Container counts must increase sequentially");
        require(reported == result.attemptedBinCounts, "Progress must expose the actual ascending attempts");
        require(!result.usedFallback, "Known feasible upper-bound attempt is a successful result");
        validatePacking(three, options, result);
        std::vector<Mesh> rectangular;
        rectangular.push_back(box({0, 0, 0}, {2, 1, 1}));
        options = {{1, 2, 1}, 0, .05};
        result = packMeshes(rectangular, options);
        require(result.binCount == 1, "A rotated part must fit");
        bool rejected = false;
        options.size = {.5, .5, .5};
        try {
            packMeshes(rectangular, options);
        } catch (const std::runtime_error &) {
            rejected = true;
        }
        require(rejected, "Oversized parts must fail without an invalid fallback");
        std::cout << "Geometry and packing regression checks passed\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
