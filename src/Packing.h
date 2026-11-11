#pragma once

#include <cstdint>
#include <functional>
#include <vector>
#include "Collisions.h"

enum class PackingAlgorithm { Anneal, Constructive };

struct PackingOptions {
    Vec3 size; // X = width, Y = length, Z = height, in input coordinate units.
    double gap = 0;
    double seconds = 60;
    uint32_t seed = 42;
    PackingAlgorithm algorithm = PackingAlgorithm::Anneal;
};
struct Placement {
    Transform transform; // Centered mesh -> bin-local coordinates.
    size_t bin = 0;
};
struct PackingResult {
    std::vector<Placement> placements; // Original input order.
    size_t binCount = 0;
    size_t iterations = 0;
    std::vector<size_t> attemptedBinCounts;
    bool usedFallback = false;
    size_t baselineBinCount = 0;
    bool baselineTimedOut = false;
    double baselineSeconds = 0;
    size_t restarts = 0;
    double searchSeconds = 0;
    double preparationSeconds = 0;
};
// Progress percent, best known feasible count, count currently being attempted.
using ProgressCallback = std::function<void(int, size_t, size_t)>;

PackingResult packMeshes(const std::vector<Mesh> &meshes, const PackingOptions &options,
                         const ProgressCallback &progress = {});
void validatePacking(const std::vector<Mesh> &meshes, const PackingOptions &options, const PackingResult &result);
double binDisplayStride(const PackingOptions &options);
