#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "Collisions.h"
#include "MeshIO.h"

int main(int argc, char** argv) {
    try {
        const auto printUsage = []() {
            std::cout << "Usage: packer [-t <time_seconds>] [-o <output.obj>] [-m <transforms.txt>] <input1.obj> "
                         "[input2.obj ...]\n"
                      << "Options:\n"
                      << "  -t <seconds>  Time limit for packing (default: 60 seconds)\n"
                      << "  -o <file>     Output OBJ file (default: output.obj, or none when -m is given)\n"
                      << "  -m <file>     Output each input's placement as a row-major 3x4 matrix, one line per mesh\n"
                      << "  -v            Show the version\n"
                      << "  -h            Show this help message\n";
        };

        float maxTimeSeconds = 60.0f;
        std::string outputFile;
        std::string transformsFile;
        std::vector<Mesh> meshes;

        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "-h" || argument == "--help") {
                printUsage();
                return 0;
            }
            if (argument == "-v" || argument == "--version") {
                std::cout << "3D Packing Solver " << PACKER_VERSION << std::endl;
                return 0;
            }
            if (argument == "-t") {
                if (++i >= argc) {
                    throw std::invalid_argument("-t requires a value");
                }
                maxTimeSeconds = std::stof(argv[i]);
            } else if (argument == "-o") {
                if (++i >= argc) {
                    throw std::invalid_argument("-o requires a filename");
                }
                outputFile = argv[i];
            } else if (argument == "-m") {
                if (++i >= argc) {
                    throw std::invalid_argument("-m requires a filename");
                }
                transformsFile = argv[i];
            } else if (!argument.empty() && argument[0] == '-') {
                throw std::invalid_argument("unknown option: " + argument);
            } else {
                std::cout << "Loading " << argument << "..." << std::endl;
                loadAndPrepareMesh(argument.c_str(), meshes);
            }
        }

        if (!std::isfinite(maxTimeSeconds) || maxTimeSeconds <= 0.0f) {
            printUsage();
            throw std::invalid_argument("-t must specify a positive number of seconds");
        }
        if (meshes.empty()) {
            printUsage();
            throw std::invalid_argument("at least one input OBJ file is required");
        }
        if (outputFile.empty() && transformsFile.empty()) {
            outputFile = "output.obj";
        }

        const size_t numMeshes = meshes.size();
        float maxDiag = 0.0f;
        float sumVolume = 0.0f;
        for (const Mesh& m : meshes) {
            const Vec3 ext = m.bvh_root->box.max - m.bvh_root->box.min;
            maxDiag = std::max(maxDiag, ext.length());
            sumVolume += std::max(1e-6f, m.bvh_root->box.volume());
        }

        std::mt19937 rng(42);
        std::uniform_real_distribution<float> unitDist(0.0f, 1.0f);

        // Initialize meshes in a loose 3D grid so they start mostly collision-free
        const int gridSide = static_cast<int>(std::ceil(std::cbrt(static_cast<float>(numMeshes))));
        const float spacing = maxDiag * 0.75f;
        std::vector<Transform> initTransforms(numMeshes);
        for (size_t i = 0; i < numMeshes; ++i) {
            const int gx = static_cast<int>(i) % gridSide;
            const int gy = (static_cast<int>(i) / gridSide) % gridSide;
            const int gz = static_cast<int>(i) / (gridSide * gridSide);
            initTransforms[i].pos = {
                (gx - 0.5f * (gridSide - 1)) * spacing,
                (gy - 0.5f * (gridSide - 1)) * spacing,
                (gz - 0.5f * (gridSide - 1)) * spacing,
            };
            initTransforms[i].rot = {0.0f, 0.0f, 0.0f};
        }

        // Scale temperature and penalty weight to the actual volume of the input meshes
        const float startTemp = sumVolume * 0.5f;
        const float endTemp = sumVolume * 1e-5f;
        const float penaltyWeight = sumVolume * 0.05f;

        std::cout << "Validating initial state exact collisions..." << std::endl;
        SolverState state = initializeSolverState(meshes, initTransforms, penaltyWeight, 0.0f);

        std::vector<Transform> bestZeroCollState = state.transforms;
        float bestVolume = std::numeric_limits<float>::max();
        if (state.totalCollisions == 0) {
            bestVolume = state.volume;
        }

        std::vector<Transform> fallbackState = state.transforms;
        long long fallbackCollisions = state.totalCollisions;
        float fallbackVolume = state.volume;

        std::cout << "Starting Incremental BVH Annealing for " << maxTimeSeconds << "s..." << std::endl;
        const auto startTime = std::chrono::steady_clock::now();
        long long iteration = 0;
        float progress = 0.0f;
        int reportedPercent = -1;
        float temperature = startTemp;

        std::vector<long long> oldRowA(numMeshes);
        std::vector<long long> oldRowB(numMeshes);

        while (progress < 1.0f) {
            const float moveRoll = unitDist(rng);
            const size_t idxA = rng() % numMeshes;

            // 10% chance to swap positions of two distinct meshes
            if (numMeshes > 1 && moveRoll >= 0.90f) {
                size_t idxB = rng() % (numMeshes - 1);
                if (idxB >= idxA) {
                    ++idxB;
                }

                const Transform oldTransA = state.transforms[idxA];
                const Transform oldTransB = state.transforms[idxB];
                const AABB oldBoxA = state.tightBoxes[idxA];
                const AABB oldBoxB = state.tightBoxes[idxB];
                const long long oldTotalCollisions = state.totalCollisions;
                const float oldVolume = state.volume;
                const float oldEnergy = state.totalEnergy;

                for (size_t j = 0; j < numMeshes; ++j) {
                    oldRowA[j] = state.pairCollisions[idxA][j];
                    oldRowB[j] = state.pairCollisions[idxB][j];
                }

                std::swap(state.transforms[idxA].pos, state.transforms[idxB].pos);
                state.tightBoxes[idxA] = computeTightAABB(meshes[idxA], state.mats[idxA], state.transforms[idxA].pos);
                state.tightBoxes[idxB] = computeTightAABB(meshes[idxB], state.mats[idxB], state.transforms[idxB].pos);

                for (size_t j = 0; j < numMeshes; ++j) {
                    if (j == idxA || j == idxB) continue;
                    state.totalCollisions -= state.pairCollisions[idxA][j];
                    state.totalCollisions -= state.pairCollisions[idxB][j];

                    const long long cA = computePairCollisions(
                        meshes[idxA], state.mats[idxA], state.transforms[idxA].pos, state.tightBoxes[idxA],
                        meshes[j], state.mats[j], state.transforms[j].pos, state.tightBoxes[j]);
                    const long long cB = computePairCollisions(
                        meshes[idxB], state.mats[idxB], state.transforms[idxB].pos, state.tightBoxes[idxB],
                        meshes[j], state.mats[j], state.transforms[j].pos, state.tightBoxes[j]);

                    state.pairCollisions[idxA][j] = state.pairCollisions[j][idxA] = cA;
                    state.pairCollisions[idxB][j] = state.pairCollisions[j][idxB] = cB;
                    state.totalCollisions += cA + cB;
                }

                state.totalCollisions -= state.pairCollisions[idxA][idxB];
                const long long cAB = computePairCollisions(
                    meshes[idxA], state.mats[idxA], state.transforms[idxA].pos, state.tightBoxes[idxA],
                    meshes[idxB], state.mats[idxB], state.transforms[idxB].pos, state.tightBoxes[idxB]);
                state.pairCollisions[idxA][idxB] = state.pairCollisions[idxB][idxA] = cAB;
                state.totalCollisions += cAB;

                recomputeTotalEnergy(state, penaltyWeight, progress);
                const float diff = state.totalEnergy - oldEnergy;

                if (!(diff < 0.0f || std::exp(-diff / temperature) > unitDist(rng))) {
                    state.transforms[idxA] = oldTransA;
                    state.transforms[idxB] = oldTransB;
                    state.tightBoxes[idxA] = oldBoxA;
                    state.tightBoxes[idxB] = oldBoxB;
                    for (size_t j = 0; j < numMeshes; ++j) {
                        state.pairCollisions[idxA][j] = state.pairCollisions[j][idxA] = oldRowA[j];
                        state.pairCollisions[idxB][j] = state.pairCollisions[j][idxB] = oldRowB[j];
                    }
                    state.totalCollisions = oldTotalCollisions;
                    state.volume = oldVolume;
                    state.totalEnergy = oldEnergy;
                }
            } else {
                // Single-mesh perturbation (45% translation, 30% continuous rotation, 15% 90-deg snap)
                const Transform oldTrans = state.transforms[idxA];
                const Mat3 oldMat = state.mats[idxA];
                const AABB oldBox = state.tightBoxes[idxA];
                const long long oldTotalCollisions = state.totalCollisions;
                const float oldVolume = state.volume;
                const float oldEnergy = state.totalEnergy;

                for (size_t j = 0; j < numMeshes; ++j) {
                    oldRowA[j] = state.pairCollisions[idxA][j];
                }

                const Vec3 ext = meshes[idxA].bvh_root->box.max - meshes[idxA].bvh_root->box.min;
                const float meshDiag = ext.length();
                const float decay = std::max(0.005f, std::pow(1.0f - progress, 2.0f));

                if (moveRoll < 0.45f) {
                    // Pure translation
                    std::normal_distribution<float> tStep(0.0f, meshDiag * 0.2f * decay);
                    state.transforms[idxA].pos = state.transforms[idxA].pos + Vec3{tStep(rng), tStep(rng), tStep(rng)};
                    const Vec3 delta = state.transforms[idxA].pos - oldTrans.pos;
                    state.tightBoxes[idxA].min = oldBox.min + delta;
                    state.tightBoxes[idxA].max = oldBox.max + delta;
                } else if (moveRoll < 0.75f) {
                    // Pure continuous rotation
                    std::normal_distribution<float> rStep(0.0f, 45.0f * decay);
                    state.transforms[idxA].rot = state.transforms[idxA].rot + Vec3{rStep(rng), rStep(rng), rStep(rng)};
                    state.mats[idxA] = state.transforms[idxA].getMatrix();
                    state.tightBoxes[idxA] = computeTightAABB(meshes[idxA], state.mats[idxA], state.transforms[idxA].pos);
                } else {
                    // Discrete 90-degree rotation around X, Y, or Z
                    const int axis = static_cast<int>(rng() % 3);
                    const float deltaAngle = (rng() % 2 == 0) ? 90.0f : -90.0f;
                    if (axis == 0) state.transforms[idxA].rot.x += deltaAngle;
                    else if (axis == 1) state.transforms[idxA].rot.y += deltaAngle;
                    else state.transforms[idxA].rot.z += deltaAngle;

                    state.mats[idxA] = state.transforms[idxA].getMatrix();
                    state.tightBoxes[idxA] = computeTightAABB(meshes[idxA], state.mats[idxA], state.transforms[idxA].pos);
                }

                for (size_t j = 0; j < numMeshes; ++j) {
                    if (j == idxA) continue;
                    state.totalCollisions -= state.pairCollisions[idxA][j];
                    const long long c = computePairCollisions(
                        meshes[idxA], state.mats[idxA], state.transforms[idxA].pos, state.tightBoxes[idxA],
                        meshes[j], state.mats[j], state.transforms[j].pos, state.tightBoxes[j]);
                    state.pairCollisions[idxA][j] = state.pairCollisions[j][idxA] = c;
                    state.totalCollisions += c;
                }

                recomputeTotalEnergy(state, penaltyWeight, progress);
                const float diff = state.totalEnergy - oldEnergy;

                if (!(diff < 0.0f || std::exp(-diff / temperature) > unitDist(rng))) {
                    state.transforms[idxA] = oldTrans;
                    state.mats[idxA] = oldMat;
                    state.tightBoxes[idxA] = oldBox;
                    for (size_t j = 0; j < numMeshes; ++j) {
                        state.pairCollisions[idxA][j] = state.pairCollisions[j][idxA] = oldRowA[j];
                    }
                    state.totalCollisions = oldTotalCollisions;
                    state.volume = oldVolume;
                    state.totalEnergy = oldEnergy;
                }
            }

            if (state.totalCollisions == 0 && state.volume < bestVolume) {
                bestVolume = state.volume;
                bestZeroCollState = state.transforms;
            }
            if (state.totalCollisions < fallbackCollisions ||
                (state.totalCollisions == fallbackCollisions && state.volume < fallbackVolume)) {
                fallbackCollisions = state.totalCollisions;
                fallbackVolume = state.volume;
                fallbackState = state.transforms;
            }

            if (++iteration % 100 == 0) {
                const std::chrono::duration<float> elapsed = std::chrono::steady_clock::now() - startTime;
                progress = std::min(1.0f, elapsed.count() / maxTimeSeconds);
                temperature = std::pow(startTemp, 1.0f - progress) * std::pow(endTemp, progress);

                // Refresh current totalEnergy so it stays synchronized with the updated progress weight
                recomputeTotalEnergy(state, penaltyWeight, progress);

                // One flushed line per percent, so programs reading stdout get live progress
                const int percent = static_cast<int>(progress * 100);
                if (percent != reportedPercent) {
                    reportedPercent = percent;
                    std::cout << "Prog: " << percent << "% | Iter: " << iteration << " | Temp: " << temperature
                              << " | Vol: " << state.volume
                              << " | BestVol: " << (bestVolume < std::numeric_limits<float>::max() ? bestVolume : -1.0f)
                              << " | Coll: " << state.totalCollisions << std::endl;
                }
            }
        }

        const bool foundZeroCollision = (bestVolume < std::numeric_limits<float>::max());
        std::vector<Transform> finalTransforms = foundZeroCollision ? bestZeroCollState : fallbackState;

        // Moving every mesh together doesn't change the energy, so the arrangement drifts during annealing.
        // Center its bounding box on the origin to give a stable result.
        AABB packedBox;
        for (size_t i = 0; i < numMeshes; ++i) {
            packedBox.expand(computeTightAABB(meshes[i], finalTransforms[i].getMatrix(), finalTransforms[i].pos));
        }
        const Vec3 packedCenter = packedBox.center();
        for (Transform& t : finalTransforms) {
            t.pos = t.pos - packedCenter;
        }

        if (!outputFile.empty()) {
            std::ofstream out(outputFile);
            if (!out.is_open()) {
                std::cerr << "Failed to open output file: " << outputFile << "\n";
                return 1;
            }

            int vOffset = 1;
            for (size_t i = 0; i < meshes.size(); ++i) {
                out << "o Object_" << i << "\n";
                const Mat3 mat = finalTransforms[i].getMatrix();

                for (const Vec3& v : meshes[i].vertices) {
                    const Vec3 tv = mat.transform(v) + finalTransforms[i].pos;
                    out << "v " << tv.x << " " << tv.y << " " << tv.z << "\n";
                }

                for (size_t j = 0; j < meshes[i].indices.size(); j += 3) {
                    out << "f " << meshes[i].indices[j] + vOffset << " " << meshes[i].indices[j + 1] + vOffset << " "
                        << meshes[i].indices[j + 2] + vOffset << "\n";
                }

                vOffset += static_cast<int>(meshes[i].vertices.size());
            }
        }

        if (!transformsFile.empty()) {
            std::ofstream out(transformsFile);
            if (!out.is_open()) {
                std::cerr << "Failed to open transforms file: " << transformsFile << "\n";
                return 1;
            }

            // Maps an input vertex v to R * (v - centroid) + pos, written as [R | pos - R * centroid]
            out << std::setprecision(9);
            for (size_t i = 0; i < meshes.size(); ++i) {
                const Mat3 mat = finalTransforms[i].getMatrix();
                const Vec3 t = finalTransforms[i].pos - mat.transform(meshes[i].centroid);
                const float tRow[3] = {t.x, t.y, t.z};
                for (int r = 0; r < 3; ++r) {
                    out << mat.m[r][0] << " " << mat.m[r][1] << " " << mat.m[r][2] << " " << tRow[r]
                        << (r < 2 ? " " : "\n");
                }
            }
        }

        std::string savedTo = outputFile;
        if (!transformsFile.empty()) {
            savedTo += (savedTo.empty() ? "" : " and ") + transformsFile;
        }

        if (!foundZeroCollision) {
            std::cout << "Done! (" << iteration << " iterations). No 0-collision configuration found in time budget. "
                      << "Saved lowest-collision state (Collisions: " << fallbackCollisions
                      << ", Vol: " << fallbackVolume << ") to " << savedTo << std::endl;
        } else {
            std::cout << "Done! (" << iteration << " iterations). Final best 0-collision volume: " << bestVolume
                      << ". Saved to " << savedTo << std::endl;
        }
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
