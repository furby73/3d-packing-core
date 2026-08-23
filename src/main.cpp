#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "Collisions.h"
#include "MeshIO.h"

int main(int argc, char **argv) {
    try {
        const auto printUsage = []() {
            std::cout << "Usage: packer [-t <time_seconds>] [-o <output.obj>] <input1.obj> [input2.obj ...]\n"
                      << "Options:\n"
                      << "  -t <seconds>  Time limit for packing (default: 60 seconds)\n"
                      << "  -o <file>     Output OBJ file (default: output.obj)\n"
                      << "  -v            Show the version\n"
                      << "  -h            Show this help message\n";
        };

        float maxTimeSeconds = 60.0f;
        std::string outputFile = "output.obj";
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

        float maxDiag = 0.0f;
        for (const Mesh &m : meshes) {
            const Vec3 ext = m.bvh_root->box.max - m.bvh_root->box.min;
            maxDiag = std::max(maxDiag, ext.length());
        }

        std::mt19937 rng(42);
        std::uniform_real_distribution<float> rDist(-1.0f, 1.0f);
        std::uniform_real_distribution<float> rotDist(-180.0f, 180.0f);

        std::vector<Transform> state(meshes.size());
        for (Transform &t : state) {
            t.pos = {
                rDist(rng) * (maxDiag * 0.1f),
                rDist(rng) * (maxDiag * 0.1f),
                rDist(rng) * (maxDiag * 0.1f),
            };
            t.rot = {rotDist(rng), rotDist(rng), rotDist(rng)};
        }

        constexpr float startTemp = 5.0f;
        constexpr float endTemp = 0.001f;
        constexpr float penaltyWeight = 0.5f;

        std::cout << "Validating initial state exact collisions..." << std::endl;
        EnergyResult currentEnergy = calculateEnergy(meshes, state, penaltyWeight, 0.0f);

        std::vector<Transform> bestState = state;
        float bestVolume = std::numeric_limits<float>::max();
        if (currentEnergy.collisions == 0) {
            bestVolume = currentEnergy.volume;
        }

        std::cout << "Starting Exact BVH Annealing for " << maxTimeSeconds << "s..." << std::endl;
        const auto startTime = std::chrono::steady_clock::now();
        int iteration = 0;
        float progress = 0.0f;
        float temperature = startTemp;

        while (progress < 1.0f) {
            std::vector<Transform> newState = state;
            const int idx = static_cast<int>(rng() % meshes.size());

            const Vec3 ext = meshes[idx].bvh_root->box.max - meshes[idx].bvh_root->box.min;
            const float meshDiag = ext.length();

            const float transStep = (meshDiag * 0.1f) * std::pow(1.0f - progress, 2.0f);
            const float rotStep = 45.0f * std::pow(1.0f - progress, 2.0f);

            std::normal_distribution<float> tStep(0.0f, std::max(0.0001f, transStep));
            std::normal_distribution<float> rStep(0.0f, std::max(0.0001f, rotStep));

            const float dx = tStep(rng);
            const float dy = tStep(rng);
            const float dz = tStep(rng);
            newState[idx].pos = newState[idx].pos + Vec3{dx, dy, dz};

            const float rx = rStep(rng);
            const float ry = rStep(rng);
            const float rz = rStep(rng);
            newState[idx].rot = newState[idx].rot + Vec3{rx, ry, rz};

            const EnergyResult newEnergy = calculateEnergy(meshes, newState, penaltyWeight, progress);
            const float diff = newEnergy.totalEnergy - currentEnergy.totalEnergy;

            if (diff < 0.0f || std::exp(-diff / temperature) > std::uniform_real_distribution<float>(0.0f, 1.0f)(rng)) {
                state = std::move(newState);
                currentEnergy = newEnergy;
                if (currentEnergy.collisions == 0 && currentEnergy.volume < bestVolume) {
                    bestVolume = currentEnergy.volume;
                    bestState = state;
                }
            }

            if (++iteration % 100 == 0) {
                const std::chrono::duration<float> elapsed = std::chrono::steady_clock::now() - startTime;
                progress = std::min(1.0f, elapsed.count() / maxTimeSeconds);
                temperature = std::pow(startTemp, 1.0f - progress) * std::pow(endTemp, progress);

                if (iteration % 2000 == 0) {
                    std::cout << "Prog: " << static_cast<int>(progress * 100) << "% | Temp: " << temperature
                              << " | Vol: " << currentEnergy.volume << " | Coll: " << currentEnergy.collisions << "\n";
                }
            }
        }

        std::ofstream out(outputFile);
        if (!out.is_open()) {
            std::cerr << "Failed to open output file: " << outputFile << "\n";
            return 1;
        }

        int vOffset = 1;
        for (size_t i = 0; i < meshes.size(); ++i) {
            out << "o Object_" << i << "\n";
            const Mat3 mat = bestState[i].getMatrix();

            for (const Vec3 &v : meshes[i].vertices) {
                const Vec3 tv = mat.transform(v) + bestState[i].pos;
                out << "v " << tv.x << " " << tv.y << " " << tv.z << "\n";
            }

            for (size_t j = 0; j < meshes[i].indices.size(); j += 3) {
                out << "f " << meshes[i].indices[j] + vOffset << " " << meshes[i].indices[j + 1] + vOffset << " "
                    << meshes[i].indices[j + 2] + vOffset << "\n";
            }

            vOffset += static_cast<int>(meshes[i].vertices.size());
        }

        if (bestVolume == std::numeric_limits<float>::max()) {
            std::cout << "Done! No 0-collision configuration found in time budget. Saved best visited state to "
                      << outputFile << std::endl;
        } else {
            std::cout << "Done! Final best 0-collision volume: " << bestVolume << ". Saved to " << outputFile
                      << std::endl;
        }
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
}
