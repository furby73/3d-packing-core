#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "MeshIO.h"
#include "Packing.h"

namespace {
namespace fs = std::filesystem;
void usage() {
    std::cout << "Usage: packer --width W --height H --length L [--gap G] [options] input.obj ...\n"
              << "  --width,  -w W    Container width (X)\n"
              << "  --height, -H H    Container height (Z)\n"
              << "  --length, -l L    Container length (Y)\n"
              << "  --gap,    -g G    Minimum surface clearance between models (default 0)\n"
              << "  -t seconds        Improvement time budget, excluding preparation and I/O (default 60)\n"
              << "  -o output.obj     Preview OBJ; also writes one local OBJ per bin for multiple bins\n"
              << "  -m transforms.txt One display placement matrix per input (12 values per row)\n"
              << "  --manifest path   JSON with bin membership and local matrices (default <output>.json)\n"
              << "  --algorithm NAME  anneal (default) or constructive (randomized backtracking)\n"
              << "  --seed N          Random seed (default 42)\n"
              << "  -h, --help        Show help\n"
              << "  -v, --version     Show version\n"
              << "All dimensions and clearance use input coordinate units. Walls need no extra gap.\n"
              << "The heuristic minimizes bin count; it does not certify global optimality.\n";
}

double number(const std::string &text) {
    size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(value))
        throw std::invalid_argument("Invalid number: " + text);
    return value;
}
std::string jsonString(const std::string &value) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value) {
        switch (c) {
        case '"':
            out << "\\\"";
            break;
        case '\\':
            out << "\\\\";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            if (c < 32)
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
            else
                out << c;
        }
    }
    out << '"';
    return out.str();
}
std::array<double, 12> matrix(const Mesh &mesh, const Placement &placement, double displayOffset = 0) {
    const Mat3 r = placement.transform.getMatrix();
    Vec3 t = placement.transform.pos - r.transform(mesh.centroid);
    t.x += displayOffset;
    const double xyz[] = {t.x, t.y, t.z};
    std::array<double, 12> out{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col)
            out[row * 4 + col] = r.m[row][col];
        out[row * 4 + 3] = xyz[row];
    }
    return out;
}
std::ofstream output(const std::string &path) {
    std::ofstream out;
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.open(path);
    out << std::setprecision(17);
    return out;
}
void writeObj(const std::string &path, const std::vector<Mesh> &meshes, const PackingResult &result, double stride,
              size_t onlyBin = std::numeric_limits<size_t>::max()) {
    auto out = output(path);
    size_t offset = 1;
    for (size_t i = 0; i < meshes.size(); ++i) {
        const auto &p = result.placements[i];
        if (onlyBin != std::numeric_limits<size_t>::max() && p.bin != onlyBin)
            continue;
        out << "o bin_" << p.bin + 1 << "_mesh_" << i + 1 << '\n';
        const Mat3 r = p.transform.getMatrix();
        for (const Vec3 &v : meshes[i].vertices) {
            Vec3 point = r.transform(v) + p.transform.pos;
            if (onlyBin == std::numeric_limits<size_t>::max())
                point.x += p.bin * stride;
            out << "v " << point.x << ' ' << point.y << ' ' << point.z << '\n';
        }
        for (size_t j = 0; j < meshes[i].indices.size(); j += 3)
            out << "f " << meshes[i].indices[j] + offset << ' ' << meshes[i].indices[j + 1] + offset << ' '
                << meshes[i].indices[j + 2] + offset << '\n';
        offset += meshes[i].vertices.size();
    }
    out.close();
}
} // namespace

int main(int argc, char **argv) {
    try {
        PackingOptions options{{0, 0, 0}};
        std::string objFile, transformsFile, manifestFile;
        std::vector<std::string> inputs;
        bool positional = false;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto value = [&]() {
                if (++i >= argc)
                    throw std::invalid_argument(arg + " requires a value");
                return std::string(argv[i]);
            };
            if (positional) {
                inputs.push_back(arg);
                continue;
            }
            if (arg == "--") {
                positional = true;
                continue;
            }
            if (arg == "-h" || arg == "--help") {
                usage();
                return 0;
            }
            if (arg == "-v" || arg == "--version") {
                std::cout << "3D Packing Solver " << PACKER_VERSION << '\n';
                return 0;
            }
            if (arg == "--width" || arg == "-w")
                options.size.x = number(value());
            else if (arg == "--height" || arg == "-H")
                options.size.z = number(value());
            else if (arg == "--length" || arg == "-l")
                options.size.y = number(value());
            else if (arg == "--gap" || arg == "-g")
                options.gap = number(value());
            else if (arg == "-t")
                options.seconds = number(value());
            else if (arg == "-o")
                objFile = value();
            else if (arg == "-m")
                transformsFile = value();
            else if (arg == "--manifest")
                manifestFile = value();
            else if (arg == "--algorithm") {
                const auto name = value();
                if (name == "anneal")
                    options.algorithm = PackingAlgorithm::Anneal;
                else if (name == "constructive")
                    options.algorithm = PackingAlgorithm::Constructive;
                else
                    throw std::invalid_argument("Unknown algorithm: " + name);
            } else if (arg == "--seed") {
                const std::string text = value();
                size_t used = 0;
                if (text.empty() || text[0] == '-')
                    throw std::invalid_argument("Invalid seed");
                const auto seed = std::stoull(text, &used);
                if (used != text.size() || seed > std::numeric_limits<uint32_t>::max())
                    throw std::invalid_argument("Invalid seed");
                options.seed = static_cast<uint32_t>(seed);
            } else if (!arg.empty() && arg[0] == '-')
                throw std::invalid_argument("Unknown option: " + arg);
            else
                inputs.push_back(arg);
        }
        if (options.size.x <= 0 || options.size.y <= 0 || options.size.z <= 0)
            throw std::invalid_argument("Specify positive --width, --height and --length");
        if (options.gap < 0 || options.seconds <= 0)
            throw std::invalid_argument("Gap must be nonnegative and time positive");
        if (inputs.empty())
            throw std::invalid_argument("At least one input OBJ file is required");
        if (objFile.empty() && transformsFile.empty())
            objFile = "output.obj";
        if (manifestFile.empty())
            manifestFile = (transformsFile.empty() ? objFile : transformsFile) + ".json";
        std::vector<Mesh> meshes;
        for (const auto &input : inputs) {
            std::cout << "Loading " << input << "...\n";
            loadAndPrepareMesh(input, meshes);
        }
        const PackingResult result = packMeshes(meshes, options, [](int percent, size_t bins, size_t target) {
            std::cout << "Prog: " << percent << "% | Trying: " << target << " | BestBins: " << bins << std::endl;
        });
        const double stride = binDisplayStride(options);
        std::vector<std::string> binFiles(result.binCount);
        if (!objFile.empty()) {
            if (result.binCount == 1)
                binFiles[0] = objFile;
            else
                for (size_t b = 0; b < result.binCount; ++b) {
                    std::ostringstream suffix;
                    suffix << "-bin-" << std::setw(3) << std::setfill('0') << b + 1 << ".obj";
                    const fs::path p = objFile;
                    binFiles[b] = (p.parent_path() / (p.stem().string() + suffix.str())).string();
                }
        }
        std::vector<std::string> outputs = {manifestFile};
        if (!objFile.empty())
            outputs.push_back(objFile);
        if (!transformsFile.empty())
            outputs.push_back(transformsFile);
        if (result.binCount > 1)
            for (const auto &file : binFiles)
                if (!file.empty())
                    outputs.push_back(file);
        // Never overwrite an input mesh, or one output with another output.
        for (size_t i = 0; i < outputs.size(); ++i) {
            const auto path = fs::weakly_canonical(outputs[i]);
            for (const auto &input : inputs)
                if (path == fs::weakly_canonical(input))
                    throw std::invalid_argument("Output would overwrite input: " + outputs[i]);
            for (size_t j = 0; j < i; ++j)
                if (path == fs::weakly_canonical(outputs[j]))
                    throw std::invalid_argument("Output paths must be distinct");
        }
        if (!objFile.empty()) {
            writeObj(objFile, meshes, result, stride);
            if (result.binCount > 1)
                for (size_t b = 0; b < result.binCount; ++b)
                    writeObj(binFiles[b], meshes, result, 0, b);
        }
        if (!transformsFile.empty()) {
            auto out = output(transformsFile);
            for (size_t i = 0; i < meshes.size(); ++i) {
                const auto values = matrix(meshes[i], result.placements[i], result.placements[i].bin * stride);
                for (size_t j = 0; j < values.size(); ++j)
                    out << values[j] << (j + 1 == values.size() ? '\n' : ' ');
            }
            out.close();
        }
        auto out = output(manifestFile);
        out << "{\n  \"schema_version\": 1,\n  \"width\": " << options.size.x << ", \"height\": " << options.size.z
            << ", \"length\": " << options.size.y << ", \"gap\": " << options.gap
            << ",\n  \"bin_count\": " << result.binCount << ",\n  \"algorithm\": "
            << jsonString(options.algorithm == PackingAlgorithm::Anneal ? "anneal" : "constructive")
            << ",\n  \"baseline_bin_count\": " << result.baselineBinCount
            << ",\n  \"baseline_timed_out\": " << (result.baselineTimedOut ? "true" : "false")
            << ",\n  \"baseline_seconds\": " << result.baselineSeconds << ",\n  \"iterations\": " << result.iterations
            << ", \"restarts\": " << result.restarts << ",\n  \"search_seconds\": " << result.searchSeconds
            << ", \"preparation_seconds\": " << result.preparationSeconds
            << ",\n  \"used_fallback\": " << (result.usedFallback ? "true" : "false")
            << ",\n  \"attempted_bin_counts\": [";
        for (size_t i = 0; i < result.attemptedBinCounts.size(); ++i)
            out << result.attemptedBinCounts[i] << (i + 1 == result.attemptedBinCounts.size() ? "" : ", ");
        out << "],\n  \"bins\": [\n";
        for (size_t b = 0; b < result.binCount; ++b) {
            out << "    {\"id\": " << b << ", \"display_offset\": [" << b * stride << ", 0, 0], \"obj\": ";
            if (binFiles[b].empty())
                out << "null";
            else
                out << jsonString(fs::absolute(binFiles[b]).lexically_normal().string());
            out << '}' << (b + 1 == result.binCount ? "\n" : ",\n");
        }
        out << "  ],\n  \"placements\": [\n";
        for (size_t i = 0; i < meshes.size(); ++i) {
            out << "    {\"input_index\": " << i << ", \"input\": " << jsonString(inputs[i])
                << ", \"bin\": " << result.placements[i].bin << ", \"matrix\": [";
            const auto values = matrix(meshes[i], result.placements[i]);
            for (size_t j = 0; j < values.size(); ++j)
                out << values[j] << (j + 1 == values.size() ? "" : ", ");
            out << "]}" << (i + 1 == meshes.size() ? "\n" : ",\n");
        }
        out << "  ]\n}\n";
        out.close();
        if (result.usedFallback)
            std::cout << "Search budget exhausted; returning the validated greedy fallback.\n";
        std::cout << "Done! Packed " << meshes.size() << " meshes into " << result.binCount << " container(s), gap "
                  << options.gap
                  << ". Best feasible result found; global minimum is not guaranteed. Manifest: " << manifestFile
                  << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
