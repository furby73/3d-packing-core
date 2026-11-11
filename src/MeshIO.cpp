#include "MeshIO.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <tiny_obj_loader.h>

void loadAndPrepareMesh(const std::string &filename, std::vector<Mesh> &meshes) {
    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(filename, tinyobj::ObjReaderConfig{}))
        throw std::runtime_error("Failed to parse " + filename + ": " + reader.Error());
    const auto &vertices = reader.GetAttrib().vertices;
    std::vector<int> remap(vertices.size() / 3, -1);
    Mesh mesh;
    for (const auto &shape : reader.GetShapes()) {
        for (auto count : shape.mesh.num_face_vertices)
            if (count != 3)
                throw std::runtime_error("Could not triangulate " + filename);
        for (const auto &index : shape.mesh.indices) {
            const int v = index.vertex_index;
            if (v < 0 || static_cast<size_t>(v) >= remap.size())
                throw std::runtime_error("Invalid vertex index in " + filename);
            if (remap[v] < 0) {
                Vec3 point = {vertices[3 * v], vertices[3 * v + 1], vertices[3 * v + 2]};
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                    throw std::runtime_error("Non-finite coordinates in " + filename);
                remap[v] = static_cast<int>(mesh.vertices.size());
                mesh.vertices.push_back(point);
            }
            mesh.indices.push_back(remap[v]);
        }
    }
    if (mesh.indices.empty())
        throw std::runtime_error("No triangle faces in " + filename);
    // Ignore unused OBJ vertices: they are not part of the printable geometry.
    Vec3 center = {0, 0, 0};
    for (const Vec3 &v : mesh.vertices)
        center = center + v;
    mesh.centroid = center * (1.0 / mesh.vertices.size());
    for (Vec3 &v : mesh.vertices)
        v = v - mesh.centroid;
    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        const Vec3 a = mesh.vertices[mesh.indices[i + 1]] - mesh.vertices[mesh.indices[i]];
        const Vec3 b = mesh.vertices[mesh.indices[i + 2]] - mesh.vertices[mesh.indices[i]];
        if (a.lengthSq() == 0 || b.lengthSq() == 0 || cross(a, b).lengthSq() <= 1e-28 * a.lengthSq() * b.lengthSq())
            throw std::runtime_error("Degenerate triangle in " + filename);
    }
    std::cout << "  Building BVH for " << mesh.indices.size() / 3 << " triangles...\n";
    buildMeshAcceleration(mesh);
    meshes.push_back(std::move(mesh));
}
