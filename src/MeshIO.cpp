#include "MeshIO.h"

#include <iostream>
#include <stdexcept>

#include <tiny_obj_loader.h>

void loadAndPrepareMesh(const std::string& filename, std::vector<Mesh>& meshes) {
    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(filename, tinyobj::ObjReaderConfig{})) {
        throw std::runtime_error("Failed to parse: " + filename);
    }

    Mesh mesh;
    const auto& attrib = reader.GetAttrib();

    for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
        mesh.vertices.push_back({
            attrib.vertices[i],
            attrib.vertices[i + 1],
            attrib.vertices[i + 2],
        });
    }

    std::vector<int> tris;
    for (const auto& shape : reader.GetShapes()) {
        for (size_t i = 0; i < shape.mesh.indices.size(); i += 3) {
            tris.push_back(static_cast<int>(mesh.indices.size() / 3));
            mesh.indices.push_back(shape.mesh.indices[i].vertex_index);
            mesh.indices.push_back(shape.mesh.indices[i + 1].vertex_index);
            mesh.indices.push_back(shape.mesh.indices[i + 2].vertex_index);
        }
    }

    Vec3 centroid = {0.0f, 0.0f, 0.0f};
    for (const Vec3& v : mesh.vertices) {
        centroid = centroid + v;
    }

    if (!mesh.vertices.empty()) {
        centroid = centroid * (1.0f / static_cast<float>(mesh.vertices.size()));
        for (Vec3& v : mesh.vertices) {
            v = v - centroid;
        }
    }

    std::cout << "  Building BVH for " << (mesh.indices.size() / 3) << " triangles..." << std::endl;
    mesh.bvh_root = buildBVH(mesh, tris);
    meshes.push_back(std::move(mesh));
}
