#include "IglMeshLoader.h"
//#include <per_vertex_normals.h>
#include <barycentric_coordinates.h>
#include <vector>
#include "readOBJ.h"
#include "ObjLoader.h"

#pragma warning(push, 3)
#pragma warning (disable: 4244) // disable annoying warnings from OBJ_Loader.h
// note: do not move this include file to ObjLoader.h (header-only library - will cause linker errors)
#pragma warning(pop)
#include "igl/read_triangle_mesh.h"
#include "igl/per_vertex_normals.h"
#include "Mesh.h"
#include "Model.h"
#include <memory>
#include <algorithm>


namespace cg3d
{

// MeshData IglLoader::MeshDataLoader(const objl::Mesh& loadedMesh)
// {
//     const auto& vertices = loadedMesh.Vertices;
//     const auto& indices = loadedMesh.Indices;

//     Eigen::MatrixXd V(vertices.size(), 3);
//     Eigen::MatrixXi F(indices.size() / 3, 3);
//     Eigen::MatrixXd V_normals(vertices.size(), 3);
//     Eigen::MatrixXd V_uv(vertices.size(), 2);

//     // import the vertices data
//     int j = 0;
//     for (const auto& vertex: vertices) {
//         V(j, 0) = vertex.Position.X;
//         V(j, 1) = vertex.Position.Y;
//         V(j, 2) = vertex.Position.Z;
//         V_normals(j, 0) = vertex.Normal.X;
//         V_normals(j, 1) = vertex.Normal.Y;
//         V_normals(j, 2) = vertex.Normal.Z;
//         V_uv(j, 0) = vertex.TextureCoordinate.X;
//         V_uv(j, 1) = vertex.TextureCoordinate.Y;
//         j++;
//     }

//     // import the faces data
//     F.resize(int(indices.size()) / 3, 3);
//     j = 0;
//     for (auto index: indices) {
//         F(j / 3, j % 3) = int(index);
//         j++;
//     }

//     // todo: add face normals?

//     return {V, F, V_normals, V_uv};
// }


std::shared_ptr<Mesh> IglLoader::MeshLoader(std::string name, const std::vector<std::string>& files)
{    
//    objl::Loader loader;
    std::vector<MeshData> dataList;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    Eigen::MatrixXd vertexNormals;
    Eigen::MatrixXd textureCoords;
    for (const auto& file: files) {
        igl::read_triangle_mesh(file,vertices,faces);
        igl::per_vertex_normals(vertices,faces,vertexNormals);
        textureCoords = Eigen::MatrixXd::Zero(vertices.rows(),2);
        // std::vector<MeshData> moreData;
        //MeshData moreData;
        //moreData.vertices = vertices;

        dataList.push_back({vertices,faces,vertexNormals,textureCoords});
        // std::move(moreData.begin(), moreData.end(), std::back_inserter(dataList));
    }

    return std::make_shared<Mesh>(std::move(name), dataList);
}

std::shared_ptr<Mesh> IglLoader::MeshLoader1(std::string name, const std::string file)
{    
    std::vector<MeshData> dataList;
    std::vector<std::vector<double>> V, TC, N;
    std::vector<std::vector<int>> F, FTC, FN;
    igl::readOBJ(file, V, TC, N, F, FTC, FN);
    Eigen::MatrixXd vertices(V.size(), 3);
    Eigen::MatrixXd textureCoords(TC.size(), 2);
    Eigen::MatrixXd vertexNormals(N.size(), 3);
    Eigen::MatrixXi faces(F.size(), 3);
    for (size_t i = 0; i < V.size(); i++) {
        vertices.row(i) = Eigen::Map<Eigen::VectorXd>(V[i].data(), V[i].size());
    }
    for (size_t i = 0; i < TC.size(); i++) {
        textureCoords.row(i) = Eigen::Map<Eigen::VectorXd>(TC[i].data(), TC[i].size());
    }
    for (size_t i = 0; i < N.size(); i++) {
        vertexNormals.row(i) = Eigen::Map<Eigen::VectorXd>(N[i].data(), N[i].size());
    }
    for (size_t i = 0; i < F.size(); i++) {
        faces.row(i) = Eigen::Map<Eigen::Vector3i>(F[i].data());
    }
    dataList.push_back({vertices,faces,vertexNormals,textureCoords});
    return std::make_shared<Mesh>(std::move(name), dataList);
}

std::shared_ptr<Mesh> IglLoader::MeshLoader2(std::string name, const std::string file)
{    
    std::vector<std::string> files;
    files.push_back(file);
    auto MESH = ObjLoader::MeshFromObj(name, files);
    return MESH;
}

} // namespace cg3d
