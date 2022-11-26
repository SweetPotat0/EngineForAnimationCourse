#include "./BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>

#include "ObjLoader.h"
// #include "AutoMorphingModel.h"
#include "SceneWithImGui.h"
#include "CamModel.h"
#include "Visitor.h"
#include "Utility.h"

#include "IglMeshLoader.h"
#include "igl/min_heap.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/collapse_edge.h"
#include "igl/per_vertex_normals.h"

using namespace cg3d;

std::shared_ptr<cg3d::AutoMorphingModel> autoCyl;
int meshDataIndex = 0;

void BasicScene::KeyCallback(cg3d::Viewport *viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_SPACE:
            collapseTenPerEdges(0.03f);
            meshDataIndex = cyl->GetMeshList()[0]->data.size() - 1;
            break;
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_UP:
            if (meshDataIndex != cyl->GetMeshList()[0]->data.size())
            {
                meshDataIndex++;
            }
            // camera->RotateInSystem(system, 0.1f, Axis::X);
            break;
        case GLFW_KEY_DOWN:
            if (meshDataIndex != 0)
            {
                meshDataIndex--;
            }
            // camera->RotateInSystem(system, -0.1f, Axis::X);
            break;
        case GLFW_KEY_LEFT:
            camera->RotateInSystem(system, 0.1f, Axis::Y);
            break;
        case GLFW_KEY_RIGHT:
            camera->RotateInSystem(system, -0.1f, Axis::Y);
            break;
        case GLFW_KEY_W:
            camera->TranslateInSystem(system, {0, 0.05f, 0});
            break;
        case GLFW_KEY_S:
            camera->TranslateInSystem(system, {0, -0.05f, 0});
            break;
        case GLFW_KEY_A:
            camera->TranslateInSystem(system, {-0.05f, 0, 0});
            break;
        case GLFW_KEY_D:
            camera->TranslateInSystem(system, {0.05f, 0, 0});
            break;
        case GLFW_KEY_B:
            camera->TranslateInSystem(system, {0, 0, 0.05f});
            break;
        case GLFW_KEY_F:
            camera->TranslateInSystem(system, {0, 0, -0.05f});
            break;
        }
    }
}

Eigen::VectorXi EMAP;
Eigen::MatrixXi OF, F, E, EF, EI;
Eigen::VectorXi EQ;
Eigen::MatrixXd OV;
igl::min_heap<std::tuple<double, int, int>> Q;
// If an edge were collapsed, we'd collapse it to these points:
Eigen::MatrixXd V, C;
int num_collapsed;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")};
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{std::make_shared<Material>("material", program)}; // empty material
                                                                    //    SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto cylMesh{IglLoader::MeshFromFiles("cube_igl", "data/3holes.off")};

    cyl = cg3d::Model::Create("cyl", cylMesh, material);
    auto morphFunc = [](cg3d::Model *model, cg3d::Visitor *visitor)
    {
        return meshDataIndex;
    };

    autoCyl = cg3d::AutoMorphingModel::Create(*cyl, morphFunc);

    autoCyl->Translate({-3, -3, 0});
    autoCyl->Scale(8.0f);
    autoCyl->showWireframe = true;
    camera->Translate(20, Axis::Z);
    root->AddChild(autoCyl);

    auto mesh = autoCyl->GetMeshList();

    // Function to reset original mesh and data structures
    OV = mesh[0]->data[0].vertices;
    OF = mesh[0]->data[0].faces;

    F = OF;
    V = OV;
    igl::edge_flaps(F, E, EMAP, EF, EI);
    C.resize(E.rows(), V.cols());
    Eigen::VectorXd costs(E.rows());
    Q = {};
    EQ = Eigen::VectorXi::Zero(E.rows());
    {
        Eigen::VectorXd costs(E.rows());
        igl::parallel_for(
            E.rows(), [&](const int e)
            {
                double cost = e;
                Eigen::RowVectorXd p(1,3);
                igl::shortest_edge_and_midpoint(e,V,F,E,EMAP,EF,EI,cost,p);
                C.row(e) = p;
                costs(e) = cost; },
            10000);
        for (int e = 0; e < E.rows(); e++)
        {
            Q.emplace(costs(e), e, 0);
        }
    }

    num_collapsed = 0;

    // std::cout<< "vertices: \n" << V <<std::endl;
    // std::cout<< "faces: \n" << F <<std::endl;

    // std::cout<< "edges: \n" << E.transpose() <<std::endl;
    // std::cout<< "edges to faces: \n" << EF.transpose() <<std::endl;
    // std::cout<< "faces to edges: \n "<< EMAP.transpose()<<std::endl;
    // std::cout<< "edges indices: \n" << EI.transpose() <<std::endl;
}

/**
 * @param ratio float between 0 to 1. e.g 0.01 will remove 10 percent of the edges
 */
bool BasicScene::collapseTenPerEdges(float ratio)
{
    std::cout << "Start collapsing:" << std::endl;
    static int numOfCollapses = 0;
    numOfCollapses++;
    // If animating then collapse 10% of edges
    if (!Q.empty())
    {
        bool something_collapsed = false;
        // collapse edge
        const int max_iter = std::ceil(ratio * Q.size());
        for (int j = 0; j < max_iter; j++)
        {
            if (!igl::collapse_edge(igl::shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, EQ, C))
            {
                break;
            }
            something_collapsed = true;
            num_collapsed++;
        }

        if (something_collapsed)
        {
            Eigen::MatrixXd normals = Eigen::MatrixXd();
            igl::per_vertex_normals(V, F, normals);
            Eigen::MatrixXd texCoords = Eigen::MatrixXd::Zero(V.rows(), 2);
            auto newlist = autoCyl->GetMeshList();
            newlist[0]->data.push_back({V, F, normals, texCoords});
            autoCyl->SetMeshList(newlist);
        }
    }
    std::cout << "End collapsing:" << std::endl;
    return false;
}

int frameCount = 0;

void BasicScene::Update(const Program &program, const Eigen::Matrix4f &proj, const Eigen::Matrix4f &view, const Eigen::Matrix4f &model)
{
    frameCount++;
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
    // cube->Rotate(0.01f, Axis::All);
}
