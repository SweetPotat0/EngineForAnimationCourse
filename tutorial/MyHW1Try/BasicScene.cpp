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
#include "igl/AABB.h"

using namespace cg3d;

std::shared_ptr<cg3d::AutoMorphingModel> autoCyl1;
std::shared_ptr<cg3d::AutoMorphingModel> autoCyl2;
int meshDataIndex1 = 0;
int meshDataIndex2 = 0;

void BasicScene::KeyCallback(cg3d::Viewport *viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_SPACE:
            // collapseTenPerEdges(0.03f);
            // meshDataIndex = cyl->GetMeshList()[0]->data.size() - 1;
            break;
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_UP:
            // if (meshDataIndex != cyl->GetMeshList()[0]->data.size())
            // {
            //     meshDataIndex++;
            // }
            // camera->RotateInSystem(system, 0.1f, Axis::X);
            break;
        case GLFW_KEY_DOWN:
            if (meshDataIndex1 != 0)
            {
                meshDataIndex1--;
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

cg3d::MeshData GetLastMesh(std::shared_ptr<cg3d::AutoMorphingModel> cyl) {
    return cyl->GetMeshList()[0]->data.back();
}

Eigen::VectorXi EMAP;
Eigen::MatrixXi OF, F, E, EF, EI;
Eigen::VectorXi EQ;
Eigen::MatrixXd OV;
igl::min_heap<std::tuple<double, int, int>> Q;
// If an edge were collapsed, we'd collapse it to these points:
Eigen::MatrixXd V, C;


cg3d::MeshData GetMeshDataFromBox(Eigen::AlignedBox3d box){   
    Eigen::MatrixXd retV(8, 3);
    Eigen::MatrixXi retF = std::move(Mesh::Cube()->data[0].faces);

    retV(0,0) = box.corner(Eigen::AlignedBox3d::CornerType::BottomLeftFloor)(0);
    retV(0,1) = box.corner(Eigen::AlignedBox3d::CornerType::BottomLeftFloor)(1);
    retV(0,2) = box.corner(Eigen::AlignedBox3d::CornerType::BottomLeftFloor)(2);

    retV(1,0) = box.corner(Eigen::AlignedBox3d::CornerType::BottomRightFloor)(0);
    retV(1,1) = box.corner(Eigen::AlignedBox3d::CornerType::BottomRightFloor)(1);
    retV(1,2) = box.corner(Eigen::AlignedBox3d::CornerType::BottomRightFloor)(2);

    retV(2,0) = box.corner(Eigen::AlignedBox3d::CornerType::TopLeftFloor)(0);
    retV(2,1) = box.corner(Eigen::AlignedBox3d::CornerType::TopLeftFloor)(1);
    retV(2,2) = box.corner(Eigen::AlignedBox3d::CornerType::TopLeftFloor)(2);

    retV(3,0) = box.corner(Eigen::AlignedBox3d::CornerType::TopRightFloor)(0);
    retV(3,1) = box.corner(Eigen::AlignedBox3d::CornerType::TopRightFloor)(1);
    retV(3,2) = box.corner(Eigen::AlignedBox3d::CornerType::TopRightFloor)(2);

    retV(4,0) = box.corner(Eigen::AlignedBox3d::CornerType::TopLeftCeil)(0);
    retV(4,1) = box.corner(Eigen::AlignedBox3d::CornerType::TopLeftCeil)(1);
    retV(4,2) = box.corner(Eigen::AlignedBox3d::CornerType::TopLeftCeil)(2);

    retV(5,0) = box.corner(Eigen::AlignedBox3d::CornerType::TopRightCeil)(0);
    retV(5,1) = box.corner(Eigen::AlignedBox3d::CornerType::TopRightCeil)(1);
    retV(5,2) = box.corner(Eigen::AlignedBox3d::CornerType::TopRightCeil)(2);

    retV(6,0) = box.corner(Eigen::AlignedBox3d::CornerType::BottomLeftCeil)(0);
    retV(6,1) = box.corner(Eigen::AlignedBox3d::CornerType::BottomLeftCeil)(1);
    retV(6,2) = box.corner(Eigen::AlignedBox3d::CornerType::BottomLeftCeil)(2);

    retV(7,0) = box.corner(Eigen::AlignedBox3d::CornerType::BottomRightCeil)(0);
    retV(7,1) = box.corner(Eigen::AlignedBox3d::CornerType::BottomRightCeil)(1);
    retV(7,2) = box.corner(Eigen::AlignedBox3d::CornerType::BottomRightCeil)(2);
    
    Eigen::MatrixXd normals = Eigen::MatrixXd();
    igl::per_vertex_normals(retV, retF, normals);

    return {retV, retF, normals, std::move(Mesh::Cube()->data[0].textureCoords)};
    //box.corner(Eigen::AlignedBox3d::CornerType::BottomLeftFloor),
    //box.corner(Eigen::AlignedBox3d::CornerType::BottomRightFloor),
    //box.corner(Eigen::AlignedBox3d::CornerType::TopLeftFloor),
    //box.corner(Eigen::AlignedBox3d::CornerType::TopRightFloor),
    //box.corner(Eigen::AlignedBox3d::CornerType::TopLeftCeil),
    //box.corner(Eigen::AlignedBox3d::CornerType::TopRightCeil),
    //box.corner(Eigen::AlignedBox3d::CornerType::BottomLeftCeil),
    //box.corner(Eigen::AlignedBox3d::CornerType::BottomRightCeil),
}

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
    auto cyl1Mesh{IglLoader::MeshFromFiles("cube_igl", "data/3holes.off")};

    cyl1 = cg3d::Model::Create("cyl", cyl1Mesh, material);
    auto morphFunc = [](cg3d::Model *model, cg3d::Visitor *visitor)
    {
        return meshDataIndex1;
    };

    autoCyl1 = cg3d::AutoMorphingModel::Create(*cyl1, morphFunc);

    autoCyl1->Translate({-3, -3, 0});
    autoCyl1->Scale(8.0f);
    autoCyl1->showWireframe = true;
    camera->Translate(20, Axis::Z);
    root->AddChild(autoCyl1);

    auto mesh = autoCyl1->GetMeshList();

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

    collapseTenPerEdges(0.8f, autoCyl1);

    auto cyl2Mesh{IglLoader::MeshFromFiles("cube_igl", "data/3holes.off")};

    cyl2 = cg3d::Model::Create("cyl", cyl2Mesh, material);

    auto morphFunc2 = [](cg3d::Model *model, cg3d::Visitor *visitor)
    {
        return meshDataIndex2;
    };

    autoCyl2 = cg3d::AutoMorphingModel::Create(*cyl2, morphFunc2);

    autoCyl2->Translate({-6, -6, 0});
    autoCyl2->Scale(8.0f);
    autoCyl2->showWireframe = true;
    root->AddChild(autoCyl2);

    // Function to reset original mesh and data structures
    OV = autoCyl2->GetMeshList()[0]->data[0].vertices;
    OF = autoCyl2->GetMeshList()[0]->data[0].faces;

    F = OF;
    V = OV;
    igl::edge_flaps(F, E, EMAP, EF, EI);
    C.resize(E.rows(), V.cols());
    Eigen::VectorXd costs2(E.rows());
    Q = {};
    EQ = Eigen::VectorXi::Zero(E.rows());
    {
        Eigen::VectorXd costs2(E.rows());
        igl::parallel_for(
            E.rows(), [&](const int e)
            {
                double cost = e;
                Eigen::RowVectorXd p(1,3);
                igl::shortest_edge_and_midpoint(e,V,F,E,EMAP,EF,EI,cost,p);
                C.row(e) = p;
                costs2(e) = cost; },
            10000);
        for (int e = 0; e < E.rows(); e++)
        {
            Q.emplace(costs2(e), e, 0);
        }
    }

    // collapseTenPerEdges(0.8f, autoCyl2);
    meshDataIndex1 = autoCyl1->GetMeshList()[0]->data.size() - 1;
    meshDataIndex2 = autoCyl2->GetMeshList()[0]->data.size() - 1;
    
    igl::AABB<Eigen::MatrixXd,3> treeCyl1;

    auto v1 = GetLastMesh(autoCyl1).vertices;
    auto f1 = GetLastMesh(autoCyl1).faces;
    treeCyl1.init(v1,f1);
    Eigen::Vector3d ay = treeCyl1.m_box.corner(Eigen::AlignedBox3d::CornerType::TopLeftFloor);
    Eigen::Vector3d az = treeCyl1.m_box.corner(Eigen::AlignedBox3d::CornerType::BottomLeftCeil);
    Eigen::Vector3d ax = treeCyl1.m_box.corner(Eigen::AlignedBox3d::CornerType::BottomRightFloor);
    Eigen::Vector3d center = treeCyl1.m_box.center();

    // Eigen::Vector3d moshe = ay * autoCyl1->GetTin().matrix() * autoCyl1->GetTout().matrix();
    
    std::cout<< "Ay: " << ay.transpose() << std::endl;
    std::cout<< "Ax: " << ax.transpose() << std::endl;
    std::cout<< "Az: " << az.transpose() << std::endl;
    std::cout<< "Center: " << center.transpose() << std::endl;

    igl::AABB<Eigen::MatrixXd,3> treeCyl2;
    auto v2 = GetLastMesh(autoCyl1).vertices;
    auto f2 = GetLastMesh(autoCyl1).faces;
    treeCyl2.init(v2,f2);

    auto customMesh = cg3d::Mesh("mycustomcube",std::vector<cg3d::MeshData>{GetMeshDataFromBox(treeCyl1.m_box)});
    auto cyl3 = cg3d::Model::Create("cyl3", std::make_shared<Mesh>(customMesh), material);
    

    cyl3->showWireframe = true;
    root->AddChild(cyl3);
    // treeCyl2.m_box;
    // treeCyl1.m_box;
}


/**
 * @param ratio float between 0 to 1. e.g 0.01 will remove 10 percent of the edges
 */
bool BasicScene::collapseTenPerEdges(float ratio, std::shared_ptr<cg3d::AutoMorphingModel> cyl)
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
            double cost = std::get<0>(Q.top());
            int edgeToCollpase = std::get<1>(Q.top());
            if (!igl::collapse_edge(igl::shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, EQ, C))
            {
                break;
            }
            // std::cout << "edge " << edgeToCollpase << ", cost = " << cost << ", new v position(" << C.row(edgeToCollpase) << ")" <<std::endl;
            something_collapsed = true;
        }

        if (something_collapsed)
        {
            Eigen::MatrixXd normals = Eigen::MatrixXd();
            igl::per_vertex_normals(V, F, normals);
            Eigen::MatrixXd texCoords = Eigen::MatrixXd::Zero(V.rows(), 2);
            auto newlist = cyl->GetMeshList();
            newlist[0]->data.push_back({V, F, normals, texCoords});
            cyl->SetMeshList(newlist);
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
    autoCyl1->Translate({0.03f, 0, 0});
    // cube->Rotate(0.01f, Axis::All);
}
