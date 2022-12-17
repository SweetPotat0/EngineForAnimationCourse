#include "./BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>

#include "ObjLoader.h"
// #include "AutoMorphingModel.h"
#include "SceneWithImGui.h"
#include "Visitor.h"

#include "IglMeshLoader.h"
#include "igl/min_heap.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/collapse_edge.h"
#include "igl/per_vertex_normals.h"



using namespace cg3d;

std::shared_ptr<cg3d::AutoMorphingModel> autoCyl1;
std::shared_ptr<cg3d::AutoMorphingModel> autoCyl2;
int meshDataIndex = 0;

void BasicScene::KeyCallback(cg3d::Viewport *viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_SPACE:
            pause = !pause;
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
    camera->Translate(10, Axis::Z);

    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{std::make_shared<Material>("material", program)}; // empty material
                                                                    //    SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());
    material->AddTexture(0, "textures/box0.bmp", 2);


    auto cyl1Mesh{IglLoader::MeshFromFiles("cube_igl", "data/bunny.off")};
    cyl1 = cg3d::Model::Create("cyl", cyl1Mesh, material);
    auto morphFunc = [](cg3d::Model *model, cg3d::Visitor *visitor)
    {
        return meshDataIndex;
    };
    autoCyl1 = cg3d::AutoMorphingModel::Create(*cyl1, morphFunc);

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

    autoCyl1->Translate({-3, -0.2, 0});
    autoCyl1->Scale(8.0f);
    root->AddChild(autoCyl1);

    auto v1 = GetLastMesh(autoCyl1).vertices;
    auto f1 = GetLastMesh(autoCyl1).faces;
    treeCyl1.init(v1,f1);


    auto cyl2Mesh{IglLoader::MeshFromFiles("cube_igl", "data/bunny.off")};

    cyl2 = cg3d::Model::Create("cyl", cyl2Mesh, material);

    autoCyl2 = cg3d::AutoMorphingModel::Create(*cyl2, morphFunc);

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

    collapseTenPerEdges(0.8f, autoCyl2);

    autoCyl2->Translate({3, -0.2, 0});
    autoCyl2->Scale(8.0f);
    root->AddChild(autoCyl2);

    meshDataIndex = autoCyl2->GetMeshList()[0]->data.size() - 1;

    auto v2 = GetLastMesh(autoCyl1).vertices;
    auto f2 = GetLastMesh(autoCyl1).faces;
    treeCyl2.init(v2,f2);

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
    if (!pause)
    {
        OBB* collidingOBB = getCollidingOBB(&treeCyl1, &treeCyl2, autoCyl1, autoCyl2);
        if (collidingOBB == NULL){
            autoCyl2->Translate({-0.05, 0, 0});
        }
        else{
            showOBB(collidingOBB);
            pause = true;
        }
    }
}
// check if there's a separating plane in between the selected axes
bool getSeparatingPlane(const Eigen::Vector3f& RPos, const Eigen::Vector3f& Plane, const OBB& box1, const OBB&box2)
{
    return (fabs(RPos.dot(Plane)) > 
        (fabs((box1.AxisX*box1.Half_size.x()).dot(Plane)) +
        fabs((box1.AxisY*box1.Half_size.y()).dot(Plane)) +
        fabs((box1.AxisZ*box1.Half_size.z()).dot(Plane)) +
        fabs((box2.AxisX*box2.Half_size.x()).dot(Plane)) + 
        fabs((box2.AxisY*box2.Half_size.y()).dot(Plane)) +
        fabs((box2.AxisZ*box2.Half_size.z()).dot(Plane))));
}

// test for separating planes in all 15 axes
bool isCollision(const OBB& box1, const OBB&box2)
{
    static Eigen::Vector3f RPos;
    RPos = box2.Pos - box1.Pos;

    return !(getSeparatingPlane(RPos, box1.AxisX, box1, box2) ||
        getSeparatingPlane(RPos, box1.AxisY, box1, box2) ||
        getSeparatingPlane(RPos, box1.AxisZ, box1, box2) ||
        getSeparatingPlane(RPos, box2.AxisX, box1, box2) ||
        getSeparatingPlane(RPos, box2.AxisY, box1, box2) ||
        getSeparatingPlane(RPos, box2.AxisZ, box1, box2) ||
        getSeparatingPlane(RPos, box1.AxisX.cross(box2.AxisX), box1, box2) ||
        getSeparatingPlane(RPos, box1.AxisX.cross(box2.AxisY), box1, box2) ||
        getSeparatingPlane(RPos, box1.AxisX.cross(box2.AxisZ), box1, box2) ||
        getSeparatingPlane(RPos, box1.AxisY.cross(box2.AxisX), box1, box2) ||
        getSeparatingPlane(RPos, box1.AxisY.cross(box2.AxisY), box1, box2) ||
        getSeparatingPlane(RPos, box1.AxisY.cross(box2.AxisZ), box1, box2) ||
        getSeparatingPlane(RPos, box1.AxisZ.cross(box2.AxisX), box1, box2) ||
        getSeparatingPlane(RPos, box1.AxisZ.cross(box2.AxisY), box1, box2) ||
        getSeparatingPlane(RPos, box1.AxisZ.cross(box2.AxisZ), box1, box2));
}

OBB getOBBfromAABB(Eigen::AlignedBox3d box, std::shared_ptr<cg3d::AutoMorphingModel> model){
    OBB obb;
    Eigen::Vector3f c =  box.center().cast <float> ();
    Eigen::Vector4f tc = model->GetTransform() * Eigen::Vector4f{c[0],c[1],c[2],1};
    obb.Pos = {tc[0], tc[1], tc[2]};
    obb.Half_size = box.sizes().cast<float>() / 2.0 ;
    auto b = model->GetScaling(model->GetTransform());
    obb.Half_size = b * obb.Half_size;
    auto a = model->GetRotation();
    obb.AxisX = a.col(0);
    obb.AxisY = a.col(1);
    obb.AxisZ = a.col(2);
    return obb;
}

void showOBB(OBB* box){
    std::cout<< "should show box with center" << box->Pos.transpose() << std::endl;
}

OBB* getCollidingOBB(igl::AABB<Eigen::MatrixXd,3> *tree1,igl::AABB<Eigen::MatrixXd,3> *tree2, std::shared_ptr<cg3d::AutoMorphingModel> model1 , std::shared_ptr<cg3d::AutoMorphingModel> model2){
    OBB obb1 = getOBBfromAABB(tree1->m_box, model1);
    OBB obb2 = getOBBfromAABB(tree2->m_box, model2);

    if (isCollision(obb1, obb2))
    {
        if (tree1->is_leaf() && tree2->is_leaf())
            return new OBB(obb1); 
        else if (tree1->is_leaf()){
            OBB* a1 = getCollidingOBB(tree1, tree2->m_left, model1, model2);
            OBB* a2 = getCollidingOBB(tree1, tree2->m_right, model1, model2);
            return (a1 != NULL) ? a1:a2;
        }
        else if (tree2->is_leaf()){
            OBB* a1 = getCollidingOBB(tree1->m_left, tree2, model1, model2);
            OBB* a2 = getCollidingOBB(tree1->m_right, tree2, model1, model2);
            return (a1 != NULL) ? a1:a2; 
        }
        else{
            OBB* a1 = getCollidingOBB(tree1->m_left, tree2->m_left, model1, model2);
            OBB* a2 = getCollidingOBB(tree1->m_left, tree2->m_right, model1, model2);
            OBB* a3 = getCollidingOBB(tree1->m_right, tree2->m_left, model1, model2);
            OBB* a4 = getCollidingOBB(tree1->m_right, tree2->m_right, model1, model2);
            return (a1 != NULL) ? a1:
                    (a2 != NULL)? a2:
                    (a3 != NULL)? a3:
                    a4; 
        }
    }
    else
        return NULL;
}
