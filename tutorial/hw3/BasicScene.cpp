#include "./BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>

#include "ObjLoader.h"
// #include "AutoMorphingModel.h"
#include "SceneWithImGui.h"
#include "Visitor.h"
#include "PickVisitor.h"
#include "Renderer.h"

#include "IglMeshLoader.h"
#include "igl/min_heap.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/collapse_edge.h"
#include "igl/per_vertex_normals.h"

using namespace cg3d;

void BasicScene::KeyCallback(cg3d::Viewport *viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();
    Eigen::Vector3f pos;

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
            case GLFW_KEY_SPACE:
                pause = !pause;
                break;
            case GLFW_KEY_P:
                std::cout << "euler angles: " << links[picked_index]->GetTout().rotation().eulerAngles(2,0,2).transpose() << std::endl;
                break;
            case GLFW_KEY_T:
                pos = std::move(root->GetTranslation());
                for (size_t i = 0; i < links.size(); i++)
                {
                    pos += links[i]->GetRotation() * Eigen::Vector3f{0,0,1.6f};
                    std::cout << "tip of link " << i << " at: " << pos.transpose() << std::endl;
                }
                break;
            case GLFW_KEY_D:
                std::cout << "destination position: " << dest.transpose() << std::endl;
                break;
            case GLFW_KEY_N:
                picked_index++;
                if (links.size() == picked_index)
                {
                    picked_index = 0;
                }
                break;
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
            case GLFW_KEY_UP:
                links[picked_index]->Rotate(0.1,Axis::X);
                // links[picked_index]->RotateInSystem(axis[picked_index]->GetTout().rotation(),0.1,Axis::X);
                break;
            case GLFW_KEY_DOWN:
                links[picked_index]->Rotate(-0.1,Axis::X);
                // links[picked_index]->RotateInSystem(axis[picked_index]->GetTout().rotation(),-0.1,Axis::X);
                break;
            case GLFW_KEY_LEFT:
                // links[picked_index]->Rotate(-0.1,Axis::Z);
                links[picked_index]->RotateInSystem(axis[picked_index]->GetTout().rotation(),-0.1,Axis::Z);
                break;
            case GLFW_KEY_RIGHT:
                // links[picked_index]->Rotate(0.1,Axis::Z);
                links[picked_index]->RotateInSystem(axis[picked_index]->GetTout().rotation(),0.1,Axis::Z);
                break;
        }
    }
}

void BasicScene::MouseCallback(Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event

    if (action == GLFW_PRESS) { // default mouse button press behavior
        PickVisitor visitor;
        visitor.Init();
        renderer->RenderViewportAtPos(x, y, &visitor); // pick using fixed colors hack
        auto modelAndDepth = visitor.PickAtPos(x, renderer->GetWindowHeight() - y);
        renderer->RenderViewportAtPos(x, y); // draw again to avoid flickering
        pickedModel = modelAndDepth.first ? std::dynamic_pointer_cast<Model>(modelAndDepth.first->shared_from_this()) : nullptr;
        pickedModel = pickedModel->modelOnPick != nullptr ? pickedModel->modelOnPick : pickedModel;
        pickedModelDepth = modelAndDepth.second;
        camera->GetRotation().transpose();
        xAtPress = x;
        yAtPress = y;

        if (pickedModel && !pickedModel->isPickable)
            pickedModel = nullptr; // for non-pickable models we need only pickedModelDepth for mouse movement calculations later

        if (pickedModel)
            {
                if ((std::count(links.begin(), links.end(), pickedModel) || std::count(axis.begin(), axis.end(), pickedModel)))
                {
                    pickedModel = root;
                }
                pickedToutAtPress = pickedModel->GetTout();
            }
        else
            cameraToutAtPress = camera->GetTout();
    }
}

Eigen::Vector3f BasicScene::getTipOfLink(int ind){
    Eigen::Vector3f pos = std::move(root->GetTranslation());
    for (size_t i = 0; i <= ind; i++)
    {
        pos += links[i]->GetRotation() * Eigen::Vector3f{0,0,1.6f};
    }
    return pos;
}

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")}; 
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    camera = Camera::Create("camera", fov, float(width) / float(height), near, far);
    auto program = std::make_shared<Program>("shaders/basicShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");
    auto material = std::make_shared<Material>("material", program);
    auto material1{ std::make_shared<Material>("material", program1)}; 
    material->AddTexture(0, "textures/box0.bmp", 2);

    auto cylMesh{IglLoader::MeshFromFiles("cyl_igl", "data/zCylinder.obj")};
    auto sphereMesh{IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj")};
    Eigen::MatrixXd vertices(6,3);
    vertices << -1,0,0,1,0,0,0,-1,0,0,1,0,0,0,-1,0,0,1;
    Eigen::MatrixXi faces(3,2);
    faces << 0,1,2,3,4,5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6,3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6,2);
    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys",vertices,faces,vertexNormals,textureCoords);

    root = cg3d::Model::Create("root", sphereMesh, material);
    root->isHidden = true;
    
    AddChild(root);

    sphere = cg3d::Model::Create("sphere", sphereMesh, material);
    sphere->showWireframe = true;
    AddChild(sphere);
    sphere->Translate(dest);

    

    axis.push_back(Model::Create("axis",coordsys,material));
    axis[0]->mode = 1; 
    root->AddChild(axis[0]);

    links.push_back(cg3d::Model::Create("link", cylMesh, material));
    links[0]->showWireframe = true;
    links[0]->Translate(0.8f, Axis::Z);
    links[0]->SetCenter(Eigen::Vector3f(0,0,-0.8));
    root->AddChild(links[0]);

    for (size_t i = 1; i < 4; i++)
    {
        links.push_back(cg3d::Model::Create("link", cylMesh, material));
        links[i]->showWireframe = true;
        links[i]->Translate(1.6f, Axis::Z);
        links[i]->SetCenter(Eigen::Vector3f(0,0,-0.8));
        links[i-1]->AddChild(links[i]);

        axis.push_back(Model::Create("axis",coordsys,material));
        axis[i]->mode = 1; 
        axis[i]->Translate(0.8f, Axis::Z);
        links[i-1]->AddChild(axis[i]);
    }

    root->RotateByDegree(-90, Axis::X);
    

    camera->Translate(20, Axis::Z);
    // cube->Scale(3);
}
void BasicScene::CCD(int steps){
    if (pause)
        return;
    Eigen::Vector3f E, R, RD, RE;
    float angle;
    for (size_t j = 0; j < steps; j++)
    {
        for (size_t i = links.size()-1; i >= 0; i--)
        {
            E = getTipOfLink(links.size()-1);
            R = getTipOfLink(i-1);
            RD = dest - R;
            RE = E - R;
            angle = acos(RD.dot(RE));

            // links[i]->RotateByDegree(angle,)
            // if ((getTip() - dest).norm() > delta)
            // {
            //     pause = true;
            // }
        }
        
    }

    
    
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);

    // cube->Rotate(0.01f, Axis::XYZ);
}