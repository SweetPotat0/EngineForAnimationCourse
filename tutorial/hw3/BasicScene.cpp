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
    Eigen::Vector3f pos, angles;
    float psi, phi, tetha, a;
    Eigen::Matrix3f A1, A2, A3;

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
            case GLFW_KEY_SPACE:
                pause = !pause;
                break;
            case GLFW_KEY_P:
                angles = links[picked_index]->GetTout().rotation().eulerAngles(2,0,2);
                phi = angles(0);
                tetha = angles(1);
                psi = angles(2);
                A1 <<  cos(phi), -sin(phi), 0,
                        sin(phi), cos(phi), 0,
                        0,0,1;
                A2 <<  1,0,0,
                        0, cos(tetha), -sin(tetha),
                        0, sin(tetha), cos(tetha);
                A3 <<  cos(psi), -sin(psi), 0,
                        sin(psi), cos(psi), 0,
                        0,0,1;
                std::cout << "rotation matrices: " << std::endl;
                std::cout << "Z: "<< A1<< std::endl;
                std::cout << "X: "<< A2<< std::endl;
                std::cout << "Z: "<< A3<< std::endl;
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
                std::cout << "destination position: " << sphere->GetTranslation().transpose() << std::endl;
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

void BasicScene::ScrollCallback(Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    auto system = camera->GetRotation().transpose();
    if (pickedModel) {
        if (pickedModel->modelOnPick != nullptr){
            pickedModel->modelOnPick->TranslateInSystem(system, {0, 0, -float(yoffset)});
            otherPickedToutAtPress = pickedModel->modelOnPick->GetTout();
        }
        else{
            pickedModel->TranslateInSystem(system, {0, 0, -float(yoffset)});
        pickedToutAtPress = pickedModel->GetTout();
        }
        
    } else {
        camera->TranslateInSystem(system, {0, 0, -float(yoffset)});
        cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::CursorPosCallback(Viewport* viewport, int x, int y, bool dragging, int* buttonState)
{
    std::shared_ptr<cg3d::Model> actuallyPicked = pickedModel;
    if (dragging) {
        auto system = camera->GetRotation().transpose();
        auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
        auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
        if (pickedModel) {
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE){
                pickedModel->SetTout(pickedToutAtPress);
                pickedModel->RotateInSystem(system, float(x - xAtPress) / moveCoeff, Axis::Y);
                pickedModel->RotateInSystem(system, float(y - yAtPress) / moveCoeff, Axis::X);
            }
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE){
                pickedModel->SetTout(pickedToutAtPress);
                pickedModel->RotateInSystem(system, float(x - xAtPress) / moveCoeff, Axis::Z);
            }
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE) {
                if (pickedModel->modelOnPick != nullptr)
                {   
                    actuallyPicked = pickedModel->modelOnPick;
                    pickedModel->modelOnPick->SetTout(otherPickedToutAtPress);
                }
                pickedModel->SetTout(pickedToutAtPress);
                actuallyPicked->TranslateInSystem(system * sceneRoot->GetRotation(), {float(x - xAtPress) / moveCoeff, float(yAtPress - y) / moveCoeff, 0});
            }
        } else {
            camera->SetTout(cameraToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE){
                sceneRoot->RotateInSystem(system, float(x - lastx) / angleCoeff* 3, Axis::Y);
                sceneRoot->RotateInSystem(system, float(y - lasty) / angleCoeff* 3, Axis::X);
                lastx = x;
                lasty = y;
            } 
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                camera->RotateInSystem(system, float(x - xAtPress) / 180, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE) {
                sceneRoot->TranslateInSystem(system, {float(lastx - x) / moveCoeff * 0.2f, float(y - lasty) / moveCoeff* 0.2f, 0});
                lastx = x;
                lasty = y;
            }
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
        // pickedModel = pickedModel->modelOnPick != nullptr ? pickedModel->modelOnPick : pickedModel;
        pickedModelDepth = modelAndDepth.second;
        camera->GetRotation().transpose();
        xAtPress = x;
        yAtPress = y;
        lastx = x;
        lasty = y;

        if (pickedModel && !pickedModel->isPickable)
            pickedModel = nullptr; // for non-pickable models we need only pickedModelDepth for mouse movement calculations later

        if (pickedModel){
            pickedToutAtPress = pickedModel->GetTout();
            otherPickedToutAtPress = pickedModel->modelOnPick != nullptr ? pickedModel->modelOnPick->GetTout() : pickedModel->GetTout();
        }
            
        else
            cameraToutAtPress = camera->GetTout();
    }
}

Eigen::Vector3f BasicScene::getTipOfLink(int ind){
    Eigen::Vector3f pos = std::move(root->GetTranslation());
    for (int i = 0; i <= ind; i++)
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

    
    sceneRoot = cg3d::Model::Create("sroot", sphereMesh, material);
    sceneRoot->isHidden = true;
    AddChild(sceneRoot);

    root = cg3d::Model::Create("root", sphereMesh, material);
    root->isHidden = true;
    sceneRoot->AddChild(root);

    sphere = cg3d::Model::Create("sphere", sphereMesh, material);
    sphere->showWireframe = true;
    sceneRoot->AddChild(sphere);
    sphere->Translate(Eigen::Vector3f{5,0,0});

    

    axis.push_back(Model::Create("axis",coordsys,material));
    axis[0]->mode = 1; 
    root->AddChild(axis[0]);
    axis[0]->modelOnPick = root;

    links.push_back(cg3d::Model::Create("link", cylMesh, material));
    links[0]->showWireframe = true;
    links[0]->Translate(0.8f, Axis::Z);
    links[0]->SetCenter(Eigen::Vector3f(0,0,-0.8));
    root->AddChild(links[0]);
    links[0]->modelOnPick = root;


    for (size_t i = 1; i < 4; i++)
    {
        links.push_back(cg3d::Model::Create("link", cylMesh, material));
        links[i]->showWireframe = true;
        links[i]->Translate(1.6f, Axis::Z);
        links[i]->SetCenter(Eigen::Vector3f(0,0,-0.8));
        links[i-1]->AddChild(links[i]);
        links[i]->modelOnPick = root;

        axis.push_back(Model::Create("axis",coordsys,material));
        axis[i]->mode = 1; 
        axis[i]->Translate(0.8f, Axis::Z);
        links[i-1]->AddChild(axis[i]);
        axis[i]->modelOnPick = root;
    }

    // links[0]->RotateByDegree(-90, Axis::X);
    // axis[0]->RotateByDegree(-90, Axis::X);
    sceneRoot->RotateByDegree(-90, Axis::X);
    

    camera->Translate(20, Axis::Z);
    // cube->Scale(3);
}
void BasicScene::CCD(){
    if (pause)
        return;
    if ((getTipOfLink(-1) - sphere->GetTranslation()).norm() > links.size() * 1.6)
    {
        std::cout << "cannot reach" << std::endl;
        return;
    }
    
    static int link_index = links.size();
    Eigen::Vector3f E, R, RD, RE;
    static float angle = 0;
    static int count = 99;
    static Eigen::Vector3f rotateAxis{0,0,0};
    if (++count == 100)
    {
        link_index--;
        if (link_index == -1)
        {
            link_index = links.size()-1;
        }
        E = getTipOfLink(links.size()-1);
        R = getTipOfLink(link_index-1);
        RD = (sphere->GetTranslation() - R).normalized();
        RE = (E - R).normalized();
        angle = acos(std::clamp(RD.dot(RE), -1.0f, 1.0f));
        rotateAxis = (RE.cross(RD)).normalized();
        count = 0;
    }

    links[link_index]->RotateInSystem(axis[link_index]->GetRotation(),angle * 0.01f, rotateAxis);
    

    if ((getTipOfLink(links.size()-1) - sphere->GetTranslation()).norm() < delta){
        pause = true;
        std::cout << "reached, distance to destination is: " << (getTipOfLink(links.size()-1) - sphere->GetTranslation()).norm() <<std::endl;
    }

    
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    if (++counter == 7)
    {
        CCD();
        counter = 0;
    }
    
    
    // cube->Rotate(0.01f, Axis::XYZ);
}