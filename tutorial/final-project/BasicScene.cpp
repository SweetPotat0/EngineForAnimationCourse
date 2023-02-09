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

#include "imgui.h"
#include "file_dialog_open.h"
#include "GLFW/glfw3.h"

using namespace cg3d;

void BasicScene::animation()
{
    std::cout << "System: " << std::endl;
    std::cout << Eigen::Affine3f(links[3]->GetAggregatedTransform()).rotation() << std::endl;

    Eigen::MatrixX3f system = Eigen::Affine3f(links[3]->GetAggregatedTransform()).rotation().transpose();
    links[0]->TranslateInSystem(system, Eigen::Vector3f(0.01f, 0, 0));
}
void BasicScene::KeyCallback(cg3d::Viewport *viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camList[1]->GetRotation().transpose();
    Eigen::Vector3f pos, angles;
    Eigen::Matrix3f A1, A2, A3;

    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_SPACE:
            animate = !animate;
            std::cout << camList[1]->GetRotation() << std::endl;
            break;

        case GLFW_KEY_ESCAPE:
            // open menu
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_W:
            // links[0]->Rotate(0.1, Axis::X);
            links[3]->RotateInSystem(system, 0.1, Axis::X);
            break;
        case GLFW_KEY_S:
            // links[0]->Rotate(-0.1, Axis::X);
            links[3]->RotateInSystem(system, -0.1, Axis::X);
            break;
        case GLFW_KEY_A:
            // links[picked_index]->Rotate(-0.1,Axis::Z);
            links[3]->RotateInSystem(system, -0.1, Axis::Z);
            break;
        case GLFW_KEY_D:
            // links[picked_index]->Rotate(0.1,Axis::Z);
            links[3]->RotateInSystem(system, 0.1, Axis::Z);
            break;
        }
    }
}

void BasicScene::ScrollCallback(Viewport *viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[])
{
    if (ImGui::GetIO().WantCaptureMouse)
        return;
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    auto system = camera->GetRotation().transpose();
    if (pickedModel)
    {
        if (pickedModel->modelOnPick != nullptr)
        {
            pickedModel->modelOnPick->TranslateInSystem(system, {0, 0, -float(yoffset)});
            otherPickedToutAtPress = pickedModel->modelOnPick->GetTout();
        }
        else
        {
            pickedModel->TranslateInSystem(system, {0, 0, -float(yoffset)});
            pickedToutAtPress = pickedModel->GetTout();
        }
    }
    else
    {
        camera->TranslateInSystem(system, {0, 0, -float(yoffset)});
        cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::CursorPosCallback(Viewport *viewport, int x, int y, bool dragging, int *buttonState)
{
    if (ImGui::GetIO().WantCaptureMouse)
        return;
    std::shared_ptr<cg3d::Model> actuallyPicked = pickedModel;
    // std::cout << "before dragging" << std::endl;
    if (dragging)
    {
        auto system = camera->GetRotation().transpose();
        auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
        auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
        if (pickedModel)
        {
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE)
            {
                pickedModel->SetTout(pickedToutAtPress);
                pickedModel->RotateInSystem(system, float(x - xAtPress) / moveCoeff, Axis::Y);
                pickedModel->RotateInSystem(system, float(y - yAtPress) / moveCoeff, Axis::X);
            }
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
            {
                pickedModel->SetTout(pickedToutAtPress);
                pickedModel->RotateInSystem(system, float(x - xAtPress) / moveCoeff, Axis::Z);
            }
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
            {
                if (pickedModel->modelOnPick != nullptr)
                {
                    actuallyPicked = pickedModel->modelOnPick;
                    pickedModel->modelOnPick->SetTout(otherPickedToutAtPress);
                }
                pickedModel->SetTout(pickedToutAtPress);
                actuallyPicked->TranslateInSystem(system * sceneRoot->GetRotation(), {float(x - xAtPress) / moveCoeff, float(yAtPress - y) / moveCoeff, 0});
            }
        }
        else
        {
            camera->SetTout(cameraToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE)
            {
                sceneRoot->RotateInSystem(system, float(x - lastx) / angleCoeff * 3, Axis::Y);
                sceneRoot->RotateInSystem(system, float(y - lasty) / angleCoeff * 3, Axis::X);
                lastx = x;
                lasty = y;
            }
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
            {
                camera->RotateInSystem(system, float(x - xAtPress) / 180, Axis::Z);
            }
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
            {
                sceneRoot->TranslateInSystem(system, {float(lastx - x) / moveCoeff * 0.2f, float(y - lasty) / moveCoeff * 0.2f, 0});
                lastx = x;
                lasty = y;
            }
        }
    }
}

void BasicScene::MouseCallback(Viewport *viewport, int x, int y, int button, int action, int mods, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    if (ImGui::GetIO().WantCaptureMouse)
        return;
    if (action == GLFW_PRESS)
    { // default mouse button press behavior
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

        if (pickedModel)
        {
            pickedToutAtPress = pickedModel->GetTout();
            otherPickedToutAtPress = pickedModel->modelOnPick != nullptr ? pickedModel->modelOnPick->GetTout() : pickedModel->GetTout();
        }

        else
            cameraToutAtPress = camera->GetTout();
    }
}

Eigen::Vector3f BasicScene::getTipOfLink(int ind)
{
    Eigen::Vector3f pos = std::move(root->GetTranslation());
    for (int i = 0; i <= ind; i++)
    {
        pos += links[i]->GetRotation() * Eigen::Vector3f{0, 0, 1.6f};
    }
    return pos;
}

void BasicScene::SetCamera(int index)
{
    camera = camList[index];
    viewport->camera = camera;
}

void BasicScene::AddViewportCallback(Viewport *_viewport)
{
    viewport = _viewport;

    Scene::AddViewportCallback(viewport);
}

void TextCentered(std::string text, float margin = 0)
{
    auto windowWidth = ImGui::GetWindowSize().x;
    auto textWidth = ImGui::CalcTextSize(text.c_str()).x;

    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + margin);
}

void BasicScene::BuildImGui()
{
    int flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    bool *pOpen = nullptr;

    ImGui::Begin("Menu", pOpen, flags);

    switch (gameState)
    {
    case GameState::Menu:
    {
        float window_width = 200;
        float window_height = 200;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("Snake 3D", 30);
        ImGui::Text("Snake 3D");
        TextCentered("Start Game", 10);
        if (ImGui::Button("Start Game"))
        {
            gameState = GameState::MidLevel;
            animate = true;
        }
        TextCentered("Quit Game", 10);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case GameState::MidLevel:
    {
        ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::Text("Camera: ");
        for (int i = 0; i < camList.size(); i++)
        {
            bool selectedCamera = camList[i] == camera;
            if (selectedCamera)
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
            }
            if (ImGui::Button(camList[i]->name.c_str()))
            {
                SetCamera(i);
            }
            if (selectedCamera)
                ImGui::PopStyleColor();
        }
        ImGui::SameLine();
        if (ImGui::Button("Center"))
            camera->SetTout(Eigen::Affine3f::Identity());
        ImGui::Text("AXES: X-RED Y-GREEN Z-BLUE");
        if (pickedModel)
        {
            ImGui::Text("Picked model: %s", pickedModel->name.c_str());
            ImGui::SameLine();
            if (ImGui::Button("Drop"))
                pickedModel = nullptr;
            if (pickedModel)
            {
                if (ImGui::CollapsingHeader("Draw options", ImGuiTreeNodeFlags_DefaultOpen))
                {
                    ImGui::Checkbox("Show wireframe", &pickedModel->showWireframe);
                    if (pickedModel->showWireframe)
                    {
                        ImGui::Text("Wireframe color:");
                        ImGui::SameLine();
                        ImGui::ColorEdit4("Wireframe color", pickedModel->wireframeColor.data(), ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel);
                    }
                    ImGui::Checkbox("Show faces", &pickedModel->showFaces);
                    ImGui::Checkbox("Show textures", &pickedModel->showTextures);
                    if (ImGui::Button("Scale down"))
                        pickedModel->Scale(0.9f);
                    ImGui::SameLine();
                    if (ImGui::Button("Scale up"))
                        pickedModel->Scale(1.1f);
                }
                if (ImGui::Button("Dump model transformations"))
                {
                    Eigen::IOFormat format(2, 0, ", ", "\n", "[", "]");
                    const Eigen::Matrix4f &transform = pickedModel->GetAggregatedTransform();
                    std::cout << "Tin:" << std::endl
                              << pickedModel->Tin.matrix().format(format) << std::endl
                              << "Tout:" << std::endl
                              << pickedModel->Tout.matrix().format(format) << std::endl
                              << "Transform:" << std::endl
                              << transform.matrix().format(format) << std::endl
                              << "--- Transform Breakdown ---" << std::endl
                              << "Rotation:" << std::endl
                              << Movable::GetTranslation(transform).matrix().format(format) << std::endl
                              << "Translation:" << std::endl
                              << Movable::GetRotation(transform).matrix().format(format) << std::endl
                              << "Rotation x Translation:" << std::endl
                              << Movable::GetTranslationRotation(transform).matrix().format(format)
                              << std::endl
                              << "Scaling:" << std::endl
                              << Movable::GetScaling(transform).matrix().format(format) << std::endl;
                }
            }
        }
        break;
    }
    case GameState::AfterLevel:
    {
        ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(0, 0), ImGuiCond_Always);
        break;
    }
    default:
    {
        break;
    }
    }

    ImGui::End();
}

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    DISPLAY_HEIGHT = height;
    DISPLAY_WIDTH = width;
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")};
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    auto program = std::make_shared<Program>("shaders/basicShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");
    auto program2 = std::make_shared<Program>("shaders/axisShader");
    auto material = std::make_shared<Material>("material", program);
    auto material1{std::make_shared<Material>("material", program1)};
    auto axis_material = std::make_shared<Material>("axis-material", program2);
    material->AddTexture(0, "textures/box0.bmp", 2);

    auto cylMesh{IglLoader::MeshFromFiles("cyl_igl", "data/snake2.obj")};
    auto sphereMesh{IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj")};
    auto coordsys = Mesh::Axis();

    sceneRoot = cg3d::Model::Create("sroot", sphereMesh, material);
    sceneRoot->isHidden = true;
    AddChild(sceneRoot);
    // sceneRoot->AddChild(Model::Create("sceneRoot axis", coordsys, axis_material));

    root = cg3d::Model::Create("root", sphereMesh, material);
    root->isHidden = true;

    // create the camera objects
    camList.push_back(Camera::Create("Third Person Camera", fov, float(width) / float(height), near, far));
    camList.push_back(Camera::Create("First Person Camera", fov, float(width) / float(height), near, far));
    camList.push_back(Camera::Create("Static Top Camera", fov, float(width) / float(height), near, far));
    camera = camList[0];

    // Third Person axis
    axis1.push_back(Model::Create("cam index 0 axis", coordsys, axis_material));
    axis1[0]->mode = 1;
    camList[0]->AddChild(axis1[0]);

    // First Person Axis
    axis1.push_back(Model::Create("cam index 1 axis", coordsys, axis_material));
    axis1[1]->mode = 1;
    camList[1]->AddChild(axis1[1]);

    sceneRoot->AddChild(root);

    sphere = cg3d::Model::Create("sphere", sphereMesh, material);
    sphere->showWireframe = true;
    sceneRoot->AddChild(sphere);
    sphere->Translate(Eigen::Vector3f{5, 0, 0});

    axis.push_back(Model::Create("root axis", coordsys, axis_material));
    axis[0]->mode = 1;
    root->AddChild(axis[0]);
    axis[0]->SetTout(Eigen::Affine3f::Identity());
    axis[0]->modelOnPick = root;

    links.push_back(cg3d::Model::Create("link 0", cylMesh, material));
    links[0]->showWireframe = true;
    links[0]->Translate(0.8f, Axis::Z);
    links[0]->SetCenter(Eigen::Vector3f(0, 0, -0.8));
    root->AddChild(links[0]);

    int linksCount = 4;

    for (size_t i = 1; i < linksCount; i++)
    {
        links.push_back(cg3d::Model::Create("link " + std::to_string(i), cylMesh, material));
        links[i]->showWireframe = true;
        links[i]->Translate(1.6f, Axis::Z);
        links[i]->SetCenter(Eigen::Vector3f(0, 0, -0.8));
        links[i - 1]->AddChild(links[i]);

        axis.push_back(Model::Create("axis of link " + std::to_string(i - 1), coordsys, axis_material));
        axis[i]->mode = 1;
        links[i - 1]->AddChild(axis[i]);
        axis[i]->SetTout(Eigen::Affine3f::Identity());
    }

    axis.push_back(Model::Create("axis of link " + std::to_string(linksCount - 1), coordsys, axis_material));
    axis[linksCount]->mode = 1;
    links[linksCount - 1]->AddChild(axis[linksCount]);
    axis[linksCount]->SetTout(Eigen::Affine3f::Identity());

    links[linksCount - 1]->AddChild(camList[0]);
    links[linksCount - 1]->AddChild(camList[1]);

    // links[0]->RotateByDegree(-90, Axis::X);
    // axis[0]->RotateByDegree(-90, Axis::X);
    sceneRoot->RotateByDegree(-90, Axis::X);

    // Third Person
    camList[0]->Translate(-10, Axis::Z);
    camList[0]->Translate(-5, Axis::Y);
    camList[0]->RotateByDegree(158.5, Eigen::Vector3f(1, 0, 0));

    float firstPersonOffset = 1;
    // First Person
    camList[1]->SetTout(Eigen::Affine3f::Identity());
    camList[1]->Translate(firstPersonOffset, Axis::Z);
    camList[1]->RotateByDegree(180, Eigen::Vector3f(1, 0, 0));

    // Static Camera above
    camList[2]->SetTout(links[linksCount - 1]->GetTout());
    camList[2]->Translate(20, Scene::Axis::Z);
    camList[2]->Translate(3, Scene::Axis::Y);
}
void BasicScene::CCD()
{
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
    static Eigen::Vector3f rotateAxis{0, 0, 0};
    if (++count == 100)
    {
        link_index--;
        if (link_index == -1)
        {
            link_index = links.size() - 1;
        }
        E = getTipOfLink(links.size() - 1);
        R = getTipOfLink(link_index - 1);
        RD = (sphere->GetTranslation() - R).normalized();
        RE = (E - R).normalized();
        angle = acos(std::clamp(RD.dot(RE), -1.0f, 1.0f));
        rotateAxis = (RE.cross(RD)).normalized();
        count = 0;
    }

    links[link_index]->RotateInSystem(axis[link_index]->GetRotation(), angle * 0.01f, rotateAxis);

    if ((getTipOfLink(links.size() - 1) - sphere->GetTranslation()).norm() < delta)
    {
        pause = true;
        std::cout << "reached, distance to destination is: " << (getTipOfLink(links.size() - 1) - sphere->GetTranslation()).norm() << std::endl;
    }
}

void BasicScene::Update(const Program &program, const Eigen::Matrix4f &proj, const Eigen::Matrix4f &view, const Eigen::Matrix4f &model)
{
    Scene::Update(program, proj, view, model);
    // cube->Rotate(0.01f, Axis::XYZ);
    static int frameCount = 0;
    if (strcmp(program.name.c_str(), "axis-material") == 0)
    {
        Eigen::Vector3f position = Eigen::Affine3f(model).translation();
        program.SetUniform3f("root", position.x(), position.y(), position.z());
    }
}