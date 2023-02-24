#define _USE_MATH_DEFINES

#include "./BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include <cmath>
#include <random>

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
#include "igl/directed_edge_parents.h"
#include "igl/directed_edge_orientations.h"
#include "igl/forward_kinematics.h"
#include "igl/dqs.h"
#include "igl/per_vertex_normals.h"

#include "imgui.h"
#include "file_dialog_open.h"
#include "GLFW/glfw3.h"
#include <Windows.h>
#include <mmsystem.h>
#pragma comment(lib, "WinMM.lib")

using namespace cg3d;

Eigen::Matrix3f Matrix3dToMatrix3f(const Eigen::Matrix3d &mat)
{
    // Create a new matrix of type Eigen::Matrix3f and copy the values of the input matrix to it
    Eigen::Matrix3f mat_f;
    mat_f << static_cast<float>(mat(0, 0)), static_cast<float>(mat(0, 1)), static_cast<float>(mat(0, 2)),
        static_cast<float>(mat(1, 0)), static_cast<float>(mat(1, 1)), static_cast<float>(mat(1, 2)),
        static_cast<float>(mat(2, 0)), static_cast<float>(mat(2, 1)), static_cast<float>(mat(2, 2));

    return mat_f;
}

Eigen::Matrix3f QuaternionToRotationMatrix(const Eigen::Quaterniond &q)
{
    // Convert quaternion to rotation matrix using Eigen's built-in functions
    Eigen::Matrix3f rotation_matrix = Matrix3dToMatrix3f(q.toRotationMatrix());

    return rotation_matrix;
}

void BasicScene::animation()
{
    if (std::chrono::steady_clock::now() - gameTime > gameDuration)
    {
        std::cout << "GAME OVER" << std::endl;
        gameState = GameState::AfterLevel;
        animate = false;
        paused = true;
    }

    static bool isTongIn = false;
    auto nanoEllapsed = std::chrono::steady_clock::now() - gameTime;
    int milliEllapsed = nanoEllapsed.count() * powl(10, -6);
    std::cout << "Elapsed milli: " << milliEllapsed << std::endl;
    long secondsElapsed = nanoEllapsed.count() * powl(10, -8);

    if (isTongIn && (secondsElapsed % 10) < 5)
    {
        snakeTongueModel->Translate({0, 0, 1});
        isTongIn = false;
    }
    else if (isTongIn == false && (secondsElapsed % 10) >= 5)
    {
        snakeTongueModel->Translate({0, 0, -1});
        isTongIn = true;
    }

    // Fix seconds
    secondsElapsed = secondsElapsed / 10.0f;

    if (boostAbility.didAbilityEnd())
        endBoostAbility();
    if (invisAbility.didAbilityEnd())
        endInvisAbility();

    Eigen::Vector3f localZAxis = links[links.size() - 1]->Model->GetAggregatedTransform().block<3, 1>(0, 2);
    snake->Translate(localZAxis.normalized() * movementSpeed);

    AnimateSnakeSkeleton();

    CheckSelfCollisions();

    CheckPointCollisions();
    if (playingLevel == 2)
    {
        // Check enemy collisions:
        for (size_t i = 0; i < enemies.size(); i++)
        {
            if (enemies[i]->ifReachedDest())
                enemies[i]->destination = GenerateRandomPoint(BasicScene::camList[2], 5, 35);
            enemies[i]->moveTowardsDest();
        }
        CheckEnemyCollisions();
    }
    if (playingLevel == 3)
    {
        int timeToVanish = pow(10, 4);
        eggAlpha = (float)(timeToVanish - (milliEllapsed % timeToVanish))/ timeToVanish;
        if (eggAlpha <= 0)
        {
            points[0]->moveToNewPosition(GenerateRandomPoint(camList[2], 5, 35), links[links.size() - 1]->Model->GetTranslation());
            eggAlpha = 1.0f;
        }
    }
}

void BasicScene::CheckSelfCollisions()
{
    for (int i = 0; i < (int)links.size() - 2; i++)
    {
        auto collidingOBB = links[links.size() - 1]->getCollidingOBB(links[i]);
        if (collidingOBB != NULL)
        {
            // Self hit
            free(collidingOBB);
            std::cout << "GAME OVER" << std::endl;
            gameState = GameState::AfterLevel;
            animate = false;
            paused = true;
        }
    }
}

float clamp(float value, float minRange, float maxRange)
{
    return std::max(minRange, std::min(value, maxRange));
}

void BasicScene::AnimateSnakeSkeleton()
{
    // Animate Skin
    RotationList anim_pose(links.size());

    float step = clamp(movementSpeed / 2, 0, 1);

    for (size_t i = links.size() - 1; i > 0; i--)
    {
        auto sonQuaternion = Eigen::Quaternionf(links[i]->Model->GetTout().rotation());
        sonQuaternion.normalize();

        auto midQ = Eigen::Quaternionf::Identity().slerp(step, sonQuaternion).normalized();
        links[i - 1]->Model->Rotate(midQ.toRotationMatrix());
        links[i]->Model->Rotate(midQ.conjugate().normalized().toRotationMatrix());
    }

    for (size_t i = 0; i < links.size(); i++)
    {
        auto qf = Eigen::Quaternionf(links[i]->Model->GetTout().rotation()).normalized();
        anim_pose[i] = Eigen::Quaterniond(qf.w(), qf.x(), qf.y(), qf.z());
    }

    RotationList vQ;
    std::vector<Eigen::Vector3d> vT;

    igl::forward_kinematics(C, BE, P, anim_pose, vQ, vT);

    igl::dqs(V, W, vQ, vT, U);
    Eigen::MatrixXd N;
    igl::per_vertex_normals(U, snake->GetMesh()->data[0].faces, N);
    Mesh nextPose("snake", U, F, N, snake->GetMesh()->data[0].textureCoords);
    snake->SetMeshList({std::make_shared<Mesh>(nextPose)});
}

void BasicScene::CheckPointCollisions()
{
    size_t pointsSize = points.size();
    for (size_t i = 0; i < pointsSize; i++)
    {
        auto point = points[i];
        auto collidingOBB = links[links.size() - 1]->getCollidingOBB(point);
        if (collidingOBB != NULL)
        {
            // Hit
            PlaySound("data/PointSound.wav", NULL, SND_FILENAME | SND_ASYNC);

            free(collidingOBB);
            std::cout << "You hit! Earned " << point->Score << " points!" << std::endl;
            levelScore += point->Score;

            if (links.size() == maxLinksCount - 1)
            {
                std::cout << "You win the level" << std::endl;
                gameState = GameState::WinLevel;
                animate = false;
                paused = true;
            }
            else
            {
                point->moveToNewPosition(GenerateRandomPoint(camList[2], 5, 35), links[links.size() - 1]->Model->GetTranslation());
                AddLinkToSnake();
                eggAlpha = 1.0f;
            }
        }
    }
}
void BasicScene::CheckEnemyCollisions()
{
    bool foundCollision = invisAbility.inUse;
    for (size_t i = 0; i < enemies.size() && !foundCollision; i++)
    {
        for (size_t j = 0; j < links.size() && !foundCollision; j++)
        {
            auto collidingOBB = links[j]->getCollidingOBB(enemies[i]);
            if (collidingOBB != NULL)
            {
                // Hit
                PlaySound("data/SwordSound.wav", NULL, SND_FILENAME | SND_ASYNC);
                free(collidingOBB);
                std::cout << "GAME OVER" << std::endl;
                gameState = GameState::AfterLevel;
                animate = false;
                paused = true;
            }
        }
    }
}

Eigen::Quaternionf get_rotation_quaternion(const Eigen::Quaternionf &fatherQ, const Eigen::Quaternionf &sonQ)
{
    // Normalize the input quaternions
    Eigen::Quaternionf fatherQ_normalized = fatherQ.normalized();
    Eigen::Quaternionf sonQ_normalized = sonQ.normalized();

    // Compute the quaternion representing the rotation from fatherQ to sonQ
    Eigen::Quaternionf rotationQ = sonQ_normalized * fatherQ_normalized.conjugate();

    return rotationQ;
}

void BasicScene::KeyCallback(cg3d::Viewport *viewport, int x, int y, int key, int scancode, int action, int mods)
{
    static bool wasd[4];
    auto rotationAngle = 4.0f;
    auto system = camList[1]->GetRotation().transpose();
    // Eigen::Matrix3f system2 = Eigen::Affine3f::Identity();
    Eigen::Vector3f pos, angles;
    Eigen::Vector3f a, b;

    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {

        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_SPACE:
            animate = !animate;
            break;

        case GLFW_KEY_ESCAPE:
            // open menu
            paused = true;
            gameState = Pause;
            animate = false;
            break;
        case GLFW_KEY_W:
            wasd[0] = true;
            break;
        case GLFW_KEY_A:
            wasd[1] = true;
            break;
        case GLFW_KEY_S:
            wasd[2] = true;
            break;
        case GLFW_KEY_D:
            wasd[3] = true;
            break;
        case GLFW_KEY_J:
            AddLinkToSnake();
            break;
        case GLFW_KEY_K:
        {
            for (size_t i = 0; i < linksCount; i++)
            {
                links[i]->Model->isHidden = !links[i]->Model->isHidden;
            }
            break;
        }

        case GLFW_KEY_E:
            if (!paused)
            {
                if (boostAbility.canUse())
                {
                    useBoostAbility();
                    boostAbility.abilityUsed();
                }
            }
            break;
        case GLFW_KEY_Q:
            if (!paused)
            {
                if (invisAbility.canUse())
                {
                    useInvisAbility();
                    invisAbility.abilityUsed();
                }
                // pointModel->material = snakeSkinTransparent;
            }
            break;

        case GLFW_KEY_1:
            if (!paused)
                SetCamera(0);
            break;
        case GLFW_KEY_2:
            if (!paused)
                SetCamera(1);
            break;
        case GLFW_KEY_3:
            if (!paused)
                SetCamera(2);
            break;
            // case GLFW_KEY_UP:
            // enemy->Translate({0,0,0.5});
            // enemyModel->RotateByDegree(10.0f,Axis::Z);
            // break;
            // case GLFW_KEY_DOWN:
            // enemy->Translate({0,0,-0.5});
            // break;
            // case GLFW_KEY_LEFT:
            // enemy->Translate({-0.5,0,0});
            // break;
            // case GLFW_KEY_RIGHT:
            // enemy->Translate({0.5,0,0});
            // break;
        }
    }
    else if (action == GLFW_RELEASE)
    {
        switch (key)
        {
        case GLFW_KEY_W:
            wasd[0] = false;
            break;
        case GLFW_KEY_A:
            wasd[1] = false;
            break;
        case GLFW_KEY_S:
            wasd[2] = false;
            break;
        case GLFW_KEY_D:
            wasd[3] = false;
            break;
        }
    }

    if (!paused)
    {
        if (wasd[0])
        {
            float angle_radians = rotationAngle * M_PI / 180.0f;
            links[links.size() - 1]->Model->Rotate(Eigen::Quaternionf(cos(angle_radians / 2.0), sin(angle_radians / 2.0), 0.0, 0.0).toRotationMatrix());
        }
        if (wasd[1])
        {
            float angle_radians = -rotationAngle * M_PI / 180.0f;
            links[links.size() - 1]->Model->Rotate(Eigen::Quaternionf(cos(angle_radians / 2.0), 0.0, sin(angle_radians / 2.0), 0.0).toRotationMatrix());
        }
        if (wasd[2])
        {

            float angle_radians = -rotationAngle * M_PI / 180.0f;
            links[links.size() - 1]->Model->Rotate(Eigen::Quaternionf(cos(angle_radians / 2.0), sin(angle_radians / 2.0), 0.0, 0.0).toRotationMatrix());
        }
        if (wasd[3])
        {
            float angle_radians = rotationAngle * M_PI / 180.0f;
            links[links.size() - 1]->Model->Rotate(Eigen::Quaternionf(cos(angle_radians / 2.0), 0.0, sin(angle_radians / 2.0), 0.0).toRotationMatrix());
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
        Eigen::Vector3f localZAxis = camera->GetAggregatedTransform().block<3, 1>(0, 2);
        camera->Translate(localZAxis.normalized() * -yoffset);
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
        pos += links[i]->Model->GetRotation() * Eigen::Vector3f{0, 0, linkMeshSize};
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

    ImGui::Text(text.c_str());
}

void cursorCentered(std::string text, float margin = 0)
{
    auto windowWidth = ImGui::GetWindowSize().x;
    auto textWidth = ImGui::CalcTextSize(text.c_str()).x;

    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + margin);
}

void BasicScene::changeNextLevel()
{
    switch (playingLevel)
    {
    case 0:
        gameState = GameState::Level1;
        break;
    case 1:
        gameState = GameState::Level2;
        break;
    case 2:
        gameState = GameState::Level3;
        break;
    case 3:
        gameState = GameState::Level1;
        break;

    default:
        break;
    }
}
void BasicScene::startLevel(int level)
{
    playingLevel = level;
    levelScore = 0;
    animate = true;
    paused = false;
    if (linksCount != startLinksCount)
    {
        snake->RemoveChild(links[0]->Model);
        for (size_t i = 0; i < linksCount - startLinksCount; i++)
        {
            links[0]->Model->RemoveChild(links[1]->Model);
            links[0]->Model->Translate(-links[0]->Model->GetTranslation());
            spareLinks.insert(spareLinks.begin(), links[0]);
            links.erase(links.begin());
        }
        links[0]->Model->Translate({0, 0, -((startLinksCount / 2.0f) + 0.5f) * linkSize});
        // links[0]->Model->Translate(-links[0]->Model->GetTout().translation());
        snake->AddChild(links[0]->Model);

        int oldLinksCount = linksCount;
        linksCount = startLinksCount;
        RecalculateSnakeMesh(oldLinksCount);
    }
    gameState = GameState::MidLevel;
    std::cout << "should start level " << level << std::endl;
    switch (level)
    {
    case 1:
        for (size_t i = 0; i < points.size(); i++)
        { // reset points
            points[i]->Model->isHidden = false;
            // points[i]->Model->SetTout(Eigen::Affine3f::Identity());
            points[i]->moveToNewPosition(GenerateRandomPoint(camList[2], 5, 35), links[links.size() - 1]->Model->GetTranslation());
        }
        snake->Translate(-snake->GetTranslation()); // reset snake
        for (size_t i = 0; i < links.size(); i++)
            links[i]->Model->Rotate(links[i]->Model->GetRotation().matrix().inverse());
        gameTime = std::chrono::steady_clock::now();
        break;
    case 2:
        for (size_t i = 0; i < points.size(); i++) // reset points
            points[i]->Model->isHidden = false;

        snake->Translate(-snake->GetTranslation()); // reset snake
        // links[0]->Model->Translate(-links[0]->Model->GetTranslation()); // reset snake
        for (size_t i = 0; i < links.size(); i++)
            links[i]->Model->Rotate(links[i]->Model->GetRotation().matrix().inverse());

        for (size_t i = 0; i < enemies.size(); i++)
        { // reset enemies
            enemies[i]->Model->isHidden = false;
            enemies[i]->Model->Translate(enemies[i]->startinPosition);
            enemies[i]->destination = GenerateRandomPoint(BasicScene::camList[2], 5, 35);
        }
        gameTime = std::chrono::steady_clock::now();
        break;

    default:
        break;
    }
}

void BasicScene::BuildImGui()
{
    int flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    bool *pOpen = nullptr;

    ImGui::Begin("Menu", pOpen, flags);

    switch (gameState)
    {
    case GameState::StartMenu:
    {
        float window_width = 200;
        float window_height = 190;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("Snake 3D", 30);
        cursorCentered("Start Game", 25);
        if (ImGui::Button("Start Game"))
        {
            gameState = GameState::Level1;

            // PlaySound("C:\\Users\\ido\\Downloads\\Run-Amok.wav", NULL, SND_FILENAME | SND_ASYNC | SND_LOOP);
        }
        cursorCentered("Quit Game", 25);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case GameState::Level1:
    {
        float window_width = 400;
        float window_height = 340;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("Level 1", 30);
        TextCentered("Controls:", 10);
        TextCentered("W - Up        1 - Third person camera", 10);
        TextCentered("S - Down      2 - First person camera", 10);
        TextCentered("A - Left      3 - Static camera      ", 10);
        TextCentered("D - Right     ESC - Pause game       ", 10);
        TextCentered("Q - Ghost     E - Boost              ", 10);

        TextCentered("Eat the Egg!", 30);
        TextCentered("The faster you get to it, the more points you get", 10);

        cursorCentered("Start Level", 10);
        if (ImGui::Button("Start Level"))
            startLevel(1);
        break;
    }
    case GameState::Level2:
    {
        float window_width = 400;
        float window_height = 250;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("Level 2", 30);

        TextCentered("Eat the Egg!", 30);
        TextCentered("The faster you catch it, the more points you get", 10);
        TextCentered("Dodge the moving Sword", 10);

        cursorCentered("Start Level", 20);
        if (ImGui::Button("Start Level"))
            startLevel(2);
        break;
    }
    case GameState::Level3:
    {
        float window_width = 400;
        float window_height = 250;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("Level 3", 30);

        TextCentered("Eat the Bunny!", 30);
        TextCentered("The faster you catch it, the more points you get", 10);
        TextCentered("Dodge the moving Lions", 10);
        TextCentered("Snake grows the more you eat", 10);

        cursorCentered("Start Level", 15);
        if (ImGui::Button("Start Level"))
            startLevel(3);
        break;
    }
    case GameState::MidLevel:
    {
        ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(0, 0), ImGuiCond_Always);

        std::string scoreStr = "Score: ";
        scoreStr = scoreStr + std::to_string(levelScore);
        TextCentered(scoreStr.c_str(), 0);

        std::string timerStr = "Timer: ";
        auto remaining_time = gameDuration - (std::chrono::steady_clock::now() - gameTime);
        auto a = std::chrono::duration_cast<std::chrono::seconds>(remaining_time).count();
        timerStr = timerStr + std::to_string(a);
        TextCentered(timerStr.c_str(), 0);

        unsigned int color;
        if (boostAbility.inUse)
            color = IM_COL32(128, 128, 128, 255);
        else if (boostAbility.canUse())
            color = IM_COL32(0, 255, 0, 255);
        else
            color = IM_COL32(255, 0, 0, 255);

        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::Text("Boost");
        ImGui::PopStyleColor();
        ImGui::SameLine();

        if (invisAbility.inUse)
            color = IM_COL32(128, 128, 128, 255);
        else if (invisAbility.canUse())
            color = IM_COL32(0, 255, 0, 255);
        else
            color = IM_COL32(255, 0, 0, 255);

        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::Text("Ghost");
        ImGui::PopStyleColor();
        ImGui::Text("AXES: X-RED Y-GREEN Z-BLUE");
        break;
    }
    case GameState::WinLevel:
    {
        float window_width = 300;
        float window_height = 300;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("You Win!", 30);
        std::string scoreStr = "Score: ";
        scoreStr = scoreStr + std::to_string(levelScore);
        TextCentered(scoreStr.c_str(), 30);

        cursorCentered("Restart Level", 30);
        if (ImGui::Button("Restart Level"))
        {
            gameState = GameState::MidLevel;
            animate = true;
            startLevel(playingLevel);
        }
        cursorCentered("Next Level", 20);
        if (ImGui::Button("Next Level"))
        {
            changeNextLevel();
        }
        cursorCentered("Quit Game", 20);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case GameState::AfterLevel:
    {
        float window_width = 300;
        float window_height = 300;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("GAME OVER", 30);
        std::string scoreStr = "Score: ";
        scoreStr = scoreStr + std::to_string(levelScore);
        TextCentered(scoreStr.c_str(), 30);

        cursorCentered("Restart Level", 30);
        if (ImGui::Button("Restart Level"))
        {
            gameState = GameState::MidLevel;
            animate = true;
            startLevel(playingLevel);
        }
        cursorCentered("Next Level", 20);
        if (ImGui::Button("Next Level"))
        {
            changeNextLevel();
        }
        cursorCentered("Quit Game", 20);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case GameState::Pause:
    {
        float window_width = 380;
        float window_height = 320;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("PAUSED", 30);
        std::string scoreStr = "Current Score: ";
        scoreStr = scoreStr + std::to_string(levelScore);
        TextCentered(scoreStr.c_str(), 30);
        TextCentered("Controls:", 10);
        TextCentered("W - Up        1 - Third person camera", 10);
        TextCentered("S - Down      2 - First person camera", 10);
        TextCentered("A - Left      3 - Static camera      ", 10);
        TextCentered("D - Right     ESC - Pause game       ", 10);

        auto restY = ImGui::GetCursorPosY();
        ImGui::SetCursorPosX(60);
        ImGui::SetCursorPosY(restY + 20);
        if (ImGui::Button("Next Level"))
        {
            changeNextLevel();
        }
        cursorCentered("Resume");
        ImGui::SetCursorPosY(restY + 20);
        if (ImGui::Button("Restart"))
        {
            gameState = GameState::MidLevel;
            animate = true;
            startLevel(playingLevel);
        }
        ImGui::SetCursorPosX(260);
        ImGui::SetCursorPosY(restY + 20);
        if (ImGui::Button("Resume"))
        {
            gameState = GameState::MidLevel;
            animate = true;
            paused = false;
        }

        cursorCentered("Quit Game", 10);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    default:
    {
        break;
    }
    }

    ImGui::End();
}

float WeightFunction(float distance)
{
    // return 1 / powf(distance, 15.0f);
    return 1 / log10f(powf(distance, 2.0f) + 1);
}

Eigen::Vector2f BasicScene::CalculateWeightByDistances(int joint1Index, float distance1, int joint2Index, float distance2)
{
    // The distance from which to give full control to the previous joint
    // Change to zero if you want always to be manipulated by previous joint
    float distanceThreshold = INFINITY;
    if (joint1Index < joint2Index)
    {
        // Give more weight to joint 1
        if (distance2 > distanceThreshold)
        {
            // Distance 2 is too big, give only to 1
            return {1, 0};
        }
        else
        {
            return Eigen::Vector2f(WeightFunction(distance1), WeightFunction(distance2)).normalized();
        }
    }
    else
    {
        // Give more weight to joint 2
        if (distance1 > distanceThreshold)
        {
            // Distance 1 is too big, give only to 2
            return {0, 1};
        }
        else
        {
            return Eigen::Vector2f(WeightFunction(distance1), WeightFunction(distance2)).normalized();
        }
    }
}

Eigen::MatrixXd scaleVertices(const Eigen::MatrixXd &vertices, const Eigen::Vector3d &scale)
{
    // Create a diagonal matrix with the scaling factors
    Eigen::DiagonalMatrix<double, 3> scalingMatrix(scale);

    // Apply the scaling to the vertices
    Eigen::MatrixXd scaledVertices = vertices * scalingMatrix;

    return scaledVertices;
}

void BasicScene::Init(float fov, int width, int height, float near1, float far1)
{
    DISPLAY_HEIGHT = height;
    DISPLAY_WIDTH = width;
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")};
    daylight->AddTexture(0, "textures/cubemaps/Nature Box/Nature Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    auto program = std::make_shared<Program>("shaders/basicShader");
    auto program1 = std::make_shared<Program>("shaders/axisShader");
    auto program2 = std::make_shared<Program>("shaders/basicShader1");
    auto eggShader = std::make_shared<Program>("shaders/eggShader");
    auto material = std::make_shared<Material>("material", program);
    auto paintedEgg{std::make_shared<Material>("paintedEgg", eggShader)};
    snakeSkin = std::make_shared<Material>("snakeSkin", program);
    snakeSkinTransparent = std::make_shared<Material>("snakeSkinTransparent", program2);
    auto swordTex{std::make_shared<Material>("sword", program)};

    auto snake_tongue_material = std::make_shared<Material>("snake_tongue_material", program);
    snake_tongue_material->AddTexture(0, "textures/tong2.jpg", 2);

    auto axis_material = std::make_shared<Material>("axis-material", program1);
    material->AddTexture(0, "textures/box0.bmp", 2);
    snakeSkin->AddTexture(0, "textures/DragonScales_diffuseOriginal.jpg", 2);
    snakeSkinTransparent->AddTexture(0, "textures/DragonScales_diffuseOriginal.jpg", 2);
    paintedEgg->AddTexture(0, "textures/paintedEgg.jpg", 2);
    swordTex->AddTexture(0, "textures/Sword_texture.png", 2);

    auto EnemyMesh{IglLoader::MeshLoader2("enemy", "data/Sword.obj")};
    auto PointMesh{IglLoader::MeshLoader2("eggPoint", "data/egg.obj")};
    auto cylMesh{IglLoader::MeshLoader2("cyl_igl", "data/zcylinder.obj")};
    auto snakeTongueMesh{IglLoader::MeshLoader2("cyl_igl", "data/BambaSnakeTongue.obj")};
    auto sphereMesh{IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj")};

    auto coordsys = Mesh::Axis();

    snakeTongueModel = cg3d::Model::Create("snakeTongue", snakeTongueMesh, snake_tongue_material);
    // snakeTongueModel->isHidden = true;

    sceneRoot = cg3d::Model::Create("sroot", sphereMesh, material);
    sceneRoot->isHidden = true;
    AddChild(sceneRoot);
    // sceneRoot->AddChild(Model::Create("sceneRoot axis", coordsys, axis_material)); //Scene root has bugs for some reasons

    root = cg3d::Model::Create("root", sphereMesh, material);
    root->isHidden = true;

    // create the camera objects
    camList.push_back(Camera::Create("Third Person Camera", fov, float(width) / float(height), near1, far1));
    camList.push_back(Camera::Create("First Person Camera", fov, float(width) / float(height), near1, far1));
    camList.push_back(Camera::Create("Static Top Camera", fov, float(width) / float(height), near1, far1));
    camera = camList[0];

    // Third Person axis
    // axis1.push_back(Model::Create("cam index 0 axis", coordsys, axis_material));
    // axis1[0]->mode = 1;
    // camList[0]->AddChild(axis1[0]);

    // First Person Axis
    // axis1.push_back(Model::Create("cam index 1 axis", coordsys, axis_material));
    // axis1[1]->mode = 1;
    // camList[1]->AddChild(axis1[1]);

    axis.push_back(Model::Create("root axis", coordsys, axis_material));
    axis[0]->mode = 1;
    root->AddChild(axis[0]);
    axis[0]->SetTout(Eigen::Affine3f::Identity());
    axis[0]->modelOnPick = root;

    sceneRoot->AddChild(root);

    auto snakeMesh{IglLoader::MeshLoader2("snake", "data/BambaSnake1.obj")};
    snake = cg3d::Model::Create("snake", snakeMesh, snakeSkin);
    snake->showWireframe = true;
    // snake->showFaces = false;
    // snake->isHidden = true;

    root->AddChild(snake);
    C = Eigen::MatrixXd(linksCount + 1, 3);
    BE = Eigen::MatrixXi(linksCount, 2);

    links.push_back(std::make_shared<Collidable>(cg3d::Model::Create("link 0", cylMesh, snakeSkin)));
    links[0]->Model->showWireframe = true;
    links[0]->Model->Translate(-linkSize * linksCount / 2.0f, Axis::Z);
    links[0]->Model->Translate(linkSize / 2.0f, Axis::Z);
    links[0]->Model->SetCenter(Eigen::Vector3f(0, 0, -linkSize / 2.0f));
    links[0]->Model->Scale(linkSize / linkMeshSize);
    snake->AddChild(links[0]->Model);
    links[0]->Model->isHidden = true;
    C.row(0) << 0, 0, -(linkSize) * (linksCount / 2);
    BE.row(0) << 0, 1;

    for (size_t i = 1; i < linksCount; i++)
    {
        links.push_back(std::make_shared<Collidable>(cg3d::Model::Create("link " + std::to_string(i), cylMesh, snakeSkin)));
        links[i]->Model->showWireframe = true;
        links[i]->Model->Translate(linkSize, Axis::Z);
        links[i]->Model->SetCenter(Eigen::Vector3f(0, 0, -linkSize / 2.0f));
        links[i - 1]->Model->AddChild(links[i]->Model);
        links[i]->Model->Scale(linkSize / linkMeshSize);
        links[i]->Model->isHidden = true;

        C.row(i) << 0, 0, (linkSize) * ((i - (linksCount / 2.0f)));
        BE.row(i) << i, i + 1;
    }

    links[links.size() - 1]->Model->AddChild(snakeTongueModel);
    snakeTongueModel->Translate({0, 0, 1});
    snakeTongueModel->RotateByDegree(180, Axis::Z);
    snakeTongueModel->showWireframe = true;
    snakeTongueModel->Scale({20, 20, 20});

    for (size_t i = 0; i < maxLinksCount - startLinksCount; i++)
    {
        spareLinks.push_back(std::make_shared<Collidable>(cg3d::Model::Create("link " + std::to_string(i), cylMesh, snakeSkin)));
        spareLinks[i]->Model->SetCenter(Eigen::Vector3f(0, 0, -linkSize / 2.0f));
        spareLinks[i]->Model->isHidden = true;
    }

    C.row(linksCount) << 0, 0, (linkSize) * ((linksCount / 2));

    V = scaleVertices(snakeMesh->data[0].vertices, {linkSize / linkMeshSize, linkSize / linkMeshSize, linkSize * linksCount / linkMeshSize});

    F = snakeMesh->data[0].faces;
    U = V;
    igl::directed_edge_parents(BE, P);

    W = Eigen::MatrixXd::Zero(V.rows(), C.rows() - 1);
    for (size_t i = 0; i < V.rows(); i++)
    {
        Eigen::Vector3f v = V.row(i).cast<float>().eval();
        auto res = getDistanceFromColsestJoints(v, C);
        auto weights = CalculateWeightByDistances(res[0], res[2], res[1], res[3]);
        W.row(i)[(int)res[0]] = weights[0];
        W.row(i)[(int)res[1]] = weights[1];
    }

    links[linksCount - 1]->Model->AddChild(camList[0]);
    links[linksCount - 1]->Model->AddChild(camList[1]);

    // Third Person
    camList[0]->Translate(-5, Axis::Z);
    camList[0]->Translate(-4, Axis::Y);
    camList[0]->RotateByDegree(158.5, Eigen::Vector3f(1, 0, 0));

    float firstPersonOffset = 1;
    // First Person
    camList[1]->SetTout(Eigen::Affine3f::Identity());
    camList[1]->Translate(firstPersonOffset, Axis::Z);
    camList[1]->RotateByDegree(180, Eigen::Vector3f(1, 0, 0));

    // Static Camera above
    camList[2]->SetTout(Eigen::Affine3f::Identity());
    camList[2]->Translate(20, Scene::Axis::Y);
    camList[2]->RotateByDegree(-90, Scene::Axis::X);

    pointModel = cg3d::Model::Create("Point", PointMesh, paintedEgg);
    pointModel->Scale(0.02f);
    sceneRoot->AddChild(pointModel);
    pointModel->isHidden = true;
    points.push_back(std::make_shared<SnakePoint>(pointModel, 0));
    pointModel->RotateByDegree(165.0f, Axis::Y);

    enemyModel = cg3d::Model::Create("Enemy", EnemyMesh, swordTex);
    enemyModel->Scale(0.2f);
    enemyModel->isHidden = true;
    sceneRoot->AddChild(enemyModel);
    enemies.push_back(std::make_shared<Enemy>(enemyModel, 0.05, 0.01, Eigen::Vector3f{5, 0, 0}));
}

void BasicScene::AddLinkToSnake()
{
    linksCount++;

    // Insert to links the last element from spareLinks
    links.insert(links.begin(), spareLinks.back());
    // Remove it from spare links
    spareLinks.pop_back();

    links[0]->Model->isHidden = links[1]->Model->isHidden;

    // Add the link to the scene
    snake->AddChild(links[0]->Model);
    snake->RemoveChild(links[1]->Model);

    links[0]->Model->Scale(linkSize / linkMeshSize);

    links[0]->Model->AddChild(links[1]->Model);

    // Move the link to the right location
    links[0]->Model->Translate(-(linksCount)*linkSize / 2.0, Axis::Z);
    links[1]->Model->Translate((linksCount)*linkSize / 2.0, Axis::Z);

    snake->Translate({0, 0, -linkSize / 2.0f});
    links[0]->Model->Translate({0, 0, linkSize / 2.0f});

    RecalculateSnakeMesh(linksCount - 1);
}

void BasicScene::RecalculateSnakeMesh(int oldLinksCount)
{
    // Initialize C and BE again
    C = Eigen::MatrixXd(linksCount + 1, 3);
    BE = Eigen::MatrixXi(linksCount, 2);

    C.row(0) << 0, 0, -(linkSize) * (linksCount / 2.0);
    BE.row(0) << 0, 1;

    for (size_t i = 1; i < linksCount; i++)
    {
        C.row(i) << 0, 0, (linkSize) * ((i - (linksCount / 2.0f)));
        BE.row(i) << i, i + 1;
    }

    C.row(linksCount) << 0, 0, (linkSize) * ((linksCount / 2.0));

    // Scale back V
    V = scaleVertices(V, {linkMeshSize / linkSize, linkMeshSize / linkSize, linkMeshSize / (linkSize * oldLinksCount)});
    // Scale V
    V = scaleVertices(V, {linkSize / linkMeshSize, linkSize / linkMeshSize, linkSize * linksCount / linkMeshSize});

    U = V;
    igl::directed_edge_parents(BE, P);

    W = Eigen::MatrixXd::Zero(V.rows(), C.rows() - 1);
    for (size_t i = 0; i < V.rows(); i++)
    {
        Eigen::Vector3f v = V.row(i).cast<float>().eval();
        auto res = getDistanceFromColsestJoints(v, C);
        auto weights = CalculateWeightByDistances(res[0], res[2], res[1], res[3]);
        W.row(i)[(int)res[0]] = weights[0];
        W.row(i)[(int)res[1]] = weights[1];
    }

    AnimateSnakeSkeleton();
}

void BasicScene::Update(const Program &program, const Eigen::Matrix4f &proj, const Eigen::Matrix4f &view, const Eigen::Matrix4f &model)
{

    Scene::Update(program, proj, view, model);
    static int frameCount = 0;
    frameCount++;
    if (strcmp(program.name.c_str(), "axis-material") == 0)
    {
        Eigen::Vector3f position = Eigen::Affine3f(model).translation();
        program.SetUniform3f("root", position.x(), position.y(), position.z());
    }
    else if (program.name == "eggShader")
    {
        std::cout << "alpha: " << eggAlpha << std::endl;
        program.SetUniform1f("alpha", eggAlpha);
    }
    // else if (strcmp(program.name.c_str(), "paintedEgg") == 0){
    //     program.SetUniform2f("alpha", 1,1);
    // }
}

// Generate a random point within the camera's field of view
Eigen::Vector3f BasicScene::GenerateRandomPoint(std::shared_ptr<cg3d::Camera> camera, float near1, float far1)
{
    // Define a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // Generate a random point within the frustum defined by the camera's field of view
    float z = std::uniform_real_distribution<>(near1, far1)(gen);
    float x = tan((camera->fov * (M_PI / 180)) / 2) * z * camera->ratio;
    float y = tan((camera->fov * (M_PI / 180)) / 2) * z;

    // Generate random x and y values proportional to the tangent of half the field of view and the distance from the camera to the point
    std::uniform_real_distribution<> dist1(-x, x);
    x = dist1(gen);
    std::uniform_real_distribution<> dist2(-y, y);
    y = dist2(gen);

    // Transform the point from camera space to world space
    Eigen::Vector3f result = camera->GetTranslation() + camera->GetRotation() * Eigen::Vector3f(x, y, -z);
    return result;
}

void BasicScene::useBoostAbility()
{
    movementSpeed *= 5;
    PlaySound("data/BoostSound.wav", NULL, SND_FILENAME | SND_ASYNC);
}

void BasicScene::endBoostAbility()
{
    movementSpeed /= 5;
}

void BasicScene::useInvisAbility()
{
    for (size_t i = 0; i < links.size(); i++)
    {
        links[i]->Model->material = snakeSkinTransparent;
    }
    PlaySound("data/GhostSound.wav", NULL, SND_FILENAME | SND_ASYNC);
}
void BasicScene::endInvisAbility()
{
    for (size_t i = 0; i < links.size(); i++)
    {
        links[i]->Model->material = snakeSkin;
    }
}

Eigen::Vector4f BasicScene::getDistanceFromColsestJoints(Eigen::Vector3f posV, Eigen::MatrixXd C)
{
    std::vector<float> distances(C.rows()); // Vector to store distances from posV to each joint
    for (int i = 0; i < C.rows(); i++)
    {
        Eigen::Vector3f posC_float = C.row(i).cast<float>().eval();
        distances[i] = (posV - posC_float).norm(); // Euclidean distance from posV to joint i
    }

    // Find the indices of the two closest joints
    int idx_closest = 0, idx_second_closest = 1;
    if (distances[1] < distances[0])
    {
        std::swap(idx_closest, idx_second_closest);
    }
    for (int i = 2; i < C.rows(); i++)
    {
        if (distances[i] < distances[idx_closest])
        {
            idx_second_closest = idx_closest;
            idx_closest = i;
        }
        else if (distances[i] < distances[idx_second_closest])
        {
            idx_second_closest = i;
        }
    }

    if (idx_closest == C.rows() - 1)
    {
        idx_closest = idx_second_closest;
    }
    else if (idx_second_closest == C.rows() - 1)
    {
        idx_second_closest = idx_closest;
    }

    Eigen::Vector4f result;
    result << idx_closest, idx_second_closest, distances[idx_closest], distances[idx_second_closest];

    return result;
}