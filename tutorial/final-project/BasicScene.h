#pragma once

#include "SceneWithImGui.h"

#include "imgui.h"
#include "Camera.h"
#include "file_dialog_open.h"
#include "GLFW/glfw3.h"
#include "igl/AABB.h"
#include <utility>
#include "./SnakePoint.h"
#include "./Enemy.h"
#include "./Ability.h"


typedef 
  std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> >
  RotationList;

class BasicScene : public cg3d::SceneWithImGui
{
public:
    enum GameState
    {
        StartMenu,
        Level1,
        Level2,
        Level3,
        MidLevel,
        AfterLevel,
        Pause
    };

    explicit BasicScene(std::string name, cg3d::Display *display) : SceneWithImGui(std::move(name), display)
    {
        ImGui::GetIO().IniFilename = nullptr;
        ImGui::StyleColorsDark();
        ImGuiStyle &style = ImGui::GetStyle();
        style.FrameRounding = 5.0f;
    };
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program &program, const Eigen::Matrix4f &proj, const Eigen::Matrix4f &view, const Eigen::Matrix4f &model) override;
    void KeyCallback(cg3d::Viewport *viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void MouseCallback(cg3d::Viewport *viewport, int x, int y, int button, int action, int mods, int buttonState[]) override;
    void CursorPosCallback(cg3d::Viewport *viewport, int x, int y, bool dragging, int *buttonState) override;
    void ScrollCallback(cg3d::Viewport *viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]) override;
    void AddViewportCallback(cg3d::Viewport *_viewport) override;
    void CCD();
    void animation() override;
    Eigen::Vector3f getTipOfLink(int ind);

private:
    RotationList rest_pose;
    void AnimateSnakeSkeleton();
    void CheckPointCollisions();
    void CheckEnemyCollisions();
    void useBoostAbility();
    void useInvisAbility();
    void endBoostAbility();
    void endInvisAbility();
    Eigen::Vector4f getDistanceFromColsestJoints(Eigen::Vector3f posV, Eigen::MatrixXd C);
    int DISPLAY_WIDTH = 0;
    int DISPLAY_HEIGHT = 0;
    GameState gameState = GameState::StartMenu;
    cg3d::Viewport *viewport = nullptr;
    Eigen::Vector3f GenerateRandomPoint(std::shared_ptr<cg3d::Camera> camera, float near, float far);
    void SetCamera(int index);
    void startLevel(int level);
    void changeNextLevel();
    void BuildImGui() override;
    std::vector<std::shared_ptr<cg3d::Camera>> camList;
    std::vector<std::shared_ptr<cg3d::Model>> axis, axis1;
    std::vector<std::shared_ptr<Collidable>> links;
    std::shared_ptr<cg3d::Model> pointModel,enemyModel, root, sceneRoot, snake;
    igl::AABB<Eigen::MatrixXd,3> treeSnakeHead;
    std::vector<std::shared_ptr<SnakePoint>> points;
    std::vector<std::shared_ptr<Enemy>> enemies;
    std::shared_ptr<cg3d::Material> snakeSkin, snakeSkinTransparent;
    bool paused = true;
    int picked_index = 0;
    int counter = 0;
    float delta = 0.05f;
    int playingLevel = 0;
    int levelScore = 0;
    float movementSpeed = 0.01f;
    std::chrono::steady_clock::time_point gameTime;
    std::chrono::seconds gameDuration = std::chrono::seconds(120);
    Ability boostAbility = Ability(15, 5);
    Ability invisAbility = Ability(10, 3);

    float linkSize = 1.6f;
    Eigen::MatrixXd C,V,U,W;
    Eigen::MatrixXi BE,F;
    Eigen::VectorXi P;
    std::vector<RotationList > poses;
    double anim_t=0.0;


    int lastx = -1, lasty = -1;
    Eigen::Affine3f otherPickedToutAtPress;
};