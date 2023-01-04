#pragma once

#include "Scene.h"

#include <utility>

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport *viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void MouseCallback(cg3d::Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[])override;
    void CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState) override;
    void ScrollCallback(cg3d::Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]) override;
    void CCD();
    Eigen::Vector3f getTipOfLink(int ind);

private:
    std::vector<std::shared_ptr<cg3d::Model>> links, axis;
    std::shared_ptr<cg3d::Model> sphere, root, sceneRoot;
    bool pause = true;
    int picked_index = 0;
    int counter = 0;
    float delta = 0.05;

    int lastx = -1, lasty = -1;
    Eigen::Affine3f otherPickedToutAtPress;
    
};
