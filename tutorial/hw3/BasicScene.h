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

private:
    std::vector<std::shared_ptr<cg3d::Model>> links, axis;
    std::shared_ptr<cg3d::Model> sphere;
    Eigen::Vector3f dest {5,0,0};
    bool pause = false;
    int picked_index = 0;
    
};
