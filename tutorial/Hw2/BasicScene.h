#pragma once

#include "Scene.h"
#include "Viewport.h"
#include "AutoMorphingModel.h"
#include "igl/AABB.h"
#include <utility>

struct OBB
{
    Eigen::Vector3f Pos, AxisX, AxisY, AxisZ, Half_size;
};

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display *display) : Scene(std::move(name), display){};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program &program, const Eigen::Matrix4f &proj, const Eigen::Matrix4f &view, const Eigen::Matrix4f &model) override;
    void KeyCallback(cg3d::Viewport *viewport, int x, int y, int key, int scancode, int action, int mods) override;

private:
    bool collapseTenPerEdges(float ratio, std::shared_ptr<cg3d::AutoMorphingModel> cyl);
    void showOBB(OBB* obb);
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> cyl1, cyl2 ,cube1, cube2, cube3, cube4;
    igl::AABB<Eigen::MatrixXd,3> treeCyl1, treeCyl2;
    bool pause = false;
};

bool getSeparatingPlane(const Eigen::Vector3f& RPos, const Eigen::Vector3f& Plane, const OBB& box1, const OBB&box2);

bool isCollision(const OBB& box1, const OBB&box2);

OBB getOBBfromAABB(Eigen::AlignedBox3d box, std::shared_ptr<cg3d::AutoMorphingModel> model);

OBB* getCollidingOBB(igl::AABB<Eigen::MatrixXd,3> *tree1,igl::AABB<Eigen::MatrixXd,3> *tree2, std::shared_ptr<cg3d::AutoMorphingModel> model1, std::shared_ptr<cg3d::AutoMorphingModel> model2);


