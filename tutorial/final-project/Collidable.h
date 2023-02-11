#pragma once

#include "Scene.h"
#include "./OBB.h"
#include "igl/AABB.h"
#include <utility>

class Collidable
{
public:
    Collidable(std::shared_ptr<cg3d::Model> model);
    void ReCalculateCollidingTree();
    OBB *GetColliderOBB(OBB::TransformType transform_type);
    OBB** getCollidingOBB(std::shared_ptr<Collidable> other);
    std::shared_ptr<cg3d::Model> Model;
    void ShowCollider();
    void HideCollider();
private:
    Eigen::Vector3f oldT1;
    Eigen::Vector3f oldS1;
    std::shared_ptr<cg3d::Model> collider;
    OBB** getCollidingOBB(igl::AABB<Eigen::MatrixXd, 3>* tree1, igl::AABB<Eigen::MatrixXd, 3>* tree2,
                                  std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2);
    std::shared_ptr<igl::AABB<Eigen::MatrixXd, 3>> collisionTree;
    
};