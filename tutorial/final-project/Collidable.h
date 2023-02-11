#pragma once

#include "Scene.h"
#include "./OBB.h"
#include "igl/AABB.h"
#include <utility>

class Collidable
{
public:
    Collidable(cg3d::Model* model);
    void ReCalculateCollidingTree();
    OBB *GetColliderOBB();
    OBB** getCollidingOBB(Collidable* other);
    cg3d::Model* Model;
    void ShowCollider();
    void HideCollider();
private:
    Eigen::Vector3f oldT1;
    Eigen::Vector3f oldS1;
    std::shared_ptr<cg3d::Model> collider;
    OBB** getCollidingOBB(igl::AABB<Eigen::MatrixXd, 3> *tree1, igl::AABB<Eigen::MatrixXd, 3> *tree2,
                                  cg3d::Model *model1, cg3d::Model *model2);
    igl::AABB<Eigen::MatrixXd, 3>* collisionTree;
    
};