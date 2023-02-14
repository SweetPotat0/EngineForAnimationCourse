#pragma once

#include "imgui.h"
#include "file_dialog_open.h"
#include "GLFW/glfw3.h"
#include <utility>
#include "./Collidable.h"

class Enemy : public Collidable
{
public:
    Enemy(std::shared_ptr<cg3d::Model> Model, Eigen::Vector3f toOrigin, float moveSpeed, Eigen::Vector3f startinPosition);
    bool ifReachedDest();
    void moveTowardsDest();
    Eigen::Vector3f destination={0,0,0};
    Eigen::Vector3f toOrigin; // idk why but the lion.off needs {-40.5,0,14} translation to be centred
    Eigen::Vector3f startinPosition;
    float moveSpeed;

};