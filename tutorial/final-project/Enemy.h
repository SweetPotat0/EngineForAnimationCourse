#pragma once

#include "imgui.h"
#include "file_dialog_open.h"
#include "GLFW/glfw3.h"
#include <utility>
#include "./Collidable.h"

class Enemy : public Collidable
{
public:
    Enemy(std::shared_ptr<cg3d::Model> Model, float moveSpeed, float spinSpeed, Eigen::Vector3f startinPosition);
    bool ifReachedDest();
    void moveTowardsDest();
    Eigen::Vector3f destination={0,0,0};
    Eigen::Vector3f startinPosition;
    float spinSpeed;
    float moveSpeed;

};