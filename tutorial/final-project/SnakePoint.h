#pragma once

#include "imgui.h"
#include "file_dialog_open.h"
#include "GLFW/glfw3.h"
#include <utility>
#include "./Collidable.h"

class SnakePoint : public Collidable
{
public:
    SnakePoint(std::shared_ptr<cg3d::Model> Model, int Score);
    int Score;
};