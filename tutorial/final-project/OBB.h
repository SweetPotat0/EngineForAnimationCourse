#pragma once

#include "Scene.h"
#include "igl/AABB.h"
#include <utility>

class OBB
{
public:
    OBB(Eigen::AlignedBox3d box, std::shared_ptr<cg3d::Model> model);
    OBB(OBB& other);
    Eigen::Vector3f Pos, AxisX, AxisY, AxisZ, Half_size;
    bool IsCollision(const OBB& other);
    ~OBB();
};