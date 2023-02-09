#pragma once

#include "Visitor.h"
#include "Camera.h"

#include <utility>
#include <memory>


namespace cg3d
{

class AnimationVisitor : public Visitor
{
public:
    void Run(Scene* scene, Camera* camera) override;
    void Visit(Model* model) override;
    // void Visit(Scene* _scene) override;
    void Init() override;


private:

    Scene* scene;
};

} // namespace cg3d
