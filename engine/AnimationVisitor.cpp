#include "AnimationVisitor.h"

#include <GL.h>
#include <string>
#include "Model.h"
#include "Scene.h"
#include "Utility.h"
#include "igl/unproject_onto_mesh.h"


namespace cg3d
{
void AnimationVisitor::Run(Scene* _scene, Camera* camera)
{
    Visitor::Run(scene = _scene, camera);
}

void AnimationVisitor::Init()
{
    // clear and set up the depth and color buffers (and the stencil buffer if outline is enabled)
    std::cout<< "here"<< std::endl;
}

// void AnimationVisitor::Visit(Scene* _scene)
// {
//     Visitor::Visit(_scene); // draw children first

//     if (_scene->pickedModel && drawOutline)
//         DrawOutline();
// }

void AnimationVisitor::Visit(Model* model)
{
    Eigen::MatrixX3f system = model->GetRotation();
    if (scene->animate)
        if(model->name == std::string("link"))
            model->TranslateInSystem(system, Eigen::Vector3f(-0.01f,0,0));
}


} // namespace cg3d
