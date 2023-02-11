#include "./Collidable.h"
#include "./OBB.h"

cg3d::MeshData GetLastMesh(std::shared_ptr<cg3d::Model> cyl)
{
    return cyl->GetMeshList()[0]->data.back();
}

Collidable::Collidable(std::shared_ptr<cg3d::Model> Model) : Model(Model), collisionTree(std::make_shared<igl::AABB<Eigen::MatrixXd, 3>>()),
                                                             oldT1(0, 0, 0), oldS1(1, 1, 1)
{
    static auto colliderProgram = std::make_shared<cg3d::Program>("shaders/basicShader");
    static auto colliderMaterial = std::make_shared<cg3d::Material>("collider material", colliderProgram);

    this->collider = cg3d::Model::Create(Model->name + " Collider", cg3d::Mesh::Cube(), colliderMaterial);

    this->collider->showFaces = false;
    this->collider->showWireframe = true;
    this->collider->isHidden = true;

    ReCalculateCollidingTree();
}

void Collidable::HideCollider()
{
    collider->isHidden = true;
}

void Collidable::ShowCollider()
{
    if (auto colliderParent = collider->parent.lock())
    {
    }
    else
    {
        if (auto modelParent = this->Model->parent.lock())
        {
            std::cout << "Success showing collider" << std::endl;
            modelParent->AddChild(this->collider);
        }
        else
        {
            std::cout << "Error showing collider" << std::endl;
            return;
        }
    }

    auto box = GetColliderOBB(OBB::TransformType::Local);

    collider->isHidden = false;
    collider->Translate(-oldT1);
    collider->Scale(Eigen::Vector3f(1 / oldS1(0), 1 / oldS1(1), 1 / oldS1(2)));

    collider->Translate(box->Pos);
    collider->Scale(box->Half_size * 2);

    oldT1 = box->Pos;
    oldS1 = box->Half_size * 2;

    free(box);
}

void Collidable::ReCalculateCollidingTree()
{
    std::cout << "Recalculate" << std::endl;
    auto vertices = GetLastMesh(this->Model).vertices;
    auto faces = GetLastMesh(this->Model).faces;
    this->collisionTree->init(vertices, faces);
}

void safeFree(void *p)
{
    if (p != NULL)
    {
        free(p);
    }
}

OBB *Collidable::GetColliderOBB(OBB::TransformType transform_type)
{
    return new OBB(this->collisionTree->m_box, this->Model, transform_type);
}

OBB **Collidable::getCollidingOBB(igl::AABB<Eigen::MatrixXd, 3> *tree1, igl::AABB<Eigen::MatrixXd, 3> *tree2,
                                  std::shared_ptr<cg3d::Model> model1, std::shared_ptr<cg3d::Model> model2)
{
    OBB *obb1 = new OBB(tree1->m_box, model1);
    OBB *obb2 = new OBB(tree2->m_box, model2);

    if (obb1->IsCollision(*obb2))
    {
        if (tree1->is_leaf() && tree2->is_leaf())
        {
            OBB **arr = (OBB **)malloc(sizeof(OBB *) * 2);
            arr[0] = obb1;
            arr[1] = obb2;
            return arr;
        }
        else if (tree1->is_leaf())
        {
            OBB **a1 = getCollidingOBB(tree1, tree2->m_left, model1, model2);
            OBB **a2 = getCollidingOBB(tree1, tree2->m_right, model1, model2);
            if (a1 != NULL)
            {
                safeFree(a2);
                return a1;
            }
            else
            {
                safeFree(a1);
                return a2;
            }
        }
        else if (tree2->is_leaf())
        {
            OBB **a1 = getCollidingOBB(tree1->m_left, tree2, model1, model2);
            OBB **a2 = getCollidingOBB(tree1->m_right, tree2, model1, model2);
            if (a1 != NULL)
            {
                safeFree(a2);
                return a1;
            }
            else
            {
                safeFree(a1);
                return a2;
            }
        }
        else
        {
            OBB **a1 = getCollidingOBB(tree1->m_left, tree2->m_left, model1, model2);
            OBB **a2 = getCollidingOBB(tree1->m_left, tree2->m_right, model1, model2);
            OBB **a3 = getCollidingOBB(tree1->m_right, tree2->m_left, model1, model2);
            OBB **a4 = getCollidingOBB(tree1->m_right, tree2->m_right, model1, model2);
            if (a1 != NULL)
            {
                safeFree(a2);
                safeFree(a3);
                safeFree(a4);
                return a1;
            }
            else if (a2 != NULL)
            {
                safeFree(a1);
                safeFree(a3);
                safeFree(a4);
                return a2;
            }
            else if (a3 != NULL)
            {
                safeFree(a1);
                safeFree(a2);
                safeFree(a4);
                return a3;
            }
            else
            {
                safeFree(a1);
                safeFree(a2);
                safeFree(a3);
                return a4;
            }
        }
    }
    else
    {
        return NULL;
    }
}

OBB **Collidable::getCollidingOBB(std::shared_ptr<Collidable> other)
{
    return getCollidingOBB(this->collisionTree.get(), other->collisionTree.get(), this->Model, other->Model);
}
