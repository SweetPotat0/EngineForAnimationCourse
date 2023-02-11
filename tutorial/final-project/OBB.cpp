#include "./OBB.h"

OBB::OBB(Eigen::AlignedBox3d box, std::shared_ptr<cg3d::Model> model) : OBB(box, model, TransformType::Global)
{
}

OBB::OBB(Eigen::AlignedBox3d box, std::shared_ptr<cg3d::Model> model, TransformType transform_type)
{
    Eigen::Matrix4f model_transform = transform_type == TransformType::Global ? model->GetAggregatedTransform() : model->GetTransform();
    Eigen::Vector3f c = box.center().cast<float>();
    Eigen::Vector4f tc = model_transform * Eigen::Vector4f{c[0], c[1], c[2], 1};
    this->Pos = {tc[0], tc[1], tc[2]};
    this->Half_size = box.sizes().cast<float>() / 2.0;
    auto b = model->GetScaling(model_transform);
    this->Half_size = b * this->Half_size;
    auto a = model->GetRotation(model_transform).rotation();
    this->AxisX = a.col(0);
    this->AxisY = a.col(1);
    this->AxisZ = a.col(2);
}

OBB::~OBB()
{
    int i = 0;
}

OBB::OBB(OBB &other)
{
    this->AxisX = other.AxisX;
    this->AxisY = other.AxisY;
    this->AxisZ = other.AxisZ;
    this->Half_size = other.Half_size;
    this->Pos = other.Pos;
}

bool getSeparatingPlane(const Eigen::Vector3f &RPos, const Eigen::Vector3f &Plane, const OBB &box1, const OBB &box2)
{
    return (fabs(RPos.dot(Plane)) >
            (fabs((box1.AxisX * box1.Half_size.x()).dot(Plane)) +
             fabs((box1.AxisY * box1.Half_size.y()).dot(Plane)) +
             fabs((box1.AxisZ * box1.Half_size.z()).dot(Plane)) +
             fabs((box2.AxisX * box2.Half_size.x()).dot(Plane)) +
             fabs((box2.AxisY * box2.Half_size.y()).dot(Plane)) +
             fabs((box2.AxisZ * box2.Half_size.z()).dot(Plane))));
}

bool OBB::IsCollision(const OBB &other)
{
    Eigen::Vector3f RPos;
    RPos = other.Pos - this->Pos;

    return !(getSeparatingPlane(RPos, this->AxisX, *this, other) ||
             getSeparatingPlane(RPos, this->AxisY, *this, other) ||
             getSeparatingPlane(RPos, this->AxisZ, *this, other) ||
             getSeparatingPlane(RPos, other.AxisX, *this, other) ||
             getSeparatingPlane(RPos, other.AxisY, *this, other) ||
             getSeparatingPlane(RPos, other.AxisZ, *this, other) ||
             getSeparatingPlane(RPos, this->AxisX.cross(other.AxisX), *this, other) ||
             getSeparatingPlane(RPos, this->AxisX.cross(other.AxisY), *this, other) ||
             getSeparatingPlane(RPos, this->AxisX.cross(other.AxisZ), *this, other) ||
             getSeparatingPlane(RPos, this->AxisY.cross(other.AxisX), *this, other) ||
             getSeparatingPlane(RPos, this->AxisY.cross(other.AxisY), *this, other) ||
             getSeparatingPlane(RPos, this->AxisY.cross(other.AxisZ), *this, other) ||
             getSeparatingPlane(RPos, this->AxisZ.cross(other.AxisX), *this, other) ||
             getSeparatingPlane(RPos, this->AxisZ.cross(other.AxisY), *this, other) ||
             getSeparatingPlane(RPos, this->AxisZ.cross(other.AxisZ), *this, other));
}