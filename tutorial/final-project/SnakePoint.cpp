#include "./SnakePoint.h"
#include "./Collidable.h"

SnakePoint::SnakePoint(std::shared_ptr<cg3d::Model> Model, int Score) : Collidable(Model), Score(Score)
{
}

void SnakePoint::moveToNewPosition(Eigen::Vector3f newPos, Eigen::Vector3f headPos){
    Model->Translate(-Model->GetTranslation());
    Model->Translate(newPos);
    Score = (newPos - headPos).norm();
}