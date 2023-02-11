#include "./SnakePoint.h"
#include "./Collidable.h"

SnakePoint::SnakePoint(std::shared_ptr<cg3d::Model> Model, int Score) : Collidable(Model), Score(Score)
{
}