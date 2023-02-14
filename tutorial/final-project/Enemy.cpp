#include "./Enemy.h"
#include "./Collidable.h"
#include "./BasicScene.h"
Enemy::Enemy(std::shared_ptr<cg3d::Model> Model, Eigen::Vector3f toOrigin, float moveSpeed, Eigen::Vector3f startinPosition) 
: Collidable(Model), toOrigin(toOrigin), moveSpeed(moveSpeed), startinPosition(startinPosition)
{
    Model->Translate(toOrigin);
}
bool Enemy::ifReachedDest(){
    return (((Model->GetTranslation() - toOrigin) - destination).norm() <= moveSpeed);
}
void Enemy::moveTowardsDest(){
    Model->Translate((destination - (Model->GetTranslation() - toOrigin)).normalized() * moveSpeed);
}