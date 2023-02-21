#include "./Enemy.h"
#include "./Collidable.h"
#include "./BasicScene.h"
Enemy::Enemy(std::shared_ptr<cg3d::Model> Model, float moveSpeed, float spinSpeed, Eigen::Vector3f startinPosition) 
: Collidable(Model), moveSpeed(moveSpeed),spinSpeed(spinSpeed), startinPosition(startinPosition)
{}
bool Enemy::ifReachedDest(){
    return (((Model->GetTranslation()) - destination).norm() <= moveSpeed);
}
void Enemy::moveTowardsDest(){
    Model->Translate((destination - (Model->GetTranslation())).normalized() * moveSpeed);
}