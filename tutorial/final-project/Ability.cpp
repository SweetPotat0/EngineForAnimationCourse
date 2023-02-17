#include "./Ability.h"

// Ability::Ability() : cooldown(0), duration(0) {}
Ability::Ability(int cooldown, int duration) : 
cooldown(std::chrono::seconds(cooldown)), duration(std::chrono::seconds(duration)){}
void Ability::abilityUsed(){
    cooldownTimer = std::chrono::steady_clock::now();
    durationTimer = std::chrono::steady_clock::now();
    inUse = true;
}
bool Ability::canUse(){
    return std::chrono::steady_clock::now() - cooldownTimer > cooldown;
}
bool Ability::didAbilityEnd(){
    bool ret = inUse && std::chrono::steady_clock::now() - durationTimer > duration;
    if (ret) inUse = false;
    return ret;
}
