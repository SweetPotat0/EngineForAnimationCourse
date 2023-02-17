#pragma once
#include <chrono>

class Ability
{
public:
    // Ability();
    Ability(int cooldown, int duration);
    std::chrono::seconds cooldown;
    std::chrono::seconds duration;
    std::chrono::steady_clock::time_point cooldownTimer;
    std::chrono::steady_clock::time_point durationTimer;
    bool inUse = false;
    bool canUse();
    void abilityUsed();
    bool didAbilityEnd();
    
    
};