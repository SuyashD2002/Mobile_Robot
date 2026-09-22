#include "Robot.h"
#include <iostream>

Robot::Robot(const std::string& robotName)
    : name(robotName),
      velocity(0.0),
      position(0.0),
      totalDistance(0.0),
      targetPosition(0.0),
      state(RobotState::IDLE)
{
}

void Robot::setVelocity(double newVelocity)
{
    if (state == RobotState::EMERGENCY)
    {
        velocity = 0.0;
        return;
    }

    velocity = newVelocity;
}

void Robot::setState(RobotState newState)
{
    state = newState;
}
bool Robot::transitionTo(RobotState newState)
{
    // Emergency can always be entered
    if (newState == RobotState::EMERGENCY)
    {
        state = newState;
        velocity = 0.0;
        return true;
    }

    // If already in emergency, do not leave it
    if (state == RobotState::EMERGENCY)
    {
        velocity = 0.0;
        return false;
    }

    // Normal transition
    state = newState;
    return true;
}
bool Robot::resetEmergency()
{
    if (state == RobotState::EMERGENCY)
    {
        state = RobotState::IDLE;
        return true;
    }

    return false;
}



void Robot::printStatus() const
{
    std::cout << "Robot: " << name << std::endl;
    std::cout << "Velocity: " << velocity << " m/s" << std::endl;
    std::cout << "Position: " << position << " m" << std::endl;
    std::cout << "Total Distance: " << totalDistance << " m" << std::endl;
    std::cout << "State: " << getStateName() << std::endl;
    std::cout << "Target Position: "<< targetPosition<< " m" << std::endl;
}
std::string Robot::getStateName() const
{
    switch (state)
    {
        case RobotState::IDLE:
            return "IDLE";

        case RobotState::MOVING:
            return "MOVING";

        case RobotState::SLOWING:
            return "SLOWING";

        case RobotState::OBSTACLE_STOP:
            return "OBSTACLE_STOP";

        case RobotState::LOW_BATTERY:
            return "LOW_BATTERY";

        case RobotState::EMERGENCY:
            return "EMERGENCY";

        case RobotState::MISSION_COMPLETE:
            return "MISSION_COMPLETE";

        default:
            return "UNKNOWN";
    }
}
void Robot::updatePosition(double deltaTime)
{
    double distanceTravelled = velocity * deltaTime;

    position += distanceTravelled;
    totalDistance += distanceTravelled;
}

double Robot::getPosition() const
{
    return position;
}

double Robot::getTotalDistance() const
{
    return totalDistance;
}

double Robot::getVelocity() const
{
    return velocity;
}

void Robot::setTargetPosition(double target)
{
    targetPosition = target;
}

bool Robot::hasReachedTarget() const
{
    return position >= targetPosition;
}
