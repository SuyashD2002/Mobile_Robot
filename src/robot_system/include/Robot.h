#ifndef ROBOT_H
#define ROBOT_H

#include <string>

enum class RobotState
{
    IDLE,
    MOVING,
    SLOWING,
    OBSTACLE_STOP,
    LOW_BATTERY,
    EMERGENCY,
    MISSION_COMPLETE
};

class Robot
{
private:
    std::string name;
    double velocity;
    RobotState state;
    double position;
    double totalDistance;
    double targetPosition;

public:
    Robot(const std::string& robotName);

    void setVelocity(double newVelocity);
    void setState(RobotState newState);
    bool transitionTo(RobotState newState);
    bool resetEmergency();
    double getVelocity() const;

    
    void updatePosition(double deltaTime);
    double getPosition() const;
    double getTotalDistance() const;

    void printStatus() const;
    std::string getStateName() const;

    void setTargetPosition(double target);
    bool hasReachedTarget() const;  

   
};

#endif