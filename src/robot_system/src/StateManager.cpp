#include "StateManager.h"

StateManager::StateManager(double emergencyDist,
                           double lowBattery,
                           double stopDist,
                           double slowDist)
    : emergencyDistance(emergencyDist),
      lowBatteryThreshold(lowBattery),
      obstacleStopDistance(stopDist),
      slowingDistance(slowDist)
{
}
void StateManager::evaluate(Robot& robot,
                            double obstacleDistance,
                            double batteryLevel)
{
    if (obstacleDistance <= emergencyDistance)
    {
        robot.setVelocity(0.0);
        robot.transitionTo(RobotState::EMERGENCY);
    }
    else if (batteryLevel <= lowBatteryThreshold)
    {
        robot.setVelocity(0.0);
        robot.transitionTo(RobotState::LOW_BATTERY);
    }
    else if (obstacleDistance <= obstacleStopDistance)
    {
        robot.setVelocity(0.0);
        robot.transitionTo(RobotState::OBSTACLE_STOP);
    }
    else if (robot.hasReachedTarget())
    {
        robot.setVelocity(0.0);
        robot.transitionTo(RobotState::MISSION_COMPLETE);
    }
    else if (obstacleDistance <= slowingDistance)
    {
        robot.setVelocity(0.5);
        robot.transitionTo(RobotState::SLOWING);
    }
    else
    {
        robot.setVelocity(1.0);
        robot.transitionTo(RobotState::MOVING);
    }
}