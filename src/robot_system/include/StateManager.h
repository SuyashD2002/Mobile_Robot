#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include "Robot.h"

class StateManager
{
private:
    double emergencyDistance;
    double lowBatteryThreshold;
    double obstacleStopDistance;
    double slowingDistance;

public:
    StateManager(double emergencyDist,
                 double lowBattery,
                 double stopDist,
                 double slowDist);

    void evaluate(Robot& robot,
                  double obstacleDistance,
                  double batteryLevel);
};

#endif