#ifndef CONTROLLER_CAR_H
#define CONTROLLER_CAR_H

#include "perception.h"
#include "pid.h"

class Car {
public:
    Perception perception;  // Perception for image processing functions
    PID pid;                // PID controller for lane keeping functions

    Car();

    /**
     *
     * while there is no intersection in front it moves the car forward while staying in lane
     *
     * @return 0 when success, -1 when fail
     */
    int autopilot();

    void toggle_mode();

    /**
     *
     * stops the car immediately
     *
     * @return 0 when success, -1 when fail
     */
    int stop();

};

#endif //CONTROLLER_CAR_H
