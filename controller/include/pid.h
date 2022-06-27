//
// Created by sofia on 26.06.22.
//

#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

#include "perception.h"

class PID{
public:
    float output;       // the output value calculated by the PID Controller
    int value_left;     // the value for the left motor
    int value_right;    // the value for the right motor

    /**
     * The PID Constructor: takes factors and creates an PID-Controller-object which has to be updated with line-measurement
     */
    PID();

    /**
     * Initiate the PID-Controller / Set prev-values to 0
     */
    void init();

    /**
     * Updates the measurement and calculates new outputs
     * @param input the measurement input
     */
    void update(float input);

    /**
     * sends wheel values to nxt
     * @return
     */
    int correct_wheels();

private:
    float Kp;
    float Ki;
    float Kd;

    float T;
    float integrator;
    float differentiator;
    float prevError;
    float prevMeasurement;
    float measurement;
    float setpoint;

    float limMax;
    float limMin;
};

#endif //CONTROLLER_PID_H
