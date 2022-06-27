#include "pid.h"
#define SPEED 80

PID::PID() {
    // TODO: this should be adjusted to our needs
    Kp = 0.2;
    Kd = 0.05;
    Ki = 0.0085;
    T = 0.2;
    setpoint = 0.0f;
    limMin = -100.0f;
    limMax = 100.0f;
    this->init();
}

void PID::init() {
    integrator = 0.0f;
    prevError = 0.0f;
    prevMeasurement = 0.0f;
    differentiator = 0.0f;
    value_left = SPEED;
    value_right = SPEED;
    output = 0;
}

void PID::update(float input) {
    measurement = input;
    float error = setpoint - measurement;

    /* proportional term */
    float proportional = Kp * error;
    /* integrator term */
    if (abs(error) < 5 and abs(prevError) < 5) {
        integrator = 0;
    }
    integrator = integrator + 0.5f * Ki * T * (error + prevError);

    /* Integrator clamping*/
    float limMinInt, limMaxInt;

    if (limMax > proportional) {
        limMaxInt = limMax - proportional;
    } else {
        limMaxInt = 0.0f;
    }

    if (limMin < proportional) {
        limMinInt = limMin + proportional;
    } else {
        limMinInt = 0.0f;
    }
    /* clamp integrator */
    if (integrator > limMaxInt) {
        integrator = limMaxInt;
    } else if (integrator < limMinInt) {
        integrator = limMinInt;
    }

    /* Derivative*/
    differentiator = Kd * (error - prevError) / T;

    output = proportional + integrator + differentiator;
    if (output > limMax) {
        output = limMax;
    } else if (output < limMin) {
        output = limMin;
    }

    prevError = error;

    if (output < 0) {
        value_left = SPEED + (int) output;
        value_right = SPEED;
    } else {
        value_left = SPEED;
        value_right = SPEED - (int) output;
    }
}


int PID::correct_wheels() {
    return 0;
}


