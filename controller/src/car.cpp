#include "car.h"

using namespace cv;

Car::Car() {
    pid.init();
}

int Car::autopilot() {

    perception.process_image();
    perception.lane_detect();
//    if (perception.lane_detect() < 0) {
//        stop();
//        return -1;
//    }
//    pid.update(perception.verify_distance());
//    pid.correct_wheels();

    return 0;
}

int Car::stop() {
    pid.value_right = 0;
    pid.value_left = 0;
    pid.correct_wheels();
    return 0;
}
