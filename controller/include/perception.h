#ifndef CONTROLLER_PERCEPTION_H
#define CONTROLLER_PERCEPTION_H

#include "opencv2/highgui.hpp"

#define HEIGHT 240
#define WIDTH 320

class Perception {
public:
    cv::VideoCapture cam;

    cv::Mat src,        // source in RGB
    dst,                // edges RGB
    frame,              // region of interest GRAY
    sva,                // sky view angle in RBG
    res;                // presentation in RGB

    int image_center;       // middle of lane
    int lane_size_low;      // size of lane at bottom of frame
    int lane_size_up;       // size of lane at top of frame

    Perception();

    int update_src();

    /**
     *
     * processes source picture
     *
     * @return 0 when success; -1 when fail
     */
    int process_image();

    /**
     *
     * detects the lanes and updates image_center
     *
     * @return 0 when success, -1 when no lanes
     */
    int lane_detect();

    /**
     *
     * tells us the offset by how much our car is off from lane center.
     * it's negative if too far left and positive if too far right.
     *
     * @return offset
     */
    int verify_distance() const;
};

int clean_lines(cv::Vec4i &l, int l_count);

#endif //CONTROLLER_PERCEPTION_H
