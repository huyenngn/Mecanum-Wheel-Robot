#include "perception.h"
#include "opencv2/imgproc.hpp"
#include <iostream>

#define ROI_TOP_LEFT 0.2
#define ROI_TOP_RIGHT 0.8
#define ROI_MID 0.7
#define ROI_HEIGHT 0.3
#define FRAME_OFFSET 50
#define LOW_SLOPE 200
#define CURVE_SLOPE 1000
#define PRECISION_FACTOR 1000
#define CAR_CENTER 160
#define CURVE_CORRECT 1.5

using namespace cv;
using namespace std;

Perception::Perception() {
    cam.open(0);
    if (!cam.isOpened()) {
        cerr << "Couldn't open camera! Exiting..." << endl;
    }
    cam.set(CAP_PROP_FRAME_WIDTH, WIDTH);
    cam.set(CAP_PROP_FRAME_HEIGHT, HEIGHT);
    image_center = CAR_CENTER;
    lane_size_up = 160;
    lane_size_low = 325;
}

int Perception::process_image() {
    // Edge detection
    cvtColor(src, dst, COLOR_RGB2GRAY);
    blur(dst, dst, Size(5, 5));
    Canny(dst, dst, 128, 255, 3);
    cvtColor(dst, dst, COLOR_GRAY2BGR);

    imshow("Edge detection", dst);

    // Define Region of Interest
    int npt = 6;
    Point corners[1][npt];
    corners[0][0] = Point(0, HEIGHT);
    corners[0][1] = Point(0, (int) (HEIGHT * ROI_MID));
    corners[0][2] = Point((int) (WIDTH * ROI_TOP_LEFT), (int) (HEIGHT * ROI_HEIGHT));
    corners[0][3] = Point((int) (WIDTH * ROI_TOP_RIGHT), (int) (HEIGHT * ROI_HEIGHT));
    corners[0][4] = Point(WIDTH, (int) (HEIGHT * ROI_MID));
    corners[0][5] = Point(WIDTH, HEIGHT);
    const Point *ppt[1] = {corners[0]};

    Mat mask(HEIGHT, WIDTH, CV_8UC3, Scalar(0, 0, 0));
    fillPoly(mask, ppt, &npt, 1, Scalar(255, 255, 255), LINE_8);
    addWeighted(src, 0.5, mask, 0.5, 0.0, res);

    bitwise_and(dst, mask, frame);
    cvtColor(frame, frame, COLOR_RGB2GRAY);

    imshow("Region of Interest 1", res);
    imshow("Region of Interest 2", frame);

    // Perspective warping
    Mat lambda = Mat::zeros(sva.rows, sva.cols, sva.type());

    Point2f inputQuad[4];
    inputQuad[0] = Point2f((float) (WIDTH * ROI_TOP_LEFT), (float) (HEIGHT * ROI_HEIGHT));
    inputQuad[1] = Point2f((float) (WIDTH * ROI_TOP_RIGHT), (float) (HEIGHT * ROI_HEIGHT));
    inputQuad[2] = Point2f(WIDTH, (float) (HEIGHT * ROI_MID));
    inputQuad[3] = Point2f(0, (float) (HEIGHT * ROI_MID));

    Point2f outputQuad[4];
    outputQuad[0] = Point2f(FRAME_OFFSET, 0);
    outputQuad[1] = Point2f((float) (WIDTH - 2 * FRAME_OFFSET), 0);
    outputQuad[2] = Point2f((float) (WIDTH - 2 * FRAME_OFFSET), HEIGHT);
    outputQuad[3] = Point2f(FRAME_OFFSET, HEIGHT);

    lambda = getPerspectiveTransform(inputQuad, outputQuad);
    warpPerspective(frame, sva, lambda, sva.size());

    imshow("Sky view angle", sva);

    return 0;
}

int Perception::lane_detect() {

    // Probabilistic Line Transform
    vector<Vec4i> lines; // will hold the results of the detection
    HoughLinesP(frame, lines, 1, CV_PI / 180, 50, 50, 20);

    if (lines.empty()) {
        fprintf(stderr, " u not even on the road no more foo\n");
        return -1;
    }

    // Averaging lines
    int left_count = 0;
    int right_count = 0;
    Vec4i left_lane = {0, 0, 0, 0};
    Vec4i right_lane = {0, 0, 0, 0};
    for (auto l: lines) {
        Point pt1 = Point(l[0], l[1]);
        Point pt2 = Point(l[2], l[3]);
        if (pt2.x == pt1.x) {
            break;
        }
        int m = (pt2.y - pt1.y) * PRECISION_FACTOR / (pt2.x - pt1.x);
        if (m > LOW_SLOPE) {
            right_lane = right_lane + l;
            right_count++;
        } else if (m < -LOW_SLOPE) {
            left_lane = left_lane + l;
            left_count++;
        }
    }

    // Deal with no or single lines
    if (left_count == 0 and right_count == 0) {
        image_center = CAR_CENTER;
        return -1;
    }
    else if (left_count == 0) {
        int m = clean_lines(right_lane, right_count);
        left_lane = {(right_lane[0] - lane_size_low), right_lane[1], (right_lane[2] - lane_size_up), right_lane[3]};
        if (m < CURVE_SLOPE) {
            printf("m left turn = %d", m);
            image_center = CAR_CENTER - (CURVE_SLOPE - m) / CURVE_CORRECT;
        }
        else {
            image_center = (right_lane[0] + left_lane[0]) / 2;
        }
    }
    else if (right_count == 0) {
        int m = clean_lines(left_lane, left_count);
        right_lane = {(left_lane[0] + lane_size_low), left_lane[1], (left_lane[2] + lane_size_up), left_lane[3]};
        if (m > -CURVE_SLOPE) {
            printf("m right turn = %d", m);
            image_center = CAR_CENTER + (CURVE_SLOPE + m) / CURVE_CORRECT;
        }
        else {
            image_center = (right_lane[0] + left_lane[0]) / 2;
        }
    }
    else {
        clean_lines(right_lane, right_count);
        clean_lines(left_lane, left_count);
        lane_size_low = right_lane[0] - left_lane[0];
        lane_size_up = right_lane[2] - left_lane[2];
        image_center = (right_lane[0] + left_lane[0]) / 2;
    }

    // Draw presentation
    vector<Vec4i> lane_lines = {left_lane, right_lane};
    int npt2 = 4;
    Point corners2[1][npt2];

    corners2[0][0] = Point(lane_lines[0][2], lane_lines[0][3]); // top left
    corners2[0][1] = Point(lane_lines[0][0], lane_lines[0][1]); // bot left
    corners2[0][2] = Point(lane_lines[1][0], lane_lines[1][1]); // bot right
    corners2[0][3] = Point(lane_lines[1][2], lane_lines[1][3]); // top right
    const Point *ppt2[1] = {corners2[0]};

    Mat mask2(HEIGHT, WIDTH, CV_8UC3, Scalar(0, 0, 0));
    fillPoly(mask2, ppt2, &npt2, 1, Scalar(0, 255, 0, 128), LINE_8);
    addWeighted(src, 0.5, mask2, 0.5, 0.0, res);

    imshow("Lane Detect", res);

    waitKey(0);
    return 0;
}

int clean_lines(Vec4i &l, int l_count) {
    for (int i = 0; i < 4; i++) {
        l[i] = l[i] / l_count;
    }
    if (l[2] == l[0]) {
        return 0;
    }
    if (l[3] == l[1]) {
        l[1] = l[1] + 1;
    }
    int m = (l[3] - l[1]) * PRECISION_FACTOR / (l[2] - l[0]);
    int n = l[1] * PRECISION_FACTOR - (m * l[0]);
    l[1] = HEIGHT;
    l[3] = HEIGHT * ROI_HEIGHT;
    l[0] = (l[1] * PRECISION_FACTOR - n) / m;
    l[2] = (l[3] * PRECISION_FACTOR - n) / m;
    return m;
}

int Perception::verify_distance() const {
    return (CAR_CENTER - image_center);
}

int Perception::update_src() {
    cam.read(src);

    const char *filename = "../tests/straight.jpg";
    src = imread(samples::findFile(filename));

    return 0;
}
