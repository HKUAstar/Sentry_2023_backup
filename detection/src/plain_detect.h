#ifndef PLAIN_DETECT_H
#define PLAIN_DETECT_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
using namespace cv;
using namespace std;

double point_distance(Point2f l, Point2f r);

class Armor
{
public:
    Armor(const RotatedRect& left, const RotatedRect& right);
    Armor();
    vector<Point2f> points();
    vector<Point2i> coordinates();
    double area();
    Point2f center, upper_l, lower_l, upper_r, lower_r;
    bool empty();
};

class Detector
{
private:
    static const int brightness_threshold = 30; // 60
    static const int light_min_area = 10;
    static constexpr double light_max_angle = 45.0;
    static constexpr double light_min_size = 3.0;
    static constexpr double light_contour_min_solidity = 0.3; // 0.5
    static constexpr double light_max_ratio = 0.6; // 0.4
    static constexpr double light_color_detect_extend_ratio = 1.1;
    static constexpr double light_max_angle_diff = 7.0;
    static constexpr double light_max_length_diff_ratio = 0.3; // 0.2
    static constexpr double light_max_y_diff_ratio = 2.0;
    static constexpr double light_min_x_diff_ratio = 0.5;
    static constexpr double armor_min_aspect_ratio = 1.0;
    static constexpr double armor_max_aspect_ratio = 5.0;

    static Mat separateColors(Mat img, char color);
    static Mat binarization(Mat img);
    static vector<vector<Point>> getContours(Mat img);
    static RotatedRect& adjustRec(RotatedRect& rec);
    static bool compareLightIndicators(RotatedRect l1, RotatedRect l2);
    static vector<vector<Point>> filterContours(vector<vector<Point>>& light_contours, vector<RotatedRect>& light_info);
    static vector<Armor> matchArmor(vector<RotatedRect>& light_info);
    static void showArmor(Mat img, vector<Armor> armors);
    static void drawAllContours(Mat img, vector<vector<Point>> light_contours);

public:
    static vector<Armor> analyze(Mat img, char color);
};

#endif