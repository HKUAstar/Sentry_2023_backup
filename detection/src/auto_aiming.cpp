#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
#include <msgs/Armors.h>
#include <msgs/ArmorItem.h>
using namespace std;

const int start_num = 37, picture_count = 10;
const int img_width = 640, img_height = 480;
bool test_speed = true, test_acc = false;

auto start = chrono::high_resolution_clock::now();

int cnt = 0;

ros::Publisher pub;

double distance(Point2f l, Point2f r)
{
    return sqrt(pow(l.x - r.x, 2) + pow(l.y - r.y, 2));
}

class Armor
{
    public:
        Armor(const RotatedRect& left, const RotatedRect& right)
        {
            RotatedRect new_left(left.center, Size(left.size.width, left.size.height * 2), left.angle);
            RotatedRect new_right(right.center, Size(right.size.width, right.size.height * 2), right.angle);
            Point2f points_l[4], points_r[4];
            new_left.points(points_l);
            new_right.points(points_r);
            upper_l = points_l[2], lower_l = points_l[3], upper_r = points_r[1], lower_r = points_r[0];
            center.x = (upper_l.x + lower_r.x) / 2;
            center.y = (upper_l.y + lower_r.y) / 2;
        }
        vector<Point2f> points()
        {
            vector<Point2f> vertices;
            vertices.push_back(lower_l);
            vertices.push_back(upper_l);
            vertices.push_back(upper_r);
            vertices.push_back(lower_r);
            return vertices;
        }
        vector<Point2i> coordinates()
        {
            vector<Point2f> vertices = this->points();
            vector<Point2i> coords;
            for (size_t i = 0; i < vertices.size(); i++)
                coords.push_back(Point2i(max((int)vertices[i].x, 0), max((int)vertices[i].y, 0)));
            return coords;
        }
        double area()
        {
            return sqrt(pow(distance(upper_l, lower_l), 2) + pow(distance(upper_l, upper_r), 2));
        }
        Point2f center, upper_l, lower_l, upper_r, lower_r;
};

class Detector
{
    private:
        /*const int brightness_threshold = 60;
        const int light_min_area = 10;
        const double light_max_angle = 45.0;
        const double light_min_size = 3.0;
        const double light_contour_min_solidity = 0.5;
        const double light_max_ratio = 0.4;
        const double light_color_detect_extend_ratio = 1.1;
        const double light_max_angle_diff = 7.0;
        const double light_max_length_diff_ratio = 0.2;
        const double light_max_y_diff_ratio = 2.0;
        const double light_min_x_diff_ratio = 0.5;
        const double armor_min_aspect_ratio = 1.0;
        const double armor_max_aspect_ratio = 5.0;*/
        const int brightness_threshold = 60;
        const int light_min_area = 10;
        const double light_max_angle = 45.0;
        const double light_min_size = 3.0;
        const double light_contour_min_solidity = 0.3;
        const double light_max_ratio = 0.6;
        const double light_color_detect_extend_ratio = 1.1;
        const double light_max_angle_diff = 7.0;
        const double light_max_length_diff_ratio = 0.3;
        const double light_max_y_diff_ratio = 2.0;
        const double light_min_x_diff_ratio = 0.5;
        const double armor_min_aspect_ratio = 1.0;
        const double armor_max_aspect_ratio = 5.0;
    
        Mat separateColors(Mat img, char color)
        {
            vector<Mat> channels;
            split(img, channels);

            Mat gray_img;

            if (color == 'r')
                gray_img = channels.at(2) - channels.at(0);
            else
                gray_img = channels.at(0) - channels.at(2);

            return gray_img;
        }

        Mat binarization(Mat img)
        {
            Mat bin_bright_img;

            threshold(img, bin_bright_img, brightness_threshold, 255, THRESH_BINARY);

            Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

            dilate(bin_bright_img, bin_bright_img, element);
            //imshow("Binarization", bin_bright_img);
            //waitKey();

            return bin_bright_img;
        }
        
        vector<vector<Point>> getContours(Mat img)
        {
            vector<vector<Point>> light_contours;
            findContours(img.clone(), light_contours, RetrievalModes::RETR_EXTERNAL, ContourApproximationModes::CHAIN_APPROX_SIMPLE);

            return light_contours;
        }

        RotatedRect& adjustRec(RotatedRect& rec)
        {
            using std::swap;

            float& width = rec.size.width;
            float& height = rec.size.height;
            float& angle = rec.angle;

            while (angle >= 90.0) angle -= 180.0;
            while (angle < -90.0) angle += 180.0;

            if (angle >= 45.0)
            {
                swap(width, height);
                angle -= 90.0;
            }
            else if (angle < -45.0)
            {
                    swap(width, height);
                    angle += 90.0;
            }

            return rec;
        }

        static bool compareLightIndicators(RotatedRect l1, RotatedRect l2)
        {
            return l1.center.x < l2.center.x;
        }

        vector<vector<Point>> filterContours(vector<vector<Point>>& light_contours, vector<RotatedRect>& light_info)
        {
            vector<vector<Point>> remaining_contours;
            for (const auto& contour:light_contours)
            {
                float light_contour_area = contourArea(contour);
                if (light_contour_area < light_min_area)
                    continue;
                RotatedRect light_rec = fitEllipse(contour);
                adjustRec(light_rec);
                if (light_rec.size.width / light_rec.size.height > light_max_ratio ||
                    light_contour_area / light_rec.size.area() < light_contour_min_solidity)
                        continue;
                light_rec.size.width *= light_color_detect_extend_ratio;
                light_rec.size.height *= light_color_detect_extend_ratio;
                
                light_info.push_back(RotatedRect(light_rec));
                remaining_contours.push_back(contour);
            }
            return remaining_contours;
        }

        vector<Armor> matchArmor(vector<RotatedRect>& light_info)
        {
            vector<Armor> armors;

            sort(light_info.begin(), light_info.end(), compareLightIndicators);
            for (size_t i = 0; i < light_info.size(); i++)
            {
                const RotatedRect& left = light_info[i];
                for (size_t j = i + 1; j < light_info.size(); j++)
                {
                    const RotatedRect& right = light_info[j];
                    
                    double angle_diff = abs(left.angle - right.angle);
                    double len_diff_ratio = abs(left.size.height - right.size.height) / max(left.size.height, right.size.height);
                    
                    if (angle_diff > light_max_angle_diff || len_diff_ratio > light_max_length_diff_ratio)
                        continue;

                    double dis = distance(left.center, right.center);
                    double mean_len = (left.size.height + right.size.height) / 2;
                    double x_diff_ratio = abs(left.center.x - right.center.x) / mean_len;
                    double y_diff_ratio = abs(left.center.y - right.center.y) / mean_len;
                    double dis_ratio = dis / mean_len;
                    if (y_diff_ratio > light_max_y_diff_ratio || 
                        x_diff_ratio < light_min_x_diff_ratio ||
                        dis_ratio > armor_max_aspect_ratio ||
                        dis_ratio < armor_min_aspect_ratio)
                        continue;
                    Armor armor(left, right);
                    armors.push_back(armor);
                }
            }
            return armors;
        }

    void showArmor(Mat img, vector<Armor> armors)
    {
        for (size_t i = 0; i < armors.size(); i++)
        {
            circle(img, armors[i].center, 1, Scalar(0, 255, 0), 10);
            vector<Point2i> armor_points = armors[i].coordinates();
            polylines(img, armor_points, true, Scalar(0, 255, 0), 1);
        }
        imshow("Armor", img);
        waitKey();
    }

    void drawAllContours(Mat img, vector<vector<Point>> light_contours)
    {
        for (size_t i = 0; i < light_contours.size(); i++)
            drawContours(img, light_contours, i, Scalar(0, 0, 255), 1);
    }

    public:
        vector<Armor> analyze(Mat img, char color)
        {
            Mat debug_img = img.clone();
            img = separateColors(img, color);
            img = binarization(img);
            Mat bin_img = img.clone();
            vector<vector<Point>> contours = getContours(img);
            //cout << contours.size() << endl;

            vector<RotatedRect> light_info;
            contours = filterContours(contours, light_info);

            drawAllContours(debug_img, contours);

            vector<Armor> armors = matchArmor(light_info);

            //cout << "Found " << armors.size() << " armor(s)" << endl;
            if (test_acc) showArmor(debug_img, armors);
            
            return armors;
        }
};

void callback(const sensor_msgs::ImageConstPtr& img, const msgs::Armors& armors) {
    
  char color = 'r';
  AutoAiming aimer;
  
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
  cv::Mat img = cv_ptr->image;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "armor_detector");
  ros::NodeHandle handler;

  message_filters::Subscriber<Image> img_sub(handler, "image", 1);
  message_filters::Subscriber<Armors> tar_sub(handler, "camera_info", 1);
  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), img_sub, tar_sub); // 1 is queue size
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  return 0;
}
