#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
using namespace cv;
using namespace std;

const auto enemy_color = 'r';
const int start_num = 635;

class LightIndicator
{
    public:
        LightIndicator(const RotatedRect& light)
        {
            width = light.size.width;
            length = light.size.height;
            center = light.center;
            angle = light.angle;
            area = light.size.area();
        }
        const LightIndicator& operator=(const LightIndicator& l)
        {
            this->width = l.width;
            this->length = l.length;
            this->center = l.center;
            this->angle = l.angle;
            this->area = l.area;
            return *this;
        }
        double width, length, angle, area;
        Point2f center;

        static bool cmp(const LightIndicator& l1, const LightIndicator& l2)
        {
            return l1.center.x < l2.center.x;
        }

        RotatedRect rec() const
        {
            return RotatedRect(center, Size2f(width, length), angle);
        }
};

class Armor
{
    public:
        Armor(const LightIndicator& left, const LightIndicator& right)
        {
            center.x = (left.center.x + right.center.x) / 2;
            center.y = (left.center.y + right.center.y) / 2;
        }
        Point2f start, endl;
        Point2f center;
};

double distance(Point2f l, Point2f r)
{
    return sqrt(pow(l.x - r.x, 2) + pow(l.y - r.y, 2));
}

class Detector
{
    private:
        const int brightness_threshold = 120;
        const int light_min_area = 10;
        const double light_max_angle = 45.0;
        const double light_min_size = 5.0;
        const double light_contour_min_solidity = 0.5;
        const double light_max_ratio = 0.4;
        const double light_color_detect_extend_ratio = 1.1;
        const double light_max_angle_diff = 7.0;
        const double light_max_length_diff_ratio = 0.2;
        const double light_max_y_diff_ratio = 2.0;
        const double light_min_x_diff_ratio = 0.5;
        const double armor_min_aspect_ratio_ = 1.0;
        const double armor_max_aspect_ratio_ = 5.0;
    
        Mat separateColors(Mat img)
        {
            vector<Mat> channels;
            split(img, channels);

            Mat gray_img;

            if (enemy_color == 'r')
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

        RotatedRect& adjustRec(RotatedRect& rec, const char mode)
        {
            using std::swap;

            float& width = rec.size.width;
            float& height = rec.size.height;
            float& angle = rec.angle;

            if (mode == 'w')
            {
                if (width < height)
                {
                    swap(width, height);
                    angle += 90.0;
                }
            }

            while (angle >= 90.0) angle -= 180.0;
            while (angle < -90.0) angle += 180.0;

            if (mode == 'a')
            {
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
            }
        return rec;
        }

        void filterContours(vector<vector<Point>>& light_contours, vector<LightIndicator>& light_info)
        {
            for (const auto& contour:light_contours)
            {
                float light_contour_area = contourArea(contour);
                if (light_contour_area < light_min_area)
                    continue;
                RotatedRect light_rec = fitEllipse(contour);
                adjustRec(light_rec, 'a');
                if (light_rec.size.width / light_rec.size.height > light_max_ratio ||
                    light_contour_area / light_rec.size.area() < light_contour_min_solidity)
                        continue;
                light_rec.size.width *= light_color_detect_extend_ratio;
                light_rec.size.height *= light_color_detect_extend_ratio;
                
                light_info.push_back(LightIndicator(light_rec));
            }
        }

        vector<Armor> matchArmor(vector<LightIndicator>& light_info)
        {
            vector<Armor> armors;

            sort(light_info.begin(), light_info.end(), LightIndicator::cmp);
            for (size_t i = 0; i < light_info.size(); i++)
            {
                const LightIndicator& left = light_info[i];
                for (size_t j = i + 1; j < light_info.size(); j++)
                {
                    const LightIndicator& right = light_info[j];
                    
                    double angle_diff = abs(left.angle - right.angle);
                    double len_diff_ratio = abs(left.length - right.length) / max(left.length, right.length);
                    
                    if (angle_diff > light_max_angle_diff || len_diff_ratio > light_max_length_diff_ratio)
                        continue;

                    double dis = distance(left.center, right.center);
                    double mean_len = (left.length + right.length) / 2;
                    double x_diff_ratio = abs(left.center.x - right.center.x) / mean_len;
                    double y_diff_ratio = abs(left.center.y - right.center.y) / mean_len;
                    double dis_ratio = dis / mean_len;
                    if (y_diff_ratio > light_max_y_diff_ratio || 
                        x_diff_ratio < light_min_x_diff_ratio ||
                        dis_ratio > armor_max_aspect_ratio_ ||
                        dis_ratio < armor_min_aspect_ratio_)
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
            circle(img, armors[i].center, 1, Scalar(0, 255, 0), 20);
        }
        imshow("Armor", img);
        waitKey();
    }

    public:
        vector<Armor> analyze(Mat img)
        {
            Mat debug_img = img.clone();
            img = separateColors(img);
            img = binarization(img);
            vector<vector<Point>> contours = getContours(img);
            vector<LightIndicator> light_info;
            filterContours(contours, light_info);
            vector<Armor> armors = matchArmor(light_info);
            //cout << "Found " << armors.size() << " armor(s)" << endl;
            //showArmor(debug_img, armors);
            
            return armors;
        }
};

int main()
{
    //double start = clock();
    
    vector<Mat> imgs;
    Mat img;
    vector<Armor> result;
    Detector detector;
    string file_path;
    int num = start_num, sum = 0;

    for (int i = 1; i <= 1; i++)
    {
        file_path = "./src/659.png";
        //file_path = "~/Desktop/1641887836_s_png.rf.018b1272ad9fab7659e95e3dc059abae.jpg";
        //cout << file_path << endl;
        img = imread(file_path, IMREAD_COLOR);
        resize(img, img, Size(640, 480));
        imgs.push_back(img);
        num++;
    }
    auto start = chrono::high_resolution_clock::now();
    for (size_t i = 0; i < imgs.size(); i++)
    {
        result = detector.analyze(imgs[i]);
        sum += result.size();
        //cout << sum << endl;
    }
    //double end = clock();
    //cout << (end - start) / CLOCKS_PER_SEC << endl;
    auto end = chrono::high_resolution_clock::now();
    cout << "Armors found in 20 pictures: " << sum << endl;
    cout << "Time elapsed: " << chrono::duration_cast<chrono::nanoseconds>(end - start).count() / 1e9 << "s" << endl;

    return 0;
}