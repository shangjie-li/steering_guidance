#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>

#define PI 3.1415926

class SteeringGuidance
{
    private:
        std::string sub_topic_;
        std::string pub_topic_;
    
        double length_b_;
        double length_l_;
        double length_offset_;
        double height_of_camera_;
        
        double cam_fx_;
        double cam_fy_;
        double cam_u0_;
        double cam_v0_;
        
        int image_width_;
        int image_height_;
        
        int num_points_;
        double min_distance_;
        double max_distance_;
        
        double steering_angle_;
    
        ros::Subscriber sub_image_;
        ros::Publisher pub_image_;
    
        void draw_guidance_points(cv::Mat &img, std::vector<double> x_left, std::vector<double> y_left, std::vector<double> x_right, std::vector<double> y_right);
    
        void compute_guidance_points(double angle_, std::vector<double> &x_left, std::vector<double> &y_left, std::vector<double> &x_right, std::vector<double> &y_right);
    
        void call_back(const sensor_msgs::ImageConstPtr &msg);

    public:
        SteeringGuidance(ros::NodeHandle &nh);
        ~SteeringGuidance();
};

