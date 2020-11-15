#include "steering_guidance_core.h"

SteeringGuidance::SteeringGuidance(ros::NodeHandle &nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/image_in");
    nh.param<std::string>("pub_topic", pub_topic_, "/image_out");
    
    nh.param<double>("length_b", length_b_, 1.6);
    nh.param<double>("length_l", length_l_, 2.3);
    nh.param<double>("length_offset", length_offset_, 1.0);
    nh.param<double>("height_of_camera", height_of_camera_, 1.8);
    
    nh.param<double>("cam_fx", cam_fx_, 581.0);
    nh.param<double>("cam_fy", cam_fy_, 604.0);
    nh.param<double>("cam_u0", cam_u0_, 605.0);
    nh.param<double>("cam_v0", cam_v0_, 332.0);
    
    nh.param<int>("image_width", image_width_, 1280);
    nh.param<int>("image_height", image_height_, 720);
    
    nh.param<int>("num_points", num_points_, 100);
    nh.param<double>("min_distance", min_distance_, 5);
    nh.param<double>("max_distance", max_distance_, 50);
    
    sub_image_ = nh.subscribe(sub_topic_, 1, &SteeringGuidance::call_back, this);
    pub_image_ = nh.advertise<sensor_msgs::Image>(pub_topic_, 1);
    
    ros::spin();
    
}

SteeringGuidance::~SteeringGuidance(){}

void SteeringGuidance::draw_guidance_points(cv::Mat &img, std::vector<double> x_left, std::vector<double> y_left, std::vector<double> x_right, std::vector<double> y_right)
{
    int r = 0;
    int g = 255;
    int b = 0;
    
    int radius = 4;
    int thickness = -1;
    int lineType = 8;
    
    int uu, vv;
    
    for (int idx = 0; idx < num_points_; idx ++)
    {
        if (x_left[idx] == 0 && y_left[idx] == 0)
        {
            continue;
        }
        
        uu = cam_fx_ * x_left[idx] / y_left[idx] + cam_u0_;
        vv = cam_fy_ * height_of_camera_ / y_left[idx] + cam_v0_;
        
        if (uu >= 0 && uu < image_width_ && vv >= 0 && vv < image_height_)
        {
            cv::circle(img, cv::Point(uu, vv), radius, cv::Scalar(r, g, b), thickness, lineType);
        }
    }
    
    for (int idx = 0; idx < num_points_; idx ++)
    {
        if (x_right[idx] == 0 && y_right[idx] == 0)
        {
            continue;
        }
        
        uu = cam_fx_ * x_right[idx] / y_right[idx] + cam_u0_;
        vv = cam_fy_ * height_of_camera_ / y_right[idx] + cam_v0_;
        
        if (uu >= 0 && uu < image_width_ && vv >= 0 && vv < image_height_)
        {
            cv::circle(img, cv::Point(uu, vv), radius, cv::Scalar(r, g, b), thickness, lineType);
        }
    }
}

void SteeringGuidance::compute_guidance_points(double angle_, std::vector<double> &x_left, std::vector<double> &y_left, std::vector<double> &x_right, std::vector<double> &y_right)
{
    double theta;
    double theta_min;
    double theta_max;
    double theta_interval;
    
    double st_radius;
    double st_x0;
    double st_y0;
    
    if (fabs(angle_) < 1)
    {
        for (int idx = 0; idx < num_points_; idx ++)
        {
            x_left[idx] = - length_b_ / 2;
            y_left[idx] = (max_distance_ - min_distance_) / num_points_ * idx;
            
            x_right[idx] = length_b_ / 2;
            y_right[idx] = (max_distance_ - min_distance_) / num_points_ * idx;
        }
    }
    
    else if (angle_ < 0)
    {
        theta = fabs(angle_ * PI / 180);
        theta_min = 0 * PI / 180;
        theta_max = 90 * PI / 180;
        theta_interval = (theta_max - theta_min) / num_points_;
        
        st_x0 = - length_l_ / tan(theta) - length_b_ / 2;
        st_y0 = length_l_ - length_offset_;
        
        st_radius = length_l_ / tan(theta);
        for (int i = 0; i < num_points_; i ++)
        {
            double theta_i = i * theta_interval + theta_min;
            x_left[i] = st_x0 + st_radius * cos(theta_i);
            y_left[i] = st_y0 + st_radius * sin(theta_i);
        }
        
        st_radius = length_l_ / tan(theta) + length_b_;
        for (int i = 0; i < num_points_; i ++)
        {
            double theta_i = i * theta_interval + theta_min;
            x_right[i] = st_x0 + st_radius * cos(theta_i);
            y_right[i] = st_y0 + st_radius * sin(theta_i);
        }
    }
    
    else if (angle_ > 0)
    {
        theta = fabs(angle_ * PI / 180);
        theta_min = 90 * PI / 180;
        theta_max = 180 * PI / 180;
        theta_interval = (theta_max - theta_min) / num_points_;
        
        st_x0 = length_l_ / tan(theta) + length_b_ / 2;
        st_y0 = length_l_ - length_offset_;
        
        st_radius = length_l_ / tan(theta);
        for (int i = 0; i < num_points_; i ++)
        {
            double theta_i = i * theta_interval + theta_min;
            x_left[i] = st_x0 + st_radius * cos(theta_i);
            y_left[i] = st_y0 + st_radius * sin(theta_i);
        }
        
        st_radius = length_l_ / tan(theta) + length_b_;
        for (int i = 0; i < num_points_; i ++)
        {
            double theta_i = i * theta_interval + theta_min;
            x_right[i] = st_x0 + st_radius * cos(theta_i);
            y_right[i] = st_y0 + st_radius * sin(theta_i);
        }
    }

    for (int idx = 0; idx < num_points_; idx ++)
    {
        if (y_left[idx] < min_distance_ || y_left[idx] > max_distance_)
        {
            x_left[idx] = 0;
            y_left[idx] = 0;
        }
        if (y_right[idx] < min_distance_ || y_right[idx] > max_distance_)
        {
            x_right[idx] = 0;
            y_right[idx] = 0;
        }
    }
}

void SteeringGuidance::call_back(const sensor_msgs::ImageConstPtr &msg_in)
{
    ros::param::get("~steering_angle", steering_angle_);
    
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_in, sensor_msgs::image_encodings::TYPE_8UC3);
    cv::Mat img = cv_ptr->image;
    
    std::vector<double> pts_x_left(num_points_);
    std::vector<double> pts_y_left(num_points_);
    std::vector<double> pts_x_right(num_points_);
    std::vector<double> pts_y_right(num_points_);
    
    compute_guidance_points(steering_angle_, pts_x_left, pts_y_left, pts_x_right, pts_y_right);
    
    draw_guidance_points(img, pts_x_left, pts_y_left, pts_x_right, pts_y_right);
    
    sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img).toImageMsg();
    pub_image_.publish(msg_out);

}


