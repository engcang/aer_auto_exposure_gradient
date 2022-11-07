#ifndef EXP_ROS_NODE_H_
#define EXP_ROS_NODE_H_

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <libgen.h>
#include <math.h>
#include <sys/time.h>
#include <string>
#include <sstream>
#include <cstdlib>
#include <iomanip>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Dense>


using namespace std;
using namespace std::chrono;


namespace exp_node {

  class ExpNode {
   public:
    ExpNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);


   private:

    void CameraCb(const sensor_msgs::CompressedImage::ConstPtr &msg);
    void GainCb(const std_msgs::Float32::ConstPtr &msg);
    void ExposureCb(const std_msgs::Float32::ConstPtr &msg);
    double image_gradient_gamma(cv::Mat &src_img, int j);
    
    double * curveFit (double x[7], double y[7]);
    double  findRoots1 (double a[6], double check);
    void generate_LUT ();
    bool check_rate = false;

    double gamma[7]={1.0/1.9, 1.0/1.5, 1.0/1.2, 1.0, 1.2, 1.5, 1.9};
    double metric[7];
    double max_metric;
    double max_gamma,alpha, expNew, expCur, shutter_cur, shutter_new, gain_cur, gain_new;
    double upper_shutter = 40000.0; // adjust if necessary [unit: micro-second]
    double lower_shutter = 18.0; // adjust if necessary [unit: micro-second]
    double gain_max = 30.0;
    double kp=0.4; // contorl the speed to convergence
    double f_number=1.8;
    double d = 0.1, R; // parameters used in the nonliear function in Shim's 2018 paper         
    int gamma_index; // index to record the location of the optimum gamma value
    bool gain_flag = false;
    std::string image_topic ="camera/image_raw";


    // Parameters that correlated to Shim's Gradient Metric
    double met_act_thresh = 0.06;
    double lamda = 1000.0; // The lamda value used in Shim's 2014 paper as a control parameter to adjust the mapping tendency (larger->steeper) 

    ros::NodeHandle nh_;
    ros::Subscriber img_sub_;
    ros::Subscriber gain_sub_;
    ros::Subscriber exp_sub_;
    ros::Publisher gain_pub_;
    ros::Publisher exp_pub_;

    double exp_cur_data_;
    double gain_cur_data_;

  };

}

#endif
