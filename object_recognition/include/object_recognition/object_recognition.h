#ifndef OBJECT_RECOGNITION_H
#define OBJECT_RECOGNITION_H


#include <math.h>

//ROS headers
#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <object_recognition/objects_poses.h>

#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>

// #include <compressed_image_transport/compressed_subscriber.h>
// #include "compressed_image_transport/compression_common.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <sensor_msgs/LaserScan.h>

// turtlesim specific msg
//#include "turtlesim/Pose.h"
// object_recognition  specific msg and srv
#include "object_recognition/SetTask.h"
#include "object_recognition/SetVelocities.h"

// Open CV
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// message_filters     subscription and synchronization
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//#include <image_transport/subscriber_filter.h>

// PCL  point cloud library  specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/impl/point_types.hpp>
//#include <pcl/console/print.h>
//#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
//#include <boost/thread/mutex.hpp>

////#define EXACT
//// exact synchronization policy is not working for the topics used, because the kinect is not publishing them with the same timestamp.
//#define APPROXIMATE

//#ifdef EXACT
//#include <message_filters/sync_policies/exact_time.h>
//#endif

//#ifdef APPROXIMATE
//#include <message_filters/sync_policies/approximate_time.h>
//#endif

// define new type
//typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
////typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;
////typedef message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_type;

//typedef image_transport::SubscriberFilter ImageSubscriber;
#include <tf/tf.h>


const static double LIDAR_THRESH = 0.8;
const static double LIDAR_THRESH_PUBLISH = 0.7;
const static double GREEN_POST_THRESH = 0.08;


class ObjectRecognition
{
public:
    ObjectRecognition();
    // initilize the node handle,  the initialization function, return a bool value
    bool init(ros::NodeHandle &nh);
    //periodicaly called inside of EXAMPLE_PACKAGE. All publisher are there
    void update(const ros::Time& time, const ros::Duration& period);

    bool adjust_blue_once;
    bool adjust_blue_once_check;
    cv::Mat adjust_blue_once_im;
    cv::Mat blue_mask_once, yellow_mask_once, green_mask_once;
    cv::Mat center_of_mass_image_blue_once, center_of_mass_image_yellow_once, center_of_mass_image_green_once;
    int h_blue_down = 110; int h_yellow_down = 19;
    int s_blue_down = 100; int s_yellow_down = 90;
    int v_blue_down = 50; int v_yellow_down = 100;
    int h_blue_up = 130; int h_yellow_up = 25;
    int s_blue_up = 255; int s_yellow_up = 255;
    int v_blue_up = 110; int v_yellow_up = 255;
    int h_green_down = 40; int h_green_up = 80;
    int s_green_down = 100; int s_green_up = 255;
    int v_green_down = 100; int v_green_up = 255;

    cv::Mat image_raw_depth;
    cv_bridge::CvImagePtr cv_ptr_depth;  // global pointer to cv_depth
    //cv::Mat *global_depth_ptr;

    // detect blob
    cv::Mat extractBlob(cv::Mat InImage, int color_index); // yellow 1, blue 2, green 3

    sensor_msgs::PointCloud2ConstPtr global_cloud_msg;

    sensor_msgs::LaserScan::ConstPtr global_scan_msg;

    const static double MIN_SCAN_ANGLE_RAD = -90.0/180 * M_PI;
    const static double MAX_SCAN_ANGLE_RAD = 90.0/180 * M_PI;




private:
    ros::NodeHandle nh_;

    ros::Publisher  pub_object_pose;


    // subscriber to the raw camera images
    //compressed_image_transport:: CompressedSubscriber sub_image_raw_compressed;

    ros::Subscriber info_depth_sub;
    image_transport::Subscriber sub_image_raw_rgb;
    image_transport::Subscriber sub_image_raw_depth;

    ros::Subscriber sub_points;

    ros::Subscriber sub_scan;

    std::vector<float> scan_ranges;

    ros::Time initial_time, safety_ton_scan, safety_ton_rgb, safety_ton_points;

    float scan_angle_min;
    float scan_angle_max;
    float scan_angle_increment;

//    message_filters::Subscriber<sensor_msgs::Image> *image_color_sub;
//    message_filters::Subscriber<sensor_msgs::Image> *image_depth_sub;
//    message_filters::Subscriber<sensor_msgs::CameraInfo> *info_depth_sub;


//    ImageSubscriber *image_color_sub;
//    ImageSubscriber *image_depth_sub;

//    message_filters::Synchronizer<MySyncPolicy> *sync;

    image_transport::ImageTransport image_tran_;

    geometry_msgs::Twist cmd_msg_;


    bool flag_image_ok_rgb;                                // an rgb image is received
    bool flag_image_ok_depth;                       // a depth image is received
    bool flag_image_ok_point;
    bool flag_scan_ok;
    bool rgb_update_ok;
    bool point_update_ok;
    bool scan_update_ok;
    bool all_update_ok;

    bool camerainfo_check;
    float fx, fy, cx, cy;


    bool is_bigendian_depth;


    tf::Transform Tcl_;
    tf::Transform Tlc_;

    tf::Transform Tlo_lidar;
    tf::Transform Tco_lidar;
    tf::Transform Tlo_PointCloud;
    tf::Transform Tco_PointCloud;




    double theta_camera_yellow, theta_camera_blue;
    geometry_msgs::Point p_mc_yellow, p_mc_blue;
    std::vector<double> theta_camera_green;
    std::vector<cv::Point2d>  lo_PointCloud_green;
    std::vector<cv::Point2d>  lo_PointCloud_green_not_NaN;
    bool puck_detection_yellow, puck_detection_blue;


    double x_lo_PointCloud_yellow;
    double y_lo_PointCloud_yellow;
    double x_lo_PointCloud_blue;
    double y_lo_PointCloud_blue;
    
    
    
    sensor_msgs::LaserScan filtered_scan;



    ////////////////////////////////// object detection

    cv::Mat hueHistogram;

    // variables for optical flow
    bool optFlowInitialized=false;
    //vector<cv::Point2f> points[2];
    cv::Mat preImgOpt;
    cv::Mat gray; //reference frame
    cv::vector<cv::Point2f> ref_points;	//feature points of reference frame
    cv::Mat sub_gray;// subsequent frames
    cv::vector<cv::Point2f> curr_points;	//feature points after calOpticalFlow
    cv::Mat lineMask;





    // mat containers for the color obstacle detection
    cv::Mat image_raw_rgb,  hsv_image_, filtered_image_, red_mask, green_mask, yellow_mask,yellow_mask_half, blue_mask;
    cv::Mat center_of_mass_image_red, center_of_mass_image_green, center_of_mass_image_yellow, center_of_mass_image_blue;
    cv::Mat center_of_mass_image_green_filtered, center_of_mass_image_yellow_filtered, center_of_mass_image_blue_filtered;

    // callback function for the synchronization (not used in this code )
    void callbackMethod(const sensor_msgs::ImageConstPtr& image_color_msg, const sensor_msgs::ImageConstPtr& image_depth_msg);


    // Callback function to image raw topic
    void getImage_rgb(const sensor_msgs::ImageConstPtr& msg_rgb);
    void getImage_depth(const sensor_msgs::ImageConstPtr& msg_depth);

    //void getCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg_camerainfo);
    void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg_camerainfo);

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    geometry_msgs::Point pixelTo3DPoint(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, int row_num, int col_num);


    std::vector<double> theta_camera_object(std::vector<float> scan_ranges);


    // determine the largest blob
    void findBiggestBlob();


    cv::Point2i computeMassCenter( cv::Point2f mc_i, cv::Mat depth_map);



    object_recognition::objects_poses object_pose;






};

#endif // OBJECT_RECOGNITION_H
