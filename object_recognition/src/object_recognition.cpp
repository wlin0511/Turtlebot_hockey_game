#include "object_recognition/object_recognition.h"

//image_transport should always be used to subscribe to and publish images.
//It provides transparent support for transporting images in low-bandwidth compressed formats.
ObjectRecognition::ObjectRecognition() : image_tran_(nh_)     // image_transport::ImageTransport image_tran_(nh_)
{}
// the initialization function ( which is called in the main function )
bool ObjectRecognition::init(ros::NodeHandle &nh)
{
    // Ros node handle for the class
    nh_ = nh;

    flag_image_ok_rgb =false;
    flag_image_ok_depth =false;
    flag_image_ok_point =false;

    flag_scan_ok = false;

    adjust_blue_once = false;
    adjust_blue_once_check = true;
    camerainfo_check = false;

    sub_image_raw_rgb = image_tran_.subscribe("/camera/rgb/image_raw", 10, &ObjectRecognition::getImage_rgb, this);
    //sub_image_raw_depth = image_tran_.subscribe("/camera/depth_registered/image_raw", 10, &ObjectRecognition::getImage_depth, this);
    //info_depth_sub = nh_.subscribe("/camera/depth_registered/camera_info", 10, &ObjectRecognition::CameraInfoCallback, this);

    sub_points = nh_.subscribe("/camera/depth/points", 1, &ObjectRecognition::cloudCallback, this);

    sub_scan = nh_.subscribe("/lidar_scan", 10, &ObjectRecognition::scanCallback,this);

    pub_object_pose = nh_.advertise<object_recognition::objects_poses>("/object_recognition/objects_poses", 10);


    //     image_color_sub = new ImageSubscriber(image_tran_, "/camera/rgb/image_raw", 10);
    //     image_depth_sub = new ImageSubscriber(image_tran_, "/camera/depth_registered/image_raw", 10);

    //     sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_color_sub, *image_depth_sub);
    //     sync->registerCallback(boost::bind(&ObjectRecognition::callbackMethod, this, _1, _2));



    Tcl_.setIdentity();
    Tcl_.setOrigin(tf::Vector3(-0.123,-0.0205, -0.170));
    Tlc_ = Tcl_.inverse();

    Tlo_lidar.setIdentity();
    Tco_PointCloud.setIdentity();


//    cv::namedWindow("Center of blob blue adjust");
//    cv::createTrackbar("h_blue_down","Center of blob blue adjust",&h_blue_down,255);
//    cv::createTrackbar("s_blue_down","Center of blob blue adjust",&s_blue_down,255);
//    cv::createTrackbar("v_blue_down","Center of blob blue adjust",&v_blue_down,255);
//    cv::createTrackbar("h_blue_up","Center of blob blue adjust",&h_blue_up,255);
//    cv::createTrackbar("s_blue_up","Center of blob blue adjust",&s_blue_up,255);
//    cv::createTrackbar("v_blue_up","Center of blob blue adjust",&v_blue_up,255);

//    cv::namedWindow("Center of blob yellow adjust");
//    cv::createTrackbar("h_yellow_down","Center of blob yellow adjust",&h_yellow_down,255);
//    cv::createTrackbar("s_yellow_down","Center of blob yellow adjust",&s_yellow_down,255);
//    cv::createTrackbar("v_yellow_down","Center of blob yellow adjust",&v_yellow_down,255);
//    cv::createTrackbar("h_yellow_up","Center of blob yellow adjust",&h_yellow_up,255);
//    cv::createTrackbar("s_yellow_up","Center of blob yellow adjust",&s_yellow_up,255);
//    cv::createTrackbar("v_yellow_up","Center of blob yellow adjust",&v_yellow_up,255);

//    cv::namedWindow("Center of blob green adjust");
//    cv::createTrackbar("h_green_down","Center of blob green adjust",&h_green_down,255);
//    cv::createTrackbar("s_green_down","Center of blob green adjust",&s_green_down,255);
//    cv::createTrackbar("v_green_down","Center of blob green adjust",&v_green_down,255);
//    cv::createTrackbar("h_green_up","Center of blob green adjust",&h_green_up,255);
//    cv::createTrackbar("s_green_up","Center of blob green adjust",&s_green_up,255);
//    cv::createTrackbar("v_green_up","Center of blob green adjust",&v_green_up,255);




    ROS_INFO ("ObjectRecognition is initialized");
    initial_time = ros::Time::now();
    safety_ton_scan = initial_time;
    safety_ton_rgb = initial_time;
    safety_ton_points = initial_time;

    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////  the update function is in      while (ros::ok())
void ObjectRecognition::update(const ros::Time& time, const ros::Duration& period){
    point_update_ok = ((ros::Time::now()- safety_ton_points).toSec()< 2.0)? true : false;
    if (!point_update_ok){
        ROS_WARN("PointCloud is not updated");
    }
    rgb_update_ok = ((ros::Time::now()- safety_ton_rgb).toSec()< 2.0)? true : false;
    if (!rgb_update_ok){
        ROS_WARN("RGB image is not updated");
    }
    scan_update_ok = ((ros::Time::now()- safety_ton_scan).toSec()< 2.0)? true : false;
    if (!scan_update_ok){
        ROS_WARN("LIDAR is not updated");
    }

    all_update_ok = (point_update_ok && rgb_update_ok && scan_update_ok );



    // if (flag_image_ok_depth && flag_image_ok_rgb && flag_image_ok_point) {
    if (flag_image_ok_rgb && flag_image_ok_point && flag_scan_ok && all_update_ok ) {
        //cv::namedWindow("Depth map");
        //cv::imshow("Depth map", image_raw_depth);

//        cv::namedWindow("Color image");
//        cv::imshow("Color image", image_raw_rgb);

        // in case of 8-bit and 16-bit images, R, G and B are converted to the floating point format and scaled to fit the 0 to 1 range
        // V = max{R,G,B}   H =....
        //   from BGR  to  HSV
        //        cv::Mat grayImage;
        //        cv::cvtColor(image_raw_rgb, grayImage, CV_BGR2GRAY);

        // turn the BGR image into HSV image
        cv::cvtColor(image_raw_rgb, hsv_image_, CV_BGR2HSV);

        //cv::GaussianBlur(image_raw_rgb, filtered_image_, cv::Size(9, 9), 1, 1);

        // Gaussian kernel size 9*9, gaussian kernel standard deviation in x direction/ and in Y direction
        cv::GaussianBlur(hsv_image_, filtered_image_, cv::Size(9, 9), 1, 1);

        // green color for the blobs
        cv::inRange(filtered_image_, cv::Scalar(40, 100, 100), cv::Scalar(80, 255, 255), green_mask);

        // yellow color for the blobs
        cv::inRange(filtered_image_, cv::Scalar(19, 90, 100), cv::Scalar(25, 255, 255), yellow_mask);
        yellow_mask_half = (yellow_mask.rowRange(0,120)).clone();

        // blue color for the blobs
        cv::inRange(filtered_image_, cv::Scalar(110, 100, 49), cv::Scalar(125, 255, 85), blue_mask);



        // /////////////////// extractBlob ////////////////////////////////
        center_of_mass_image_green = extractBlob(green_mask, 3);  // green
        center_of_mass_image_yellow = extractBlob(yellow_mask_half, 1);  // yellow
        center_of_mass_image_blue = extractBlob(blue_mask, 2);   // blue

        int num_yellow_lidar = 0;
        int num_blue_lidar = 0;

///////////////////////////////////////////////////////// Check Data from Lidar

        object_pose.green_post_poses.clear();
        object_pose.yellow_puck_poses.clear();
        object_pose.blue_puck_poses.clear();

        geometry_msgs::Pose2D yellow_puck_pose_temp;
        geometry_msgs::Pose2D blue_puck_pose_temp;
        geometry_msgs::Pose2D green_post_pose_temp;
        cv::Point2d lo_PointCloud_green_not_NaN_temp;



        std::vector<cv::Point2d> coord_yellow_lidar_vec;
        std::vector<cv::Point2d> coord_blue_lidar_vec;
        cv::Point2d coord_yellow_lidar(100.0,100.0);

        cv::Point2d coord_blue_lidar(100.0,100.0);

        cv::Point2d point2d_temp;




        for (int i_left = 31; i_left>= 0; i_left--) {
            if (scan_ranges.at(i_left) <= LIDAR_THRESH ){
                double theta_lidar_left = i_left * scan_angle_increment;
                double y_lidar_left = scan_ranges.at(i_left) * sin (theta_lidar_left);
                double x_lidar_left = scan_ranges.at(i_left) * cos (theta_lidar_left);
                Tlo_lidar.setOrigin(tf::Vector3(x_lidar_left,y_lidar_left, 0));
                // the multiplication of Tcl and Tlo
                Tco_lidar=Tcl_* Tlo_lidar;

                double x_co_lidar = Tco_lidar.getOrigin().x();
                double y_co_lidar = Tco_lidar.getOrigin().y();
                double theta_co = atan2(y_co_lidar,x_co_lidar);
                if (puck_detection_yellow) {
                    if (std::abs(theta_camera_yellow-theta_co)<0.05){

                        point2d_temp.x = x_lidar_left;
                        point2d_temp.y = y_lidar_left;
                        coord_yellow_lidar_vec.push_back(point2d_temp);
                    }
                }

            }
        }

        for (int i_right = 359; i_right>=329; i_right--) {
            if (scan_ranges.at(i_right)<=LIDAR_THRESH ){

                // if (scan_ranges.at(i_right) < 2.0) {
                double theta_lidar_right = (360 - i_right)* scan_angle_increment;
                double y_lidar_right = -scan_ranges.at(i_right) * sin (theta_lidar_right);
                double x_lidar_right= scan_ranges.at(i_right) * cos (theta_lidar_right);
                Tlo_lidar.setOrigin(tf::Vector3(x_lidar_right,y_lidar_right, 0));
                // the multiplication of Tcl and Tlo
                Tco_lidar=Tcl_* Tlo_lidar;
                double x_co_lidar = Tco_lidar.getOrigin().x();
                double y_co_lidar = Tco_lidar.getOrigin().y();
                double theta_co = atan2(y_co_lidar,x_co_lidar);
                if (puck_detection_yellow) {
                    if (std::abs(theta_camera_yellow-theta_co)<0.05){
                        point2d_temp.x = x_lidar_right;
                        point2d_temp.y = y_lidar_right;
                        coord_yellow_lidar_vec.push_back(point2d_temp);
                    }
                }
            }
        }


        for (int i_left = 31; i_left>= 0; i_left--) {
            if (scan_ranges.at(i_left) <=LIDAR_THRESH ){
                double theta_lidar_left = i_left * scan_angle_increment;
                double y_lidar_left = scan_ranges.at(i_left) * sin (theta_lidar_left);
                double x_lidar_left = scan_ranges.at(i_left) * cos (theta_lidar_left);
                Tlo_lidar.setOrigin(tf::Vector3(x_lidar_left,y_lidar_left, 0));
                // the multiplication of Tcl and Tlo
                Tco_lidar=Tcl_* Tlo_lidar;

                double x_co_lidar = Tco_lidar.getOrigin().x();
                double y_co_lidar = Tco_lidar.getOrigin().y();
                double theta_co = atan2(y_co_lidar,x_co_lidar);
                if (puck_detection_blue) {
                    if (std::abs(theta_camera_blue-theta_co)<0.05){
                        point2d_temp.x = x_lidar_left;
                        point2d_temp.y = y_lidar_left;
                        coord_blue_lidar_vec.push_back(point2d_temp);
                    }
                }


            }
        }


        for (int i_right = 359; i_right>=329; i_right--) {
            if (scan_ranges.at(i_right)<=LIDAR_THRESH ){

                // if (scan_ranges.at(i_right) < 2.0) {
                double theta_lidar_right = (360 - i_right)* scan_angle_increment;
                double y_lidar_right = -scan_ranges.at(i_right) * sin (theta_lidar_right);
                double x_lidar_right= scan_ranges.at(i_right) * cos (theta_lidar_right);
                Tlo_lidar.setOrigin(tf::Vector3(x_lidar_right,y_lidar_right, 0));
                // the multiplication of Tcl and Tlo
                Tco_lidar=Tcl_* Tlo_lidar;
                double x_co_lidar = Tco_lidar.getOrigin().x();
                double y_co_lidar = Tco_lidar.getOrigin().y();
                double theta_co = atan2(y_co_lidar,x_co_lidar);
                if (puck_detection_blue) {
                    if (std::abs(theta_camera_blue-theta_co)<0.05){
                        point2d_temp.x = x_lidar_right;
                        point2d_temp.y = y_lidar_right;
                        coord_blue_lidar_vec.push_back(point2d_temp);
                    }
                }
            }
        }



        double smallest_yellow_lidar_norm;
        if (coord_yellow_lidar_vec.size()==1){
            coord_yellow_lidar = coord_yellow_lidar_vec[0];
//            std::cout<< "Yellow Lidar x:"<<coord_yellow_lidar.x<<" y:"<<coord_yellow_lidar.y<<std::endl;
        }
        else if(coord_yellow_lidar_vec.size()>=2){
            int yellow_lidar_min_index = 0;
            smallest_yellow_lidar_norm = std::sqrt(coord_yellow_lidar_vec[0].x*coord_yellow_lidar_vec[0].x + coord_yellow_lidar_vec[0].y*coord_yellow_lidar_vec[0].y  );
            for (int i=1; i<coord_yellow_lidar_vec.size();i++){
                if (std::sqrt(coord_yellow_lidar_vec[i].x*coord_yellow_lidar_vec[i].x + coord_yellow_lidar_vec[i].y*coord_yellow_lidar_vec[i].y )<smallest_yellow_lidar_norm) {
                    smallest_yellow_lidar_norm = std::sqrt(coord_yellow_lidar_vec[i].x*coord_yellow_lidar_vec[i].x + coord_yellow_lidar_vec[i].y*coord_yellow_lidar_vec[i].y);
                    yellow_lidar_min_index = i;
                }

            }
            coord_yellow_lidar =  coord_yellow_lidar_vec[yellow_lidar_min_index];
//            std::cout<< "Yellow Lidar x:"<<coord_yellow_lidar_vec[yellow_lidar_min_index].x<<" y:"<<coord_yellow_lidar_vec[yellow_lidar_min_index].y<<std::endl;
        }

        double smallest_blue_lidar_norm;
        if (coord_blue_lidar_vec.size()==1){
            coord_blue_lidar = coord_blue_lidar_vec[0];
//            std::cout<< "Blue Lidar x:"<<coord_blue_lidar.x<<" y:"<<coord_blue_lidar.y<<std::endl;
        }
        else if(coord_blue_lidar_vec.size()>=2){
            int blue_lidar_min_index = 0;
            smallest_blue_lidar_norm = std::sqrt(coord_blue_lidar_vec[0].x*coord_blue_lidar_vec[0].x + coord_blue_lidar_vec[0].y*coord_blue_lidar_vec[0].y  );
            for (int i=1; i<coord_blue_lidar_vec.size();i++){
                if (std::sqrt(coord_blue_lidar_vec[i].x*coord_blue_lidar_vec[i].x + coord_blue_lidar_vec[i].y*coord_blue_lidar_vec[i].y )<smallest_blue_lidar_norm) {
                    smallest_blue_lidar_norm = std::sqrt(coord_blue_lidar_vec[i].x*coord_blue_lidar_vec[i].x + coord_blue_lidar_vec[i].y*coord_blue_lidar_vec[i].y);
                    blue_lidar_min_index = i;
                }

            }
            coord_blue_lidar = coord_blue_lidar_vec[blue_lidar_min_index];
//            std::cout<< "Blue Lidar x:"<<coord_blue_lidar_vec[blue_lidar_min_index].x<<" y:"<<coord_blue_lidar_vec[blue_lidar_min_index].y<<std::endl;
        }





        //////////////////////////////////////// Publish the data ///////////////////////////////////////////////////////////////////////////
        ///////////// Yellow ///////////////////////////////
        if ( std::sqrt(coord_yellow_lidar.x * coord_yellow_lidar.x + coord_yellow_lidar.y + coord_yellow_lidar.y)<= LIDAR_THRESH_PUBLISH ) {  //range of lidar is within 0.7
            yellow_puck_pose_temp.x = coord_yellow_lidar.x;  // Keep in mind:  the initial value of coord_yellow_lidar.x is 100,0
            yellow_puck_pose_temp.y = coord_yellow_lidar.y;
            yellow_puck_pose_temp.theta = 0;
            object_pose.yellow_puck_poses.push_back( yellow_puck_pose_temp );
                       std::cout<< "Yellow Lidar x:"<<yellow_puck_pose_temp.x<<" y:"<<yellow_puck_pose_temp.y<<std::endl;
        }
        else {     // the range of LIDAR data is beyond 0.7
            if (!std::isnan(x_lo_PointCloud_yellow) && !std::isnan(y_lo_PointCloud_yellow) && x_lo_PointCloud_yellow<10.0 && y_lo_PointCloud_yellow<10.0){
                yellow_puck_pose_temp.x = x_lo_PointCloud_yellow;   // Keep in mind:  the initial value of x_lo_PointCloud_yellow is 100.0
                yellow_puck_pose_temp.y = y_lo_PointCloud_yellow;
                yellow_puck_pose_temp.theta = 0;
                object_pose.yellow_puck_poses.push_back( yellow_puck_pose_temp );
                       std::cout<< "Yellow PointCloud x:"<<yellow_puck_pose_temp.x<<" y:"<<yellow_puck_pose_temp.y<<std::endl;
            }
        }
        ///////////// Blue //////////////////////////////////
        if ( std::sqrt(coord_blue_lidar.x * coord_blue_lidar.x + coord_blue_lidar.y + coord_blue_lidar.y)<= LIDAR_THRESH_PUBLISH ) {  // 0.7
            blue_puck_pose_temp.x = coord_blue_lidar.x;
            blue_puck_pose_temp.y = coord_blue_lidar.y;
            blue_puck_pose_temp.theta = 0;
            object_pose.blue_puck_poses.push_back( blue_puck_pose_temp );
                       std::cout<< "Blue Lidar x:"<<blue_puck_pose_temp.x<<" y:"<<blue_puck_pose_temp.y<<std::endl;
        }
        else {     // the range of LIDAR data is beyond 0.7
            if (!std::isnan(x_lo_PointCloud_blue) && !std::isnan(y_lo_PointCloud_blue) && x_lo_PointCloud_blue<10.0 && y_lo_PointCloud_blue<10.0){
                blue_puck_pose_temp.x = x_lo_PointCloud_blue;
                blue_puck_pose_temp.y = y_lo_PointCloud_blue;
                blue_puck_pose_temp.theta = 0;
                object_pose.blue_puck_poses.push_back( blue_puck_pose_temp );
                       std::cout<< "Blue PointCloud x:"<<blue_puck_pose_temp.x<<" y:"<<blue_puck_pose_temp.y<<std::endl;
            }
        }
        ///////////////// Green ///////////////////////////

        lo_PointCloud_green_not_NaN.clear();

        if (!lo_PointCloud_green.empty()) {     // if we received
            for (int i=0; i< lo_PointCloud_green.size(); i++){
                if (!std::isnan( lo_PointCloud_green[i].x) && !std::isnan( lo_PointCloud_green[i].y)) {   // first filter out all the NaN
                    lo_PointCloud_green_not_NaN_temp.x = lo_PointCloud_green[i].x;
                    lo_PointCloud_green_not_NaN_temp.y = lo_PointCloud_green[i].y;
                    lo_PointCloud_green_not_NaN.push_back(lo_PointCloud_green_not_NaN_temp);

                }
            }
        }
        if (!lo_PointCloud_green_not_NaN.empty()){
            green_post_pose_temp.x = lo_PointCloud_green_not_NaN[0].x;   // push the first element into the vector
            green_post_pose_temp.y = lo_PointCloud_green_not_NaN[0].y;
            green_post_pose_temp.theta = 0;
            object_pose.green_post_poses.push_back(green_post_pose_temp);
    //                                           std::cout<<"Green PointCloud x:"<<green_post_pose_temp.x <<" y:"<<green_post_pose_temp.y<<std::endl;
            if (lo_PointCloud_green_not_NaN.size()>=2){
                for (int i=1; i<lo_PointCloud_green_not_NaN.size();i++){
                    int green_post_poses_vec_size = object_pose.green_post_poses.size();
                    bool is_different_green_post = true;
                    for(int j=0;j<green_post_poses_vec_size;j++){                 // compare the new element with all the values which are already in the green_post_poses vector
                        if ( std::abs(object_pose.green_post_poses[j].y - lo_PointCloud_green_not_NaN[i].y)<=GREEN_POST_THRESH ){
                            is_different_green_post = false;      //  the new element is close to one of the values which is already in the green_post_poses vector, break
                            break;
                        }
                    }
                    if (is_different_green_post) {
                        green_post_pose_temp.x = lo_PointCloud_green_not_NaN[i].x;   // push the first element into the vector
                        green_post_pose_temp.y = lo_PointCloud_green_not_NaN[i].y;
                        green_post_pose_temp.theta = 0;
                        object_pose.green_post_poses.push_back(green_post_pose_temp);
  //                                                         std::cout<<"Green PointCloud x:"<<green_post_pose_temp.x <<" y:"<<green_post_pose_temp.y<<std::endl;
                    }
                }
            }

        }

 //       std::cout<<"green number:"<<object_pose.green_post_poses.size()<<std::endl;

        pub_object_pose.publish(object_pose);

//        imshow("Center of blob yellow", center_of_mass_image_yellow);
//        imshow("Center of blob blue", center_of_mass_image_blue);
//        imshow("Center of blob green", center_of_mass_image_green);
//        cv::inRange(filtered_image_, cv::Scalar(h_blue_down, s_blue_down , v_blue_down), cv::Scalar(h_blue_up,s_blue_up, v_blue_up), blue_mask_once);
//        cv::inRange(filtered_image_, cv::Scalar(h_yellow_down, s_yellow_down , v_yellow_down), cv::Scalar(h_yellow_up,s_yellow_up, v_yellow_up), yellow_mask_once);
//        cv::inRange(filtered_image_, cv::Scalar(h_green_down, s_green_down , v_green_down), cv::Scalar(h_green_up,s_green_up, v_green_up), green_mask_once);

//        center_of_mass_image_blue_once = extractBlob(blue_mask_once,  2);
//        center_of_mass_image_yellow_once = extractBlob(yellow_mask_once, 1);
//        center_of_mass_image_green_once = extractBlob(green_mask_once, 3);
//        imshow("Center of blob blue adjust", center_of_mass_image_blue_once);
//        imshow("Center of blob yellow adjust", center_of_mass_image_yellow_once);
//        imshow("Center of blob green adjust", center_of_mass_image_green_once);
//        cv::waitKey(1);
    }
}

///////////////////////////////////////////////////////// theta co (camera object) calculation //////////////////////////////////////////////////////////////
//std::vector<float> ObjectRecognition::theta_camera_object(std::vector<float> scan_ranges){
////    int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan_angle_min)/scan_angle_increment);
////    int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan_angle_min)/scan_angle_increment);
//    for (int i_left = 31; i_left>= 0; i_left--) {
//        float theta_lidar = i_left * scan_angle_increment;
//        std::cout<< "angle_left "<< i_left << " :"<< theta_lidar<< std::endl;
//    }

//}

/////////////////////////////////////////////////////////////// callbackMethod  for Synchronization function ///////////////////////////////////////////////////
//void ObjectRecognition::callbackMethod(const sensor_msgs::ImageConstPtr& image_color_msg, const sensor_msgs::ImageConstPtr& image_depth_msg){
//    double stamp_1 = image_color_msg->header.stamp.toSec();
//    double stamp_2 = image_depth_msg->header.stamp.toSec();
//    ROS_INFO("Callback gets called\n TIME_STAMP_RGB: %f\n TIME_STAMP_DEPTH: %f\n", stamp_1, stamp_2);
//    cv_bridge::CvImagePtr cv_ptr_rgb;
//    cv_bridge::CvImagePtr cv_ptr_depth;
//    try
//    {
//        // transform ROS image into OpenCV image
//        cv_ptr_rgb = cv_bridge::toCvCopy(image_color_msg, sensor_msgs::image_encodings::BGR8);
//        image_raw_rgb = cv_ptr_rgb->image;
//        flag_image_ok_rgb = true;
//    }
//    catch (cv_bridge::Exception& e)		// throw an error msg. if conversion fails
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//    try
//    {
//        cv_ptr_depth = cv_bridge::toCvCopy(image_depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
//        image_raw_depth = cv_ptr_depth->image;
//        flag_image_ok_depth = true;
//    }
//    catch(cv_bridge::Exception& e)    // throw an error msg. if conversion fails
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }

// //   if (flag_image_ok_depth && flag_image_ok_rgb ) {   // for the initial flag check, later this flag check is useless

//        std::cout << "depth[320,240]="<< image_raw_depth.at<float>(cv::Point(320,240))  <<std::endl;
//        cv::namedWindow("Depth map");
//        cv::imshow("Depth map", image_raw_depth);
//        cv::namedWindow("Color image");
//        cv::imshow("Color image", image_raw_rgb);
//        cv::Mat grayImage;
//        cv::cvtColor(image_raw_rgb, grayImage, CV_BGR2GRAY);
//        // turn the BGR image into HSV image
//        cv::cvtColor(image_raw_rgb, hsv_image_, CV_BGR2HSV);
//        // Gaussian kernel size 9*9, gaussian kernel standard deviation in x direction/ and in Y direction
//        cv::GaussianBlur(hsv_image_, filtered_image_, cv::Size(9, 9), 1, 1);
//        // send one filtered image for adjustment of HSV value
//        if (!adjust_blue_once) {
//            adjust_blue_once_im = filtered_image_.clone();
//            adjust_blue_once = true;
//        }
//        else{
//        adjust_blue_once_check = false; }
//        // green color for the blobs
//        cv::inRange(filtered_image_, cv::Scalar(40, 100, 100), cv::Scalar(80, 255, 255), green_mask);
//        // yellow color for the blobs
//        cv::inRange(filtered_image_, cv::Scalar(19, 90, 100), cv::Scalar(25, 255, 255), yellow_mask);
//        // blue color for the blobs
//        cv::inRange(filtered_image_, cv::Scalar(110, 100, 50), cv::Scalar(130, 255, 110), blue_mask);

//        // /////////////////// extractBlob ////////////////////////////////
//        center_of_mass_image_green = extractBlob(green_mask, image_raw_depth, 3);  // green
//        center_of_mass_image_yellow = extractBlob(yellow_mask, image_raw_depth, 1);  // yellow
//        center_of_mass_image_blue = extractBlob(blue_mask, image_raw_depth, 2);   // blue


//        imshow("Center of blob green", center_of_mass_image_green);
//        imshow("Center of blob yellow", center_of_mass_image_yellow);
//        imshow("Center of blob blue", center_of_mass_image_blue);
//        cv::waitKey(1);

//    }
//}

//void ObjectRecognition::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg_camerainfo) {
//    fx = msg_camerainfo->K[0];
//    fy = msg_camerainfo->K[4];
//    cx = msg_camerainfo->K[2];
//    cy = msg_camerainfo->K[5];
//    //std::cout << "fx= "<< fx << " fy="<<fy<<" cx="<<cx<<" cy="<<cy<<std::endl;
//    camerainfo_check = true;
//}

///////////////////////////////////////// depth image callback function //////////////////////////////////////////////////////////////////////////////////
//void ObjectRecognition::getImage_depth(const sensor_msgs::ImageConstPtr& msg_depth){
//    //double stamp_1 = image_color_msg->header.stamp.toSec();
//    double stamp_2 = msg_depth->header.stamp.toSec();
//    //ROS_INFO("Depth Callback gets called\n TIME_STAMP_DEPTH: %f\n", stamp_2);


//    try
//    {  //cv_ptr_depth is a global pointer
//        cv_ptr_depth = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_32FC1);
//        //global_depth_ptr =&( cv_ptr_depth->image );
//        image_raw_depth = cv_ptr_depth->image;
//        flag_image_ok_depth = true;
//    }
//    catch(cv_bridge::Exception& e)    // throw an error msg. if conversion fails
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//}
///////////////////////////////////////////////////////////  rgb image callback function //////////////////////////////////////////////////////////////////////////
void ObjectRecognition::getImage_rgb(const sensor_msgs::ImageConstPtr& msg_rgb){
    //double stamp_1 = msg_rgb->header.stamp.toSec();
    //  double stamp_2 = image_depth_msg->header.stamp.toSec();
    //ROS_INFO("RGB Callback gets called\n TIME_STAMP_RGB: %f\n", stamp_1);
    // pointer on OpenCV image
    cv_bridge::CvImagePtr cv_ptr_rgb;
    try
    {
        // transform ROS image into OpenCV image
        cv_ptr_rgb = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
        image_raw_rgb = cv_ptr_rgb->image;
        flag_image_ok_rgb = true;
    }
    catch (cv_bridge::Exception& e)		// throw an error msg. if conversion fails
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    safety_ton_rgb = ros::Time::now();
}

//////////////////////////////////////////////////////// Point Cloud Callback //////////////////////////////////////////////////////////////////////////////
void ObjectRecognition::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    global_cloud_msg = cloud_msg;
    //double stamp_3 = cloud_msg->header.stamp.toSec();
    //ROS_INFO("PointCloud Callback gets called\n TIME_STAMP_PointCloud: %f\n", stamp_3);
    if( (cloud_msg->width * cloud_msg->height) == 0 ) {
        // not a dense cloud; return
        return;
    }
    try{

        flag_image_ok_point = true;
        //        geometry_msgs::Point p1;
        //        p1 = pixelTo3DPoint(cloud_msg, 240, 320);
        //        std::cout<<"x="<<p1.x<<", y="<<p1.y<<", z="<<p1.z<<std::endl;
    }
    catch( std::runtime_error e) {
        ROS_ERROR_STREAM("Error in converting point cloud to image: "<<e.what());
    }
    safety_ton_points = ros::Time::now();
}

/////////////////////////////////////////////////////////////// Scan Callback /////////////////////////////////////////////////////////////////////////////
void ObjectRecognition::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
    //double stamp_4 = scan_msg->header.stamp.toSec();
    //ROS_INFO("Scan Callback gets called\n TIME_STAMP_LidarScan: %f\n", stamp_4);
    global_scan_msg = scan_msg;
    scan_ranges = scan_msg->ranges;
    scan_angle_min = scan_msg ->angle_min;
    scan_angle_min = scan_msg ->angle_max;
    scan_angle_increment = scan_msg ->angle_increment;
    flag_scan_ok = true;
    safety_ton_scan = ros::Time::now();

//    filtered_scan.header = scan_msg ->header;
//    filtered_scan.angle_increment = scan_msg ->angle_increment;
//    filtered_scan.angle_max = scan_msg->angle_max;
//    filtered_scan.angle_min = scan_msg->angle_min;
//    filtered_scan.ranges = scan_msg ->ranges;


//    std::cout<<"scan_msg ranges[0]:"<<scan_msg->ranges[0]<<std::endl;
//    std::cout<<"filtered_scan ranges[0]:"<<filtered_scan.ranges[0]<<std::endl;

}

////////////////////////////////////////////////// pixelTo3DPoint ////////////////////////////////////////////////////////////////////
geometry_msgs::Point ObjectRecognition::pixelTo3DPoint(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,  int row_num,  int col_num){
    geometry_msgs::Point p;
    int arrayPosition = row_num  * 5120 + col_num * 16 ;
    int arrayPosX = arrayPosition + cloud_msg->fields[0].offset ;
    int arrayPosY =arrayPosition + cloud_msg->fields[1].offset ;
    int arrayPosZ =arrayPosition + cloud_msg->fields[2].offset ;
    float X=0.0; float Y=0.0; float Z=0.0;
    memcpy(&X, &cloud_msg->data[arrayPosX], sizeof(float));
    memcpy(&Y, &cloud_msg->data[arrayPosY], sizeof(float));
    memcpy(&Z, &cloud_msg->data[arrayPosZ], sizeof(float));
    p.x = X;
    p.y = Y;
    p.z = Z;
    return p;
}

//////////////////////////////////////////////// extractBlob ////////////////////////////////////////////
cv::Mat ObjectRecognition::extractBlob(cv::Mat InImage,  int color_index){
    cv::Mat blob_image_, contour_image_ , center_of_mass_image_;
    // if element = Mat(), a 3*3 (default) rectangular structuring element is used
    // position of the anchor with the element, default value (-1,-1) means that the anchor is at the element center
    // iterations: numer of times dilation is applied
    // borderTypra:0.210e   borderValue

    //   cv::dilate(InImage, blob_image_, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
    //   // Create a structuring element
    //   int erosion_size = 6;
    //   // get a cross shaped structuring element for erosion
    //   cv::Mat element = getStructuringElement(cv::MORPH_CROSS,
    //                 cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
    //                 cv::Point(erosion_size, erosion_size) ); // the position of the anchor

    //   // Apply erosion or dilation on the image
    //   cv::erode(blob_image_,blob_image_,element);  // dilate(image,dst,element);

    cv::erode(InImage,blob_image_,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
    cv::dilate(blob_image_,blob_image_,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(7,7)));

    cv::dilate(blob_image_,blob_image_,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(6,6)));
    cv::erode(blob_image_,blob_image_,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));


    /// Find biggest blob
    std::vector<std::vector<cv::Point> > contours;
    // Find contours

    // cv::findContours    the image is treated as binary image
    // contours    eaach contour is stored as a vector of points
    blob_image_.copyTo(contour_image_);
    cv::findContours( contour_image_, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

    if (!contours.empty()) {

        // Get the moments
        std::vector<cv::Moments> mu(contours.size() );

        double contourArea_tolerance_green = 100.0;       // to filter the small features and noise

        double biggest_contourArea_yellow = 50.0;// to filter the small features and noise
        double biggest_contourArea_blue = 50.0;


        cv::Moments biggest_mu;      //  Moments  :   class of image moment

        std::vector<cv::Point2f> mc(contours.size()); // the actual mass center

        std::vector<cv::Point2i> mc_approximate(contours.size()); // the approximate mass center

        if (color_index == 3) {                                                            // detection of green sticks
            //  Get the mass centers:
            // biggest_mu.m00   0 moment , the area of the image    .m10  .m01  the first moment
            // Mat drawing used to show mask + blob's centers
            cv::cvtColor(blob_image_,center_of_mass_image_,CV_GRAY2RGB);

    //        theta_camera_green.clear();    // std::vector<double>
            lo_PointCloud_green.clear();    // std::vector<cv::Point2d>

 //           double theta_camera_green_temp;
            geometry_msgs::Point p_mc_green_temp;
            cv::Point2d lo_PointCloud_green_temp;


            std::vector<cv::Point2i> mc_int_sorted;
            cv::Point2i mc_int_not_sorted_temp;


            // the mass center of the image
            for( int i=0; i<contours.size(); i++) {
                mu[i] = cv::moments( contours[i], false );
                mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
                //mc_approximate[i] = computeMassCenter(mc[i], depth_map);  // mc[i] is float, but mc_approximate[i] is int
                mc_approximate[i].x = int(mc[i].x);
                mc_approximate[i].y = int(mc[i].y);
                if  ( contourArea(contours[i]) > contourArea_tolerance_green ) {  // we only care out the contours with large areas
                    // draw a red cross at the center of the blob
                    cv::line(center_of_mass_image_, cv::Point2f (mc[i].x-10, mc[i].y+10), cv::Point2f (mc[i].x+10, mc[i].y-10), cv::Scalar(0,0,255),2);
                    cv::line(center_of_mass_image_, cv::Point2f (mc[i].x+10, mc[i].y+10), cv::Point2f (mc[i].x-10, mc[i].y-10), cv::Scalar(0,0,255),2);
//                    if (mc[i].x <= 159){    //  we don't use LIDAR to detect green pforsten, so theta_camera_green is actually useless
//                        theta_camera_green_temp= (159 - mc[i].x )/160 * (29.0/180 * M_PI);
//                        theta_camera_green.push_back(theta_camera_green_temp );
//                    }
//                    else {
//                        theta_camera_green_temp=  -( mc[i].x - 159 )/160 * (29.0/180 * M_PI);
//                        theta_camera_green.push_back(theta_camera_green_temp );
//                    }
                    mc_int_not_sorted_temp.x = mc_approximate[i].x;
                    mc_int_not_sorted_temp.y = mc_approximate[i].y;
                    mc_int_sorted.push_back(mc_int_not_sorted_temp);
                }
            }

             // sort the mass center from left to right (based on the x )
            if (!mc_int_sorted.empty()){
                for (int i=0; i<mc_int_sorted.size();i++){
                    for (int j=i+1; j< mc_int_sorted.size();j++){
                        if ( mc_int_sorted[i].x > mc_int_sorted[j].x ){
                             cv::Point2i sort_temp = mc_int_sorted[i];
                            mc_int_sorted[i] = mc_int_sorted[j];
                            mc_int_sorted[j] = sort_temp;
                        }
                    }
                }

                for (int i=0; i<mc_int_sorted.size();i++){

     //               std::cout<< "mc_int_sorted:"<<mc_int_sorted[i]<<std::endl;

                    p_mc_green_temp = pixelTo3DPoint(global_cloud_msg, mc_int_sorted[i].y, mc_int_sorted[i].x);
                    Tco_PointCloud.setOrigin(tf::Vector3(p_mc_green_temp.z,-p_mc_green_temp.x, 0));
                    Tlo_PointCloud=Tlc_* Tco_PointCloud;
                    lo_PointCloud_green_temp.x = Tlo_PointCloud.getOrigin().x();
                    lo_PointCloud_green_temp.y = Tlo_PointCloud.getOrigin().y();
                    lo_PointCloud_green.push_back(lo_PointCloud_green_temp);

                }
            }


        }

        else if ((color_index == 1)){   // detection of yellow pucks
            x_lo_PointCloud_yellow = 100.0;
            y_lo_PointCloud_yellow = 100.0;
            puck_detection_yellow = false;
            for( int i = 0; i < contours.size(); i++ )
            {
                mu[i] = cv::moments( contours[i], false );
                if (  contourArea(contours[i]) > biggest_contourArea_yellow)
                {
                    biggest_contourArea_yellow = contourArea(contours[i]);
                    biggest_mu = mu[i];
                    // get the mu of the biggest contour
                    puck_detection_yellow = true;
                }
            }
            //  Get the mass centers:
            // biggest_mu.m00   0 moment , the area of the image    .m10  .m01  the first moment

            // Mat drawing used to show mask + blob's centers
            cv::cvtColor(blob_image_,center_of_mass_image_,CV_GRAY2RGB);

            if (puck_detection_yellow) {      // if we can detect a biggest contour which is also greater than the threshold

                // the mass center of the image
                cv::Point2f mc_;
                mc_=cv::Point2f(biggest_mu.m10/biggest_mu.m00, biggest_mu.m01/biggest_mu.m00);

                cv::Point2i mc_approximate_;
                mc_approximate_.x = int(mc_.x);
                mc_approximate_.y = int(mc_.y);


                // draw a red cross at the center of the blob
                cv::line(center_of_mass_image_, cv::Point2f (mc_.x-10, mc_.y+10), cv::Point2f (mc_.x+10, mc_.y-10), cv::Scalar(0,0,255),2);
                cv::line(center_of_mass_image_, cv::Point2f (mc_.x+10, mc_.y+10), cv::Point2f (mc_.x-10, mc_.y-10), cv::Scalar(0,0,255),2);
                //std::cout<<"contours[i][1].x = "<<contours[i][1].x<<std::endl;

                //double theta_camera = (61.0/180 * M_PI ) +  (mc[i].x - 0)/320 * (29.0/180 * M_PI);

                if (mc_.x <= 159){  //left
                    theta_camera_yellow =  (159 - mc_.x )/160 * (29.0/180 * M_PI);
                }
                else {
                    theta_camera_yellow =  -( mc_.x - 159 )/160 * (29.0/180 * M_PI);
                }

                p_mc_yellow = pixelTo3DPoint(global_cloud_msg, mc_approximate_.y, mc_approximate_.x);

                Tco_PointCloud.setOrigin(tf::Vector3(p_mc_yellow.z,-p_mc_yellow.x, 0));
                Tlo_PointCloud=Tlc_* Tco_PointCloud;
                x_lo_PointCloud_yellow = Tlo_PointCloud.getOrigin().x();
                y_lo_PointCloud_yellow = Tlo_PointCloud.getOrigin().y();

            }
        }
        else {                                           // detection of blue pucks
            x_lo_PointCloud_blue = 100.0;
            y_lo_PointCloud_blue = 100.0;
            puck_detection_blue = false;
            for( int i = 0; i < contours.size(); i++ )
            {
                mu[i] = cv::moments( contours[i], false );
                if (  contourArea(contours[i]) > biggest_contourArea_blue)
                {
                    biggest_contourArea_blue = contourArea(contours[i]);
                    biggest_mu = mu[i];
                    // get the mu of the biggest contour
                    puck_detection_blue = true;
                }
            }


            //  Get the mass centers:
            // biggest_mu.m00   0 moment , the area of the image    .m10  .m01  the first moment

            // Mat drawing used to show mask + blob's centers
            cv::cvtColor(blob_image_,center_of_mass_image_,CV_GRAY2RGB);
            if (puck_detection_blue) {

                // the mass center of the image
                cv::Point2f mc_;
                mc_=cv::Point2f(biggest_mu.m10/biggest_mu.m00, biggest_mu.m01/biggest_mu.m00);

                cv::Point2i mc_approximate_;
                mc_approximate_.x = int(mc_.x);
                mc_approximate_.y = int(mc_.y);


                // draw a red cross at the center of the blob
                cv::line(center_of_mass_image_, cv::Point2f (mc_.x-10, mc_.y+10), cv::Point2f (mc_.x+10, mc_.y-10), cv::Scalar(0,0,255),2);
                cv::line(center_of_mass_image_, cv::Point2f (mc_.x+10, mc_.y+10), cv::Point2f (mc_.x-10, mc_.y-10), cv::Scalar(0,0,255),2);
                //std::cout<<"contours[i][1].x = "<<contours[i][1].x<<std::endl;

                //double theta_camera = (61.0/180 * M_PI ) +  (mc[i].x - 0)/320 * (29.0/180 * M_PI);

                if (mc_.x <= 159){  //left
                    theta_camera_blue =  (159 - mc_.x )/160 * (29.0/180 * M_PI);
                }
                else {
                    theta_camera_blue =  -( mc_.x - 159 )/160 * (29.0/180 * M_PI);
                }

                p_mc_blue = pixelTo3DPoint(global_cloud_msg, mc_approximate_.y, mc_approximate_.x);


                Tco_PointCloud.setOrigin(tf::Vector3(p_mc_blue.z,-p_mc_blue.x, 0));
                Tlo_PointCloud=Tlc_* Tco_PointCloud;
                x_lo_PointCloud_blue = Tlo_PointCloud.getOrigin().x();
                y_lo_PointCloud_blue = Tlo_PointCloud.getOrigin().y();

//                std::cout<<"Blue Puck detected. "<<"theta_camera_blue:"<<theta_camera_blue<<std::endl;
//                std::cout<<"PointCloud Position:"<<"x:"<<x_lo_PointCloud_blue<<", y:"<<y_lo_PointCloud_blue <<std::endl;

            }
        }
        return center_of_mass_image_;
    }
    else   // if no contours can be found
        return blob_image_;
}

//cv::Point2i ObjectRecognition::computeMassCenter( cv::Point2f mc_i, cv::Mat depth_map) {
//    cv::Point2i mc_approximate_i;
//    int sum_x = 0;
//    int sum_y = 0;
//    int mc_neighbor_counter = 0;
//    int search_begin_x, search_begin_y, search_end_x, search_end_y;
//    if ((depth_map.at<float>(mc_i.x,mc_i.y)!=0)&& !std::isnan(depth_map.at<float>(mc_i.x,mc_i.y))){  // if the depth value of  mass center is neither 0 nor NaN
//        mc_approximate_i.x = int( mc_i.x );
//        mc_approximate_i.y = int( mc_i.y );
//    }
//    else {   // keep in mind that mc_i.x  mc_i.y  are both float, but search_begin_x is int
//        if ( int(mc_i.x - 10 + 0.5) < 0)
//            search_begin_x = 0;
//        else
//            search_begin_x = int(mc_i.x - 10 + 0.5);

//        if ( int(mc_i.x + 10 + 0.5) > 639)
//            search_end_x = 639;
//        else
//            search_end_x = int(mc_i.x + 10 +0.5);

//        if ( int(mc_i.y - 10 + 0.5) < 0)
//            search_begin_y = 0;
//        else
//            search_begin_y = int(mc_i.y - 10 + 0.5);

//        if ( int(mc_i.y + 10 + 0.5) > 479)
//            search_end_y = 479;
//        else
//            search_end_y = int(mc_i.y + 10 + 0.5);

//        for (int j = search_begin_x; j<= search_end_x; j++) {
//            for (int k = search_begin_y; k<= search_end_y; k++){
//                if ( (depth_map.at<float>(j,k)!=0) && !std::isnan(depth_map.at<float>(j,k))){
//                    mc_neighbor_counter = mc_neighbor_counter + 1;
//                    sum_x = sum_x +j;
//                    sum_y = sum_y +k;
//                }
//            }
//        }
//        mc_approximate_i.x = int ( sum_x / mc_neighbor_counter  + 0.5 );
//        mc_approximate_i.y = int ( sum_y / mc_neighbor_counter  + 0.5 );
//    }
//    return mc_approximate_i;
//}




