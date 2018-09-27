#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <cstdlib>
#include <std_srvs/SetBool.h>
#include <communication/pos.h>
#include <communication/ratio.h>
#include <communication/ab.h>

//// callback to get real color information
bool colorCallBack(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response&){
    if (req.data == 0){
        ROS_INFO("real color is yellow");
    }
    else
        ROS_INFO("real color is blue");
    return true;
}

////// callback to get ab information
bool abCallBack(communication::ab::Request& req, communication::ab::Response&){
    ROS_INFO("a=%f, b=%f", req.a, req.b);
    return true;
}

//// subscribe connected status
void connectionCallBack(const std_msgs::Bool::ConstPtr& msg){
    if (msg->data == false){
        ROS_INFO("not connected");
    }
    else
        ROS_INFO("connected");
}

//// subscribe stop command
void stopCallBack(const std_msgs::Bool::ConstPtr& msg){
    if (msg->data == false){
        ROS_INFO("not stop");
    }
    else
        ROS_INFO("stop");
}

//// for robot, detection start command
//void detectCallBack(const std_msgs::Bool::ConstPtr& msg){
//    if (msg->data == false){
//        ROS_INFO("wait for detection permission command");
//    }
//    else
//        ROS_INFO("start detection");
//}

//// for robot, game start command
//void gameStartCallBack(const std_msgs::Bool::ConstPtr& msg){
//    if (msg->data == false){
//        ROS_INFO("wait for game start command");
//    }
//    else
//        ROS_INFO("game start");
//}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "send_color");
    ros::NodeHandle n;

////****************** color part ******************//

//    ros::ServiceClient setColorClient = n.serviceClient<std_srvs::SetBool>("set_color");                    // client to send the required 0: yellow  1: blue
//    ros::ServiceServer getColorServer = n.advertiseService("/path_planning/send_color", colorCallBack);     // server to get color

//    std_srvs::SetBool srv_color;
//    srv_color.request.data = true;
//    if (srv_color.request.data == true){
//        ROS_INFO("Required Color Is Yellow");
//    }
//    else
//        ROS_INFO("Required Color Is Blue");

//    if (setColorClient.call(srv_color)){
//        ROS_INFO("Get Color Sucess");
//    }
//    else{
//        ROS_ERROR("Get Color Error");
//        return 1;
//    }

////****************** ratio and ab part ******************//

//    ros::ServiceClient checkRatioClient = n.serviceClient<communication::ratio>("check_ratio");     // client asks to check the ratio
//    ros::ServiceServer abServer = n.advertiseService("ab_information", abCallBack);                 // server to get ab
//    communication::ratio srv_ratio;
//    srv_ratio.request.ratio = 3;

//    if (checkRatioClient.call(srv_ratio)){
//       ROS_INFO("Read Ratio Sucess");
//    }
//    else{
//        ROS_INFO("Read Ratio Error");
//    }


////****************** connection and stop signals part ******************//

    ros::Subscriber connectionSub = n.subscribe("connected", 1, connectionCallBack);
    ros::Subscriber stopSub = n.subscribe("/stop_signal", 1, stopCallBack);

// don't use any more !!!!
//    ros::Subscriber detectStartSub = n.subscribe("detect_start", 1, detectCallBack);
//    ros::Subscriber gameStartSub = n.subscribe("game_start", 1, gameStartCallBack);


////****************** report ready report goal report done part ******************//

//    // set ready
//    ros::ServiceClient cReportReady = n.serviceClient<std_srvs::SetBool>("report_ready");
//    std_srvs::SetBool srv_ready;
//    srv_ready.request.data = true;
//    ROS_INFO("report ready");

//    if (cReportReady.call(srv_ready)){
//        ROS_INFO("report ready received");
//    }
//    else{
//        ROS_ERROR("Error");
//        return 1;
//    }

//    // set done
//    ros::ServiceClient cReportDone = n.serviceClient<std_srvs::SetBool>("report_done");
//    std_srvs::SetBool srv_done;
//    srv_done.request.data = true;
//    ROS_INFO("report done");

//    if (cReportDone.call(srv_done)){
//        ROS_INFO("report done received");
//    }
//    else{
//        ROS_ERROR("Error");
//        return 1;
//    }

//    // report goal
//    ros::ServiceClient cReportGoal = n.serviceClient<std_srvs::SetBool>("report_goal");
//    std_srvs::SetBool srv_goal;
//    srv_goal.request.data = true;
//    ROS_INFO("report goal");

//    if (cReportGoal.call(srv_goal)){
//        ROS_INFO("report goal received");
//    }
//    else{
//        ROS_ERROR("Error");
//        return 1;
//    }


////****************** update position part ******************//

//    ros::Publisher posPub = n.advertise<communication::pos>("pos_info", 1);
//    communication::pos curr_pos;
//    curr_pos.x = 1;
//    curr_pos.y = 2;

    ros::Rate loop_rate(10);
    while (ros::ok()){

////****************** update position part ******************//
//        curr_pos.x = curr_pos.x + 0.01;
//        curr_pos.y = 2;
//        posPub.publish(curr_pos);

        ros::spinOnce();
        loop_rate.sleep();

     }

    return 0;

}

