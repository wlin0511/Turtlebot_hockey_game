#include <ros/ros.h>
#include "object_recognition/object_recognition.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_recognition_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    ObjectRecognition ep;    //define an object   ep of the class ObjectRecognition, call the constructor function
    ep.init(nh);    // /////////////////////////////////////     call the init  function subscription and synchronization

    ros::Time init_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();


    while(ros::ok())
    {
        ros::spinOnce();
        ep.update(init_time, ros::Time::now() - last_time);
        last_time = ros::Time::now();
        loop_rate.sleep();
    }
    return 0;
}

