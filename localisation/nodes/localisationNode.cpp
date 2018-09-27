#include <ros/ros.h>
#include "localisation.h"
#include "position.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localisation_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(25);

    Localisation loc;    //define an object loc of the class Localisation, call the constructor function
    loc.init(nh);    // /////////////////////////////////////     call the init  function subscription and synchronization

    ros::Time init_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();


    while(ros::ok())
    {
        ros::spinOnce();
        loc.update(init_time, ros::Time::now() - last_time);
        last_time = ros::Time::now();
        loop_rate.sleep();
    }
    return 0;
}

