#include <ros/ros.h>
#include <path_planning/path_planning.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planning_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    PathPlanning pp;
    pp.init(nh);

    ros::Time init_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();

    while(ros::ok())
    {
        ros::spinOnce();
        pp.update(init_time, ros::Time::now() - last_time);
        last_time = ros::Time::now();
        loop_rate.sleep();
    }
    return 0;
}

