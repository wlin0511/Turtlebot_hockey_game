#include "test.h"
#include "referee.h"
#include "ros/ros.h"
#include "std_msgs/String.h"


#include <QDebug>
#include <QApplication>
#include <QThread>


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    ros::init(argc, argv, "chatter");
    Test test;
    test.testConnect();
    ros::AsyncSpinner spinner(2); // Use 2 threads
    ros::Rate loop_rate(100);
    spinner.start();
    while (ros::ok){
        ros::spinOnce();
        if(!test.getConnectionInfo()){
            test.testConnect();
        }
        app.processEvents();
        loop_rate.sleep();
    }

    return app.exec(); // the program will keep running

}

