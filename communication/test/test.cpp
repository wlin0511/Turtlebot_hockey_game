#include <QDebug>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include "test.h"
#include "referee.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <cstdlib>
#include <std_srvs/SetBool.h>
#include <communication/pos.h>
#include <communication/ab.h>

Test::Test(QWidget *parent): QWidget(parent)
{
    // send from angelina
    connectedPub_ = nh_.advertise<std_msgs::Bool>("connected", 1);                                      // publish connection status
    stopPub_ = nh_.advertise<std_msgs::Bool>("/stop_signal", 1);                                        // publish stop command
    gameOverPub_ = nh_.advertise<std_msgs::Bool>("/game_over_signal", 1);                               // publish game over command
    posSub_ = nh_.subscribe("pos_info", 1, &Test::tellEgoPos, this);                                    // subscribe position info
    aliveSub_ = nh_.subscribe("/path_planning/report_alive", 1, &Test::sendAlive, this);                // subscribe alive info

    // send to angelina
    reportReadyService_ = nh_.advertiseService("report_ready", &Test::reportReady, this);               // report ready
    reportDoneService_ = nh_.advertiseService("report_done", &Test::reportDone, this);                  // report all pucks have been done
    reportGoalService_ = nh_.advertiseService("report_goal", &Test::reportGoal, this);                  // report goal
    setColorService_ = nh_.advertiseService("set_color", &Test::tellTeamColor, this);                   // send required color
    checkRatioService_ = nh_.advertiseService("check_ratio", &Test::tellAbRatio, this);                 // send required color

    // send out from angelina
    abClient_ = nh_.serviceClient<communication::ab>("ab_information");
    colorClient_PP = nh_.serviceClient<std_srvs::SetBool>("/path_planning/send_color");                   // send true color back to robot 0: blue 1: yellow
    colorClient_LOC = nh_.serviceClient<std_srvs::SetBool>("/localisation/send_color");                   // send true color back to robot 0: blue 1: yellow
    detectionStartClientPP_ = nh_.serviceClient<std_srvs::SetBool>("/path_planning/start_detection");     // send detection start
//    detectionStartClientLoc_ = nh_.serviceClient<std_srvs::SetBool>("/localisation/start_detection");     // send detection start
    gameStartClient_ = nh_.serviceClient<std_srvs::SetBool>("/path_planning/start_game");               // send game start


    pubTimer_ = nh_.createTimer(ros::Duration(1), &Test::publishCommand, this);                         // timer for publisher

    bColorSrv_.request.data = false;
    bDetectionStartSrvPP_.request.data = false;
	bDetectionStartSrvLoc_.request.data = false;
    bGameStartSrv_.request.data = false;
}

void Test::publishCommand(const ros::TimerEvent&){                                                      // publish stop command
    stopCommandMsgs_.data = bStopMovement_;
    stopPub_.publish(stopCommandMsgs_);
    gameOverMsgs_.data = bGameOver_;
    gameOverPub_.publish(gameOverMsgs_);
    connectedMsgs_.data = bConnected_;
    connectedPub_.publish(connectedMsgs_);
}

// libereferee: methods

void Test::testConnect() {
    referee = new Referee(0, this);
    connect(referee, SIGNAL(connected()), this, SLOT(slotConnected()));
    connect(referee, SIGNAL(disconnected()), this, SLOT(slotDisconnected()));
    connect(referee, SIGNAL(gameStart()), this, SLOT(slotGameStart()));
    connect(referee, SIGNAL(detectionStart()), this, SLOT(slotDetectionStart()));
    connect(referee, SIGNAL(gameOver()), this, SLOT(slotGameOver()));
    connect(referee, SIGNAL(stopMovement()), this, SLOT(slotStopMovement()));
    connect(referee, SIGNAL(trueColorOfTeam(TeamColor)), this, SLOT(slotTeamColor(TeamColor)));
    connect(referee, SIGNAL(abValues(double,double)), this, SLOT(slotAbValues(double,double)));
    referee->connectToServer("129.187.240.111", 10000);
}

bool Test::reportReady(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res){
    qDebug() << "robot is ready" << endl;
    referee->reportReady();
    res.success=true;
    return true;
}

bool Test::reportDone(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&){
    qDebug() << "robot reports done" << endl;
    referee->reportDone();
    return true;
}

void Test::sendAlive(const std_msgs::Bool::ConstPtr& msg){
    if (msg->data == true){
        referee->sendAlive();
    }
}

bool Test::tellTeamColor(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response&){
    TeamColor color;
    if (req.data == 0){
        color = yellow;
    }
    else{
        color = blue;
    }
    referee->tellTeamColor(color);
    return true;
}

bool Test::reportGoal(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&){
    qDebug() << "robot reports goal" << endl;
    referee->reportGoal();
    return true;
}

void Test::tellEgoPos(const communication::pos::ConstPtr& msg){
    referee->tellEgoPos(double(msg->x), double(msg->y));
}

bool Test::tellAbRatio(communication::ratio::Request& req, communication::ratio::Response&){
    referee->tellAbRatio(double(req.ratio));
    return true;
}

// libereferee: signals

void Test::disconnected(){
    qDebug() << "disconnected" << endl;
}

void Test::detectionStart(){
    qDebug() << "detection start" << endl;
}

void Test::gameStart(){
    qDebug() << "game start" << endl;
}

void Test::trueColorOfTeam(TeamColor tcolor){
    if (tcolor == yellow){
        bTeamColor_ = false;
    }
    else{
        bTeamColor_ = true;
    }
}

void Test::gameOver(){
    qDebug() << "game over" << endl;
}

void Test::stopMovement(){
    qDebug() << "stop movement" << endl;
}

bool Test::getConnectionInfo(){
    return bConnected_;
}

// libereferee: slots

void Test::slotConnected(){
    qDebug() << "Connected" << endl;
    bConnected_ = true;
}

void Test::slotDisconnected(){
    qDebug() << "Disconnected" << endl;
    bDisconnected_ = true;
    bConnected_ = false;
}

void Test::slotDetectionStart(){
    qDebug() << "Detection Start" << endl;
    bDetectionStartSrvPP_.request.data = true;
    detectionStartClientPP_.call(bDetectionStartSrvPP_);
//    bDetectionStartSrvLoc_.request.data = true;
//    detectionStartClientLoc_.call(bDetectionStartSrvLoc_);
    qDebug() << "Detection Start called" << endl;
}

void Test::slotGameStart() {
     qDebug() << "Game has been started!" << endl;
     bGameStartSrv_.request.data = true;
     //bDetectionStartSrvPP_.request.data = true;
     //detectionStartClientPP_.call(bDetectionStartSrvPP_);
     gameStartClient_.call(bGameStartSrv_);
}

void Test::slotTeamColor(TeamColor color)   // return the real team color
{
    if (color == yellow){
        qDebug() << "Real Team color: Yellow" << endl;
        bColorSrv_.request.data = false;
    }
    else{
        qDebug() << "Real Team color: Blue" << endl;
        bColorSrv_.request.data = true;
    }
    if(colorClient_LOC.call(bColorSrv_))
        colorClient_PP.call(bColorSrv_);
    else
        ROS_WARN ("color send service localisation do not responce");
}

void Test::slotAbValues(double a, double b)
{
    qDebug() << "a: "<< a <<" b: "<< b << endl;
    sendABSrv_.request.a = a;
    sendABSrv_.request.b = b;
    abClient_.call(sendABSrv_);
}

void Test::slotGameOver(){
    qDebug() << "Game Over" << endl;
    bStopMovement_ = true;
    bGameOver_ = true;
}

void Test::slotStopMovement()
{
    qDebug() << "Stop immediately!" << endl;
    bStopMovement_ = true;
}
