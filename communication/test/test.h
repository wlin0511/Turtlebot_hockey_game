#ifndef TEST_H
#define TEST_H

#include <QLineEdit>
#include <QListWidget>
#include <QRadioButton>
#include <QTimer>
#include <QtGui>
#include <QWidget>
#include <QMap>
#include "referee.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <std_srvs/SetBool.h>
#include "std_msgs/Bool.h"
#include <communication/pos.h>
#include <communication/ratio.h>
#include <communication/ab.h>

class Referee;

class Test: public QWidget
{
    Q_OBJECT
    public:
        Test(QWidget *parent=0);
        Referee *referee;
        void testConnect();
        bool reportReady(std_srvs::SetBool::Request&req, std_srvs::SetBool::Response&res);
        bool reportDone(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);
        void sendAlive(const std_msgs::Bool::ConstPtr& msg);
        bool tellTeamColor(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response&);
        bool reportGoal(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);
        void tellEgoPos(const communication::pos::ConstPtr& msg);
        bool tellAbRatio(communication::ratio::Request& req, communication::ratio::Response&);
        void disconnected();
        void detectionStart();
        void gameStart();
        void trueColorOfTeam(TeamColor);
        void gameOver();
        void stopMovement();
        void publishCommand(const ros::TimerEvent&);
        bool getConnectionInfo();

    private slots:
        void slotGameStart();
        void slotGameOver();
        void slotStopMovement();
        void slotTeamColor(TeamColor);
        void slotConnected();
        void slotDisconnected();
        void slotDetectionStart();
        void slotAbValues(double a, double b);

    private:
        ros::NodeHandle nh_;
        ros::Publisher stopPub_;
        ros::Publisher gameOverPub_;
        ros::Publisher connectedPub_ ;
        ros::Subscriber posSub_;
        ros::Subscriber aliveSub_;
        ros::ServiceServer reportReadyService_;
        ros::ServiceServer reportDoneService_;
        ros::ServiceServer reportGoalService_ ;
        ros::ServiceServer checkRatioService_;
        ros::ServiceServer setColorService_;

        ros::ServiceClient colorClient_PP;
        ros::ServiceClient colorClient_LOC;
        ros::ServiceClient abClient_ ;
		ros::ServiceClient detectionStartClientPP_;
//        ros::ServiceClient detectionStartClientLoc_;
        ros::ServiceClient gameStartClient_;

        ros::Timer sendAliveTimer_;
        ros::Timer pubTimer_;

        std_msgs::Bool stopCommandMsgs_;
        std_msgs::Bool detectCommandMsgs_;
        std_msgs::Bool gameStartMsgs_;
        std_msgs::Bool gameOverMsgs_;
        std_msgs::Bool connectedMsgs_;
        std_srvs::SetBool bColorSrv_;
        std_srvs::SetBool bDetectionStartSrvPP_;
		std_srvs::SetBool bDetectionStartSrvLoc_;
        std_srvs::SetBool bGameStartSrv_;

        communication::ab sendABSrv_;

        bool bConnected_ = false;
        bool bDisconnected_ = false;
        bool bDetectionStart_ = false;
        bool bGameStart_ = false;
        bool bGameOver_ = false;
        bool bStopMovement_ = false;
        bool bTeamColor_ = false;

};

#endif /* TEST_H */
