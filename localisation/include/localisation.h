#ifndef LOCALISATION_H
#define LOCALISATION_H

#include <math.h>

//ROS headers
#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <localisation/objects_poses.h>
#include <localisation/map_dimensions.h>
#include <defineLocalisationConsts.h>
#include <communication/ab.h>
#include <communication/ratio.h>
#include <communication/pos.h>


// external library
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <eigen3/Eigen/Dense>

//node specific header files
#include "localisation/SetTask.h"
#include "localisation/SetPose.h"


class Position;

class Localisation {

	public:

		Localisation();

		// initialize node handle, the initialization function, return a bool value
		bool init(ros::NodeHandle &nh);


		//periodically called inside of LOCALISATION_PACKAGE -> publishers are inside of there
		void update(const ros::Time& time, const ros::Duration& period);

        // filter poste poses for duplicates
        std::vector<KDL::Frame2> filterPostePoses(std::vector<KDL::Frame2> poste_wrt_odom_vec_tmp, std::vector<KDL::Frame2> &poste_wrt_odom_vec_saved);

        //filter puck poses for duplicates
        void filterPuckPoses(std::vector<KDL::Frame2> pucks_wrt_odom_vec_tmp, std::vector<KDL::Frame2> &puck_wrt_odom_vec_saved);

		/* input is a vector containing the positions of found objects
		 * output is an estimated position of the robot
		 */

        void getPositionInMap();

        //get distance from odom to obstacle or input position, respectively
        double getDistance(KDL::Frame2 &object);

        // returns global position in the field frame
        //Position getGlobalPolarPosition(const Position &relativePosition, const Position &robotPosition);

        void getGlobalPosition();

        double getKDLDistance(KDL::Frame2 &posteSaved, KDL::Frame2 &posteTmp);

        //calculate a and b values from poste poses
        void get_aValue(std::vector<KDL::Frame2> &polesInField);
        void get_bValue(std::vector<KDL::Frame2> &polesInField);

        //calculate own color from puck poses in the beginning
        bool getMyColor(std::vector<KDL::Frame2> &bluePucks_wrt_odom_vec_saved, std::vector<KDL::Frame2> &yellowPucks_wrt_odom_vec_saved);
        bool gotMapFrame;



	private:
		//the ROS node handle
		ros::NodeHandle nh_;
        ros::Subscriber subObjects;
        ros::Subscriber subTurtleOdom;            // subsriber for the turtle odometry
        ros::Publisher  pubObjects;          // publisher to locallizing
        ros::Publisher posPub;
        ros::Publisher alivePub;            // publish alive to path_planning
        ros::Timer pubTimer_;               // timer for position update

        //services
        ros::ServiceClient client_posts_stored;    // client to confirm that posts are stored
        ros::ServiceClient client_map_dimensions;  // client to send map dimensions to path_planning node
        ros::ServiceClient client_set_color;     // client to send the required 0: yellow  1: blue

        ros::ServiceServer srv_turning_done;       // service for storing posts
        ros::ServiceServer detectionServer;
        std_srvs::SetBool bool_req_;
        ros::ServiceClient setColorClient;
        ros::ServiceServer getColorServer;
        std_srvs::SetBool srv_color;
        ros::Subscriber subPuckGrasbed;

        ros::ServiceClient checkRatioClient;
        ros::ServiceServer abServer;
        communication::ratio srv_ratio;

        bool detectionStartCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response&);

        // Service from path planning to see whether turning was successful
        bool turningDone(std_srvs::SetBool::Request &req , std_srvs::SetBool::Response &res);
        void getOdometry(const nav_msgs::Odometry &msg);
        void poseObjectsCb(const localisation::objects_poses &msg);

        bool colorCallBack(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response&);
        bool abCallBack(communication::ab::Request& req, communication::ab::Response&);

        void publishCommand(const ros::TimerEvent&);                                 // publish stop command


        bool isInField(KDL::Frame2 &input);


        void getPuckGrasped(const std_msgs::Bool& msg);

        std_msgs::Bool isAlive;                     //localisation alive flag
		
		//Callback functions
        bool odom_received_;                        // flag for received data from odometry callback
        bool poseCb_ok;
        bool map_received_;                         // flag for received data from map callback
        bool recognition_received_;                 // flag for received data from recognition callback
        bool filter_poste_poses_;                   // flag for filtering poste poses
        bool puck_grasped;

        bool poste_already_exists;                  // flag for filtering postes
        bool puck_already_exists;                   // flag for filtering pucks

        bool color_reported;
        bool position_reported;
        bool detection_start;

        ros::Time safety_ton_scan_, safety_ton_odom_, safety_ton_mapping_, safety_ton_recognition_ ;





        KDL::Frame fullFmr_;                        // robot tf wrt to map (full 4x4)
        KDL::Twist Tmr_;                            // robot twist wrt to map
        KDL::Twist Tsr_;                            // robot twist wrt to start (odometry)
        KDL::Frame2 Fmf_;                           // final square tf wrt to map /// TODO goal puck tf wrt to robot  from calback ?
        KDL::Frame2 Fsf_;                           // final square wrt to odom
        KDL::Frame2 Fmr_;                           // robot tf wrt to map
        KDL::Frame2 Fsr_;                           // robot tf wrt to start point (odometry)
        KDL::Frame2 Fsp_;                           // poles wrt to starting position /odom
        KDL::Frame2 Fmg_;                           // goal tf wrt to map
        KDL::Frame2 Frg_;                           // goal tf wrt to robot
        KDL::Frame2 Frp_;                           /// TODO goal puck tf wrt to robot  from calback ?
        KDL::Frame2 Fexp_;                          // initial pose for the exploration
        KDL::Frame2 Fpuck_offset_;                   // offset of the puck before graspingmsg.green_post_poses.at(i).y
        KDL::Frame2 Fms_;

        KDL::Frame2 tmp_;                            //temporary frame for filter operations

        double a_value;                              // the value for a in the map
        double b_value;                              // the value for b in the map

        //for mapping purposes
        bool a_value_confirmed;
        bool b_value_confirmed;
        double abRatio;
        bool turning_done;                            // flag for rotation in the beginning
        bool posts_stored;
        bool abRatio_sent = false;
        bool reported_posts_stored = false;

        //calculate own color
        bool work_with_blue_pucks_;                 //true: blue pucks; false: yellow pucks
        bool color_calculated;                      //flag for whether color has been successfully calculated
        bool tellEgoPos;
        communication::pos egoPos;



        //callback for poles // pucks data
        void getRecognitionData(const localisation::objects_poses &msg);

        //callback for mapping data
        void getMapppingData(const localisation::objects_poses &msg);

        // all Poles wrt to odom base
        std::vector<geometry_msgs::Pose2D> poles_wrt_odom;

        // all Puck Frames wrt to robot base
        std::vector<KDL::Frame2> pucks_frames_s;

        // all Puck Frames wrt to robot base
        std::vector<KDL::Frame2> pucks_frames_r_;

        // all Puck Frames wrt to map
        std::vector<KDL::Frame2> pucks_frames_m_;

        std::vector<double> a_vector;

        //temporary input vector
        //std::vector<geometry_msgs::Pose2D> green_post_poses_vec_tmp;
        std::vector<KDL::Frame2> poste_wrt_odom_vec_tmp;
        std::vector<KDL::Frame2> poste_wrt_odom_vec_filtered;
        std::vector<KDL::Frame2> poste_wrt_odom_vec_saved;
        std::vector<KDL::Frame2> poste_neg_y;
        std::vector<KDL::Frame2> poste_pos_y;

        //vectors for blue and yellow pucks
        std::vector<KDL::Frame2> bluePucks_wrt_odom_vec_tmp;
        std::vector<KDL::Frame2> bluePucks_wrt_odom_vec_saved;
        std::vector<KDL::Frame2> yellowPucks_wrt_odom_vec_tmp;
        std::vector<KDL::Frame2> yellowPucks_wrt_odom_vec_saved;

        geometry_msgs::Pose2D tmp_pole_pose;

        enum State {searching, nPoles, threePoles, twoPoles, onePole};

};


#endif // LOCALISATION_H
