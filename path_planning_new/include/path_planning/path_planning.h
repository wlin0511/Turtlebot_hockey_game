/*
 * path_planning.cpp
 *
 *  Created on: 18 Dec 2016
 *      Author: Dimitar Rakov
 *      email: dimitar.rakov@tum.de
 */


#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H



//ROS headers
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>


// path_planning  specific
#include "path_planning/SetTask.h"
#include "path_planning/SetPose.h"
#include "path_planning/SetVelocities.h"
#include "path_planning/map_dimensions.h"
#include "path_planning/action.h"
#include "path_planning/node.h"
#include "path_planning/problem.h"
#include "path_planning/objects_poses.h"

// external library
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <eigen3/Eigen/Dense>

// Tunable parameters
const static double MIN_PROX_RANGE = 0.10;                  // should be smaller than sensor_msgs::LaserScan::range_max
const static double ROBOT_RADIUS = 0.18;                    // robot radius from official specs /0.177
const static double SAFETY_CURVE_RADIUS = 0.40;             // safety curve consiering velocity > ROBOT_RADIUS+MIN_PROXIMITY_RANGE_M
const static double V_MAX = 0.65;                           // max Velocity 0.65m/sec
const static double V_MIN = 0.07;                           // min linear velocity 0.05m/sec
const static double OMEGA_MAX = M_PI;                       // max angular velocity pi/sec
const static double OMEGA_MIN = 0.25*M_PI;                  // min angular velocity 10.pi /sec
const static double CELL_SIZE = 0.1;                        // size of square cells with, chosen 0.1m
const static int ACT_WIN_SIZE = 21;                         // size of active windows  around 2m
const static int MAX_CERTAINLY =9;                          // maximum certantly values for active region grid
const static double HIST_ALPHA= 5.0/180*M_PI;               // histogram sector angle default 5 deg
const static double HIST_TAU_MIN = 10.0;                    // binary histogram tau_min parameters
const static double HIST_TAU_MAX = 50.0;                    // binary histogram tau_max parameters
const static int S_MAX = 2;                                 // threshold for wide oppening default 16
const static double SAFETY_DIST_TO_GOAL = 0.4;              // safety distance in front of the goal for reaching task
const static double PUCK_LOWER_RADIUS = 0.07;               // lower radius of the puck
const static double PUCK_UPPER_RADIUS = 0.08;               // upper radius of the puck TODO
const static double ACT_CIRCLE_RAD = 1.0;                   // radius of active circle
const static double SAFETY_DIST_TO_FIELD = 0.2;             // safety distance  from field sizes

/// from KRIS
const static double STEP_LENGTH = 0.35;						// moving step length
const static double MU1 = 5;								// parameter for primary candidate * refer to VFH* *
const static double MU2 = 1;								// * refer to VFH* *
const static double MU3 = 1;								// * refer to VFH* *
const static double MU1_PH = 1;								// parameter for projected candidate and heuristic * refer to VFH* *
const static double MU2_PH = 3;								// * refer to VFH* *
const static double MU3_PH = 5;								// * refer to VFH* *
const static double LAMBDA_C = 0.8;							// discount factor for cost function
const static double LAMBDA_H = 0.8;							// discount factor for heuristic function
const static int NUM_STEPS = 8;                             // number of steps in one cycle


class PathPlanning
{
public:



    PathPlanning();
    // initilize the node handle
    bool init(ros::NodeHandle &nh);

    //periodicaly called inside of PATH_PLANNING. All publisher are there
    void update(const ros::Time& time, const ros::Duration& period);



private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_turtle_odom;            // subsriber for the turtle odometry
    ros::Subscriber sub_sim_turtle_odom;        /// SIM subsriber for the simulation only turtle odometry in respect to world
    ros::Subscriber sub_laser_;                 // subscriber to the robot's laser scan topic
    ros::Subscriber sub_stop_signal_;           // subscriber to stop signal
    ros::Subscriber sub_angelina_connected_;    // subscriber to connected topic
    ros::Subscriber sub_objects_poses_;         // subscriber to objects_poses
    ros::Subscriber sub_localisation_poses_;    // subscriber to localisation poses
    ros::Subscriber sub_localisation_alive_;    // subscriber to localisation alive
    ros::Publisher  pub_cmd_vel_real_;          // publisher to turtle velocity real robot
    ros::Publisher  pub_cmd_vel_sim_;           // publisher to turtle velocity sim
    ros::Publisher  pub_puck_grasped_;          // publisher to locallizing
    ros::Publisher  pub_report_alive_;          // publisher to robot alive
    ros::ServiceServer srv_task_number_;        // servise for setting task number DEBUG
    ros::ServiceServer srv_des_pose_;           // servise for setting desired pose DEBUG
    ros::ServiceServer srv_send_color_;         // service for puck color
    ros::ServiceServer srv_start_detection_;    // service for start exploration
    ros::ServiceServer srv_start_game_;         // service for start game
    ros::ServiceServer srv_posts_stored_;       // service for storing posts
    ros::ServiceServer srv_set_map_dimensions_; // service for map dimensions
    ros::ServiceClient client_report_goal_;     // client to confirm that one puck is placed correctly
    ros::ServiceClient client_report_done_;     // client to confirm that all puck are placed correctly
    ros::ServiceClient client_report_ready_;    // client to confirm that the robot is ready
    ros::ServiceClient client_turning_done_;    // client for 2 rotation done
    geometry_msgs::Twist cmd_msg_;              // velocity setpoint msg
    std_srvs::SetBool bool_req_;                // for temporal use

    bool stop_received_;                        // flag for received stop from angelina
    bool start_detection_;                      // flag for received start exploration from angelina
    bool start_game_;                           // flag for received start game from angelina
    bool odom_ready_;                           // flag for receiving data from odometry callback
    bool scan_ready_;                           // flag for receiving data from Lidar callback
    bool localisation_ready_;                   // flag for receiving data from map callback
    bool map_ready_;                            // flag for receiving data from map callback
    bool recognition_ready_;                    // flag for receiving data from recognition callback
    bool angelina_connected_;                   // flag succesfull commnucation with angelina
    bool comm_ready_;                           // flag for receiving data from recognition callback
    bool cmd_received_;                         // flag for new received desired position
    bool all_blocked_;                          // flag if all direction are blocked              
    bool obst_avoidance_ready;                  // flag if obstacle avoidance algorithm is finished
    bool puck_goal_is_set_;                     // flag for new desired goal
    bool puck_in_front_;                        // flag for puck in front of the robot
    bool puck_grasped_;                         // flag for successfuly grasped puck
    bool puck_placed_;                          // flag for successfuly placed puck
    bool puck_released_;                        // flag for successfuly released puck
    bool robot_blocked_;                        // flag for robot been blocked
    bool all_nodes_ready_;                      // flag for all nodes are ready
    bool posts_stored_;                         // flag for strored posts from localisation node
    bool robot_unblocking_;                     // flag for start robot unblocking
    bool robot_out_out_;                        // flag robot out of field
    bool making_step_;                          // enable calculated  step from  search the alghorthm
    bool work_with_blue_pucks_;                 // define which pucks color is used
    bool problem_initialized_;                  // problem initialized flag
    bool turning_done_send_;                    // turning done flag
    bool puck_not_grasped_;                     // puck is no longor grasped signal flag
    bool ready_sended;                          // ready flag sended

    int task_number_;                           // define which task is going to be started
    int placed_puck_num_;                       // howa many pucks are alredy placed on
    int rotation_step_;                         // rotation step for the exploration mode 16 for 4pi
    int detection_step_;                        // steps in  exploration mode
    int current_step_;                          // used for defining the curent trajectory point from search alghorithm
    int chosen_puck_idx_ ;                      // index of choosen puck candidat from map coordinates

    double min_scan_angle_;                     // lidar min angle
    double max_scan_angle_;                     // lidar max angle
    double map_x_lenght_ ;                      // maps x lenght
    double map_y_lenght_;                       // maps y lenght
    double final_y_lenght_ ;                    // final square x lenght
    double final_x_lenght_;                     // final square y lenght

    // Timers
    ros::Time safety_ton_scan_, safety_ton_odom_, safety_ton_localisation_;
    ros::Time safety_ton_recognition_, safety_ton_comm_ , safety_ton_angelina_ ;
    ros::Time exploration_start_time, robot_ton_blocked_, puck_grasped_ton_, angelina_ready_ton_;


    KDL::Frame fullFmr_;                        // robot tf wrt to map (full 4x4)
    KDL::Twist Tmr_;                            // robot twist wrt to map
    KDL::Twist Tsr_;                            // robot twist wrt to start (odometry)
    KDL::Frame2 Fmf_;                           // final square tf wrt to map /// TODO goal puck tf wrt to robot  from calback ?
    KDL::Frame2 Fmr_;                           // robot tf wrt to map
    KDL::Frame2 Fsr_;                           // robot tf wrt to start point (odometry)
    KDL::Frame2 desFmr_;                        // current desired robot tf wrt to map
    KDL::Frame2 Fmp_;                           // choosen puck wrt to map
    KDL::Frame2 Fmg_;                           // goal tf wrt to map
    KDL::Frame2 Frg_;                           // goal tf wrt to robot
    KDL::Frame2 Frp_;                           // goal puck tf wrt to robot
    KDL::Frame2 Fm_blue_center;                 // approximate where are the blue pucks
    KDL::Frame2 Fm_yellow_center;               // approximate where are the yellow pucks

    KDL::Frame2 Fs_exp_init_;                   // initial pose for the exploration
    KDL::Frame2 Fpuck_offset_;                  // offset of the puck before grasping
    KDL::Frame2 Fmr_blocked_;                   // robot position by out of field
    KDL::Frame2 Fsr_blocked_;                   // robot position by blocking
    KDL::Frame2 Fsr_release_;                   // robot position for releasing
    KDL::Frame2 desF_;                          // desired frame used for move function
    KDL::Frame2 goal_offset_;                   // used in problem creation


    Eigen::VectorXd laser_angles_;
    Eigen::VectorXd laser_ranges_;
    Eigen::VectorXd act_obs_;
    std::vector <Eigen::Vector3d > non_zero_obj_;

    // primary histogram
    Eigen::VectorXd H_p_;
    // binary histogram
    Eigen::VectorXi H_b_;
    // masked binary histogram
    Eigen::VectorXi H_m_;
    // Directions
    std::vector<int> cn_, cr_ ,cl_, ct_, c_best_;

    // sampled angles used for cost functions  all those int can be initialized to 0
    int kc_, kt_, ke_, kpre_, cpre_;

    // all Puck Frames wrt to robot base
    std::vector<KDL::Frame2 > pucks_frames_r_;

    // all Puck Frames wrt to map
    std::vector<KDL::Frame2 > pucks_frames_m_;

    Problem problem_;                       // problem for every search
    Action action_;                         // single action
    std::vector<Action> actions_;           // all actions from search algorithm

    // Service to set a task number through the terminal
    bool setTaskNumber( path_planning::SetTask::Request &req ,path_planning::SetTask::Response &res);

    // Service to set a desired pose through the terminal
    bool setDesiredPose( path_planning::SetPose::Request &req ,path_planning::SetPose::Response &res);

    // Service from comm to set a color number
    bool sendColor(std_srvs::SetBool::Request &req , std_srvs::SetBool::Response &res);

    // Service from comm to start exploaration
    bool startDetection(std_srvs::SetBool::Request &req ,std_srvs::SetBool::Response &res);

    // Service from comm to start game
    bool startGame(std_srvs::SetBool::Request &req ,std_srvs::SetBool::Response &res);

    // Service from localisation for stored posts
    bool postsStored(std_srvs::SetBool::Request &req ,std_srvs::SetBool::Response &res);

    // Service from localisation for map dimensions
    bool setMapDimensions(path_planning::map_dimensions::Request &req ,path_planning::map_dimensions::Response &res);

    // Subcriber from localisation alive signal
    void localicsationAlive(const std_msgs::Bool &msg);

    // Subcriber from comm to stop signal
    void stopSignal(const std_msgs::Bool &msg);

    // Subcriber to angelina connected
    void angelinaConnected(const std_msgs::Bool &msg);

    //Callback for the range sensor data
    void getScan(const sensor_msgs::LaserScan::ConstPtr& scan);

    // callback for the turtle odometry
    void getOdometry(const nav_msgs::Odometry &msg);

    // callback for mapping
    void getMapppingData(const path_planning::objects_poses &msg);

    // callback for recognition
    void getRecognitionData(const path_planning::objects_poses &msg);

    // SIM callback for the simulated turtle odometry
    void getSimOdometry(const nav_msgs::Odometry &msg);



    // Consist the controll alghorithm
    bool moveToPose (KDL::Frame2 Fmsr, KDL::Frame2 Fdes, double max_lin_vel,  double max_ang_vel, double err_eps = 0.005);

    // Update active grid region values bounded up to max_certainly
    void calcObstacleCertainty();

    // Project goal in the active window
    void projectGoalInActWindow(std::vector<Eigen::Vector3d> &non_zero_obj, const KDL::Frame2  &Frg, bool do_mask);

    // Calculate a polar histogram consists of pi/alpha sectors
    void calcPolarHistograms(const std::vector<Eigen::Vector3d> &non_zero_obj);

    // Find Direction
    void calcCandidateDirection(const Eigen::VectorXi &H);

    // calculate transformated active window
    std::vector<Eigen::Vector3d> calcTransformedActWindow(const std::vector<Eigen::Vector3d> &non_zero_obj, const KDL::Frame2 &Fnc);

    // Create Problem
    void createProblemNew();

    // For single obstacle avoidance VHF+
    Action calcObstAvoidanceAction();

    // Check if the robot is inside of the field
    void inFieldCheck();

    // Check for min proximity
    void proximityCheck();

    // Make exploration (detection) at start
    void makeExploration();

    // Go to the puck candidate
    void goToPuck();

    // Grasp to the puck candidate
    void graspPuck();

    // Go to the final square with grasped puck
    void goToFinal();

    // Release the grasped puck
    void releasePuck ();

    // Choise candidate puck from the map coordinates
    void choicePuckGoal();






    /// FROM KRIS

    // Calculate absolute angle difference between two sectors
    int calcDeltaAngular(int c1, int c2);

    // Calculate cost function for primary candidate
    double calcPriCost(int c, int kc, int kt, int kpre);

    // Calculate cost function for projected candidates
    double calcProCost(int c, int kt, int ke, int kc, int kpre);

    // Calculate heuristic function
    double calcHeuristic(int ke, int kt, int kc, int kpre);

    // Calculate the action vector if goal can be reached within NUM_STEPS
    std::vector<Action> calcActionVector();

    // Calculate the action vector if goal can not be reached within NUM_STEPS
    std::vector<Action> calcActionVectorAStar();



};

#endif // PATH_PLANNING_H
