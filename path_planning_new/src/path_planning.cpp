#include "path_planning/path_planning.h"
#include "angles/angles.h"


PathPlanning::PathPlanning(){ }

bool PathPlanning::init(ros::NodeHandle &nh)
{
    // Ros node handle for the class
    nh_ = nh;


    stop_received_ = false;
    start_detection_ = false;
    start_game_ = false;
    localisation_ready_ = false;
    recognition_ready_ = false;
    comm_ready_ = false;
    angelina_connected_= false;
    odom_ready_ = false;
    scan_ready_ = false;
    cmd_received_= false;
    all_blocked_ = false;
    obst_avoidance_ready= false;
    puck_goal_is_set_= false;
    puck_in_front_=false;
    puck_grasped_=false;
    puck_placed_=false;
    puck_released_ =false;
    making_step_= false;
    work_with_blue_pucks_= false;
    problem_initialized_ =false;
    robot_blocked_=false;
    posts_stored_ = false;
    map_ready_ =false;
    all_nodes_ready_ =false;
    robot_unblocking_ =false;
    robot_out_out_= false;
    turning_done_send_ = false;
    puck_not_grasped_ = false;
    ready_sended = false;


    task_number_ =0;
    placed_puck_num_ = 0;
    chosen_puck_idx_ = 0;
    safety_ton_odom_.fromSec(-10.0);
    safety_ton_scan_.fromSec(-10.0);
    safety_ton_localisation_.fromSec(-10.0);
    safety_ton_recognition_.fromSec(-10.0);
    safety_ton_comm_.fromSec(-10.0);

    kc_= 0;
    kt_= 0;
    ke_= 0;
    kpre_= 0;
    cpre_= 0;

    map_x_lenght_ = 10.0;
    map_y_lenght_= 4.0;
    final_y_lenght_ = 5.2;
    final_x_lenght_= 1.5;


    // Initialize all subcribers, services and publishers
    sub_turtle_odom= nh_.subscribe("/odom", 10, &PathPlanning::getOdometry, this);
    sub_sim_turtle_odom = nh_.subscribe("/base_pose_ground_truth", 10, &PathPlanning::getSimOdometry, this);
    sub_laser_ = nh_.subscribe("/lidar_scan", 10, &PathPlanning::getScan, this);
    sub_stop_signal_ = nh_.subscribe("/stop_signal", 10, &PathPlanning::stopSignal, this);
    sub_angelina_connected_ = nh_.subscribe("/connected", 10, &PathPlanning::angelinaConnected, this);
    sub_objects_poses_ = nh_.subscribe("/object_recognition/objects_poses", 10, &PathPlanning::getRecognitionData, this);
    sub_localisation_poses_ = nh_.subscribe("/localisation/objects_poses", 10, &PathPlanning::getMapppingData, this);
    sub_localisation_alive_= nh_.subscribe("/localisation/localisation_alive", 10, &PathPlanning::localicsationAlive, this);

    srv_task_number_ = nh_.advertiseService("/path_planning/set_task_number", &PathPlanning::setTaskNumber, this);
    srv_des_pose_ = nh_.advertiseService("/path_planning/set_desired_pose", &PathPlanning::setDesiredPose, this);
    srv_send_color_= nh_.advertiseService("/path_planning/send_color", &PathPlanning::sendColor, this);
    srv_start_detection_= nh_.advertiseService("/path_planning/start_detection", &PathPlanning::startDetection, this);
    srv_start_game_= nh_.advertiseService("/path_planning/start_game", &PathPlanning::startGame, this);
    srv_posts_stored_ = nh_.advertiseService("/path_planning/posts_stored", &PathPlanning::postsStored, this);
    srv_set_map_dimensions_ = nh_.advertiseService("/path_planning/set_map_dimensions", &PathPlanning::setMapDimensions, this);
    client_report_goal_ = nh_.serviceClient<std_srvs::SetBool>("report_goal");
    client_report_done_ = nh_.serviceClient<std_srvs::SetBool>("report_done");
    client_report_ready_= nh_.serviceClient<std_srvs::SetBool>("report_ready");
    client_turning_done_= nh_.serviceClient<std_srvs::SetBool>("/localisation/turning_done");
    pub_report_alive_ =  nh_.advertise<std_msgs::Bool>("/path_planning/report_alive", 10);
    pub_puck_grasped_ =  nh_.advertise<std_msgs::Bool>("/path_planning/puck_grasped", 10);
    pub_cmd_vel_real_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    pub_cmd_vel_sim_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);       ///SIM

    // SIM default transformation somewhere in the middle of the field for start
    pucks_frames_m_.push_back( KDL::Frame2( KDL::Rotation2(0), KDL::Vector2(1.40, 2.0)));
    pucks_frames_m_.push_back( KDL::Frame2( KDL::Rotation2(0), KDL::Vector2(1.40, 1.5)));
    pucks_frames_m_.push_back( KDL::Frame2( KDL::Rotation2(0), KDL::Vector2(1.40, 1.0)));
    Fmf_ = KDL::Frame2( KDL::Rotation2(0), KDL::Vector2(3.2, 1.4));
    Fmp_ = KDL::Frame2( KDL::Rotation2(0), KDL::Vector2(2.0, 1.5));

    ROS_INFO ("PathPlanning is initialized");
    return true;

}

void PathPlanning::update(const ros::Time& time, const ros::Duration& period){

    // Safety timers
    odom_ready_= ((ros::Time::now()- safety_ton_odom_).toSec()< 2.0)? true : false;
    if (!odom_ready_) ROS_WARN("Odometry topic is no longer available");

    scan_ready_= ((ros::Time::now()- safety_ton_scan_).toSec()< 2.0)? true : false;
    if (!scan_ready_) ROS_WARN("Scan topic is no available");

    localisation_ready_= ((ros::Time::now()- safety_ton_localisation_).toSec()< 2.0)? true : false;  ///SIM
    if (!localisation_ready_) ROS_WARN("Localisation node is no longer available");

    recognition_ready_= ((ros::Time::now()- safety_ton_recognition_).toSec()< 2.0)? true : false;
    if (!recognition_ready_) ROS_WARN("Recognition node is no longer available");           ///SIM

    comm_ready_= ((ros::Time::now()- safety_ton_comm_).toSec()< 2.0)? true : false;
    if (!comm_ready_) ROS_WARN("Comm node is no longer available");           ///SIM



    ////////////////////////////////SIM //////////////////////////////////
//    recognition_ready_ = true;
//    localisation_ready_ = true;
//    comm_ready_ = true;
//    angelina_connected_= true;

    //////////////////////////////////////////////////////////////////////

    all_nodes_ready_ = (odom_ready_ && scan_ready_ && localisation_ready_ && recognition_ready_ && comm_ready_);

    if(scan_ready_ && odom_ready_){
        if (cmd_received_){
            /// DEBUG
            std::cout << "Fmr_: x: "<<Fmr_.p.x()<<"   y: "<<Fmr_.p.y()<<"   theta: "<<Fmr_.M.GetRot()<<"\n";
            std::cout << "Msr velocities: linear  x: "<<Tmr_.vel.x() << "   angular z: "<<Tmr_.rot.z()<<"\n";
            std::cout << "Fsr_: x: "<<Fsr_.p.x()<<"   y: "<<Fsr_.p.y()<<"   theta: "<<Fsr_.M.GetRot()<<"\n";
            std::cout << "desFmr_: x: "<<desFmr_.p.x()<<"   y: "<<desFmr_.p.y()<<"   theta: "<<desFmr_.M.GetRot()<<"\n";
            if (moveToPose(Fsr_, desFmr_, 0.4*V_MAX,  0.2*M_PI ,0.03)){
                cmd_received_ =false;
                ROS_INFO ("On target");
            }
        }
    }

    if (all_nodes_ready_){
        ROS_INFO_ONCE ("all node are ready");

        if (angelina_connected_ && start_detection_ ){
            makeExploration();
        }

        if (start_game_ && angelina_connected_ && !stop_received_ && map_ready_){

            /// Set the puck goal from map;
            if (!puck_goal_is_set_ && placed_puck_num_<3){
                choicePuckGoal();
            }


            Fpuck_offset_ = KDL::Frame2(KDL::Rotation2(0.0), KDL::Vector2(-SAFETY_DIST_TO_GOAL, 0.0));
            Fmg_ = Fmp_*Fpuck_offset_;

            if(!pucks_frames_r_.empty()){
                double dist = std::abs((Fmp_.p - (Fmr_ * pucks_frames_r_.back()).p).Norm());
                if (dist< STEP_LENGTH){
                    Frp_ = pucks_frames_r_.back();
                    ROS_INFO("Frp_ from kinect/lidar");
                }
                else{
                    Frp_ = Fmr_.Inverse() * Fmp_;
                    ROS_INFO("Frp_ from map");
                }
            }
            else{
                Frp_ = Fmr_.Inverse() * Fmp_;
                ROS_INFO("Frp_ from map");
            }


            //////////////////////////////////////////////////////////////////////////////

            if (puck_grasped_ && !puck_placed_ && !pucks_frames_m_.empty()) {
               Fmp_ = Fmr_* KDL::Frame2(KDL::Rotation2(0.0), KDL::Vector2(ROBOT_RADIUS + PUCK_LOWER_RADIUS +0.01, 0.0));
            }

            ROS_INFO("____________________________________________________________________");
            ROS_INFO("map_x_lenght: %lf, map_y_lenght: %lf", map_x_lenght_, map_y_lenght_);     //SIM
            ROS_INFO("Fmr_.x: %lf, Fmr_.y: %lf, Fmr_.theta: %lf", Fmr_.p.x(), Fmr_.p.y(), angles::to_degrees(Fmr_.M.GetRot()));
            ROS_INFO("Frp_.x: %lf, Frp_.y: %lf, Frp_.theta: %lf", Frp_.p.x(), Frp_.p.y(), angles::to_degrees(Frp_.M.GetRot()));
            ROS_INFO("Fmp.x: %lf, Fmp.y: %lf, Fmp.theta: %lf", Fmp_.p.x(), Fmp_.p.y(), angles::to_degrees(Fmp_.M.GetRot()));
            ROS_INFO("Fmf.x: %lf, Fmf.y: %lf, Fmf.theta: %lf", Fmf_.p.x(), Fmf_.p.y(), angles::to_degrees(Fmf_.M.GetRot()));
            //for (auto Fmp : pucks_frames_m_){ROS_INFO("Fmp: x: %lf, y %lf , theta %lf", Fmp.p.x(), Fmp.p.y(), Fmp.M.GetRot());}
            //////////////////////////////////////////////////////////////////////////////



            // Original active window, histograms and candidate direction based on real data
            calcObstacleCertainty();
            if(std::abs(Frp_.p.Norm())<ACT_CIRCLE_RAD -PUCK_LOWER_RADIUS)
                projectGoalInActWindow(non_zero_obj_ , Frp_, true);


            /// Go in front of puck
            if (puck_goal_is_set_ && !puck_in_front_){
                goToPuck();
                ROS_INFO("working in goToPuck");
            }

            /// Grasping
            if (puck_in_front_ && !puck_grasped_){
                graspPuck();
                ROS_INFO("working in graspPuck");
            }

            /// Go to the final square
            if (puck_grasped_ && !puck_placed_){
                goToFinal();
                ROS_INFO("working in goToFinal");
            }

            /// release the puck
            if (puck_placed_ && puck_grasped_ && !puck_released_){
                releasePuck();
                ROS_INFO("working in releasePuck");
            }

            /// repeat cycle till all 3 pucs are on place
            if (puck_released_){
                puck_goal_is_set_ = false;
                puck_in_front_ =false;
                puck_grasped_= false;
                puck_placed_= false;
                puck_released_= false;
                moveToPose(KDL::Frame2::Identity(), KDL::Frame2::Identity(), V_MAX,  M_PI, 0.04);       // stop
            }


            /// Puck dropped or released
            if (puck_grasped_ && std::abs(pucks_frames_r_.back().p.Norm())> (ROBOT_RADIUS + PUCK_LOWER_RADIUS + 0.10) && !puck_not_grasped_){
                puck_not_grasped_ =true;
                puck_grasped_ton_= ros::Time::now();
            }
            else{
                puck_not_grasped_ =false;
            }

            if (puck_not_grasped_ && (ros::Time::now()- puck_grasped_ton_).toSec() > 5.0){
                puck_goal_is_set_ = false;
                puck_in_front_ =false;
                puck_grasped_= false;
                puck_placed_= false;
                puck_released_= false;
            }

            inFieldCheck();
            //proximityCheck();
        }
    }

    if (stop_received_ || !angelina_connected_ || !all_nodes_ready_){
        moveToPose(KDL::Frame2::Identity(), KDL::Frame2::Identity(), 0.2*V_MAX,  0.1*M_PI, 0.04);        // Stop
        ROS_WARN("Robot is stoped 0");
        problem_initialized_ =false;
        making_step_=false;
    }

    // publish grasped puck flag to locallization
    std_msgs::Bool msg_puck_grasped;
    msg_puck_grasped.data = (puck_grasped_ && !puck_placed_);
    pub_puck_grasped_.publish(msg_puck_grasped);

    // publish robot ready alive signal for angelina
    if (!all_nodes_ready_){
        angelina_ready_ton_ = ros::Time::now();
    }
    else if ((ros::Time::now() - angelina_ready_ton_).toSec() > 5.0){
        angelina_ready_ton_ = ros::Time::now();
        std_msgs::Bool msg_ready;
        msg_ready.data = all_nodes_ready_;
        pub_report_alive_.publish(msg_ready);
    }


    // Send ready
    bool_req_.request.data = (all_nodes_ready_ && scan_ready_ && odom_ready_);
    if(angelina_connected_ && bool_req_.request.data && !ready_sended){
        if (client_report_ready_.call(bool_req_) ){
            ready_sended =true;
        }
        else
            ROS_ERROR("Failed to call service report ready");
    }


    // Publish the command velocity
    pub_cmd_vel_real_.publish(cmd_msg_);
    pub_cmd_vel_sim_.publish(cmd_msg_);

}

bool PathPlanning::setDesiredPose( path_planning::SetPose::Request &req ,path_planning::SetPose::Response &res){
    desFmr_= KDL::Frame2( KDL::Rotation2(req.des_pose.theta), KDL::Vector2(req.des_pose.x, req.des_pose.y));

    /// TEST
    cmd_received_ =true;
    res.send = true;
    return true;
}

bool PathPlanning::setTaskNumber( path_planning::SetTask::Request &req ,path_planning::SetTask::Response &res){

    if ((long int)req.task_number < 5 && (long int)req.task_number >= 0){
        ROS_INFO("requested task: %ld", (long int)req.task_number);
        task_number_ = (long int)req.task_number;
        if (task_number_ == 1)
            ROS_INFO("Do task: %ld", (long int)req.task_number);
        else {
            ROS_INFO("Do task: %ld", (long int)req.task_number);
        }
    }
    else{
        ROS_INFO("requested task: %ld is out of range [1-4]",(long int)req.task_number);
    }
    res.send =true;
    return true;
}

bool PathPlanning::sendColor(std_srvs::SetBool::Request &req , std_srvs::SetBool::Response &res){
    work_with_blue_pucks_= req.data;
    if (work_with_blue_pucks_) ROS_INFO("Received color: blue");
    else ROS_INFO("Received color: yellow");
    res.success =true;
    res.message = std::string("Send color service responsed");
    return true;
}

bool PathPlanning::startDetection(std_srvs::SetBool::Request &req ,std_srvs::SetBool::Response &res){
    if (req.data){
        stop_received_ = false;
        start_game_ = false;
        start_detection_  = true;
        rotation_step_ = 0;
        detection_step_ =0;
        ROS_INFO("Start detection request received");
        exploration_start_time = ros::Time::now();
        Fs_exp_init_ =Fsr_;
    }
    res.success =true;
    res.message = std::string("Start detection service responsed");
    return true;

}

bool PathPlanning::startGame(std_srvs::SetBool::Request &req ,std_srvs::SetBool::Response &res){
    if (req.data){
        stop_received_ = false;
        start_game_ = true;
        start_detection_  = false;
        ROS_INFO("Start game request received");
    }
    res.success =true;
    res.message = std::string("Start game service responsed");
    return true;
}

bool PathPlanning::postsStored(std_srvs::SetBool::Request &req ,std_srvs::SetBool::Response &res){
    if (req.data){
        posts_stored_ = true;
        ROS_INFO("Post stored request received");
    }
    else{
        posts_stored_ = false;
    }
    res.success =true;
    res.message = std::string("Post stored service responsed");
    return true;
}

bool PathPlanning::setMapDimensions(path_planning::map_dimensions::Request &req ,path_planning::map_dimensions::Response &res){
    map_x_lenght_ = req.x_lenght;
    map_y_lenght_ = req.y_lenght;
    ROS_INFO("Map dimensions request received map_x_lenght_: %lf  map_y_lenght_: %lf" ,map_x_lenght_, map_y_lenght_);
    Fm_blue_center.p = KDL::Vector2 (map_x_lenght_/3, map_y_lenght_/2);
    Fm_yellow_center.p = KDL::Vector2 (map_x_lenght_*2/3, map_y_lenght_/2);
    res.send =true;
    return true;
}

void PathPlanning::localicsationAlive(const std_msgs::Bool &msg){
    localisation_ready_ =msg.data;
    safety_ton_localisation_=ros::Time::now();
}

void PathPlanning::stopSignal(const std_msgs::Bool &msg){

    if (msg.data){
        stop_received_ = true;
        start_game_ = false;
        start_detection_  = false;
        puck_goal_is_set_ =false;
        puck_in_front_ =false;
        puck_grasped_ =false;
        puck_placed_=false;
        puck_released_=false;
        ROS_INFO("Stop signal received");
    }

    comm_ready_ = true;
    safety_ton_comm_ = ros::Time::now();
}

void PathPlanning::angelinaConnected(const std_msgs::Bool &msg){
    angelina_connected_ =msg.data;
}

void PathPlanning::getScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{

    // Initialize sizes at first cycle
    if (!scan_ready_){
        min_scan_angle_ = scan->angle_min;
        max_scan_angle_ = scan->angle_max;
        // define vectors with all angles and ranges
        // REAL robot the lidar data
        laser_angles_.setLinSpaced(scan->ranges.size(), 0.0, std::abs(min_scan_angle_) + std::abs(max_scan_angle_));
        ///SIM
        //laser_angles_.setLinSpaced(scan->ranges.size(), min_scan_angle_, max_scan_angle_);

        laser_ranges_ = Eigen::VectorXd::Zero(scan->ranges.size());
        act_obs_ =  Eigen::VectorXd::Zero(scan->ranges.size());
        H_p_ = Eigen::VectorXd::Zero((round(std::abs(min_scan_angle_) + std::abs(max_scan_angle_))/HIST_ALPHA));
        H_b_ = Eigen::VectorXi::Zero((round(std::abs(min_scan_angle_) + std::abs(max_scan_angle_))/HIST_ALPHA));
        H_m_ = Eigen::VectorXi::Zero((round(std::abs(min_scan_angle_) + std::abs(max_scan_angle_))/HIST_ALPHA));
    }
    // Take all ranges
    for (size_t i= 0; i < scan->ranges.size(); i++)
        laser_ranges_(i) = scan->ranges[i];

    scan_ready_ =true;
    safety_ton_scan_ = ros::Time::now();
}

void PathPlanning::getOdometry(const nav_msgs::Odometry &msg){
    KDL::Rotation Rot =KDL::Rotation::Quaternion(  msg.pose.pose.orientation.x,
                                                   msg.pose.pose.orientation.y,
                                                   msg.pose.pose.orientation.z,
                                                   msg.pose.pose.orientation.w);

    Tsr_.vel(0) =msg.twist.twist.linear.x;
    Tsr_.vel(1) =msg.twist.twist.linear.y;
    Tsr_.vel(2) =msg.twist.twist.linear.z;
    Tsr_.rot(0) =msg.twist.twist.angular.x;
    Tsr_.rot(1) =msg.twist.twist.angular.y;
    Tsr_.rot(2) =msg.twist.twist.angular.z;

    // data for Fsr_
    double alfa, beta, gama;
    Rot.GetEulerZYX(alfa, beta, gama);
    Fsr_.p= KDL::Vector2(msg.pose.pose.position.x, msg.pose.pose.position.y);
    Fsr_.M.SetRot(alfa);


    ///SIM data for Fmr_
//    KDL::Frame2 Fms = KDL::Frame2(KDL::Rotation2(0), KDL::Vector2(0, 0) );
//    Fmr_= Fms*Fsr_;


    odom_ready_ =true;
    safety_ton_odom_=ros::Time::now();

}

void PathPlanning::getSimOdometry(const nav_msgs::Odometry &msg){
    fullFmr_ =KDL::Frame(KDL::Rotation::Quaternion(msg.pose.pose.orientation.x,
                                                   msg.pose.pose.orientation.y,
                                                   msg.pose.pose.orientation.z,
                                                   msg.pose.pose.orientation.w),
                         KDL::Vector(msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.position.z));

    Tmr_.vel(0) =msg.twist.twist.linear.x;
    Tmr_.vel(1) =msg.twist.twist.linear.y;
    Tmr_.vel(2) =msg.twist.twist.linear.z;
    Tmr_.rot(0) =msg.twist.twist.angular.x;
    Tmr_.rot(1) =msg.twist.twist.angular.y;
    Tmr_.rot(2) =msg.twist.twist.angular.z;

    double alfa, beta, gama;
    fullFmr_.M.GetEulerZYX(alfa, beta, gama);

    ///SIM data for msrFmr_
    Fmr_.p= KDL::Vector2(msg.pose.pose.position.x, msg.pose.pose.position.y);
    Fmr_.M.SetRot(alfa);

    odom_ready_ =true;
    safety_ton_odom_=ros::Time::now();
}

void PathPlanning::getMapppingData(const path_planning::objects_poses &msg){
    if (msg.robot_pose.x < 20 && msg.robot_pose.y <20 && msg.robot_pose.x > -10 && msg.robot_pose.y >-10) {
        Fmr_ = KDL::Frame2(KDL::Rotation2(msg.robot_pose.theta), KDL::Vector2(msg.robot_pose.x, msg.robot_pose.y));
        Fmf_ = KDL::Frame2(KDL::Rotation2(msg.final_pose.theta), KDL::Vector2(msg.final_pose.x, msg.final_pose.y));
        map_ready_ =true;
    }
    else {
        ROS_WARN ("Sended robot position is wrong");
        map_ready_ =false;
    }
    pucks_frames_m_.clear();
    if (work_with_blue_pucks_){
        for(auto pose : msg.blue_puck_poses)
            pucks_frames_m_.push_back(KDL::Frame2(KDL::Rotation2(pose.theta), KDL::Vector2(pose.x, pose.y) ));
    }
    else{
        for(auto pose : msg.yellow_puck_poses)
            pucks_frames_m_.push_back(KDL::Frame2(KDL::Rotation2(pose.theta), KDL::Vector2(pose.x, pose.y) ));
    }
}

void PathPlanning::getRecognitionData(const path_planning::objects_poses &msg){
    pucks_frames_r_.clear();
    if (work_with_blue_pucks_){
        for(auto pose : msg.blue_puck_poses)
            pucks_frames_r_.push_back(KDL::Frame2(KDL::Rotation2(pose.theta), KDL::Vector2(pose.x, pose.y) ));
    }
    else{
        for(auto pose : msg.yellow_puck_poses)
            pucks_frames_r_.push_back(KDL::Frame2(KDL::Rotation2(pose.theta), KDL::Vector2(pose.x, pose.y) ));
    }

    recognition_ready_ =true;
    safety_ton_recognition_=ros::Time::now();
}



bool PathPlanning::moveToPose (KDL::Frame2 Fmsr, KDL::Frame2 Fdes, double max_lin_vel,  double max_ang_vel, double err_eps){
    const double k_rho = 0.5;
        const double k_beta = -3.0;
        const double k_alfa = 2.0;
        double err_x = Fdes.p.x()- Fmsr.p.x();
        double err_y = Fdes.p.y()- Fmsr.p.y();
        double err_theta = angles::shortest_angular_distance( Fmsr.M.GetRot(),Fdes.M.GetRot());
        double rho = std::sqrt(err_x*err_x + err_y*err_y);
        double alpha =0.0;
        double delta = (std::abs(err_x)== 0.0 && std::abs(err_y)== 0.0)? Fmsr.M.GetRot(): std::atan2(err_y, err_x) ;
        if (max_lin_vel > 0.0)
            alpha = angles::shortest_angular_distance( Fmsr.M.GetRot(), delta);
        else
            alpha = angles::shortest_angular_distance( Fmsr.M.GetRot(), M_PI+delta);  //reverse
        double beta = -delta + Fdes.M.GetRot() ;
        double cmd_lin_vel,cmd_ang_vel;



        ///  base alghorithm: first translation to the goal position and then orientation
        if (!Equal(Fdes.p, Fmsr.p, err_eps)){
            // if goal is rotated stop linear motion and only rotate
            if (max_lin_vel > 0.0){
                if(std::abs(alpha) > 7*err_eps )
                    cmd_lin_vel = 0.0;
                else if (std::abs(alpha) < 3*err_eps ){
                    cmd_lin_vel = k_rho*rho;
                    cmd_lin_vel = (cmd_lin_vel > max_lin_vel)?   max_lin_vel  :  cmd_lin_vel;
                    cmd_lin_vel = (cmd_lin_vel < V_MIN)?   V_MIN  :  cmd_lin_vel;
                }
                // limit linear accelaration
                cmd_lin_vel = (cmd_lin_vel > 0.0 && std::abs(Tsr_.vel.x()-cmd_lin_vel) > V_MIN)?   Tsr_.vel.x()+ V_MIN :  cmd_lin_vel;
            }
            else{
                if(std::abs(alpha) > 7*err_eps )
                    cmd_lin_vel = 0.0;
                else if (std::abs(alpha) <3*err_eps ){
                    cmd_lin_vel = -k_rho*rho;
                    cmd_lin_vel = (cmd_lin_vel < max_lin_vel)?   max_lin_vel  :  cmd_lin_vel;
                    cmd_lin_vel = (cmd_lin_vel > -V_MIN)?   -V_MIN  :  cmd_lin_vel;
                }
                // limit linear accelaration
                cmd_lin_vel = (cmd_lin_vel < 0.0 && std::abs(Tsr_.vel.x()-cmd_lin_vel) > V_MIN)?   Tsr_.vel.x()- V_MIN :  cmd_lin_vel;
            }
            cmd_ang_vel = k_alfa*(alpha);
        }
        else
            cmd_ang_vel = k_alfa*(err_theta);


        /// advanced alghorithm:translation + rotation simultaneously (!!! Unstable)
        //cmd_lin_vel = k_rho*rho;
        //cmd_ang_vel = k_alfa*(alpha) + k_beta*(beta);

        /// DEBUG
        // ROS_INFO("alpha: %lf", alpha);
        // ROS_INFO("cmd_lin_vel1: %lf, cmd_ang_vel1: %lf",cmd_lin_vel, cmd_ang_vel);


        // check velocities limits
        cmd_ang_vel = (cmd_ang_vel > 0 && cmd_ang_vel > max_ang_vel) ?  max_ang_vel  :  cmd_ang_vel;
        cmd_ang_vel = (cmd_ang_vel > 0 && cmd_ang_vel < V_MIN) ?  V_MIN  :  cmd_ang_vel;
        cmd_ang_vel = (cmd_ang_vel < 0 && cmd_ang_vel < -max_ang_vel)? -max_ang_vel  :  cmd_ang_vel;
        cmd_ang_vel = (cmd_ang_vel < 0 && cmd_ang_vel > -OMEGA_MIN)? -OMEGA_MIN  :  cmd_ang_vel;

        /// DEBUG
        //ROS_INFO("cmd_lin_vel3: %lf, cmd_ang_vel3: %lf",cmd_lin_vel, cmd_ang_vel);

        if (Equal(Fdes.p, Fmsr.p, err_eps) && Equal(Fdes.M, Fmsr.M, err_eps)){
            cmd_msg_.linear.x = 0;
            cmd_msg_.angular.z = 0;
            return true;
        }
        else{
            cmd_msg_.linear.x = cmd_lin_vel;
            cmd_msg_.angular.z = cmd_ang_vel;
            return false;
        }

}

void PathPlanning::calcObstacleCertainty(){
    non_zero_obj_.clear();

    // update
    for (size_t i= 0; i < laser_ranges_.size(); i++){

        // decrement all positive values by 1 considering dynamical enviroment
        if(act_obs_(i) > 0)
            act_obs_(i) -=1;

        // check the boundary
        if (laser_ranges_(i) < ACT_CIRCLE_RAD){

            // check the max_certainly and increment by 2 to neglect the  the decrementing from beginning
            if(act_obs_(i) <= (MAX_CERTAINLY-2))
                act_obs_(i) +=2;
            else if(act_obs_(i) == (MAX_CERTAINLY-1))
                act_obs_(i) +=1;
            // save the non zeros indeces for reducing of the computaional effort
            if(act_obs_(i) > 0){
                double x = laser_ranges_(i)*cos(laser_angles_(i));
                double y = laser_ranges_(i)*sin(laser_angles_(i));
                non_zero_obj_.push_back(Eigen::Vector3d(act_obs_(i), x, y ));
            }
        }
    }
}

void PathPlanning::projectGoalInActWindow(std::vector<Eigen::Vector3d> &non_zero_obj, const KDL::Frame2 &Frg, bool mask){

    // remove the old projection
    std::vector<int> idx_to_delete ;
    for (auto i = 0; i < non_zero_obj.size(); i++){
        if (non_zero_obj[i](0) == -9)
            idx_to_delete .push_back(i);
    }
    for (auto i : idx_to_delete ){
        non_zero_obj.erase(non_zero_obj.begin()+i);
    }

    double theta= angles::normalize_angle_positive(atan2(Frg.p.y(), Frg.p.x()));
    double dist = (std::abs(Frg.p.Norm())<= ACT_CIRCLE_RAD-PUCK_LOWER_RADIUS)?  std::abs(Frg.p.Norm()) : ACT_CIRCLE_RAD -PUCK_LOWER_RADIUS;
    non_zero_obj.push_back(Eigen::Vector3d(-9, dist*cos(theta), dist*sin(theta)));  // center of goal at the end of vector
    kt_= (round((theta)/HIST_ALPHA)<H_b_.size())?  round((theta)/HIST_ALPHA) :  0;
    goal_offset_ = KDL::Frame2(KDL::Rotation2(Frg.M.GetRot()), KDL::Vector2(non_zero_obj.back()(1), non_zero_obj.back()(2)));

    // Mask the goal
    for (auto i = 0; i < non_zero_obj.size(); i++){
        double dx = non_zero_obj[i](1) - non_zero_obj.back()(1);
        double dy = non_zero_obj[i](2) - non_zero_obj.back()(2);
        if (1.5*pow(PUCK_LOWER_RADIUS, 2) > dx*dx+ dy*dy && mask)
            non_zero_obj[i](0) =-9;

        // Mask the if grasped
        if (puck_grasped_){
            dx = non_zero_obj[i](1) - Frp_.p.x();
            dy = non_zero_obj[i](2) - Frp_.p.y();
            if (1.5*pow(PUCK_UPPER_RADIUS, 2) > dx*dx+ dy*dy)
                non_zero_obj[i](0) =-9;
        }
    }

    //    ///////////////////////////////////////////////// DEBUG////////////////////////////////////
    //    Eigen::MatrixXi tmp_region_grid = Eigen::MatrixXi::Zero(ACT_WIN_SIZE, ACT_WIN_SIZE);
    //    for (auto obj : non_zero_obj){
    //            // find cartesian coordinates from polar. x=r*cos(theta), y=r*sin(theta)
    //            size_t idx_col = round(0.5 * ACT_WIN_SIZE + obj(1)/CELL_SIZE);
    //            size_t idx_row = round(0.5 * ACT_WIN_SIZE - obj(2)/CELL_SIZE);
    //            if (idx_row > tmp_region_grid.rows()-1)
    //                idx_row = tmp_region_grid.rows()-1;
    //            else if (idx_row < 0 )
    //                idx_row=0;

    //            if (idx_col > tmp_region_grid.cols()-1)
    //                idx_col = tmp_region_grid.cols()-1;
    //            else if (idx_col < 0 )
    //                idx_col=0;
    //            tmp_region_grid(idx_row, idx_col) = obj(0);
    //    }

    //    Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
    //    std::cout <<"tmp_region_grid_1\n"<< tmp_region_grid.format(OctaveFmt) << "\n\n";
    //    ///////////////////////////////////////////////////////////////////////////////////////////


}

void PathPlanning::calcPolarHistograms(const std::vector<Eigen::Vector3d> &non_zero_obj){

    H_p_.setZero();
    // calculate histogram alghorithm constant parameters for speed optimisation
    double a =1+pow(ACT_CIRCLE_RAD,2);                               // positive constant
    double b =1.0;                                                   // positive constant
    double r_safety= ROBOT_RADIUS + MIN_PROX_RANGE;                  // safety radius
    double dx_r = 0;                                                 // safety center x coordinates from right side
    double dx_l = 0;                                                 // safety center x coordinates from left side
    double dy_r =-SAFETY_CURVE_RADIUS;                               /// check for upper left corner
    double dy_l = SAFETY_CURVE_RADIUS;                               /// check for upper left corner

    // calculate polar histogram considering the size of the robot
    for (auto obj : non_zero_obj){

        double theta= angles::normalize_angle_positive( std::atan2(obj(2), obj(1)));
        double dist = sqrt(pow(obj(1), 2) + pow(obj(2), 2));
        double m = (dist <=ACT_CIRCLE_RAD)?  obj(0) * a-b*dist  : 0;
        double gamma = angles::normalize_angle_positive(std::atan2(r_safety, dist));
        int range_gamma = ceil( angles::normalize_angle_positive(2*gamma)/HIST_ALPHA);

        for (int p = 0; p<=range_gamma; p++){
            int k_ext =round((angles::normalize_angle_positive(theta- gamma + p*HIST_ALPHA))/HIST_ALPHA);
            k_ext = (k_ext < H_p_.size())?  k_ext  : 0;
            if ((H_p_(k_ext)>= 0 && m>0) || (H_p_(k_ext)<= 0 && m<0))
                H_p_(k_ext) += m;
            else if (H_p_(k_ext)> 0 && m < 0)            // if exist any obstacle before the goal, block this orientation
                H_p_(k_ext) += abs(m);
        }
    }


    // calculate polar and binary
    for (size_t k= 0; k < H_p_.size(); k++){
        H_b_(k) =  (H_p_(k) > HIST_TAU_MAX)? 1  :  H_b_(k);
        H_b_(k) =  (H_p_(k) < HIST_TAU_MIN)? 0  :  H_b_(k);
    }


    // masked polar histogram
    H_m_ = H_b_;

    for (auto obj : non_zero_obj){
        if (obj(0) > 0.5*MAX_CERTAINLY){
            // Check if beta is to the right of theta
            double beta = angles::normalize_angle_positive( std::atan2(obj(2), obj(1)));
            double dr_squared = pow( dx_r - obj(1),2 ) + pow(dy_r - obj(2), 2);
            double dl_squared = pow( dx_l - obj(1),2 ) + pow(dy_l - obj(2), 2);

            if (beta> M_PI && dr_squared < pow(SAFETY_CURVE_RADIUS, 2) ){
                for (int k = round(M_PI/HIST_ALPHA); k < round(2*M_PI/HIST_ALPHA) ; k++){
                    int k_tmp = (k <H_p_.size())?  k  : 0;
                    H_m_(k_tmp) = 1.0;
                }
            }

            // Check if beta(i,j) is to the left of theta
            if (beta< M_PI && dl_squared < pow(SAFETY_CURVE_RADIUS, 2) ){
                for (int k = 0; k < round(M_PI/HIST_ALPHA) ; k++){
                    int k_tmp = (k < H_p_.size())?  k  : 0;
                    H_m_(k_tmp) = 1.0;
                }
            }
        }
    }

    /// DEBUG
    //std::cout<<"Hp_:"<<H_p_<<"\n\n";
    //std::cout<<"Hb_ size:"<<H_b_.size()<<"\n" <<H_b_.transpose()<<"\n\n";
    //std::cout<<"Hm_:"<<H_m_<<"\n\n";

}

void PathPlanning::calcCandidateDirection(const Eigen::VectorXi &H){
    std::vector <int> kr;
    std::vector <int> kl;
    cn_.clear();
    cr_.clear();
    cl_.clear();
    ct_.clear();
    c_best_.clear();

    for (size_t k =0; k< H.size(); k++){
        if (H(k) == 0){
            if (kl.empty() && kr.empty()){
                kr.push_back(k);
                kl.push_back(k);
            }
            if (k-(kl.back()) >1){              // new gap
                kr.push_back(k);
                kl.push_back(k);
            }

            if (!kr.empty() && !kl.empty())     // update the end
                kl[kl.size()-1] = k;
        }
    }
    // if there are a gaps at the begining and at the end of histogram and ranges is 2pi combine both gaps
    if (H(0)==0 && H(H.size()-1)==0 && kr.size()>1 && kl.size()>1 && max_scan_angle_> (M_PI-0.2)){
        kr[0] =kr[kr.size()-1];
        kr.pop_back();
        kl.pop_back();
    }

    if (!kr.empty() && !kl.empty()){
        for (size_t i =0; i< kl.size(); i++ ){

            // normal sequence
            if (kl[i]>kr[i] &&  kl[i]+1-kr[i] > S_MAX){
                cn_.push_back(round(0.5*(kl[i] + kr[i])));
                cr_.push_back(kr[i] + round(0.5*S_MAX));
                cl_.push_back(kl[i] - round(0.5*S_MAX));
                if (cr_.back()>=0 && cr_.back()<H_b_.size()){
                    if (H_b_(cr_.back()) == 0 )
                        c_best_.push_back(cr_.back());
                }
                else  ROS_INFO("wrong index1 normal sequence 1  cr_.back()>=0 %d, cr_.back()<H_b_.size() %d ", cr_.back()>=0 , cr_.back()<H_b_.size());
                if (cl_.back()>=0 && cl_.back()<H_b_.size()){
                    if(H_b_(cl_.back()) == 0)
                        c_best_.push_back(cl_.back());
                }
                else  ROS_INFO("wrong index2 normal sequence 1 cl_.back()>=0 %d, cl_.back()<H_b_.size() %d ", cl_.back()>=0 , cl_.back()<H_b_.size());
                //ROS_INFO("normal sequence 1");
                if ((kt_ >= cr_.back() && kt_ <= cl_.back()) || (kl[i] - kr[i] == H.size()-1)){                   
                    if (kt_>=0 && kt_ <H_b_.size()){
                        ct_.push_back(kt_);
                        if (H_b_(kt_) == 0){
                            c_best_.insert(c_best_.begin(), kt_);
                            //ROS_INFO("normal sequence 2");
                        }
                    }
                }
            }

            // inverted sequence
            if (kl[i]<kr[i] &&  (kl[i]+1 + H.size()-1 - kr[i]+1) > S_MAX){
                int avg = round(0.5*(kl[i]+ H.size()-1 - kr[i]));
                int cn = ((kr[i]+avg) <= H.size()-1)? kr[i]+avg  : kl[i]-avg;
                int cr = (round(kr[i] + 0.5*S_MAX) < H.size())? round(kr[i] + 0.5*S_MAX) : round(0.5*S_MAX-1 + kr[i])-(H.size()-1);
                int cl = (round(kl[i]- 0.5*S_MAX) >= 0)? round(kl[i]- 0.5*S_MAX)  : H.size()-1 +(round(kl[i] - 0.5*S_MAX-1)) ;

                cn_.push_back(cn);
                cr_.push_back(cr);
                cl_.push_back(cl);
                if (cr_.back()>=0 && cr_.back()<H_b_.size()){
                    if (H_b_(cr_.back()) == 0 )
                        c_best_.push_back(cr_.back());
                }
                else  ROS_INFO("wrong index1 inverted sequence 1  cr_.back()>=0 %d, cr_.back()<H_b_.size() %d ", cr_.back()>=0 , cr_.back()<H_b_.size());
                if (cl_.back()>=0 && cl_.back()<H_b_.size()){
                    if(H_b_(cl_.back()) == 0)
                        c_best_.push_back(cl_.back());
                }
                else  ROS_INFO("wrong index2 inverted sequence 1 cl_.back()>=0 %d, cl_.back()<H_b_.size() %d ", cl_.back()>=0 , cl_.back()<H_b_.size());
                //ROS_INFO("inverted sequence 1");
                if (kt_ >= cr_.back() || kt_ <= cl_.back()){
                    if (H_b_(kt_) == 0 && kt_>=0 && kt_ <H_b_.size()){
                        if (H_b_(kt_) == 0){
                            c_best_.insert(c_best_.begin(), kt_);
                            //ROS_INFO("inverted sequence 2");
                        }
                    }
                    //ROS_INFO("inverted sequence 2");
                }
            }

        }

        /// DEBUG
//        std::cout <<"kl:  "; for(auto i : kl){std::cout <<i<<"  ";} std::cout <<"\n";
//        std::cout <<"kr:  "; for(auto i : kr){std::cout <<i<<"  ";} std::cout <<"\n";
//        std::cout <<"kt:  "<< kt_<<"\n";
//        std::cout <<"cn:  "; for(auto i : cn_){std::cout <<i<<"  ";} std::cout <<"\n";
//        std::cout <<"cr:  "; for(auto i : cr_){std::cout <<i<<"  ";} std::cout <<"\n";
//        std::cout <<"cl:  "; for(auto i : cl_){std::cout <<i<<"  ";} std::cout <<"\n";
//        std::cout <<"c_best_:  "; for(auto i : c_best_){std::cout <<i<<"  ";} std::cout <<"\n";

    }
}

std::vector<Eigen::Vector3d> PathPlanning::calcTransformedActWindow(const std::vector<Eigen::Vector3d> &non_zero_obj, const KDL::Frame2 &Fnc){

    std::vector<Eigen::Vector3d> new_non_zero_obj;
    KDL::Frame2 Fco, Fno;
    for (auto obj : non_zero_obj){
        Fco = KDL::Frame2 (KDL::Rotation2(0.0), KDL::Vector2(obj(1), obj(2)));
        Fno = Fnc * Fco;
        if (obj(0)>0 && std::abs(Fno.p.Norm()) <= ACT_CIRCLE_RAD)
            new_non_zero_obj.push_back(Eigen::Vector3d(obj(0), Fno.p.x(), Fno.p.y()));

    }
    return new_non_zero_obj;
}

void PathPlanning::createProblemNew(){

    ROS_INFO("Problem is started");
    projectGoalInActWindow(non_zero_obj_ , Frg_, false);
    calcPolarHistograms(non_zero_obj_);
    calcCandidateDirection(H_b_);
    KDL::Frame2 Frg =KDL::Frame2(KDL::Rotation2(goal_offset_.M.GetRot()), KDL::Vector2(goal_offset_.p.x(), goal_offset_.p.y()));

    Action action;
    double cost, heuristic;                                     // g cost and heuristic h cost
    problem_.eraseProblem();                                    // init new problem
    problem_.resizeNet(NUM_STEPS);								// new added by Kris 07.01
    problem_.setStartState({0, 0});                             // set start state (0, 0)for problem
    problem_.addNodeStateToLayer(0, {0,0});						// new added by Kris 07.01
    std::cout <<"c_best_ in problem:  "; for(auto i : c_best_){std::cout <<i<<"  ";} std::cout <<"\n";
    for (auto idx_layer=0; idx_layer< NUM_STEPS-1; idx_layer++){

        //        ROS_INFO("***********************************************************************");
        //        ROS_INFO("New layer: %d", idx_layer);
        //        ROS_INFO("***********************************************************************");
        // Check if the goal was alredy reached in previus step
        if(problem_.goalFound()){
            ROS_INFO ("ok the goal was found");
            break;
        }

        int unique_node_index = 0;
        std::vector<std::vector <int> > parent_states = problem_.getLayer(idx_layer);

        for (auto parent_state : parent_states){
            //            ROS_INFO(" _______________________________________________");
            //            ROS_INFO("New parent state:[ %d, %d] ", parent_state[0], parent_state[1]);
            //            ROS_INFO(" _______________________________________________");
            std::vector<Node> children_nodes;

            // Check if the goal was alredy reached in previus step
            if(problem_.goalFound()){
                break;
            }

            KDL::Frame2 F0_cn = problem_.getNode(parent_state).getFrame();
            KDL::Frame2 Fcn_g = F0_cn.Inverse() * Frg;

            // Calculate parent offset
            std::vector<Eigen::Vector3d> new_non_zero_obj = calcTransformedActWindow(non_zero_obj_, F0_cn.Inverse());
            projectGoalInActWindow(new_non_zero_obj , Fcn_g, false);
            calcPolarHistograms(new_non_zero_obj);
            calcCandidateDirection(H_b_);                                           // here  c_best_ is updated

            all_blocked_ =  c_best_.empty();
            if (all_blocked_)   c_best_.push_back(M_PI_2);                          // when is blocked Rotate pi/2

            size_t max_num_children = (idx_layer<1)? 4 : 2;                         // reduce amount of successors after first layer
            // find chidren
            for (auto j= 0; j< std::min(c_best_.size(), max_num_children); j++){

                if (all_blocked_){
                    action = Action(0, 0, M_PI_2);
                    ROS_WARN("Problem creation all blocked");
                }
                else if(std::abs(Fcn_g.p.Norm())<=STEP_LENGTH && c_best_[j] == kt_){
                    action = Action(Fcn_g.p.x(), Fcn_g.p.y(), Fcn_g.M.GetRot());
                    // ROS_INFO ("almost on the goal");
                }
                else
                    action = Action(STEP_LENGTH*cos(c_best_[j]*HIST_ALPHA), STEP_LENGTH*sin(c_best_[j]*HIST_ALPHA), c_best_[j]*HIST_ALPHA);

                // calculate cost and heuristic
                if (all_blocked_){
                    cost = 1000000;                                                 // calculate some large cost and heuristic for blocked node
                    heuristic = 1000000;
                }
                else if(idx_layer==0){
                    cost = calcPriCost(c_best_[j], kc_, kt_, kpre_);
                    heuristic = pow(LAMBDA_H, idx_layer)*calcHeuristic(ke_, kt_, kc_, cpre_);
                }
                else{
                    cost = pow(LAMBDA_C, idx_layer)*calcProCost(c_best_[j], kt_, ke_, kc_, cpre_);
                    heuristic = pow(LAMBDA_H, idx_layer)*calcHeuristic(ke_, kt_, kc_, cpre_);
                }

                // Calculate new node transformation wrt to active window
                KDL::Frame2 Fcn_nn =  KDL::Frame2(KDL::Rotation2(action.theta()), KDL::Vector2(action.x(), action.y()));
                KDL::Frame2 F0_nn = F0_cn * Fcn_nn;
                KDL::Frame2 Fnn_g = F0_nn.Inverse() * Frg;

//                // DEBUG
//                ROS_INFO("Fcn_g.x %lf, Fcn_g.y %lf , Fcn_g.theta %lf",Fcn_g.p.x(), Fcn_g.p.y(), Fcn_g.M.GetRot());
//                ROS_INFO("F0_cn.x: %lf, F0_cn.y: %lf, F0_cn.theta: %lf", F0_cn.p.x(), F0_cn.p.y(), F0_cn.M.GetRot());
//                ROS_INFO("Fcn_nn.x: %lf, Fcn_nn.y: %lf, Fcn_nn.theta: %lf", Fcn_nn.p.x(), Fcn_nn.p.y(), Fcn_nn.M.GetRot());
//                ROS_INFO("F0_nn.x: %lf, F0_nn.y: %lf, F0_nn.theta: %lf", F0_nn.p.x(), F0_nn.p.y(), F0_nn.M.GetRot());
//                ROS_INFO("Fnn_g.x: %lf, Fnn_g.y: %lf, Fnn_g.theta: %lf", Fnn_g.p.x(), Fnn_g.p.y(), Fnn_g.M.GetRot());

                kpre_ = 0.0;
                ke_  = 0.0;
                kc_ = 0.0;

                std::vector<int> child_state {idx_layer+1, unique_node_index};
                children_nodes.push_back(Node(parent_state, action, cost, heuristic, F0_nn));
                problem_.addNodeStateToLayer(child_state[0], child_state);			// new added by Kris 07.01
                problem_.setStateNodeMap(child_state, children_nodes.back());       // map state to node
                problem_.setParent(child_state, parent_state);
                unique_node_index++;
                if(KDL::Equal(Fnn_g, KDL::Frame2::Identity(), 0.01)){
                    problem_.setGoalState(child_state);
                    problem_.setGoalFound();
                    break;
                }
            }

            problem_.setSuccessors(parent_state, children_nodes);                   // map the state s and its successive nodes
        }
    }
    ROS_INFO("Problem is initialized");
}

Action PathPlanning::calcObstAvoidanceAction(){


    projectGoalInActWindow(non_zero_obj_ , Frg_, false);
    calcPolarHistograms(non_zero_obj_);
    calcCandidateDirection(H_b_);

    double min_cost = calcPriCost(c_best_[0], kc_, kt_, kpre_);
    int idx_c = 0;
    if (!c_best_.empty()){
        for (auto i= 0; i< c_best_.size(); i++){
            double current_cost = calcPriCost(c_best_[i], kc_, kt_, kpre_);
            if (min_cost> current_cost){
                min_cost = current_cost;
                idx_c = i;
            }
        }
        int r1 = rand() % 100 + 1;              // r1 in the range 1 to 100
        int r2 = rand() % c_best_.size();       // r2 in the range 0 to c_best_.size()
        if (r1 <= 50){      // explorer mode to avoid local minima situation, 60% takes the proposed action
            idx_c = r2;
        }

        if((std::abs(Frg_.p.Norm())<STEP_LENGTH && c_best_[idx_c] == kt_)){
            kpre_ = 0;
            ROS_INFO("1 Choosen idx: %d value: %d", idx_c , c_best_[idx_c]);
            return Action(goal_offset_.p.x(), goal_offset_.p.y(), goal_offset_.M.GetRot());
        }
        else{
            kpre_ = 0;
            ROS_INFO("2 Choosen idx: %d value: %d", idx_c , c_best_[idx_c]);
            return Action(STEP_LENGTH*cos(c_best_[idx_c]*HIST_ALPHA), STEP_LENGTH*sin(c_best_[idx_c]*HIST_ALPHA), c_best_[idx_c]*HIST_ALPHA);
        }

    }

}

void PathPlanning::proximityCheck(){


    projectGoalInActWindow(non_zero_obj_ , Frg_, false);
    calcPolarHistograms(non_zero_obj_);
    calcCandidateDirection(H_b_);


    for (auto i = 0; i < non_zero_obj_.size(); i++){
        if(non_zero_obj_[i](0) > 0.2*MAX_CERTAINLY){
            double dist = sqrt (pow(non_zero_obj_[i](1), 2) + pow(non_zero_obj_[i](2), 2));
            if (ROBOT_RADIUS + MIN_PROX_RANGE > dist ){
                problem_initialized_ =false;
                making_step_=false;
                if (!robot_blocked_){
                    robot_blocked_ = true;
                    robot_ton_blocked_= ros::Time::now();
                }          
            }
        }
    }

    if (robot_blocked_ && (ros::Time::now()- robot_ton_blocked_).toSec()>3.0 && !robot_unblocking_){
        action_ = calcObstAvoidanceAction();
        desF_= Fsr_blocked_* KDL::Frame2( KDL::Rotation2(action_.theta()), KDL::Vector2(action_.x(), action_.y()));
        robot_unblocking_= true;
    }

    int k = round(angles::normalize_angle_positive(std::atan2(action_.y() , action_.x()))/HIST_ALPHA);
    k = (k < H_b_.size())?  k  : 0;
    if (robot_unblocking_ && H_b_(k)==0){
        ROS_INFO("Fsr_x: %lf, Fsr_y: %lf, Fsr_theta: %lf", Fsr_.p.x(), Fsr_.p.y(), angles::to_degrees(Fsr_.M.GetRot()));
        ROS_INFO("des_x: %lf, des_: %lf, des_theta: %lf", desF_.p.x(), desF_.p.y(), angles::to_degrees(desF_.M.GetRot()));
        if (moveToPose(Fsr_, desF_, 0.4*V_MAX,  0.2*M_PI, 0.04)){
            robot_unblocking_= false;
            robot_blocked_ = false;
        }
    }
    else{
        robot_unblocking_ =false;
    }

    if (robot_blocked_ && !robot_unblocking_){
        ROS_WARN("Robot is blocked");
        Fsr_blocked_ =Fsr_;
        moveToPose(KDL::Frame2::Identity(), KDL::Frame2::Identity(), V_MAX,  M_PI, 0.04);          // stop
    }
}

void PathPlanning::inFieldCheck(){

    if (map_x_lenght_ -SAFETY_DIST_TO_FIELD < Fmr_.p.x() || Fmr_.p.x()< SAFETY_DIST_TO_FIELD||
        map_y_lenght_ -SAFETY_DIST_TO_FIELD < Fmr_.p.y()|| Fmr_.p.y()< SAFETY_DIST_TO_FIELD  ){
        robot_out_out_ = true;
    }
    else {
        robot_out_out_ = false;
    }

    if (robot_out_out_){
        ROS_INFO("Robot out of field");
        Fmg_ =  Fmg_ = KDL::Frame2(KDL::Rotation2(0.0), KDL::Vector2(0.5 * map_x_lenght_, 0.5 * map_y_lenght_));;
        Frg_ =Fmr_.Inverse()*Fmg_;

        if (!problem_initialized_){
            ros::Time tic =ros::Time::now();                            // time consumed for creating of a problem and solution of a A*
            createProblemNew();

            if(problem_.goalFound()){
                actions_ = calcActionVector();
            }
            else{
                // actions_ = calcActionVectorAStar();
                actions_.clear();
                actions_.push_back(calcObstAvoidanceAction());
            }
            current_step_ = actions_.size()-1;
            problem_initialized_= true;
            ROS_INFO("toc: %lf",(ros::Time::now() - tic).toSec());
            ///DEBUG
            for (auto action : actions_){ROS_INFO("action: x: %lf, y %lf , theta %lf", action.x(), action.y(), action.theta());}
        }

        projectGoalInActWindow(non_zero_obj_ , Frg_, false);
        calcPolarHistograms(non_zero_obj_);
        calcCandidateDirection(H_b_);

        if (!making_step_ && problem_initialized_){
            action_ = actions_[current_step_];
            desF_= Fsr_* KDL::Frame2( KDL::Rotation2(action_.theta()), KDL::Vector2(action_.x(), action_.y()));         // use odomemetry for steps measurments
        }

        int k = round(angles::normalize_angle_positive(std::atan2(action_.y() , action_.x()))/HIST_ALPHA);
        k = (k < H_b_.size())?  k  : 0;
        ROS_INFO("k: %d , H_b_(k) %d", k, H_b_(k));
        problem_initialized_= (H_b_(k)==0);
        making_step_= (H_b_(k)==0);

        if (making_step_ ){
            ROS_INFO("working on step: %d",current_step_);
            if (moveToPose(Fsr_, desF_, 0.2*V_MAX,  0.1*M_PI, 0.04)){
                making_step_= false;
                if(current_step_>=1)
                    current_step_-- ;
                else
                    problem_initialized_= false;
            }
        }
        else{
            ROS_WARN("Robot is stoped from inFieldCheck");
            moveToPose(KDL::Frame2::Identity(), KDL::Frame2::Identity(), V_MAX,  M_PI, 0.04);
        }
    }
    if (robot_out_out_ && !puck_grasped_){
        puck_goal_is_set_ = false;
        puck_in_front_ =false;
    }
}

void PathPlanning::makeExploration(){

    double rot_setpoint;
    double time =  (ros::Time::now() - exploration_start_time).toSec();
    if (rotation_step_ <24){ // 24* 45 deg = 3 rotation  otherwise max 70 sec rotation
        desF_.p = Fs_exp_init_.p;
        rot_setpoint = angles::normalize_angle_positive(Fs_exp_init_.M.GetRot() + (1 +rotation_step_ )*M_PI_4);
        desF_.M.SetRot(rot_setpoint);
        if(moveToPose(Fsr_, desF_, 0.01*V_MAX,  0.5*M_PI, 0.1)){;
            rotation_step_++;
            if (rotation_step_ ==24){
                bool_req_.request.data = true;
                if(client_turning_done_.call(bool_req_)){
                    turning_done_send_ = true;
                }
                else{
                    ROS_WARN ("Turning done service do not respond");
                }
            }
        }
    }

    if (time > 80.0 && !turning_done_send_){
        bool_req_.request.data = true;
        if(client_turning_done_.call(bool_req_)){
            turning_done_send_ = true;
        }
        else{
            ROS_WARN ("Turning done service do not respond");
        }
    }
    ROS_INFO("Fs_exp_init__x: %lf, Fs_exp_init__y: %lf,Fs_exp_init__theta: %lf", Fs_exp_init_.p.x(), Fs_exp_init_.p.y(), angles::to_degrees(Fs_exp_init_.M.GetRot()));
    ROS_INFO("Fsr_x: %lf, Fsr_y: %lf, Fsr_theta: %lf", Fsr_.p.x(), Fsr_.p.y(), angles::to_degrees(Fsr_.M.GetRot()));
    ROS_INFO("des_x: %lf, des_: %lf, des_theta: %lf", desF_.p.x(), desF_.p.y(), angles::to_degrees(desF_.M.GetRot()));

    /// Exploration mode
//    if (detection_step_ ==0 && (!posts_stored_ && time > 70.0)){

//        Frg_ = KDL::Frame2(KDL::Rotation2(0.0), KDL::Vector2(0.5, 0.0));
//        Action action = calcObstAvoidanceAction();
//        int k = round(angles::normalize_angle_positive(std::atan2(action.y() , action.x()))/HIST_ALPHA);
//        k = (k < H_b_.size())?  k  : 0;
//        desF_= Fsr_* KDL::Frame2( KDL::Rotation2(action.theta()), KDL::Vector2(action.x(), action.y()));
//        detection_step_ = (H_b_(k)==0)? 1 :  0;
//    }


//    if (detection_step_ ==0 && posts_stored_ && map_ready_){

//        Fmg_ = KDL::Frame2(KDL::Rotation2(0.0), KDL::Vector2(1.0, 1.0));
//        Frg_ = Fmr_.Inverse() * Fmg_;
//        Action action = calcObstAvoidanceAction();
//        int k = round(angles::normalize_angle_positive(std::atan2(action.y() , action.x()))/HIST_ALPHA);
//        k = (k < H_b_.size())?  k  : 0;
//        desF_= Fsr_* KDL::Frame2( KDL::Rotation2(action.theta()), KDL::Vector2(action.x(), action.y()));
//        detection_step_ = (H_b_(k)==0)? 1 :  0;
//    }

    if (detection_step_ ==1){
        moveToPose(Fsr_, desF_, 0.2*V_MAX,  0.2*M_PI, 0.04);
    }

}

void PathPlanning::goToPuck(){

    Fpuck_offset_ = KDL::Frame2(KDL::Rotation2(0.0), KDL::Vector2(-SAFETY_DIST_TO_GOAL, 0.0));
    Fmg_ = Fmp_*Fpuck_offset_;
    if (std::abs((Fmr_.p - Fmg_.p).Norm())< 3*STEP_LENGTH && std::abs(Frp_.p.Norm())< 3*STEP_LENGTH){      //TODO check
        Frg_ = Frp_ * Fpuck_offset_;
    }
    else {
        Frg_ =Fmr_.Inverse()*Fmg_;
    }

    if (!problem_initialized_){
        ros::Time tic =ros::Time::now();                            // time consumed for creating of a problem and solution of a A*
        createProblemNew();

        if(problem_.goalFound()){
            actions_ = calcActionVector();
        }
        else{
            // actions_ = calcActionVectorAStar();
            actions_.clear();
            actions_.push_back(calcObstAvoidanceAction());
        }
        current_step_ = actions_.size()-1;
        problem_initialized_= true;
        ROS_INFO("toc: %lf",(ros::Time::now() - tic).toSec());
        ///DEBUG
        for (auto action : actions_){ROS_INFO("action: x: %lf, y %lf , theta %lf", action.x(), action.y(), action.theta());}
    }


    projectGoalInActWindow(non_zero_obj_ , Frg_, false);
    calcPolarHistograms(non_zero_obj_);
    calcCandidateDirection(H_b_);

    if(std::abs(Frg_.p.Norm())>= STEP_LENGTH){
        if (!making_step_ && problem_initialized_){
            action_ = actions_[current_step_];
            desF_= Fsr_* KDL::Frame2( KDL::Rotation2(action_.theta()), KDL::Vector2(action_.x(), action_.y()));
        }

        int k = round(angles::normalize_angle_positive(std::atan2(action_.y() , action_.x()))/HIST_ALPHA);
        k = (k < H_b_.size())?  k  : 0;
        ROS_INFO("k: %d , H_b_(k) %d", k, H_b_(k));
        problem_initialized_= (H_b_(k)==0);
        making_step_= (H_b_(k)==0);

        if (making_step_ ){
            ROS_INFO("working on step: %d",current_step_);
            if (moveToPose(Fsr_, desF_, 0.4*V_MAX,  0.2*M_PI, 0.04)){
                making_step_= false;
                if(current_step_>=1)
                    current_step_-- ;
                else
                    problem_initialized_= false;
            }
        }
        else{
            moveToPose(KDL::Frame2::Identity(), KDL::Frame2::Identity(), V_MAX,  M_PI, 0.04);                   // stop
        }
    }
    else{
        ROS_INFO("almost in front of the puck");
        desF_ = Fsr_*Frg_;
        if(moveToPose(Fsr_, desF_, 0.3*V_MAX,  0.1*M_PI, 0.03)){
            if (pucks_frames_r_.back().p.x() < 1.5*SAFETY_DIST_TO_GOAL && std::abs(pucks_frames_r_.back().p.y()) < 3*PUCK_LOWER_RADIUS ){
                puck_in_front_ = true;
                ROS_INFO("Robot in front of the puck");
            }
            else  {
                puck_goal_is_set_ =false;                   // there was no puck at this position search for new goal
            }
            problem_initialized_= false;
        }
    }

    ROS_INFO("x_offset: %lf, y_offset: %lf, theta_offset: %lf", action_.x(), action_.y(), angles::to_degrees(action_.theta()));
    ROS_INFO("Frg_x: %lf, Frg_y: %lf, Frg_theta: %lf", Frg_.p.x(), Frg_.p.y(), angles::to_degrees(Frg_.M.GetRot()));
    ROS_INFO("Fsr_x: %lf, Fsr_y: %lf, Fsr_theta: %lf", Fsr_.p.x(), Fsr_.p.y(), angles::to_degrees(Fsr_.M.GetRot()));
    ROS_INFO("des_x: %lf, des_: %lf, des_theta: %lf", desF_.p.x(), desF_.p.y(), angles::to_degrees(desF_.M.GetRot()));
}

void PathPlanning::graspPuck(){
    /// SIM
    Fpuck_offset_ = KDL::Frame2(KDL::Rotation2(0.0), KDL::Vector2(ROBOT_RADIUS + PUCK_LOWER_RADIUS -0.02, 0.0));
    ROS_INFO("Frp_.x: %lf, Frp_.y: %lf, Frp_.theta: %lf", Frp_.p.x(), Frp_.p.y(), angles::to_degrees(Frp_.M.GetRot()));
    Frg_ = Frp_ *Fpuck_offset_.Inverse();
    desF_ = Fsr_*Frg_;
    if (moveToPose(Frg_.Inverse(), KDL::Frame2::Identity(), 0.2*V_MAX,  0.1*M_PI, 0.03)){
        puck_grasped_ = true;
        ROS_INFO("Grasping finished");
    }

}

void PathPlanning::goToFinal(){


    if (placed_puck_num_ == 0)
        Fpuck_offset_ = KDL::Frame2(KDL::Rotation2(0), KDL::Vector2(-ROBOT_RADIUS - PUCK_LOWER_RADIUS, 0.0 ));
    else if (placed_puck_num_ == 1)
        Fpuck_offset_ = KDL::Frame2(KDL::Rotation2(0), KDL::Vector2(-ROBOT_RADIUS - PUCK_LOWER_RADIUS, 4*PUCK_LOWER_RADIUS  + ROBOT_RADIUS));
    else if (placed_puck_num_ == 2)
        Fpuck_offset_ = KDL::Frame2(KDL::Rotation2(0), KDL::Vector2(-ROBOT_RADIUS - PUCK_LOWER_RADIUS, -4*PUCK_LOWER_RADIUS - ROBOT_RADIUS ));
    else
        Fpuck_offset_ = KDL::Frame2(KDL::Rotation2(0), KDL::Vector2(-ROBOT_RADIUS - PUCK_LOWER_RADIUS + 2*PUCK_LOWER_RADIUS , 0.0));

    Frg_ =Fmr_.Inverse()*Fmf_ *Fpuck_offset_;

    if (!problem_initialized_){
        ros::Time tic =ros::Time::now();                            // time consumed for creating of a problem and solution of a A*
        createProblemNew();

        if(problem_.goalFound()){
            actions_ = calcActionVector();
        }
        else{
            // actions_ = calcActionVectorAStar();
            actions_.clear();
            actions_.push_back(calcObstAvoidanceAction());
        }
        current_step_ = actions_.size()-1;
        problem_initialized_= true;
        ROS_INFO("toc: %lf",(ros::Time::now() - tic).toSec());
        ///DEBUG
        for (auto action : actions_){ROS_INFO("action: x: %lf, y %lf , theta %lf", action.x(), action.y(), action.theta());}
    }


    projectGoalInActWindow(non_zero_obj_ , Frg_, false);
    calcPolarHistograms(non_zero_obj_);
    calcCandidateDirection(H_b_);

    if(std::abs(Frg_.p.Norm())>= STEP_LENGTH){
        if (!making_step_){
            action_ = actions_[current_step_];
            desF_= Fsr_* KDL::Frame2( KDL::Rotation2(action_.theta()), KDL::Vector2(action_.x(), action_.y()));         // use odomemetry for steps measurments
        }

        int k = round(angles::normalize_angle_positive(std::atan2(action_.y() , action_.x()))/HIST_ALPHA);
        k = (k < H_b_.size())?  k  : 0;
        ROS_INFO("k: %d , H_b_(k) %d", k, H_b_(k));

        problem_initialized_= (H_b_(k)==0);
        making_step_= (H_b_(k)==0);

        if (making_step_){
            ROS_INFO("working on step: %d",current_step_);
            if (moveToPose(Fsr_, desF_, 0.4*V_MAX,  0.2*M_PI, 0.04)){
                making_step_= false;
                if(current_step_>=1)
                    current_step_-- ;
                else
                    problem_initialized_= false;;
            }
        }
        else{
            moveToPose(KDL::Frame2::Identity(), KDL::Frame2::Identity(), V_MAX,  M_PI, 0.04);                   // stop
        }

    }
    else{
        ROS_INFO("Almost on final");
        desF_ = Fsr_*Frg_;
        if(moveToPose(Fsr_, desF_, 0.3*V_MAX,  0.1*M_PI, 0.04)){
            placed_puck_num_ ++;
            bool_req_.request.data = true;
            ROS_INFO("Go to final square finished");
            if (!client_report_goal_.call(bool_req_))
                ROS_ERROR("Failed to call service report goal");            

            if (placed_puck_num_ == 3)
                    ROS_ERROR("Failed to call service report done");           

            Fsr_release_ =Fsr_;
            puck_placed_ = true;
            problem_initialized_= false;
        }
    }

    ROS_INFO("x_offset: %lf, y_offset: %lf, theta_offset: %lf", action_.x(), action_.y(), angles::to_degrees(action_.theta()));
    ROS_INFO("Fsr_x: %lf, Fsr_y: %lf, Fsr_theta: %lf", Fsr_.p.x(), Fsr_.p.y(), angles::to_degrees(Fsr_.M.GetRot()));
    ROS_INFO("des_x: %lf, des_: %lf, des_theta: %lf", desF_.p.x(), desF_.p.y(), angles::to_degrees(desF_.M.GetRot()));
    ROS_INFO("Frg_x: %lf, Frg_y: %lf, Frg_theta: %lf", Frg_.p.x(), Frg_.p.y(), angles::to_degrees(Frg_.M.GetRot()));
    ROS_INFO("des_x: %lf, des_: %lf, des_theta: %lf", desF_.p.x(), desF_.p.y(), angles::to_degrees(desF_.M.GetRot()));
}

void PathPlanning::releasePuck (){


    double x = 3*PUCK_LOWER_RADIUS* cos(Fsr_release_.M.GetRot() + M_PI);
    double y = 3*PUCK_LOWER_RADIUS* sin(Fsr_release_.M.GetRot() + M_PI);
    desF_ = Fsr_release_*KDL::Frame2(KDL::Rotation2(0.0),KDL::Vector2(x, y));
    //desF_ = KDL::Frame2(KDL::Rotation2(0.0), KDL::Vector2((ROBOT_RADIUS + 3*PUCK_RADIUS), 0.0));
    if (moveToPose(Fsr_, desF_, -0.1*V_MAX,  0.1*M_PI, 0.03)){
        puck_released_ = true;
        ROS_INFO("Releasing of the puck finished");
    }

    ROS_INFO("Fsr_.x: %lf, Fsr_.y: %lf, Fsr_.theta: %lf", Fsr_.p.x(), Fsr_.p.y(), angles::to_degrees(Fsr_.M.GetRot()));
    ROS_INFO("des_x: %lf, des_: %lf, des_theta: %lf", desF_.p.x(), desF_.p.y(), angles::to_degrees(desF_.M.GetRot()));
}

void PathPlanning::choicePuckGoal(){

    KDL::Frame2 Fmp;
    if (!pucks_frames_m_.empty() && placed_puck_num_ <3){
        chosen_puck_idx_ =0;
        double min_dist= 100.0;
        for (int i=0; i<pucks_frames_m_.size() ; i++){
            Fmp = pucks_frames_m_[i];
            KDL::Vector2 diff = Fmp.p - Fmf_.p;
            if (((std::abs(diff.x()) > 0.5*final_x_lenght_)  || (std::abs(diff.y()) > 0.5*final_y_lenght_ )) && std::abs((Fmp_.p-Fmp.p).Norm()) > 4*PUCK_LOWER_RADIUS ){
                if (min_dist > std::abs((Fmp.p - Fmr_.p).Norm())){
                     min_dist= std::abs((Fmp.p - Fmr_.p).Norm());
                     chosen_puck_idx_ =i;
                }
            }
        }
        Fmp_= pucks_frames_m_[chosen_puck_idx_];
        puck_goal_is_set_ =true;
        ROS_INFO("Puck goal is set");
    }
}


/// FROM KRIS

int PathPlanning::calcDeltaAngular(int c1, int c2){
    double min_angle =angles::shortest_angular_distance(c1 * HIST_ALPHA ,c2 * HIST_ALPHA);
    return round(std::abs(min_angle) / HIST_ALPHA);
}

// kc is always 0, kt is sector of the goal, kpre is sector of the previous step
double PathPlanning::calcPriCost(int c, int kc, int kt, int kpre){
    return MU1*calcDeltaAngular(c, kt)+MU2*calcDeltaAngular(c, kc)+MU3*calcDeltaAngular(c, kpre);
}

double PathPlanning::calcProCost(int c, int kt, int ke, int kc,  int cpre){
    return MU1_PH*std::max(calcDeltaAngular(c, kt), calcDeltaAngular(ke, kt))+ MU2_PH*calcDeltaAngular(c, kc)+MU3_PH*calcDeltaAngular(c, cpre);
}

double PathPlanning::calcHeuristic(int ke, int kt, int kc, int cpre){
    return MU1_PH*calcDeltaAngular(ke, kt)+MU2_PH*calcDeltaAngular(kt, kc)+MU3_PH*calcDeltaAngular(kt, cpre);
}

// if goal can be reached in NUM_STEPS, use this
std::vector<Action> PathPlanning::calcActionVector(){
    std::vector<Action> v_action;
    std::vector<int> curr = problem_.getGoalState();
    v_action.push_back(problem_.getNode(curr).getAction());    // new line

    // iterate till the root node
    while(problem_.getParent(curr)!= problem_.getStartState()){
        std:: vector<int> parent = problem_.getParent(curr);
        v_action.push_back(problem_.getNode(parent).getAction());
        curr = parent;
    }
    return v_action;

}

// if goal can't be reached in NUM_STEPS, use this
std::vector<Action> PathPlanning::calcActionVectorAStar(){
    std::vector<double> path_cost;                                      // cost vector contains the total cost for each trajectory
    std::vector<std::vector<Action> > v_action;
    std::vector<std::vector<int> > v_last_layer_state = problem_.getLayer(NUM_STEPS-1);
    for(int i=0; i<v_last_layer_state.size(); i++){
        std::vector<int> s = v_last_layer_state[i];
        double total_cost = 0;
        std::vector<Action> v_action_branch;
        while(s!=problem_.getStartState()){
            v_action_branch.push_back(problem_.getNode(s).getAction()); // add the action to the corresponding vector
            total_cost+=problem_.getNode(s).getCost();					// sum the cost and heuristic
            total_cost+=problem_.getNode(s).getHeuristic();
            s = problem_.getParent(s);
        }
        v_action.push_back(v_action_branch);
        path_cost.push_back(total_cost);                                // store the total cost for one trajectory
    }
    // return the action sequence with minimal total cost
    return v_action[distance(begin(path_cost), min_element(path_cost.begin(),path_cost.end()))];
}
