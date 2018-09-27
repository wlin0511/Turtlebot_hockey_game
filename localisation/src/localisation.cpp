#include "localisation.h"
#include "position.h"

Localisation::Localisation(){}

bool Localisation::colorCallBack(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response&){
    if (req.data == 0){
        ROS_INFO("real color is yellow");
        work_with_blue_pucks_ = false;
    }
    else
        ROS_INFO("real color is blue");
    return true;
}

bool Localisation::detectionStartCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response&){
    if (req.data == 1){
        ROS_INFO("Detection started");
        turning_done = true;
    }
    return true;
}

////// callback to get ab information
bool Localisation::abCallBack(communication::ab::Request& req, communication::ab::Response&){
    ROS_INFO("a=%f, b=%f", req.a, req.b);
    a_value = req.a;
    b_value = req.b;
    return true;
}

bool Localisation::isInField(KDL::Frame2 &input){
    bool test = false;
    if(input.p.x() < 3*a_value - FIELD_TOLERANCE && input.p.y() < b_value -FIELD_TOLERANCE && input.p.x() > FIELD_TOLERANCE && input.p.y() > FIELD_TOLERANCE){
        test =true;
    }
    return test;
}

void Localisation::getPuckGrasped(const std_msgs::Bool &msg)
{
    puck_grasped = msg.data;
}

////****************** color part ******************//

//


bool Localisation::init(ros::NodeHandle &nh){
    ROS_INFO("Initialisation of localisation node..");
    // Ros node handle for the class
    turning_done = false;
    poseCb_ok = false;
    work_with_blue_pucks_ = false;                 //true: blue pucks; false: yellow pucks
    color_calculated = false;
    a_value_confirmed = false;
    b_value_confirmed = false;
    posts_stored = false;
    color_calculated = false;
    gotMapFrame = false;
    isAlive.data = false;
    tellEgoPos = false;
    position_reported = false;
    turning_done = false;
    reported_posts_stored = false;
    puck_grasped = false;

    //position_reported = false;

    nh_ = nh;

    // Initialize all subcribers, services and publishers
    subTurtleOdom = nh_.subscribe("/odom", 10, &Localisation::getOdometry, this);
    subObjects = nh_.subscribe("/object_recognition/objects_poses", 10, &Localisation::poseObjectsCb, this);
    pubObjects = nh_.advertise<localisation::objects_poses>("/localisation/objects_poses", 10);
    srv_turning_done = nh_.advertiseService("/localisation/turning_done", &Localisation::turningDone, this);
    client_posts_stored = nh_.serviceClient<std_srvs::SetBool>("/path_planning/posts_stored");
    client_map_dimensions = nh_.serviceClient<localisation::map_dimensions>("/path_planning/set_map_dimensions");
    posPub = nh_.advertise<communication::pos>("pos_info", 1);
    setColorClient = nh_.serviceClient<std_srvs::SetBool>("set_color");     // client to send the required 0: yellow  1: blue
    getColorServer = nh_.advertiseService("/localisation/send_color", &Localisation::colorCallBack, this); // server to get color
    checkRatioClient = nh_.serviceClient<communication::ratio>("check_ratio");     // client asks to check the ratio
    abServer = nh_.advertiseService("ab_information", &Localisation::abCallBack, this);     // server to get ab
    alivePub = nh_.advertise<std_msgs::Bool>("localisation/localisation_alive", 10);
    subPuckGrasbed = nh_.subscribe("/path_planning/puck_grasped", 10, &Localisation::getPuckGrasped, this);
    detectionServer = nh_.advertiseService("/localisation/start_detection", &Localisation::detectionStartCb, this);     // get detection start

    pubTimer_ = nh_.createTimer(ros::Duration(1), &Localisation::publishCommand, this);                         // timer for publisher



    ROS_INFO ("Localisation is initialized");

    return true;
}//END_UPDATE//


// Timer
void Localisation::publishCommand(const ros::TimerEvent&){
    if(color_calculated && gotMapFrame && turning_done){
        posPub.publish(egoPos);
    }

}


/* :::::::::UPDATE FUNCTION:::::::
 * ::::::::::::::::::::::::::::::: */
void Localisation::update(const ros::Time& time, const ros::Duration& period){
    isAlive.data = true;
    alivePub.publish(isAlive);

    odom_received_= ((ros::Time::now()- safety_ton_odom_).toSec()< 2.0)? true : false;

    // filter duplicate poste poses from input buffer and save to Postes Vector
    if(poste_wrt_odom_vec_tmp.size() > 2){
        poste_wrt_odom_vec_saved = filterPostePoses(poste_wrt_odom_vec_tmp, poste_wrt_odom_vec_saved);
    }

    // filter duplicate puck poses from input buffer and save to pucks Vector
    filterPuckPoses(yellowPucks_wrt_odom_vec_tmp, yellowPucks_wrt_odom_vec_saved);
    filterPuckPoses(bluePucks_wrt_odom_vec_tmp, bluePucks_wrt_odom_vec_saved);

    // std::cout << "The size of YELLOW puck vector:" << yellowPucks_wrt_odom_vec_saved.size() << std::endl;

    //    std::cout << "The size of BLUE puck vector:" << bluePucks_wrt_odom_vec_saved.size() << std::endl;

    //        for(int i = 0 ; i< bluePucks_wrt_odom_vec_saved.size(); i++){
    //            std::cout << "THE VALUE FOR BLUE X: " << bluePucks_wrt_odom_vec_saved.at(i).p(0) << std::endl;
    //            std::cout << "THE VALUE FOR BLUE Y: " << bluePucks_wrt_odom_vec_saved.at(i).p(1) << std::endl;
    //            std::cout << std::endl;

    //        }

    // publish puck
    if(!color_calculated && turning_done && yellowPucks_wrt_odom_vec_saved.size() > 2 && bluePucks_wrt_odom_vec_saved.size() > 2){
        color_calculated = getMyColor(bluePucks_wrt_odom_vec_saved, yellowPucks_wrt_odom_vec_saved);
    }
    if(color_calculated && DEBUG == true && !color_reported && turning_done){
        std::cout << "Color calculated! Our color is: ";
        if(work_with_blue_pucks_){
            std::cout << "BLUE" << std::endl;
            Fsf_.p = KDL::Vector2(2*a_value + a_value/2, 0);    //was (1/2)*a_value
            Fsf_.M.SetRot(0.0);
        }else{
            std::cout << "YELLOW" << std::endl;
            Fsf_.p = KDL::Vector2(2*a_value + a_value/2, 0);
            Fsf_.M.SetRot(0.0);
        }
        Fsf_.p = KDL::Vector2(2*a_value, 0);
        Fsf_.M.SetRot(0.0);
    }

    //get a_value
    if(!a_value_confirmed && poste_wrt_odom_vec_tmp.size() > 2)
        get_aValue(poste_wrt_odom_vec_tmp);

    //get b_value
    if(a_value_confirmed && !b_value_confirmed)
        get_bValue(poste_wrt_odom_vec_saved);

    // report map dimensions to path_planning
    if(a_value_confirmed && b_value_confirmed && posts_stored && !abRatio_sent && turning_done){
        localisation::map_dimensions tmp;
        tmp.request.x_lenght = a_value * 3;
        tmp.request.y_lenght = b_value;

        client_map_dimensions.call(tmp);

        srv_ratio.request.ratio = a_value / b_value;

        if (checkRatioClient.call(srv_ratio)){
            ROS_INFO("Read Ratio Sucess");
            abRatio_sent = true;
        }
        else{
            ROS_WARN("Read Ratio Error");
        }
    }

    // report color
    if(color_calculated && !color_reported && turning_done){
        srv_color.request.data = work_with_blue_pucks_;

        if (srv_color.request.data == false){
            ROS_INFO("Required Color Is Yellow");
        }
        else
            ROS_INFO("Required Color Is Blue");

        if (setColorClient.call(srv_color)){
            ROS_INFO("Get Color Success");
            color_reported = true;
        }
        else{
            ROS_ERROR("Get Color Error");
        }
    }

    //get map frame:
    if(color_calculated && work_with_blue_pucks_){
        Fms_.M.SetRot(0.0);
        Fms_.p = KDL::Vector2( a_value / 2, b_value/2);
        gotMapFrame = true;
    }else if(color_calculated && !work_with_blue_pucks_){
        Fms_.M.SetRot(PI);
        Fms_.p = KDL::Vector2( (3*a_value - (a_value/2)), b_value/2);
        gotMapFrame = true;
    }


    //tell path_planning starting position
    if(color_calculated && gotMapFrame && turning_done){
        getGlobalPosition();
    }

    if(turning_done && !reported_posts_stored){
        poste_wrt_odom_vec_saved = filterPostePoses(poste_wrt_odom_vec_tmp, poste_wrt_odom_vec_saved);
        bool_req_.request.data = a_value_confirmed && b_value_confirmed;
        if(!client_posts_stored.call(bool_req_)){
            ROS_WARN("Service posts_stored doesn't respond.");
        }else{
            ROS_INFO("posts_stored sent!");
        }
        reported_posts_stored = true;
    }
}



void Localisation::poseObjectsCb(const localisation::objects_poses &msg){
    State state = searching;

    poste_wrt_odom_vec_tmp.clear();
    bluePucks_wrt_odom_vec_tmp.clear();
    yellowPucks_wrt_odom_vec_tmp.clear();
    for(int i = 0; i < msg.green_post_poses.size(); i++){
        Frp_.M.SetRot(0.0);
        Frp_.p = KDL::Vector2(msg.green_post_poses.at(i).x, msg.green_post_poses.at(i).y);
        Fsp_ = Fsr_ * Frp_;
        poste_wrt_odom_vec_tmp.push_back(Fsp_);
        /* DEBUG ROS_INFO("Fsp_.x %lf, Fsp_.y %lf , Fsp_.theta %lf",Fsp_.p.x(), Fsp_.p.y(), Fsp_.M.GetRot());
         *
         *
         *
        */

    }
    for(int i = 0; i < msg.blue_puck_poses.size(); i++){
        Frp_.M.SetRot(0.0);
        Frp_.p = KDL::Vector2(msg.blue_puck_poses.at(i).x, msg.blue_puck_poses.at(i).y);
        Fsp_ = Fsr_ * Frp_;
        bluePucks_wrt_odom_vec_tmp.push_back(Fsp_);
        /* DEBUG ROS_INFO("Fsp_.x %lf, Fsp_.y %lf , Fsp_.theta %lf",Fsp_.p.x(), Fsp_.p.y(), Fsp_.M.GetRot());
         *
         *
         *
        */
    }
    for(int i = 0; i < msg.yellow_puck_poses.size(); i++){
        Frp_.M.SetRot(0.0);
        Frp_.p = KDL::Vector2(msg.yellow_puck_poses.at(i).x, msg.yellow_puck_poses.at(i).y);
        Fsp_ = Fsr_ * Frp_;
        yellowPucks_wrt_odom_vec_tmp.push_back(Fsp_);
        /* DEBUG ROS_INFO("Fsp_.x %lf, Fsp_.y %lf , Fsp_.theta %lf",Fsp_.p.x(), Fsp_.p.y(), Fsp_.M.GetRot());
         *
         *
         *
        */
    }
    poseCb_ok = true;
}

void Localisation::getOdometry(const nav_msgs::Odometry &msg){
    KDL::Rotation Rot = KDL::Rotation::Quaternion( msg.pose.pose.orientation.x,
                                                   msg.pose.pose.orientation.y,
                                                   msg.pose.pose.orientation.z,
                                                   msg.pose.pose.orientation.w);

    // data forFsr_
    Fsr_.p= KDL::Vector2(msg.pose.pose.position.x, msg.pose.pose.position.y);
    double alfa, beta, gama;
    Rot.GetEulerZYX(alfa, beta, gama);
    Fsr_.M.SetRot(alfa);

    odom_received_ = true;
    safety_ton_odom_ = ros::Time::now();
}



std::vector<KDL::Frame2> Localisation::filterPostePoses(std::vector<KDL::Frame2> poste_wrt_odom_vec_tmp, std::vector<KDL::Frame2> &poste_wrt_odom_vec_saved){
    if(poste_wrt_odom_vec_saved.empty()){
        poste_wrt_odom_vec_saved = poste_wrt_odom_vec_tmp;
        /* Earlier version for case : empty poste position vector wrt robot base frame
        for(int i = 0; i < poste_wrt_odom_vec_tmp.size(); i++){
            poste_wrt_odom_vec_saved.push_back(poste_wrt_odom_vec_tmp.at(i));
            for(int j = 0; j < poste_wrt_odom_vec_tmp.size(); j++){
                if(std::abs(std::abs(poste_wrt_odom_vec_saved.at(i).p.Norm()) - std::abs(poste_wrt_odom_vec_tmp.at(j).p.Norm())) <= POSTE_UNCERTAINTY){
                    tmp_.p = (poste_wrt_odom_vec_tmp.at(j).p + poste_wrt_odom_vec_saved.at(i).p / 2.0);
                    poste_wrt_odom_vec_saved.at(i).p = tmp_.p;
                }
            }
        }
        */

        /* DEBUG
        ROS_INFO("TEMPORARY VECTOR COPIED TO SAVED POSTE VECTOR.");
        */

        return poste_wrt_odom_vec_saved;
    }else{
        for(int i = 0; i < poste_wrt_odom_vec_tmp.size(); i++){
            poste_already_exists = false;
            for(int j = 0; j < poste_wrt_odom_vec_saved.size(); j++){
                double dPoleDist = getKDLDistance(poste_wrt_odom_vec_saved.at(j), poste_wrt_odom_vec_tmp.at(i));
                if(dPoleDist < POSTE_UNCERTAINTY){
                    tmp_.p = (poste_wrt_odom_vec_tmp.at(i).p + poste_wrt_odom_vec_saved.at(j).p) / 2.0;
                    poste_wrt_odom_vec_saved.at(j).p = tmp_.p;
                    poste_already_exists = true;
                    /* DEBUG
                    std::cout << "POSTE ALREADY EXISTS! Values: i = " << i << ", j = " << j << ". Exists-Flag: " << poste_already_exists << std::endl;
                    */
                }
            }
            if(poste_already_exists == false){
                poste_wrt_odom_vec_saved.push_back(poste_wrt_odom_vec_tmp.at(i));
                /* DEBUG
                std::cout << "NEW POSTE DETECTED: i = " << i <<  ". Exists-Flag: " << poste_already_exists << std::endl;
                */

            }
        }
        if(poste_wrt_odom_vec_saved.size() > 14){
            posts_stored = true;
        }
        return poste_wrt_odom_vec_saved;
    }

    //set flag
    //TODO
    //return poste_wrt_odom_vec_saved;s
}

void Localisation::filterPuckPoses(std::vector<KDL::Frame2> pucks_wrt_odom_vec_tmp, std::vector<KDL::Frame2> &puck_wrt_odom_vec_saved){
    if(puck_wrt_odom_vec_saved.empty() && !pucks_wrt_odom_vec_tmp.empty()){
        puck_wrt_odom_vec_saved.push_back(pucks_wrt_odom_vec_tmp.back());
    }else if(!pucks_wrt_odom_vec_tmp.empty()){
        puck_already_exists = false;
        for(int j = 0; j < puck_wrt_odom_vec_saved.size(); j++){
            double dPoleDist = getKDLDistance(puck_wrt_odom_vec_saved.at(j), pucks_wrt_odom_vec_tmp.back());
            if(dPoleDist < PUCK_UNCERTAINTY){
                if(!puck_grasped){
                    puck_wrt_odom_vec_saved.at(j).p = (pucks_wrt_odom_vec_tmp.back().p + puck_wrt_odom_vec_saved.at(j).p) / 2.0;
                }else{
                    puck_wrt_odom_vec_saved.at(j).p = pucks_wrt_odom_vec_tmp.back().p;
                }
                puck_already_exists = true;
                /* DEBUG
                    std::cout << "PUCK ALREADY EXISTS! Values: i = " << i << ", j = " << j << ". Exists-Flag: " << puck_already_exists << std::endl;
                    */
            }
            /* DEBUG
                std::cout << "NEW PUCK DETECTED: i = " << i <<  ". Exists-Flag: " << puck_already_exists << std::endl;
                */
        }

        if(!puck_already_exists){
            puck_wrt_odom_vec_saved.push_back(pucks_wrt_odom_vec_tmp.back());
        }
    }

}



/*
void Localisation::getPositionInMap(){
    int vectorLength = poste_wrt_odom_vec_tmp.size();
    double dist1 = 0.0;
    double dist2 = 0.0;
    double dist3 = 0.0;

    for(int i = 0; i < vectorLength; ++i){
        // reducing the possible combinations
        for(int j = i + 1; j < vectorLength; ++j){
            /*dist1 = Localisation::getDistance(poste_wrt_odom_vec_tmp.at(i).p(0), poste_wrt_odom_vec_tmp.at(i).p(1),
                                              poste_wrt_odom_vec_tmp.at(j).p(0), poste_wrt_odom_vec_tmp.at(j).p(1));
            for(int k = 0; k < vectorLength; ++k){
            }
        }
    }
}
*/


double Localisation::getDistance(KDL::Frame2 &object){
    double vecNorm =sqrt(object.p(0)*object.p(0) + object.p(1)*object.p(1));
    return vecNorm;
}

// get position of objects, robot and final goal w.r.t. map frame and publish data
void Localisation::getGlobalPosition(){
    localisation::objects_poses robot_pose;
    geometry_msgs::Pose2D puck_pose_tmp;

    robot_pose.blue_puck_poses.clear();
    robot_pose.yellow_puck_poses.clear();


    Fmr_ = Fms_ * Fsr_;
    Fmf_ = Fms_ * Fsf_;
    robot_pose.robot_pose.x = Fmr_.p.x();
    robot_pose.robot_pose.y = Fmr_.p.y();
    robot_pose.robot_pose.theta = Fmr_.M.GetRot();

    if(work_with_blue_pucks_ == 1){
        robot_pose.final_pose.x = Fmf_.p.x() + FINAL_CORRECTION;
    }else{
        robot_pose.final_pose.x = Fmf_.p.x() - FINAL_CORRECTION;

    }
    robot_pose.final_pose.y = Fmf_.p.y();
    robot_pose.final_pose.theta = Fmf_.M.GetRot();

    for(int i = 0; i < bluePucks_wrt_odom_vec_saved.size(); i++){
        Fsp_ = bluePucks_wrt_odom_vec_saved.at(i);
        KDL::Frame2 Fmp_ = Fms_ * Fsp_;
        if(isInField(Fmp_)){
            puck_pose_tmp.x = Fmp_.p.x();
            puck_pose_tmp.y = Fmp_.p.y();
            puck_pose_tmp.theta = Fmp_.M.GetRot();
            robot_pose.blue_puck_poses.push_back(puck_pose_tmp);
        }
    }

    for(int i = 0; i < yellowPucks_wrt_odom_vec_saved.size(); i++){
        Fsp_ = yellowPucks_wrt_odom_vec_saved.at(i);
        KDL::Frame2 Fmp_ = Fms_ * Fsp_;
        if(isInField(Fmp_)){
            puck_pose_tmp.x = Fmp_.p.x();
            puck_pose_tmp.y = Fmp_.p.y();
            puck_pose_tmp.theta = Fmp_.M.GetRot();
            robot_pose.yellow_puck_poses.push_back(puck_pose_tmp);
        }
    }


    pubObjects.publish(robot_pose);
    position_reported = true;



    if(!tellEgoPos){
        //tell communication ego Position for start
        egoPos.x = robot_pose.robot_pose.x;
        egoPos.y = b_value - robot_pose.robot_pose.y;
        tellEgoPos = false;  // for continiously updating the robot's position in the map for angelina
    }

    //std::cout << "Robot in map, x-coordinate: " << robot_pose.robot_pose.x << std::endl;
    //std::cout << "Robot in map, y-coordinate: " << robot_pose.robot_pose.y << std::endl;

}

double Localisation::getKDLDistance(KDL::Frame2 &posteSaved, KDL::Frame2 &posteTmp){
    double dx = posteSaved.p(0) - posteTmp.p(0);
    double dy = posteSaved.p(1) - posteTmp.p(1);
    double vecNorm = sqrt(dx*dx + dy*dy);
    return vecNorm;
}

void Localisation::get_aValue(std::vector<KDL::Frame2> &polesInField){
    double a, ratio, d1, d2, tmp1, tmp2;
    bool success = false;
    for(int i = 0; i < polesInField.size() - 2; i++){
        d1 = getKDLDistance(polesInField.at(i), polesInField.at(i+1));
        d2 = getKDLDistance(polesInField.at(i+1), polesInField.at(i+2));
        ratio = d2/d1;
        // case 1
        if(ratio-PDIST_UNCERTAINTY < D2_D1 && ratio+PDIST_UNCERTAINTY > D2_D1){
            tmp1 = d1 * 4.0;
            tmp2 = d2 * 4.0/3.0;
            a_vector.push_back(tmp1);
            a_vector.push_back(tmp2);
            success = true;
        }
        // case 2
        if(ratio-PDIST_UNCERTAINTY < D3_D2 && ratio+PDIST_UNCERTAINTY > D3_D2){
            tmp1 = d1 * 4.0/3.0;
            tmp2 = d2 * 2.0;
            a_vector.push_back(tmp1);
            a_vector.push_back(tmp2);
            success = true;
        }
        // case 3
        if(ratio-PDIST_UNCERTAINTY < D4_D3 && ratio+PDIST_UNCERTAINTY > D4_D3){
            tmp1 = d1 * 2.0;
            tmp2 = d2 * 2.0;
            a_vector.push_back(tmp1);
            a_vector.push_back(tmp2);
            success = true;
        }

        // case 4
        if(ratio-PDIST_UNCERTAINTY < D5_D4 && ratio+PDIST_UNCERTAINTY > D5_D4){
            tmp1 = d1 * 2.0;
            tmp2 = d2 * 4.0/3.0;
            a_vector.push_back(tmp1);
            a_vector.push_back(tmp2);
            success = true;
        }
        // case 5
        if(ratio-PDIST_UNCERTAINTY < D6_D5 && ratio+PDIST_UNCERTAINTY > D6_D5){
            tmp1 = d1 * 4.0/3.0;
            tmp2 = d2 * 4.0;
            a_vector.push_back(tmp1);
            a_vector.push_back(tmp2);
            success = true;
        }
    }

    /* OLD
    // calculate mean for a
    if(success){
        tmp1 = 0.0;
        for(int i = 0; i < a_vector.size(); i++){
            tmp1 = tmp1 + a_vector.at(i);
        }
        a = tmp1/a_vector.size();
        a_value = a;
        // DEBUG
    }*/

    if(a_vector.size() > A_VALUE_ACCURACY){
        // calculate mean for a
        tmp1 = 0.0;
        for(int i = 0; i < a_vector.size(); i++){
            tmp1 = tmp1 + a_vector.at(i);
        }
        a = tmp1/a_vector.size();
        a_value = a;
        a_value_confirmed = true;
        std::cout << "FOUND A. VALUE: " << a_value << std::endl;
        a_vector.clear();
    }

}


void Localisation::get_bValue(std::vector<KDL::Frame2> &polesInField){
    //define temporary variables
    std::vector<double> distances;
    std::vector<double> distancesFiltered;
    double tmp;

    for(int i = 0; i < polesInField.size(); i++){
        if(polesInField.at(i).p(1) > 0){
            poste_pos_y.push_back(polesInField.at(i));
        }else{
            poste_neg_y.push_back((polesInField.at(i)));
        }
    }
    for(int i = 0; i < poste_pos_y.size(); i++){
        for(int j = i+1; j < poste_neg_y.size(); j++){
            tmp = getKDLDistance(poste_pos_y.at(i), poste_neg_y.at(j));
            distances.push_back(tmp);

        }
    }

    //calculating minimal distances with condition b > a
    double min = 0.0;
    for(int i = 0; i < distances.size(); i++){
        if(distances.at(i) > 2*a_value){
            distancesFiltered.push_back(distances.at(i));
        }
    }
    //calculate b_value from filtered distances vector
    tmp = 1000.0;
    for(int i = 0; i < distancesFiltered.size(); i++){
        if(distancesFiltered.at(i) < tmp){
            tmp = distancesFiltered.at(i);
        }
    }
    if(tmp > a_value && tmp < 5*a_value){
        b_value = tmp;
        b_value_confirmed = true;
        std::cout << "Value for b is: " << b_value << "." << std::endl;
    }else{
        b_value = 2.93134515;      // this case shouldnt happen and should be removed
        std::cout << "Value for b is: " << b_value << "." << std::endl;
        b_value_confirmed = true;
    }
}

bool Localisation::getMyColor(std::vector<KDL::Frame2> &bluePucks_wrt_odom_vec_saved, std::vector<KDL::Frame2> &yellowPucks_wrt_odom_vec_saved){
    std::vector<double> blueDistances;
    std::vector<double> yellowDistances;
    double tmp;
    double minBlue = 3.0;
    double minYellow = 3.0;

    for(int i = 0; i < bluePucks_wrt_odom_vec_saved.size(); i++){
        tmp = getDistance(bluePucks_wrt_odom_vec_saved.at(i));
        if(tmp < minBlue && tmp > 0.5)
            minBlue = tmp;
        blueDistances.push_back(tmp);
    }
    for(int i = 0; i < yellowPucks_wrt_odom_vec_saved.size(); i++){
        tmp = getDistance(yellowPucks_wrt_odom_vec_saved.at(i));
        if(tmp < minYellow && tmp > 0.5)
            minYellow = tmp;
        yellowDistances.push_back(tmp);
    }
    if(minBlue < minYellow){
        work_with_blue_pucks_ = true;
        std::cout << "The distance of the nearest BLUE puck was: " << minBlue << "." << std::endl;
        return true;
    }else if(minYellow < minBlue){
        work_with_blue_pucks_ = false;
        std::cout << "The distance of the nearest BLUE puck was: " << minYellow << "." << std::endl;
        return true;
    }else
        return false;
}

bool Localisation::turningDone(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    turning_done = req.data;
    if (turning_done) ROS_INFO("Received service: turning is done.");
    else ROS_INFO("Received message: turning..");
    res.success =true;
    res.message = std::string("Turning was successfully done.");
    return true;
}

void Localisation::getMapppingData(const localisation::objects_poses &msg){
    Fmr_ = KDL::Frame2(KDL::Rotation2(msg.robot_pose.theta), KDL::Vector2(msg.robot_pose.x, msg.robot_pose.y));
    Fmf_ = KDL::Frame2(KDL::Rotation2(msg.final_pose.theta), KDL::Vector2(msg.final_pose.x, msg.final_pose.y));
    pucks_frames_m_.clear();
    if (work_with_blue_pucks_){
        for(auto pose : msg.blue_puck_poses)
            pucks_frames_m_.push_back(KDL::Frame2(KDL::Rotation2(pose.theta), KDL::Vector2(pose.x, pose.y) ));

    }
    else{
        for(auto pose : msg.yellow_puck_poses)
            pucks_frames_m_.push_back(KDL::Frame2(KDL::Rotation2(pose.theta), KDL::Vector2(pose.x, pose.y) ));
    }

    if (!map_received_)
        map_received_ = true;
    safety_ton_mapping_=ros::Time::now();
}


void Localisation::getRecognitionData(const localisation::objects_poses &msg){
    pucks_frames_m_.clear();
    if (work_with_blue_pucks_){
        for(auto pose : msg.blue_puck_poses)
            pucks_frames_r_.push_back(KDL::Frame2(KDL::Rotation2(pose.theta), KDL::Vector2(pose.x, pose.y) ));
    }
    else{
        for(auto pose : msg.yellow_puck_poses)
            pucks_frames_r_.push_back(KDL::Frame2(KDL::Rotation2(pose.theta), KDL::Vector2(pose.x, pose.y) ));
    }

    if (!recognition_received_)
        recognition_received_ =true;
    safety_ton_recognition_=ros::Time::now();

}

