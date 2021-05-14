#include <CoherentD.h>
#include <pluginlib/class_list_macros.h>

namespace coherentD
{
/*onInit() */
void CoherentD::onInit() {
    /* set flags to false */
    is_initialized_ = false;

    ros::NodeHandle nh("~");

    ros::Time::waitForValid();

    mrs_lib::ParamLoader param_loader(nh, "CoherentD");

    /* load parameters */
    param_loader.loadParam("uav_name", _this_uav_name_);
    param_loader.loadParam("uav/uav_names", _uav_names_);
    param_loader.loadParam("uav/localization_distance", localization_distance);
    param_loader.loadParam("uav/max_distance_in_one_step", dist_max_one_step);
    param_loader.loadParam("uav/avoid_distance", avoid_distance);

    param_loader.loadParam("controller/alpha", alpha);
    param_loader.loadParam("controller/coherence_loops", coherence_loops);

    
    param_loader.loadParam("goal/sub_goal", tmp_goals);
    param_loader.loadParam("goal/sub_goal_change_distance", distance_goal_change);

    param_loader.loadParam("altitude/constant", maintain_constant_altitude);
    param_loader.loadParam("altitude/value", altitude_constant_value);
    param_loader.loadParam("altitude/lower_altitude", altitude_lower_value);
    param_loader.loadParam("altitude/upper_altitude", altitude_upper_value);

    param_loader.loadParam("sectors/margins", margin); 

    param_loader.loadParam("loop/rate", run_rate);

    param_loader.loadParam("debug", debug_mode);


    if (!param_loader.loadedSuccessfully()){
        ROS_ERROR("[Coherent - ERROR]: failed to load non-optional parameters!");
        ros::shutdown();
    }

    /* subscribes */

    sub_uav_odom = nh.subscribe<nav_msgs::Odometry>("/" + _this_uav_name_ + "/odometry/odom_main", 1, 
                                                                &CoherentD::callbackUAVOdometry, this);

    sub_position_cmd_ = nh.subscribe<mrs_msgs::PositionCommand>("/" + _this_uav_name_ + "/control_manager/position_cmd", 1, 
                                                                &CoherentD::callbackPositionCMD, this);

    sub_uav_uvdar = nh.subscribe<mrs_msgs::PoseWithCovarianceArrayStamped>("/" + _this_uav_name_ + 
                                    "/uvdar/filteredPoses", 1, &CoherentD::callbackNeighborsUVDAR, this);

    sub_uav_rplidar = nh.subscribe<mrs_msgs::ObstacleSectors>("/" + _this_uav_name_ + 
                                "/bumper/obstacle_sectors", 1,&CoherentD::callbackObstacleLIDAR, this);
    
    /* timers */
    timer_pub_coherent_ = nh.createTimer(ros::Rate(run_rate), &CoherentD::callbackUAVCoherence, this);

    /* service */
    srv_client_goto_ = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("/" + _this_uav_name_ + 
                                                                        "/control_manager/reference");
    
    /* Initializing the swarm */
    ROS_INFO_ONCE("[Coherent]: initialized");
    ros::console::Level logger_level = (debug_mode) ? ros::console::levels::Debug : ros::console::levels::Info;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, logger_level);
    way_point = rotate_2d(Eigen::Vector3d(1, 0, 0), random_number(0, 2 * M_PI));
    ROS_DEBUG("[Coherent - DEBUG]: Random initial waypoint: [%0.2f, %0.2f, %0.2f]", way_point[0], way_point[1], way_point[2]);
    is_initialized_ = true;
    goal();
    set_margin();
    ROS_INFO_ONCE("[Coherent]: Goal initialized");
    ros::spin();    
}

/*

    | ---------------------- subscriber callbacks ------------------------- |

*/

/* callbackUAVOdometry - odom of /odometry/odom_main*/
void CoherentD::callbackUAVOdometry(const nav_msgs::Odometry::ConstPtr& odom) {
    if (!is_initialized_) {
        return;
    }

    // Put this uav pose in Eigen variable
    current_position[0] = odom->pose.pose.position.x;
    current_position[1] = odom->pose.pose.position.y;
    current_position[2] = odom->pose.pose.position.z;
    ROS_DEBUG("[Coherent]: Current pose: %f, %f, %f", current_position[0], current_position[1],current_position[2]);
    
}

/* callbackPositionCMD - odom of /control_manager/position_cmd*/
void CoherentD::callbackPositionCMD(const mrs_msgs::PositionCommand::ConstPtr& pose_cmd) {
    if (!is_initialized_) {
        return;
    }

    // Put this uav pose in Eigen variable
    this_pose_cmd[0] = pose_cmd->position.x;
    this_pose_cmd[1] = pose_cmd->position.y;
    this_pose_cmd[2] = pose_cmd->position.z;
    ROS_DEBUG("[Coherent]: Current pose: %f, %f, %f", this_pose_cmd[0], this_pose_cmd[1],this_pose_cmd[2]);
    
}

/* callbackNeighborsUVDAR - pose and number of neighbors using UVDAR*/
void CoherentD::callbackNeighborsUVDAR(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr& array_poses) {
    if (!is_initialized_){
        return;
    }

    {
    std::scoped_lock lock(mutex_neighbors_pose);

        /* array for take the pose of every neighbor */
        if (!array_poses->poses.empty()){

            for (unsigned int i = 0; i < array_poses->poses.size(); i++) {

                /* create Eigen3d vector */
                Eigen::Vector3d             eigen_pose;
                
                eigen_pose[0] = array_poses->poses[i].pose.position.x;
                eigen_pose[1] = array_poses->poses[i].pose.position.y;
                eigen_pose[2] = array_poses->poses[i].pose.position.z;

                /* save estimated position */
                const unsigned int uav_id = array_poses->poses[i].id;

                if (neighbors_pose.find(uav_id) == neighbors_pose.end()) {
                    neighbors_pose.insert(std::pair<int, Eigen::Vector3d>(uav_id, eigen_pose));
                    neighbors_range.insert(std::pair<int, double>(uav_id, dist(current_position, eigen_pose)));
                } 
                else {
                    neighbors_pose[uav_id] = eigen_pose;
                    neighbors_range[uav_id] = dist(current_position, eigen_pose);
                }

            } // for (unsigned int i = 0; i < array_poses->poses.size(); i++)

        } // if (!array_poses->poses.empty())
        else{
            neighbors_pose.clear();
            neighbors_range.clear();

        }
    } // std::scoped_lock lock(mutex_neighbors_position_)

}

/* callbackObstacleLIDAR - take info about anything close using RpLidar*/
void CoherentD::callbackObstacleLIDAR(const mrs_msgs::ObstacleSectors::ConstPtr& scan){
    if (!is_initialized_) {
        return;
    }
    
    sector[0] = scan->sectors[0];
    sector[1] = scan->sectors[1];
    sector[2] = scan->sectors[2];
    sector[3] = scan->sectors[3];
    sector[4] = scan->sectors[4];
    sector[5] = scan->sectors[5];
    sector[6] = scan->sectors[6];
    sector[7] = scan->sectors[7];     
}

/*

    | ---------------------- timers callbacks ------------------------- |

*/

/* callbackUAVCoherence - main code of swarm*/
void CoherentD::callbackUAVCoherence(const ros::TimerEvent& event){
    
    ROS_INFO("[Coherent - DEBUG]: Counter: %d, coherence loop: %d.", counter, coherence_loops);
    
    /* Checking current goal */
    double coherence_goal_offset = 0;
    Eigen::Vector3d goal_ = sub_goals[current_goal_index];

    /* Checking objects to avoid */
    check_avoidance();

    /* Pub state */
    switch (state)
    {
    case FORWARD:
        ROS_INFO("\n[Coherent]: State: %s.", state_to_string(state).c_str());
        break;

    case COHERENCE:
        ROS_WARN("\n[Coherent]: State: %s.", state_to_string(state).c_str());
        break;

    case AVOIDANCE:
        ROS_ERROR("\n[Coherent]: State: %s.", state_to_string(state).c_str());
        break;
    default:
        break;
    }

    /* states */
    switch (state) {

        /* Case when the drone has acceptable alpha, with this do one step to the goal */
        case FORWARD:
        {
            ROS_DEBUG("[Coherent - DEBUG]: UAV is in FORWARD state.");
            ROS_DEBUG("[Coherent - DEBUG]: neighbors count: %d.", neighbors_count);
            
            /* Checking if it is a coherent case */
            if (neighbors_previous >= neighbors_count && neighbors_count < alpha) {

                ROS_INFO("\n[Coherent]: State: %s DENIED.", state_to_string(state).c_str());
                state = COHERENCE;
                counter ++;
                ROS_INFO("\n[Coherent]: neighbors count: %d.", neighbors_count);
                ROS_INFO("\n[Coherent]: goal: %d. \n \n ", current_goal_index);
                return;


            } else {

                /* Get angle of goal, and rotate to it */
                way_angle = get_angle(current_position, goal_);
                way_point = rotate_2d(Eigen::Vector3d(1, 0, 0), way_angle);
                
                /* Reseting counter */
                counter = 0;

            }
            
        break;
        }

        case COHERENCE:
        {
            ROS_DEBUG("[Coherent - DEBUG]:UAV is in COHERENCE state.");
            /* Cheking if counter is a an aceptable value */
            if (counter >= coherence_loops) {

                coherence_goal_offset = get_positive_angle(
                    atan2(goal_[1] - current_position[1], goal_[0] - current_position[0]));
                ROS_DEBUG("[Coherent - DEBUG]: Angle to current goal [%0.2f, %0.2f, %0.2f]: %0.2f rad.", goal_[0], goal_[1], goal_[2],
                        coherence_goal_offset);
                ROS_DEBUG("[Coherent - DEBUG]: Current goal [%0.2f %0.2f %0.2f].", sub_goals[current_goal_index][0],
                          sub_goals[current_goal_index][1], sub_goals[current_goal_index][2]);
                ROS_DEBUG("[Coherent - DEBUG]: Coherence goal offset = %0.2f", coherence_goal_offset);


                /* Get angle of goal, and rotate to it with a random angle added */
                double turn_angle = coherence_goal_offset + random_number(-M_PI / 10.0f, M_PI / 10.0f);
                way_point = rotate_2d(Eigen::Vector3d(10, 0, 0), turn_angle);
                /* Reseting counter */
                counter = 0;
                ROS_WARN("\n[Coherent]: State: %s FORCED FORWARD.", state_to_string(state).c_str());

                /* Sends goto */
                ROS_INFO("\n[Coherent]: neighbors count: %d.", neighbors_count);
                ROS_INFO("\n[Coherent]: goal: %d. \n \n ", current_goal_index);
                way_point = limit(way_point, dist_max_one_step*2);
                ROS_DEBUG("[Coherent - DEBUG]: Way point FINAL: [%0.2f %0.2f %0.2f]", way_point[0], way_point[1], way_point[2]);
                GoTo( this_pose_cmd + way_point );
                return;

            } else {

                /* Rotating 180 degrees to trying found your neighbors */
                way_point = rotate_2d(way_point, M_PI);
                ROS_DEBUG("[Coherent - DEBUG]: Way point COHERENT: [%0.2f %0.2f %0.2f]", way_point[0], way_point[1], way_point[2]);
                ROS_DEBUG("[Coherent - DEBUG]: Counter: %d, coherence loop: %d.", counter, coherence_loops);

            }
        
            state = FORWARD;

        break;
        }

        case AVOIDANCE: 
        {

            ROS_DEBUG("[Coherent - DEBUG]: UAV is in AVOIDANCE state.");
            // Get average of all avoidance forces
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d one_x(1, 0, 0);
            for (int i = 0; i < sector_information.size(); i++) {

                int sector_value = sector_information[i];
                ROS_DEBUG("[Coherent - DEBUG]: Avoidance in sector %d; sector_value: %d  .", i, sector_value);
                // For each sector with critically close object

                if (sector_value == 1) {

                    // Sector from + 1/2 of sector
                    double rot_angle = sectors_margin[i][0] + (sectors_margin[i][1] - sectors_margin[i][0]) / (double) 2;
                    ROS_DEBUG("[Coherent]: Angle of rotation: %f", rot_angle);
                    sum -= rotate_2d(one_x, rot_angle);

                }

            }

            way_point = sum / (double) critically_close;
            ROS_DEBUG("[Coherent - DEBUG]: Waypoint: [%0.2f %0.2f %0.2f].", way_point[0], way_point[1], way_point[2]);
            state = FORWARD;

        break;
        }

        default:
        
            ROS_ERROR("[Coherent - ERROR]: No state present.");
            state = FORWARD;
            ROS_INFO("\n[Coherent]: neighbors count: %d.", neighbors_count);
            ROS_INFO("\n[Coherent]: goal: %d. \n \n ", current_goal_index);
            return;

        break;
            
    }

    // Update navigation goal    
    if (sub_goals.size() > 0) {

        ROS_INFO("\n[Coherent]: neighbors count: %d.", neighbors_count);
        ROS_INFO("\n[Coherent]: goal: %d. \n \n ", current_goal_index);

        if (dist(current_position, goal_) > distance_goal_change) {

            coherence_goal_offset = get_positive_angle(
                    atan2(goal_[1] - current_position[1], goal_[0] - current_position[0]));
            ROS_DEBUG("[Coherent - DEBUG]: Angle to current goal [%0.2f, %0.2f, %0.2f]: %0.2f rad.", goal_[0], goal_[1], goal_[2],
                        coherence_goal_offset);
            // If not already the last goal && is in range to the subgoal

        } else if (current_goal_index != sub_goals.size() - 1) {

            current_goal_index++;

        }
        
    }

    neighbors_previous = neighbors_count;
    /* Sends goto */
    way_point = limit(way_point, dist_max_one_step);
    ROS_DEBUG("[Coherent - DEBUG]: Way point FINAL: [%0.2f %0.2f %0.2f]", way_point[0], way_point[1], way_point[2]);
    GoTo( this_pose_cmd + way_point );

}

/*

    | ---------------------- service callbacks ------------------------- |

*/

/* callbackGoTo */
void CoherentD::GoTo(Eigen::Vector3d position) {
    
    if (maintain_constant_altitude) {
        position[2] = altitude_constant_value;
    }

    ROS_DEBUG("[Coherent]: %s flies to [%0.2f, %0.2f, %0.2f].", _this_uav_name_.c_str() , position[0], position[1], position[2]);
    
    /* Point */
    mrs_msgs::ReferenceStampedSrv srv_reference_stamped_msg;
    srv_reference_stamped_msg.request.header.stamp    = ros::Time::now();
    srv_reference_stamped_msg.request.header.frame_id = _this_uav_name_ + "/" + "stable_origin" ;
    srv_reference_stamped_msg.request.reference.position.x = position[0];
    srv_reference_stamped_msg.request.reference.position.y = position[1];
    srv_reference_stamped_msg.request.reference.position.z = position[2];
    srv_reference_stamped_msg.request.reference.heading = 0;
    srv_client_goto_.call(srv_reference_stamped_msg);

    ros::spinOnce();
}

/*

    | ---------------------- suport functions ------------------------- |

*/

/* state */
std::string CoherentD::state_to_string(State s) {
    switch (s) {
        case (FORWARD):
            return "FORWARD";
            break;
        case (AVOIDANCE):
            return "AVOIDANCE";
            break;
        case (COHERENCE):
            return "COHERENCE";
            break;
        default:
            return "";
    }
}

/* rotate 2D */
Eigen::Vector3d CoherentD::rotate_2d(Eigen::Vector3d vector, double angle_in_rad) {
    double x, y;
    x = vector[0] * cos(angle_in_rad) - vector[1] * sin(angle_in_rad);
    y = vector[0] * sin(angle_in_rad) + vector[1] * cos(angle_in_rad);
    ROS_DEBUG("[Coherent - DEBUG]: Rotating way point by %0.2f rad.", angle_in_rad);
    return Eigen::Vector3d(x, y, 0);
}

/* Margins of sectors */
void CoherentD::set_margin(){
    range_limited_to_sectors = false;
    if (margin.size() % 2 != 0) {
        ROS_ERROR("Sensor sectors are wrongly defined in config file. Ending rosnode.");
        ros::shutdown();
    } else if (margin.size() > 0) {
        range_limited_to_sectors = true;
    }
    for (int i = 0; i < margin.size(); i += 2) {
        std::vector<double> one_sector;
        one_sector.push_back(margin[i]);
        one_sector.push_back(margin[i + 1]);
        sectors_margin.push_back(one_sector);
    }  
}

/* random number */
double CoherentD::random_number(double min, double max) {
    return (max - min) * ((double) rand() / (double) RAND_MAX) + min;
}

/* Goal */
void CoherentD::goal(){
    if (tmp_goals.size() % 3 != 0) {
        ROS_ERROR("[Coherent - ERROR]: Goals were wrongly defined in goal.yaml file.");
        std::exit(-1);
    }
    current_goal_index = 0;
    for (int i = 0; i < tmp_goals.size(); i += 3) {
        double altitude = (tmp_goals[i + 2] == -1) ? altitude_constant_value : tmp_goals[2];
        Eigen::Vector3d sub_goal(tmp_goals[i], tmp_goals[i + 1], altitude);
        sub_goals.push_back(sub_goal);
    }
    for (int i = 0; i < sub_goals.size(); i++) {
        ROS_DEBUG_ONCE("[Coherent]: Sub goals:");
        ROS_DEBUG("[Coherent - DEBUG]: \t[%0.1f, %0.1f, %0.1f]", sub_goals[i][0], sub_goals[i][1], sub_goals[i][2]);
    }
    current_goal = sub_goals[0];
}

/* Sector information */
std::vector<int> CoherentD::get_sector_information() {
    std::vector<int> sector_information;
    neighbor();
    // For each sector
    for (int i = 0; i < sectors_margin.size(); i++) {
        int status = 0;      
        // Sector is defined as angle from/to
        double from = sectors_margin[i][0], to = sectors_margin[i][1];
        if (to < from) {
            to += 2 * M_PI;
        }
        // For each uav
        for (auto itr = neighbors_pose.begin(); itr != neighbors_pose.end(); ++itr) {
            // If is in range of sight
            for (auto itr = neighbors_range.begin(); itr != neighbors_range.end(); ++itr){
                if (itr->second <= localization_distance) {
                    // Get angle of to the target uav
                    for (auto itr = neighbors_pose.begin(); itr != neighbors_pose.end(); ++itr) {
                        double uav_angle = get_angle(current_position, itr->second);
                        ROS_DEBUG("[Coherent]: uav angle: %f, sector: F:%f T:%f", uav_angle, from, to);
                        if (uav_angle < from) {
                            uav_angle += 2 * M_PI;
                        }
                        if (from < uav_angle && uav_angle < to ){
                            for (auto itr = neighbors_range.begin(); itr != neighbors_range.end(); ++itr){
                                ROS_DEBUG("[Coherent - DEBUG]: neighbors range: %f id: %d.", itr->second, itr->first);
                                if (itr->second <= avoid_distance) {
                                    status = 1;
                                }
                            }
                        }
                    }
                }
            }    
        }
        // If not already critical distance in sector *LIDAR*
        if (sector[i] <= avoid_distance && sector[i] != -1) {
            ROS_DEBUG("[Coherent - DEBUG]: Sector: %d; distance: %f.", i, sector[i]);
            status = 1;
        }
        sector_information.push_back(status);    
    }
    return sector_information;
}

/* Neighbors localization range */
void CoherentD::neighbor() {
    neighbors_count = 0;
    for (auto itr = neighbors_range.begin(); itr != neighbors_range.end(); ++itr){
        if (itr->second <= localization_distance){
            neighbors_count++;
        }
    }
    ROS_DEBUG("\n[Coherent]: neighbors count: %d.", neighbors_count);

}

/* Check avoidance state */
void CoherentD::check_avoidance() {

    /* Setting info about objects surrounding */
    sector_information = get_sector_information();

    /* Reseting number of critically close */
    critically_close = 0;
    
    /* Check each sector looking for something to avoid */
    for (int i = 0; i < sector_information.size(); i++) {
        int sector_value = sector_information[i];
        if (sector_value == 1) {
        ROS_DEBUG("[Coherent - DEBUG]: close in %d, sector value = %d.", i, sector_value);
            critically_close++;
        }
    }

    /* Declere state of avoidance */
    if (critically_close > 0) {
        if (state != AVOIDANCE) {
            ROS_DEBUG("[Coherent - DEBUG]: No Avoidance.");
            state = FORWARD;
        }
        state = AVOIDANCE;
    }

}

/* Get positive angle */
double CoherentD::get_positive_angle(double angle) {
    double ret_angle = angle;
    while (ret_angle < 0) {
        ret_angle += 2 * M_PI;
        ROS_DEBUG("Converted angle %0.2f to %0.2f.", angle, ret_angle);
    }
    while (ret_angle > 2 * M_PI) {
        ret_angle -= 2 * M_PI;
        ROS_DEBUG("Converted angle %0.2f to %0.2f.", angle, ret_angle);
    }
    return ret_angle;
}

/* Get angle */
double CoherentD::get_angle(Eigen::Vector3d uav, Eigen::Vector3d object) {
    // angle between uav and object
    double angle = get_positive_angle(atan2(object[1] - uav[1] , object[0] - uav[0]));
    //ROS_DEBUG("For uav at [%0.2f, %0.2f, %0.2f] and object at [%0.2f, %0.2f, %0.2f], the angle is: %0.2f", uav[0], uav[1], uav[2], object[0], object[1], object[2], angle);
    return angle;
}

/* Limit */
Eigen::Vector3d CoherentD::limit(Eigen::Vector3d vec, double value) {
    if (vec.norm() > value) {
        vec = normalize(vec);
        vec *= value;
        ROS_DEBUG("[Coherent - DEBUG]: Way point needed to be limited.");
    }
    return vec;
}

/* Normalize */
Eigen::Vector3d CoherentD::normalize(Eigen::Vector3d vec) {
    double m = vec.norm();
    if (m > 1e-15 && m != 1) {
        vec /= m;
    }
    return vec;
}

/* Distance */
double CoherentD::dist(Eigen::Vector3d v1, Eigen::Vector3d v2) {
    Eigen::Vector3d dif = v1 - v2;
    return dif.norm();
}

} //namespace coherentD
PLUGINLIB_EXPORT_CLASS(coherentD::CoherentD, nodelet::Nodelet);