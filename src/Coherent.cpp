#include <Coherent.h>
#include <pluginlib/class_list_macros.h>

namespace coherent
{
/*onInit() */
void Coherent::onInit() {
    /* set flags to false */
    is_initialized_ = false;
    has_this_pose_ = false;

    ros::NodeHandle nh("~");

    ros::Time::waitForValid();

    mrs_lib::ParamLoader param_loader(nh, "Coherent");

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


    if (!param_loader.loadedSuccessfully()){
        ROS_ERROR("[Coherent - ERROR]: failed to load non-optional parameters!");
        ros::shutdown();
    }

    /* subscribes */
    sub_uav_odom = nh.subscribe<nav_msgs::Odometry>("/" + _this_uav_name_ + "/odometry/odom_main", 1, 
                                                                &Coherent::callbackUAVOdometry, this);
    sub_uav_uvdar = nh.subscribe<mrs_msgs::PoseWithCovarianceArrayStamped>("/" + _this_uav_name_ + 
                                    "/uvdar/filteredPoses", 1, &Coherent::callbackNeighborsUVDAR, this);
    sub_uav_rplidar = nh.subscribe<mrs_msgs::ObstacleSectors>("/" + _this_uav_name_ + 
                                "/bumper/obstacle_sectors", 1,&Coherent::callbackObstacleLIDAR, this);
    /* timers */
    timer_pub_coherent_ = nh.createTimer(ros::Rate(run_rate), &Coherent::callbackUAVCoherence, this);

    /* service */
    srv_client_goto_ = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("/" + _this_uav_name_ + 
                                                                        "/control_manager/reference");
    /* transformer */
    tfr_ = mrs_lib::Transformer("SensorNeighbor", _this_uav_name_);

    ROS_INFO_ONCE("[Coherent]: initialized");
    way_point = rotate_2d(Eigen::Vector3d(1, 0, 0), random_number(0, 2 * M_PI));
    ROS_DEBUG("[Coherent - DEBUG]: Random initial waypoint: [%0.2f, %0.2f, %0.2f]", way_point[0], way_point[1], way_point[2]);
    is_initialized_ = true;    
    goal();
    set_margin();
    ROS_INFO_ONCE("[Coherent]: Goal initialized");
    state = FORWARD;
    ros::spin();    
}

/*

    | ---------------------- subscriber callbacks ------------------------- |

*/

/* callbackUAVOdometry */
void Coherent::callbackUAVOdometry(const nav_msgs::Odometry::ConstPtr& odom) {
    if (!is_initialized_) {
        return;
    }

    {
        std::scoped_lock lock(mutex_this_uav_pose_);

        /* update position */
        this_uav_pose_.pose = odom->pose.pose;
        this_uav_pose_.header = odom->header;

        /* turn on flag */
        has_this_pose_ = true;

    }
    // Put this uav pose in Eigen variable
    current_position[0] = odom->pose.pose.position.x;
    current_position[1] = odom->pose.pose.position.y;
    current_position[2] = odom->pose.pose.position.z;
}

/* callbackNeighborsUVDAR */
void Coherent::callbackNeighborsUVDAR(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr& array_poses) {
    if (!is_initialized_ || !has_this_pose_){
        return;
    }

    std::string odom_frame_id;
    {
        std::scoped_lock lock(mutex_this_uav_pose_);
        odom_frame_id = this_uav_pose_.header.frame_id;
    }

    auto tf = tfr_.getTransform(array_poses->header.frame_id, odom_frame_id);
    if (!tf.has_value()) {
    ROS_WARN("[Coherent - WARN]: Could not transform pose from %s to %s", array_poses->header.frame_id.c_str(), odom_frame_id.c_str());
    return;
    }

    {
    std::scoped_lock lock(mutex_neighbors_position_);

    for (unsigned int i = 0; i < array_poses->poses.size(); i++) {
        /* create new msg */
        geometry_msgs::PointStamped   uav_point;

        uav_point.point.x = array_poses->poses[i].pose.position.x;
        uav_point.point.y = array_poses->poses[i].pose.position.y;
        uav_point.point.z = array_poses->poses[i].pose.position.z;
        uav_point.header  = array_poses->header;

        auto uav_point_transformed = tfr_.transform(tf.value(), uav_point);

        /* create Eigen3d vector */
        Eigen::Vector3d             eigen_pose;
        
        eigen_pose[0] = array_poses->poses[i].pose.position.x;
        eigen_pose[1] = array_poses->poses[i].pose.position.y;
        eigen_pose[2] = array_poses->poses[i].pose.position.z;



    if (uav_point_transformed.has_value()) {
        /* save estimated position */
        const unsigned int uav_id = array_poses->poses[i].id;
        neighbors_count = array_poses->poses.size();


        if (neighbors_position_.find(uav_id) == neighbors_position_.end()) {
            neighbors_position_.insert(std::pair<unsigned int, geometry_msgs::PointStamped>(uav_id, uav_point_transformed.value()));
            neighbors_pose.insert(std::pair<int, Eigen::Vector3d>(uav_id, eigen_pose));
        } 
        else {
            neighbors_position_[uav_id] = uav_point_transformed.value();
            neighbors_pose[uav_id] = eigen_pose;
        }
    }

    } // for (unsigned int i = 0; i < array_poses->poses.size(); i++)

    double focal_x, focal_y, focal_heading;
    focal_x       = this_uav_pose_.pose.position.x;
    focal_y       = this_uav_pose_.pose.position.y;
    focal_heading = mrs_lib::AttitudeConverter(this_uav_pose_.pose.orientation).getHeading();

    for (auto itr = neighbors_position_.begin(); itr != neighbors_position_.end(); ++itr) {
        const double range = sqrt(pow(focal_x - itr->second.point.x, 2) + pow(focal_y - itr->second.point.y, 2));
        if ( range <= 0 ){
            continue;
        }
        else {
            neighbors_range = range;
        }
    }
    } // std::scoped_lock lock(mutex_neighbors_position_)

}

/* callbackObstacleLIDAR */
void Coherent::callbackObstacleLIDAR(const mrs_msgs::ObstacleSectors::ConstPtr& scan){
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

/* callbackUAVCoherence */
void Coherent::callbackUAVCoherence(const ros::TimerEvent& event){

    get_sector_information();

    /* Check avoidance state */
    int critically_close = 0;

    for (int i = 0; i < sector_information.size(); i++) {
        int sector_value = sector_information[i];
        if (sector_value == 1) {
            critically_close++;
        }
    }

    if (critically_close > 0) {
        if (state != AVOIDANCE) {
            previous_state = state;
        }
        state = AVOIDANCE;
    }

    /* Pub state */
    ROS_INFO("[Coherent]: State: %s.", state_to_string(state).c_str());

    std_msgs::String advertise;
    char buff[25];
    snprintf(buff, sizeof(buff), "[%0.2f %0.2f %0.2f]", current_position[0], current_position[1], current_position[2]);

    /* states */
    switch (state) {
        case FORWARD:
            ROS_DEBUG("[Coherent - DEBUG]: UAV is in FORWARD state.");
            if (neighbors_count <= neighbors_previous && neighbors_count < alpha) {
                way_point = rotate_2d(way_point, M_PI);
                counter = 0;
                state = COHERENCE;
            }
            break;
        case COHERENCE:
            ROS_DEBUG("[Coherent - DEBUG]:UAV is in COHERENCE state.");
            if (counter >= coherence_loops) {
                // Update navigation goal
                double coherence_goal_offset = 0;
                if (sub_goals.size() > 0) {
                    Eigen::Vector3d goal = sub_goals[current_goal_index];
                    if (dist(current_position, goal) > distance_goal_change) {
                        coherence_goal_offset = get_positive_angle(
                                atan2(goal[1] - current_position[1], goal[0] - current_position[0]));
                        ROS_INFO("[Coherent - DEBUG]: Angle to current goal [%0.2f, %0.2f, %0.2f]: %0.2f rad.", goal[0], goal[1], goal[2],
                                  coherence_goal_offset);
                        // If not already the last goal && is in range to the subgoal
                    } else if (current_goal_index != sub_goals.size() - 1) {
                        current_goal_index++;
                    }
                }
                ROS_INFO("[Coherent - DEBUG]: Current goal [%0.2f %0.2f %0.2f].", sub_goals[current_goal_index][0],
                          sub_goals[current_goal_index][1], sub_goals[current_goal_index][2]);
                ROS_INFO("[Coherent - DEBUG]: Coherence goal offset = %0.2f", coherence_goal_offset);
                double turn_angle = coherence_goal_offset + random_number(-M_PI / 10.0f, M_PI / 10.0f);
                way_point = rotate_2d(Eigen::Vector3d(10, 0, 0), turn_angle);
                state = FORWARD;
            } else {
                ROS_INFO("[Coherent - DEBUG]:Counter: %d, coherence loop: %d.", counter, coherence_loops);
            }
            break;
        case AVOIDANCE: {
            ROS_DEBUG("[Coherent - DEBUG]: UAV is in AVOIDANCE state.");
            // Get average of all avoidance forces
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d one_x(1, 0, 0);
            for (int i = 0; i < sector_information.size(); i++) {
                int sector_value = sector_information[i];
                // For each sector with critically close object
                if (sector_value == 1) {
                    // Sector from + 1/2 of sector
                    double rot_angle = sectors_margin[i][0] + (sectors_margin[i][1] - sectors_margin[i][0]) / (double) 2;
                    sum -= rotate_2d(one_x, rot_angle);
                }
            }
            way_point = sum / (double) critically_close;
            ROS_DEBUG("[Coherent - DEBUG]: Waypoint: [%0.2f %0.2f %0.2f].", way_point[0], way_point[1], way_point[2]);
            state = previous_state;
        }
            break;
        default:
            ROS_ERROR("[Coherent - ERROR]: No state present.");
            break;
    }
    counter++;
    neighbors_previous = neighbors_count;
    /* sends goto */
    ROS_DEBUG("[Coherent - DEBUG]: Way point: [%0.2f %0.2f %0.2f]", way_point[0], way_point[1], way_point[2]);
    way_point = limit(way_point, dist_max_one_step);
    GoTo(current_position + way_point);
}

/*

    | ---------------------- service callbacks ------------------------- |

*/

/* callbackGoTo */
void Coherent::GoTo(Eigen::Vector3d position) {
    if (maintain_constant_altitude) {
        position[2] = altitude_constant_value;
    }
    ROS_INFO("[Coherent]: %s flies to [%0.2f, %0.2f, %0.2f].", _this_uav_name_.c_str() , position[0], position[1], position[2]);
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
std::string Coherent::state_to_string(State s) {
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
Eigen::Vector3d Coherent::rotate_2d(Eigen::Vector3d vector, double angle_in_rad) {
    double x, y;
    x = vector[0] * cos(angle_in_rad) - vector[1] * sin(angle_in_rad);
    y = vector[0] * sin(angle_in_rad) + vector[1] * cos(angle_in_rad);
    ROS_DEBUG("[Coherent - DEBUG]: Rotating way point by %0.2f rad.", angle_in_rad);
    return Eigen::Vector3d(x, y, 0);
}

/* Margins of sectors */
void Coherent::set_margin(){
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
double Coherent::random_number(double min, double max) {
    return (max - min) * ((double) rand() / (double) RAND_MAX) + min;
}

/* Goal */
void Coherent::goal(){
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
std::vector<int> Coherent::get_sector_information() {
    sector_information.clear();
    // For each sector
    for (int i = 0; i < sectors_margin.size(); i++) {
        int status = 0;
        // Sector is defined as angle from/to
        double from = sectors_margin[i][0], to = sectors_margin[i][1];
        if (to < from) {
            to += 2 * M_PI;
        }
        // For each uav
        for (int u = 0; u < neighbors_count; u++) {
            // If is in range of sight
            if (neighbors_range <= localization_distance) {
                // Get angle of to the target uav
                for (auto itr = neighbors_pose.begin(); itr != neighbors_pose.end(); ++itr) {
                    double uav_angle = get_angle_to_object(current_position, itr->second);
                    if (uav_angle < from) {
                        uav_angle += 2 * M_PI;
                    }
                    // // If between margins
                    if (from <= uav_angle && uav_angle <= to) {
                        status = -1;
                        if (neighbors_range <= avoid_distance) {
                            status = 1;
                            break;
                        }
                    }
                }
            }
        }
        // If not already critical distance in sector
        if (sector[i] <= avoid_distance && sector[i] != -1) {
            status = 1;
            break;
        }
        sector_information.push_back(status);
        std::cout << "\n" << status << "\n";
    }
    return sector_information;
}


/* Get positive angle */
double Coherent::get_positive_angle(double angle) {
    double ret_angle = angle;
    while (ret_angle < 0) {
        ret_angle += 2 * M_PI;
        //ROS_DEBUG("Converted angle %0.2f to %0.2f.", angle, ret_angle);
    }
    while (ret_angle > 2 * M_PI) {
        ret_angle -= 2 * M_PI;
        //ROS_DEBUG("Converted angle %0.2f to %0.2f.", angle, ret_angle);
    }
    return ret_angle;
}

double Coherent::get_angle_to_object(Eigen::Vector3d uav, Eigen::Vector3d object) {
    // angle between uav and object
    double angle = get_positive_angle(atan2(object[1] - uav[1] , object[0] - uav[0]));
    //ROS_DEBUG("For uav at [%0.2f, %0.2f, %0.2f] and object at [%0.2f, %0.2f, %0.2f], the angle is: %0.2f", uav[0], uav[1], uav[2], object[0], object[1], object[2], angle);
    return angle;
}

/* Limit */
Eigen::Vector3d Coherent::limit(Eigen::Vector3d vec, double value) {
    if (vec.norm() > value) {
        vec = normalize(vec);
        vec *= value;
        ROS_DEBUG("[Coherent - DEBUG]: Way point needed to be limited.");
    }
    return vec;
}

/* Normalize */
Eigen::Vector3d Coherent::normalize(Eigen::Vector3d vec) {
    double m = vec.norm();
    if (m > 1e-15 && m != 1) {
        vec /= m;
    }
    return vec;
}

/* Distance */
double Coherent::dist(Eigen::Vector3d v1, Eigen::Vector3d v2) {
    Eigen::Vector3d dif = v1 - v2;
    return dif.norm();
}

} //namespace coherent
PLUGINLIB_EXPORT_CLASS(coherent::Coherent, nodelet::Nodelet);