#pragma once
#ifndef COHERENTD_H
#define COHERENTD_H

#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/SpeedTrackerCommand.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ObstacleSectors.h>
#include <mrs_msgs/PositionCommand.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

#include <math.h>

#include <eigen3/Eigen/Core>

#include <map>
#include <mutex>
#include <random>

using test_t = geometry_msgs::PointStamped;
template std::optional<test_t> mrs_lib::Transformer::transform<test_t>(const mrs_lib::TransformStamped& to_frame, const test_t& what);

namespace coherentD
{

class CoherentD : public nodelet::Nodelet{
public:
    virtual void onInit();
private:
    /* flags */
    bool                                                is_initialized_;
    bool                                                debug_mode = false;

    /* Parameters */
    std::string                                         _this_uav_name_; 
    std::vector<std::string>                            _uav_names_;    
    std::optional<mrs_lib::TransformStamped>            tf_output_;
     

    mrs_lib::Transformer                                tfr_;

    Eigen::Vector3d                                     current_position;    

    int alpha,
        coherence_loops,
        counter;

    double localization_distance,
            dist_max_one_step,
            altitude_constant_value,
            altitude_lower_value,
            altitude_upper_value,
            run_rate,
            avoid_distance;

    bool maintain_constant_altitude,
         range_limited_to_sectors;

    // | ------------------------ subscriber callbacks --------------------------- |
    
    void callbackUAVOdometry(const nav_msgs::Odometry::ConstPtr& odom);
    ros::Subscriber                                     sub_uav_odom;
    geometry_msgs::Point                                this_uav_pose_;
    std::mutex                                          mutex_this_uav_pose_;
    bool                                                has_this_pose_;

    void callbackNeighborsUVDAR(const mrs_msgs::PoseWithCovarianceArrayStamped::ConstPtr& array_pose);
    ros::Subscriber                                     sub_uav_uvdar;
    std::map<int, Eigen::Vector3d>                      neighbors_pose;
    std::map<int, double>                               neighbors_range;
    std::mutex                                          mutex_neighbors_pose;
    int                                                 neighbors_count;
    int                                                 neighbors_previous;

    void callbackObstacleLIDAR(const mrs_msgs::ObstacleSectors::ConstPtr& scan);
    ros::Subscriber                                     sub_uav_rplidar;
    std::vector<double>                                 sector = {0,0,0,0,0,0,0,0};

    // | --------------------------- timer callbacks ----------------------------- |

    void callbackUAVCoherence(const ros::TimerEvent& event);
    ros::Timer                                          timer_pub_coherent_;

    // | --------------------------- service callbacks --------------------------- |
    void GoTo(Eigen::Vector3d position);
    ros::ServiceClient                                  srv_client_goto_;
    Eigen::Vector3d                                     limit(Eigen::Vector3d vec, double value);
    Eigen::Vector3d                                     normalize(Eigen::Vector3d vec);

    // | --------------------------- suport functions ---------------------------- |

    /* WAYPOINT FUNCTION */
    Eigen::Vector3d                                     way_point;
    Eigen::Vector3d                                     prev_way_point;
    Eigen::Vector3d                                     rotate_2d(Eigen::Vector3d vector, double angle_in_rad);
    double                                              random_number(double a, double b);
    double                                              way_angle;
    double                                              coherence_goal_offset = 0;



    /* STATE FUNCTION */
    enum State {
      FORWARD,
      COHERENCE,
      AVOIDANCE
    };
    State state, previous_state;
    std::string state_to_string(State state);    

    /* GOAL FUNCTION */
    void goal();
    double                                             distance_goal_change;
    double                                             get_positive_angle(double angle);
    double                                             dist(Eigen::Vector3d v1, Eigen::Vector3d v2);
    int                                                current_goal_index;
    std::vector<double>                                tmp_goals;
    std::vector<Eigen::Vector3d>                       sub_goals;
    Eigen::Vector3d                                    current_goal;

    /* SECTOR INFORMATION */
    std::vector<int>                                   get_sector_information();
    std::vector<std::vector<double>>                   sectors_margin;
    std::vector<double>                                margin;
    std::vector<int>                                   sector_information;
    void check_avoidance();
    int                                                critically_close;

    /* SET MARGIN */
    void set_margin(); 

    /* GET ANGLE */
    double                                             get_angle(Eigen::Vector3d uav, Eigen::Vector3d object);

    /* Neighbor */
    void neighbor();
};

} // namespace coherentD

#endif