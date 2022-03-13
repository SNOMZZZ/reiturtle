#ifndef TURTLE_CONTROLLER_H
#define TURTLE_CONTROLLER_H

#include <ros/ros.h>
#include <vector>
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

class TurtleController{
    public:
        TurtleController();
        void process();

    private:
        void pose_callback(const turtlesim::Pose::ConstPtr &msg);
        void turn();
        void go_straight();
        void square();
        int hz;
        int N;
        double outside;
        double len;
        double theta_sum;
        double dx;
        double dy;
        double dl;
        double dtheta;

        int time_step;
        bool pose_checker;
        bool hit_flag;
        bool super_mode_flag;
        double turtle_angle;
        double length;
        double alpha_;

        ros::Publisher pub_cmd_vel;
        ros::Subscriber sub_pose;
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        turtlesim::Pose current_pose;
        turtlesim::Pose old_pose;
};
#endif
