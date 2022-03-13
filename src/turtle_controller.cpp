#include "reiturtle/turtle_controller.h"

TurtleController::TurtleController():private_nh("~"){
    private_nh.param("hz",hz, {10});
    private_nh.param("N",N, {4});

    sub_pose = nh.subscribe("/turtle1/pose", 10, &TurtleController::pose_callback, this);

    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
}

void TurtleController::pose_callback(const turtlesim::Pose::ConstPtr &msg){
    outside = double(2*M_PI/N);
    old_pose = current_pose;
    current_pose = *msg;
    if(!pose_checker) old_pose = current_pose;

    dx = fabs(current_pose.x - old_pose.x);
    dy = fabs(current_pose.y - old_pose.y);
    dl = sqrt(dx*dx + dy*dy);
    if(current_pose.theta * old_pose.theta < 0) dtheta = 0;
    else dtheta = fabs(current_pose.theta - old_pose.theta);
    pose_checker = true;
}

void TurtleController::go_straight(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.5;
    pub_cmd_vel.publish(cmd_vel);
    len += dl;
    std::cout<<"len="<<len<<std::endl;
    if(len > 0.2) theta_sum = 0;
}

void TurtleController::turn(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = M_PI/4;
    pub_cmd_vel.publish(cmd_vel);
    theta_sum += dtheta;
    std::cout<<"theta_sum="<<theta_sum<<std::endl;
    if(theta_sum*2*M_PI >= outside) len=0;
}

void TurtleController::square(){
    if(pose_checker){
        if(len > 0.2) turn();
        else go_straight();
    }
}

void TurtleController::process(){
    ros::Rate loop_rate(hz);
    while(ros::ok()){
        square();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char**argv){
    ros::init(argc, argv, "turtle_controller");
    TurtleController turtle_controller;
    turtle_controller.process();
    return 0;
}

