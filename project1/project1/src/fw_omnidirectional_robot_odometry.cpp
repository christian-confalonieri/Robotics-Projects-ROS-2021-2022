#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "project1/WheelSpeed.h"
#include <math.h>

// publishers includes
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

// tf2 broadcaster includes
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

// dynamic reconfigure includes
#include "dynamic_reconfigure/server.h"
#include "project1/dynamic_recConfig.h"

// service includes
#include "project1/SetPose.h"

#define _USE_MATH_DEFINES
#define WHEEL_RADIUS 0.070
#define WHEEL_POSITION_X 0.200
#define WHEEL_POSITION_Y 0.169 
#define GEAR_RATIO 5
#define ENCODER_CPR 42

#define WHEEL_RADIUS_BAG1 0.040957
#define WHEEL_POSITION_X_BAG1 0.182268
#define WHEEL_POSITION_Y_BAG1 0.154017
#define ENCODER_CPR_BAG1 38.028638

#define WHEEL_RADIUS_BAG2 0.021110
#define WHEEL_POSITION_X_BAG2 0.181404
#define WHEEL_POSITION_Y_BAG2 0.153286
#define ENCODER_CPR_BAG2 38.101910

#define WHEEL_RADIUS_BAG3 0.044373
#define WHEEL_POSITION_X_BAG3 0.183019
#define WHEEL_POSITION_Y_BAG3 0.154651
#define ENCODER_CPR_BAG3 38.513785

typedef struct data {

    double motor_position_fl;
    double motor_position_fr;
    double motor_position_rr;
    double motor_position_rl;

    double motor_rpm_fl;
    double motor_rpm_fr;
    double motor_rpm_rr;
    double motor_rpm_rl;

    double time;

}Data;

typedef struct velocities {
    
    double linear_velocity_x;
    double linear_velocity_y;
    double angular_velocity;

}Velocity;

typedef struct pose {

    double x;
    double y;
    double theta;

}Pose;

class fw_omnidirectional_robot_odometry {

    private:

        ros::NodeHandle pub_node;
        ros::Publisher velocities_pub; // publisher for velocities messages --> geometry_msgs/TwistStamped
        ros::Publisher odometry_pub; // publisher for odometry messages --> nav_msgs/Odometry
        ros::Publisher wheel_speed_pub; // publisher for wheel speed --> project1/WheelSpeed

        tf2_ros::TransformBroadcaster odom_broadcaster;

        ros::ServiceServer set_pose_service; 

        Velocity velocities;
        Data wheel_speed;
        Pose current_pose;
        Pose prev_pose;
        double current_time;
        double prev_time;
        double delta_time;

        int chosen_odometry_type;

        // attributes and variables used in the method for calculating speed via wheel ticks
        double prev_position_fl;
        double prev_position_fr;
        double prev_position_rr;
        double prev_position_rl;
        bool first_position;

    public: 

        fw_omnidirectional_robot_odometry() {

            velocities_pub = pub_node.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
            odometry_pub = pub_node.advertise<nav_msgs::Odometry>("odom", 1000);
            wheel_speed_pub = pub_node.advertise<project1::WheelSpeed>("wheels_rpm", 1000);

            set_pose_service = pub_node.advertiseService("set_pose", &fw_omnidirectional_robot_odometry::set_pose_service_function, this);

            pub_node.getParam("/initial_x", current_pose.x);
            pub_node.getParam("/initial_y", current_pose.y);
            pub_node.getParam("/initial_theta", current_pose.theta);

            prev_time = 0.0;

            first_position = true; //boolean used in the method for calculating speed via wheel ticks

            prev_pose.x = current_pose.x;
            prev_pose.y = current_pose.y;
            prev_pose.theta = current_pose.theta;

            chosen_odometry_type = 0;

        }

        bool set_pose_service_function(project1::SetPose::Request &req, project1::SetPose::Response &res ){

            prev_pose.x = req.x;
            prev_pose.y = req.y;
            prev_pose.theta = req.theta;

            ROS_INFO("new pose: [x: [%f],y: [%f],z: [%f]]",prev_pose.x,prev_pose.y,prev_pose.theta);

            res.response = "a new pose has been set";
            return true;

        }

        void set_chosen_odometry(int odometryMethod) {

            if(odometryMethod == 0 || odometryMethod == 1) {
                chosen_odometry_type = odometryMethod;
            }

        }

        int get_odometry_method() {

            return chosen_odometry_type;

        }

        Pose get_current_pose() {

            return current_pose;

        }

        void compute_velocities(Data *input) {

            current_time = input->time;

            delta_time = current_time - prev_time;
            prev_time = current_time;            

            // compute velocity with rpm ----------------------------------------------

            // double velocity_fl = input->motor_rpm_fl / (60*GEAR_RATIO);
            // double velocity_fr = input->motor_rpm_fr / (60*GEAR_RATIO);
            // double velocity_rr = input->motor_rpm_rr / (60*GEAR_RATIO);
            // double velocity_rl = input->motor_rpm_rl / (60*GEAR_RATIO);

            // compute velocity with wheel ticks --------------------------------------

            double current_position;
            double delta_position;

            // in this if you enter only in the first iteration
            if(first_position) {
                prev_position_fl = input->motor_position_fl;
                prev_position_fr = input->motor_position_fr;
                prev_position_rr = input->motor_position_rr;
                prev_position_rl = input->motor_position_rl;
                first_position = false;
            }

            current_position = input->motor_position_fl;
            delta_position = current_position - prev_position_fl;
            double velocity_fl = (delta_position*2*M_PI)/(delta_time*ENCODER_CPR*GEAR_RATIO);
            prev_position_fl = current_position;

            current_position = input->motor_position_fr;
            delta_position = current_position - prev_position_fr;
            double velocity_fr = (delta_position*2*M_PI)/(delta_time*ENCODER_CPR*GEAR_RATIO);
            prev_position_fr = current_position;

            current_position = input->motor_position_rr;
            delta_position = current_position - prev_position_rr;
            double velocity_rr = (delta_position*2*M_PI)/(delta_time*ENCODER_CPR*GEAR_RATIO);
            prev_position_rr = current_position;

            current_position = input->motor_position_rl;
            delta_position = current_position - prev_position_rl;
            double velocity_rl = (delta_position*2*M_PI)/(delta_time*ENCODER_CPR*GEAR_RATIO);
            prev_position_rl = current_position;

            velocities.linear_velocity_x = (WHEEL_RADIUS * (velocity_fl+velocity_fr+velocity_rr+velocity_rl))/4;
            velocities.linear_velocity_y = (WHEEL_RADIUS * (-velocity_fl+velocity_fr+velocity_rr-velocity_rl))/4;
            velocities.angular_velocity = (WHEEL_RADIUS * (-velocity_fl+velocity_fr-velocity_rr+velocity_rl))/
            (4*(WHEEL_POSITION_X+WHEEL_POSITION_Y));

        }

        void publish_velocities(geometry_msgs::TwistStamped *computed_velocities) {

            ros::Time time = ros::Time::now();

            computed_velocities->header.stamp = time;
            computed_velocities->header.frame_id = "cmd_vel";

            // set linear velocity
            computed_velocities->twist.linear.x = velocities.linear_velocity_x;
            computed_velocities->twist.linear.y = velocities.linear_velocity_y;
            computed_velocities->twist.linear.z = 0;

            // set angular velocity
            computed_velocities->twist.angular.x = 0;
            computed_velocities->twist.angular.y = 0;
            computed_velocities->twist.angular.z = velocities.angular_velocity;

            velocities_pub.publish(*computed_velocities);

        }

        void compute_odometry(Data *input) {

            if(chosen_odometry_type == 0) {
                compute_odometry_euler(input);
            }
            if(chosen_odometry_type == 1) {
                compute_odometry_rungekutta(input);
            }

        }

        void compute_odometry_euler(Data *input) {

            // euler formula
            double local_x = (velocities.linear_velocity_x * delta_time);
            double local_y = (velocities.linear_velocity_y * delta_time);
            
            current_pose.x = prev_pose.x + local_x * cos(prev_pose.theta) - local_y * sin(prev_pose.theta);
            prev_pose.x = current_pose.x;
            
            current_pose.y = prev_pose.y + local_x * sin(prev_pose.theta) + local_y * cos(prev_pose.theta);
            prev_pose.y = current_pose.y;
            
            current_pose.theta = prev_pose.theta + (velocities.angular_velocity * delta_time);
            prev_pose.theta = current_pose.theta;

        }

        void compute_odometry_rungekutta(Data *input) {

            // runge kutta formula
            double local_x = (velocities.linear_velocity_x * delta_time);
            double local_y = (velocities.linear_velocity_y * delta_time);
            
            current_pose.x = prev_pose.x + local_x * cos(prev_pose.theta + ((velocities.angular_velocity*delta_time)/2.0)) - local_y * sin(prev_pose.theta + ((velocities.angular_velocity*delta_time)/2.0));
            prev_pose.x = current_pose.x;
            
            current_pose.y = prev_pose.y + local_x * sin(prev_pose.theta + ((velocities.angular_velocity*delta_time)/2.0)) + local_y * cos(prev_pose.theta + ((velocities.angular_velocity*delta_time)/2.0));
            prev_pose.y = current_pose.y;
            
            current_pose.theta = prev_pose.theta + (velocities.angular_velocity * delta_time);
            prev_pose.theta = current_pose.theta;

        }

        void publish_odometry(nav_msgs::Odometry *computed_odometry) {
            
            ros::Time time = ros::Time::now();
            geometry_msgs::TransformStamped odom_trans;
            tf2::Quaternion q;
            q.setRPY(0, 0, current_pose.theta);
            geometry_msgs::Quaternion odom_quat;
            
            odom_quat.x = q.x();
            odom_quat.y = q.y();
            odom_quat.z = q.z();
            odom_quat.w = q.w();

            //odometry header
            computed_odometry->header.stamp = time;
            computed_odometry->header.frame_id = "odom";

            //set position
            computed_odometry->pose.pose.position.x = current_pose.x;
            computed_odometry->pose.pose.position.y = current_pose.y;
            computed_odometry->pose.pose.position.z = 0;
            computed_odometry->pose.pose.orientation = odom_quat;

            //set velocity
            computed_odometry->child_frame_id = "base_link";
            computed_odometry->twist.twist.linear.x = velocities.linear_velocity_x;
            computed_odometry->twist.twist.linear.y = velocities.linear_velocity_y;
            computed_odometry->twist.twist.linear.z = 0.0;

            computed_odometry->twist.twist.angular.x = 0.0;
            computed_odometry->twist.twist.angular.y = 0.0;
            computed_odometry->twist.twist.angular.z = velocities.angular_velocity;

            //transform
            odom_trans.header.stamp = time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = current_pose.x;
            odom_trans.transform.translation.y = current_pose.y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            odom_broadcaster.sendTransform(odom_trans);
            odometry_pub.publish(*computed_odometry);

        }

        void compute_wheel_speed(Velocity *velocities) {

            wheel_speed.motor_rpm_fl = (velocities->linear_velocity_x-velocities->linear_velocity_y-((WHEEL_POSITION_X+WHEEL_POSITION_Y)*velocities->angular_velocity))/WHEEL_RADIUS;
            wheel_speed.motor_rpm_fl *= 60*GEAR_RATIO;

            wheel_speed.motor_rpm_fr = (velocities->linear_velocity_x+velocities->linear_velocity_y+((WHEEL_POSITION_X+WHEEL_POSITION_Y)*velocities->angular_velocity))/WHEEL_RADIUS;
            wheel_speed.motor_rpm_fr *= 60*GEAR_RATIO;

            wheel_speed.motor_rpm_rr = (velocities->linear_velocity_x+velocities->linear_velocity_y-((WHEEL_POSITION_X+WHEEL_POSITION_Y)*velocities->angular_velocity))/WHEEL_RADIUS;
            wheel_speed.motor_rpm_rr *= 60*GEAR_RATIO;

            wheel_speed.motor_rpm_rl = (velocities->linear_velocity_x-velocities->linear_velocity_y+((WHEEL_POSITION_X+WHEEL_POSITION_Y)*velocities->angular_velocity))/WHEEL_RADIUS;
            wheel_speed.motor_rpm_rl *= 60*GEAR_RATIO;

        }

        void publish_wheel_speed(project1::WheelSpeed *computed_wheel_speed) {

            ros::Time time = ros::Time::now();

            computed_wheel_speed->header.stamp = time;
            computed_wheel_speed->header.frame_id = "wheels_rpm";

            computed_wheel_speed->rpm_fl = wheel_speed.motor_rpm_fl;
            computed_wheel_speed->rpm_fr = wheel_speed.motor_rpm_fr;
            computed_wheel_speed->rpm_rr = wheel_speed.motor_rpm_rr;
            computed_wheel_speed->rpm_rl = wheel_speed.motor_rpm_rl;

            wheel_speed_pub.publish(*computed_wheel_speed);

        }
};

// callback to compute and publish velocities and odometry
void wheel_states_callback(const sensor_msgs::JointState::ConstPtr& msg,fw_omnidirectional_robot_odometry *robot, Data *input){

    input->motor_position_fl = msg->position[0];
    input->motor_rpm_fl = msg->velocity[0];
    input->motor_position_fr = msg->position[1];
    input->motor_rpm_fr = msg->velocity[1];
    input->motor_position_rr = msg->position[2];
    input->motor_rpm_rr = msg->velocity[2];
    input->motor_position_rl = msg->position[3];
    input->motor_rpm_rl = msg->velocity[3];

    input->time = msg->header.stamp.toSec();

    // compute and publish velocities
    geometry_msgs::TwistStamped computed_velocities;
    robot->compute_velocities(input);
    robot->publish_velocities(&computed_velocities);

    // compute and publish odometry
    nav_msgs::Odometry computed_odometry;
    robot->compute_odometry(input);
    robot->publish_odometry(&computed_odometry);

    // position and orientation print
    tf2::Quaternion q;
    q.setRPY(0, 0, robot->get_current_pose().theta);

    ROS_INFO("odometry method: [%d]\nposition:\nx: [%lf]\ny: [%lf]\nz: [%lf]\norientation:\nx: [%lf]\ny: [%lf]\nz: [%lf]\nw: [%lf]\n",robot->get_odometry_method(),robot->get_current_pose().x,robot->get_current_pose().y,0.0,q.x(),q.y(),q.z(),q.w());

}

// callback to compute wheel speeds (RPM) from the linear and angular velocity
void velocities_callback(const geometry_msgs::TwistStamped::ConstPtr& msg,fw_omnidirectional_robot_odometry *robot, Velocity *velocities) {
            
    velocities->linear_velocity_x = msg->twist.linear.x;
    velocities->linear_velocity_y = msg->twist.linear.y;
    velocities->angular_velocity = msg->twist.angular.z;

    project1::WheelSpeed computed_wheel_speed;
    robot->compute_wheel_speed(velocities);
    robot->publish_wheel_speed(&computed_wheel_speed);

}

// dynamic reconfigure callback
void param_callback(project1::dynamic_recConfig &config, uint32_t level, fw_omnidirectional_robot_odometry *robot) {
    
    robot->set_chosen_odometry(config.odometry_integration_type);

    ROS_INFO("reconfigure request: %d", config.odometry_integration_type);
    
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "fw_omnidirectional_robot_node");
    
    Data input;
    Velocity velocities;

    fw_omnidirectional_robot_odometry *robot = NULL;
    robot = new fw_omnidirectional_robot_odometry();

    //dynamic reconfigure for odometry type
    dynamic_reconfigure::Server<project1::dynamic_recConfig> server;
    dynamic_reconfigure::Server<project1::dynamic_recConfig>::CallbackType f;
    f = boost::bind(&param_callback,_1,_2,robot);
    server.setCallback(f);

    ros::NodeHandle sub_node;
    ros::Subscriber odometry_sub = sub_node.subscribe<sensor_msgs::JointState>("wheel_states", 1000, boost::bind(&wheel_states_callback,_1, robot, &input));
    ros::Subscriber velocities_sub = sub_node.subscribe<geometry_msgs::TwistStamped>("cmd_vel", 1000, boost::bind(&velocities_callback,_1, robot, &velocities));

    ros::spin();

    return 0;

}