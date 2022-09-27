#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <math.h>

// tf broadcaster includes
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// message filters includes
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

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

typedef message_filters::sync_policies::
        ApproximateTime<sensor_msgs::JointState,geometry_msgs::PoseStamped> MySyncPolicy;

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

    double time;

}Pose;

class fw_omnidirectional_robot_odometry {

    private:

        Velocity velocities;
        Pose current_pose;
        Pose prev_pose;
        
        double prev_time_joint_state;
        double prev_time_pose_stamped;

        double prev_position_fl;
        double prev_position_fr;
        double prev_position_rr;
        double prev_position_rl;
        bool first_position;

        double estimated_wheel_radius;
        double estimated_wheel_position_x;
        double estimated_wheel_position_y;
        double estimated_encoder_cpr;

        int count1;
        int count2;
        int count3;

    public: 

        fw_omnidirectional_robot_odometry() {

            prev_time_joint_state = 0.0;
            prev_time_pose_stamped = 0.0;

            first_position = true;

            prev_pose.x = current_pose.x;
            prev_pose.y = current_pose.y;
            prev_pose.theta = current_pose.theta;

            estimated_wheel_radius = 0.0;
            estimated_wheel_position_x = 0.0;
            estimated_wheel_position_y = 0.0;
            estimated_encoder_cpr = 0.0;
            
            count1 = 0;
            count2 = 0;
            count3 = 0;

        }

        void calibrate_parameters(Data *input, Pose *pose_stamped) {

            double current_time = input->time;

            double delta_time = current_time - prev_time_joint_state;
            prev_time_joint_state = current_time;   
             
            double current_position;
            double delta_position;
             
            // in this if you enter only the first iteration
            if(first_position) {
                prev_position_fl = input->motor_position_fl;
                prev_position_fr = input->motor_position_fr;
                prev_position_rr = input->motor_position_rr;
                prev_position_rl = input->motor_position_rl;
                first_position = false;
            }
            
            double current_estimated_encoder_cpr = 0.0;
            int correct = 0;

            current_position = input->motor_position_fl;
            delta_position = current_position - prev_position_fl;
            if(input->motor_rpm_fl!=0.0) {
                current_estimated_encoder_cpr += (delta_position*2*M_PI*60) / (delta_time*input->motor_rpm_fl);
                correct++;
            }
            prev_position_fl = current_position;

            current_position = input->motor_position_fr;
            delta_position = current_position - prev_position_fr;
            if(input->motor_rpm_fr!=0.0) {
                current_estimated_encoder_cpr += (delta_position*2*M_PI*60) / (delta_time*input->motor_rpm_fr);
                correct++;
            }
            prev_position_fr = current_position;

            current_position = input->motor_position_rr;
            delta_position = current_position - prev_position_rr;
            if(input->motor_rpm_rr!=0.0) {
                current_estimated_encoder_cpr += (delta_position*2*M_PI*60) / (delta_time*input->motor_rpm_rr);
                correct++;
            }
            prev_position_rr = current_position;

            current_position = input->motor_position_rl;
            delta_position = current_position - prev_position_rl;
            if(input->motor_rpm_rl!=0.0) {
                current_estimated_encoder_cpr += (delta_position*2*M_PI*60) / (delta_time*input->motor_rpm_rl);
                correct++;
            }
            prev_position_rl = current_position;

            if(correct != 0) {

                current_estimated_encoder_cpr /= correct;

                if(current_estimated_encoder_cpr >= 30 && current_estimated_encoder_cpr <= 50) {
                    estimated_encoder_cpr += current_estimated_encoder_cpr;
                    count1++;
                }
            
            }

            if(count1 != 0) {
                ROS_INFO("estimated encoder cpr: [%lf], iteration: [%d]", estimated_encoder_cpr / count1,count1);
            }
            else {
                ROS_INFO("estimated encoder cpr: [%lf], iteration: [%d]", 0.0,0);
            }

            current_pose.x = pose_stamped->x;
            current_pose.y = pose_stamped->y;
            current_pose.theta = pose_stamped->theta;

            current_time = pose_stamped->time;
            delta_time = current_time - prev_time_pose_stamped;
            prev_time_pose_stamped = current_time;

            double local_x_estimation = (current_pose.x - prev_pose.x + (((prev_pose.x - current_pose.x) * sin(prev_pose.theta) + (current_pose.y - prev_pose.y) * cos(prev_pose.theta)) / 2.0) * sin(prev_pose.theta))/cos(prev_pose.theta);
            double local_y_estimation = ((prev_pose.x - current_pose.x) * sin(prev_pose.theta) + (current_pose.y - prev_pose.y) * cos(prev_pose.theta)) / 2.0;

            velocities.linear_velocity_x = local_x_estimation / delta_time;
            velocities.linear_velocity_y = local_y_estimation / delta_time;
            velocities.angular_velocity = (current_pose.theta-prev_pose.theta) / delta_time;

            double current_estimated_wheel_radius = 0.0;
            correct = 0;
            if(input->motor_rpm_fl+input->motor_rpm_fr+input->motor_rpm_rr+input->motor_rpm_rl!=0.0) {
                current_estimated_wheel_radius += (velocities.linear_velocity_x * 4 * (60*GEAR_RATIO)) / (input->motor_rpm_fl+input->motor_rpm_fr+input->motor_rpm_rr+input->motor_rpm_rl);
                correct++;
            }
            if(-input->motor_rpm_fl+input->motor_rpm_fr+input->motor_rpm_rr-input->motor_rpm_rl!=0.0) {
                current_estimated_wheel_radius += (velocities.linear_velocity_y * 4 * (60*GEAR_RATIO)) / (-input->motor_rpm_fl+input->motor_rpm_fr+input->motor_rpm_rr-input->motor_rpm_rl);
                correct++;
            }

            if(correct!=0) {
                
                current_estimated_wheel_radius /= correct;
                
                if(current_estimated_wheel_radius>=0.00 && current_estimated_wheel_radius<=0.15) {
                    estimated_wheel_radius += current_estimated_wheel_radius;
                    count2++;
                } 
            
            }

            if(count2 != 0) {
                ROS_INFO("estimated wheel radius: [%lf], iteration: [%d]", estimated_wheel_radius / count2,count2);
            }
            else {
                ROS_INFO("estimated wheel radius: [%lf], iteration: [%d]", 0.0,0);
            }

            prev_pose.x = current_pose.x;
            prev_pose.y = current_pose.y;
            prev_pose.theta = current_pose.theta;

            double estimated_sum_wheel_position = 0.0;
            correct = 0;

            if(velocities.angular_velocity != 0.0) {
                if(estimated_wheel_radius == 0.0) {
                    estimated_sum_wheel_position += ( ((input->motor_rpm_fl * WHEEL_RADIUS) / (60*GEAR_RATIO)) - velocities.linear_velocity_x + velocities.linear_velocity_y ) /  -velocities.angular_velocity;
                    estimated_sum_wheel_position += ( ((input->motor_rpm_fr * WHEEL_RADIUS) / (60*GEAR_RATIO)) - velocities.linear_velocity_x - velocities.linear_velocity_y ) /  velocities.angular_velocity;
                    estimated_sum_wheel_position += ( ((input->motor_rpm_rr * WHEEL_RADIUS) / (60*GEAR_RATIO)) - velocities.linear_velocity_x - velocities.linear_velocity_y ) /  -velocities.angular_velocity;
                    estimated_sum_wheel_position += ( ((input->motor_rpm_rl * WHEEL_RADIUS) / (60*GEAR_RATIO)) - velocities.linear_velocity_x + velocities.linear_velocity_y ) /  velocities.angular_velocity;
                } 
                else {
                    estimated_sum_wheel_position += ( ((input->motor_rpm_fl * estimated_wheel_radius)) / (60*GEAR_RATIO) - velocities.linear_velocity_x + velocities.linear_velocity_y ) /  -velocities.angular_velocity;
                    estimated_sum_wheel_position += ( ((input->motor_rpm_fr * estimated_wheel_radius)) / (60*GEAR_RATIO) - velocities.linear_velocity_x - velocities.linear_velocity_y ) /  velocities.angular_velocity;
                    estimated_sum_wheel_position += ( ((input->motor_rpm_rr * estimated_wheel_radius)) / (60*GEAR_RATIO) - velocities.linear_velocity_x - velocities.linear_velocity_y ) /  -velocities.angular_velocity;
                    estimated_sum_wheel_position += ( ((input->motor_rpm_rl * estimated_wheel_radius)) / (60*GEAR_RATIO) - velocities.linear_velocity_x + velocities.linear_velocity_y ) /  velocities.angular_velocity;
                }
                estimated_sum_wheel_position /= 4;
                if(estimated_sum_wheel_position >= 0.2 && estimated_sum_wheel_position<= 0.5) {
                    estimated_wheel_position_x += 0.5420054201 * estimated_sum_wheel_position;
                    estimated_wheel_position_y += 0.4579945799 * estimated_sum_wheel_position;
                    count3++;
                }
            }
           
            if(count3 != 0) {
                ROS_INFO("estimated wheel position x: [%lf], iteration: [%d]", estimated_wheel_position_x / count3, count3);
                ROS_INFO("estimated wheel position y: [%lf], iteration: [%d]", estimated_wheel_position_y / count3, count3);
            }
            else {
                ROS_INFO("estimated wheel position x: [%lf], iteration: [%d]", 0.0, 0);
                ROS_INFO("estimated wheel position y: [%lf], iteration: [%d]", 0.0, 0);
            }

        }
};

// callback to compute the estimated parameters
void calibration_callback(const sensor_msgs::JointState::ConstPtr& msg_joint_state,const geometry_msgs::PoseStamped::ConstPtr& msg_pose_stamped,fw_omnidirectional_robot_odometry *robot, Data *input, Pose *pose_stamped){

    input->motor_position_fl = msg_joint_state->position[0];
    input->motor_rpm_fl = msg_joint_state->velocity[0];
    input->motor_position_fr = msg_joint_state->position[1];
    input->motor_rpm_fr = msg_joint_state->velocity[1];
    input->motor_position_rr = msg_joint_state->position[2];
    input->motor_rpm_rr = msg_joint_state->velocity[2];
    input->motor_position_rl = msg_joint_state->position[3];
    input->motor_rpm_rl = msg_joint_state->velocity[3];

    input->time = msg_joint_state->header.stamp.toSec();

    pose_stamped->x = msg_pose_stamped->pose.position.x;
    pose_stamped->y = msg_pose_stamped->pose.position.y;

    double roll,pitch,yaw;
    tf2::Quaternion q(msg_pose_stamped->pose.orientation.x,msg_pose_stamped->pose.orientation.y,msg_pose_stamped->pose.orientation.z,msg_pose_stamped->pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);

    pose_stamped->theta = yaw;
    pose_stamped->time = msg_pose_stamped->header.stamp.toSec();    

    robot->calibrate_parameters(input,pose_stamped);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "calibration_node");
    
    Data input;
    Pose pose_stamped;

    fw_omnidirectional_robot_odometry *robot = NULL;
    robot = new fw_omnidirectional_robot_odometry();

    ros::NodeHandle sub_node;

    message_filters::Subscriber<sensor_msgs::JointState> sub_joint_state(sub_node,"wheel_states", 1000);
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose_stamped(sub_node,"robot/pose",1000);

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),sub_joint_state,sub_pose_stamped);
    sync.registerCallback(boost::bind(&calibration_callback, _1, _2, robot, &input, &pose_stamped));

    ros::spin();

    return 0;

}