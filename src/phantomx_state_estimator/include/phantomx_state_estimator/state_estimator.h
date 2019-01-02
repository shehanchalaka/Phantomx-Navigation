#pragma once

#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#define INF 999999
#define a 0.12
#define b 0.06
#define c 0.10
#define L1 0.048
#define L2 0.083
#define L3 0.143

class StateEstimator {

    public:
        StateEstimator(ros::NodeHandle& nodeHandle);
        virtual ~StateEstimator();

        void KalmanPredict();
        void KalmanUpdate();

    private:
        void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
        void IMUCallback(const geometry_msgs::Quaternion::ConstPtr& msg);        
        void ContactSensorCallback(const geometry_msgs::Twist::ConstPtr& msg);
        
        void timerCallback(const ros::TimerEvent&);
        
        void calculateMeasurementMatrix();
        void calculateMeasurementJacobian();
        void publishTransform();

        ros::NodeHandle nodeHandle_;
        ros::Subscriber imu_subscriber_;
        ros::Subscriber joint_state_subscriber_;
        ros::Subscriber contact_subscriber_;
        
        ros::Publisher state_publisher_;

        std::string imu_topic_name_;
        int imu_queue_size_;
        std::string contact_topic_name_;
        int contact_queue_size_;

        ros::Timer timer1;
        

        double q[18]; // encoder values
        double quat[4]; // quaternoin from IMU

        Eigen::Matrix<double, 24, 1> state;
        Eigen::Matrix<double, 24, 24> covariance;

        Eigen::Matrix<double, 24, 24> Gt; // state transition matrix 
        Eigen::Matrix<double, 18, 24> Ht; // observation matrix jacobian

        Eigen::Matrix<double, 24, 24> Rt; // process noise covariance matrix
        Eigen::Matrix<double, 18, 18> Qt; // observation noise covariance matrix
        Eigen::Matrix<double, 18, 18> J_fk; // jacobian of forward kinematics to transform measurement noise covariance matrix

        Eigen::Matrix<double, 3, 3> Q_pos; // position noise covariance matrix
        Eigen::Matrix<double, 3, 3> Q_vel; // velocity noise covariance matrix
        Eigen::Matrix<double, 3, 3> Q_foot; // foot contact noise covariance matrix

        Eigen::Matrix<double, 3,3> Ct; // rotation matrix from inertial coordinate from to body coordinate frame
        Eigen::Matrix<double, 18, 1> Ft; // measurement matrix
        
        Eigen::Matrix<double, 18, 18> St;
        Eigen::Matrix<double, 24, 18> Kt; // kalman gain

        Eigen::Matrix<double, 24, 24> I; // 24x24 identity matrix
        
        double update_rate;
        double dt;
        
        // noise parameters
        double n_pos, n_vel, n_foot;
        double n_enc, n_fkin;
        double n_obs;
        
};