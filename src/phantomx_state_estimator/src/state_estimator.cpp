#include "phantomx_state_estimator/state_estimator.h"

StateEstimator::StateEstimator(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle)
{
    ROS_INFO("Kalman Filter Constructor!");

    if(!nodeHandle.getParam("imu/topic_name", imu_topic_name_)){
        ROS_ERROR("Could not find topic parameter!");
    }

    nodeHandle_.getParam("imu/topic_name", imu_topic_name_);
    nodeHandle_.getParam("imu/queue_size", imu_queue_size_);

    nodeHandle_.getParam("contact_sensors/topic_name", contact_topic_name_);
    nodeHandle_.getParam("contact_sensors/queue_size", contact_queue_size_);

    nodeHandle_.getParam("kalman_filter/update_rate", update_rate);
    dt = (1/update_rate);    

    nodeHandle_.getParam("position_noise", n_pos);
    nodeHandle_.getParam("velocity_noise", n_vel);
    nodeHandle_.getParam("foot_noise", n_foot);
    nodeHandle_.getParam("encoder_measurement_noise", n_enc);
    nodeHandle_.getParam("forward_kinematic_noise", n_fkin);
    nodeHandle_.getParam("observation_noise", n_obs);

    joint_state_subscriber_ = nodeHandle_.subscribe("/joint_states", 1, &StateEstimator::JointStateCallback, this);
    imu_subscriber_ = nodeHandle_.subscribe(imu_topic_name_, imu_queue_size_, &StateEstimator::IMUCallback, this);
    contact_subscriber_ = nodeHandle_.subscribe(contact_topic_name_, contact_queue_size_, &StateEstimator::ContactSensorCallback, this);

    state_publisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/phantomx_state_estimator/pose", 10);

    // kalman filter thread
    timer1 = nodeHandle_.createTimer(ros::Duration((float)dt), &StateEstimator::timerCallback, this);

    // initialize sensor callback values
    for(int i = 0; i < 18; i ++){ q[i] = 0.0; } // encoder values
    quat[0] = 0; quat[1] = 0; quat[2] = 0; quat[3] = 1; // imu 

    // initially set state to zero matrix and covariance to infinity
    state.setZero();
    state(6,0) = 0.2127;
    state(7,0) = 0.1527;
    state(8,0) = -0.1430;

    state(9,0) = 0.2127;
    state(10,0) = -0.1527;
    state(11,0) = -0.1430;
    
    state(12,0) = 0.0;
    state(13,0) = 0.2311;
    state(14,0) = -0.1432;
    
    state(15,0) = 0;
    state(16,0) = -0.2311;
    state(17,0) = -0.1430;
    
    state(18,0) = -0.2127;
    state(19,0) = 0.1527;
    state(20,0) = -0.1430;

    state(21,0) = -0.2127;
    state(22,0) = -0.1527;
    state(23,0) = -0.1430;


    covariance = INF * covariance.setIdentity();

    // state transition matrix
    Gt.setIdentity();
    Gt(0,3) = dt;    Gt(1,4) = dt;    Gt(2,5) = dt;

    // process noise covariance matrix
    Q_pos = pow(n_pos, 2) * Q_pos.setIdentity();
    Q_vel = pow(n_vel, 2) * Q_vel.setIdentity();
    Q_foot = pow(n_foot, 2) * Q_foot.setIdentity();

    Rt.block<3, 3>(0,0) = (pow(dt,3)/3) * Q_pos + (pow(dt,5)/20) * Q_vel;
    Rt.block<3, 3>(0,3) = (pow(dt,2)/2) * Q_vel + (pow(dt,4)/8) * Q_vel;
    Rt.block<3, 3>(3,0) = (pow(dt,2)/2) * Q_pos + (pow(dt,4)/8) * Q_vel;
    Rt.block<3, 3>(3,3) = dt * Q_vel + (pow(dt,3)/3) * Q_vel;

    // assuming all the legs are in contact with the ground initially
    Rt.block<3, 3>(6,6) = Q_foot;
    Rt.block<3, 3>(9,9) = Q_foot;
    Rt.block<3, 3>(12,12) = Q_foot;
    Rt.block<3, 3>(15,15) = Q_foot;
    Rt.block<3, 3>(18,18) = Q_foot;
    Rt.block<3, 3>(21,21) = Q_foot;    

    // std::cout << Rt << std::endl;

    // observation noise covariance matrix
    Qt = pow(n_obs, 2) * Qt.setIdentity();

    // jacobian of forward kinematics to transform measurement noise covariance matrix
    J_fk.setIdentity();

    // set Ct to identity matrix initially
    Ct.setIdentity();
    
    // initialize Ht matrix depending on Ct
    Ht.block<3, 3>(0,0) = -Ct;
    Ht.block<3, 3>(3,0) = -Ct;
    Ht.block<3, 3>(6,0) = -Ct;
    Ht.block<3, 3>(9,0) = -Ct;
    Ht.block<3, 3>(12,0) = -Ct;
    Ht.block<3, 3>(15,0) = -Ct;
    Ht.block<3, 3>(0,6) = Ct;
    Ht.block<3, 3>(3,9) = Ct;
    Ht.block<3, 3>(6,12) = Ct;
    Ht.block<3, 3>(9,15) = Ct;
    Ht.block<3, 3>(12,18) = Ct;
    Ht.block<3, 3>(15,21) = Ct;

    Ft(0,0) = 0.2127;
    Ft(1,0) = 0.1527;
    Ft(2,0) = -0.1430;

    Ft(3,0) = 0.2127;
    Ft(4,0) = -0.1527;
    Ft(5,0) = -0.1430;
    
    Ft(6,0) = 0.0;
    Ft(7,0) = 0.2311;
    Ft(8,0) = -0.1432;
    
    Ft(9,0) = 0;
    Ft(10,0) = -0.2311;
    Ft(11,0) = -0.1430;
    
    Ft(12,0) = -0.2127;
    Ft(13,0) = 0.1527;
    Ft(14,0) = -0.1430;

    Ft(15,0) = -0.2127;
    Ft(16,0) = -0.1527;
    Ft(17,0) = -0.1430; 

    I.setIdentity();   

}

StateEstimator::~StateEstimator(){    
}

void StateEstimator::KalmanPredict(){

    // update Gt matrix if dt is updated
    // Gt(0,3) = dt;    Gt(1,4) = dt;    Gt(2,5) = dt;

    state = Gt * state;
    covariance = Gt * covariance * Gt.transpose() + Rt;

}

void StateEstimator::KalmanUpdate(){

    Eigen::Matrix<double, 18, 1> yt;
    Eigen::Matrix<double, 18, 1> Obs;
    Eigen::Matrix<double, 18, 18> Rot;

    Obs(0,0) = state(6,0)-state(0,0);    Obs(1,0) = state(7,0)-state(1,0);    Obs(2,0) = state(8,0)-state(2,0);
    Obs(3,0) = state(9,0)-state(0,0);    Obs(4,0) = state(10,0)-state(1,0);    Obs(5,0) = state(11,0)-state(2,0);
    Obs(6,0) = state(12,0)-state(0,0);    Obs(7,0) = state(13,0)-state(1,0);    Obs(8,0) = state(14,0)-state(2,0);
    Obs(9,0) = state(15,0)-state(0,0);    Obs(10,0) = state(16,0)-state(1,0);    Obs(11,0) = state(17,0)-state(2,0);
    Obs(12,0) = state(18,0)-state(0,0);    Obs(13,0) = state(19,0)-state(1,0);    Obs(14,0) = state(20,0)-state(2,0);
    Obs(15,0) = state(21,0)-state(0,0);    Obs(16,0) = state(22,0)-state(1,0);    Obs(17,0) = state(23,0)-state(2,0);

    Rot.setIdentity();
    Rot.block<3, 3>(0,0) = Ct;
    Rot.block<3, 3>(3,3) = Ct;
    Rot.block<3, 3>(6,6) = Ct;
    Rot.block<3, 3>(9,9) = Ct;
    Rot.block<3, 3>(12,12) = Ct;
    Rot.block<3, 3>(15,15) = Ct;

    calculateMeasurementMatrix(); // Ft calculation
    yt = Ft - (Rot * Obs);

    calculateMeasurementJacobian(); // J_fk calculation
    St = Ht * covariance * Ht.transpose() + J_fk * Qt * J_fk.transpose();
    
    Kt = covariance * Ht.transpose() * St.inverse();
    
    state = state + Kt * yt;
    covariance = (I - Kt * Ht) * covariance;
    
}

void StateEstimator::JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){

    for(int i = 0; i < 18; i ++){
        q[i] = msg->position[i];
    }
}

void StateEstimator::calculateMeasurementMatrix(){

    Ft(0,0) = a + (1.414*(L1*cos(q[0]) + L2*cos(q[0])*cos(q[1]) + L3*cos(q[2] - 1.57)*cos(q[0])*cos(q[1]) + L3*sin(q[2] - 1.57)*cos(q[0])*sin(q[1])))/2 - (1.414*(L1*sin(q[0]) + L2*cos(q[1])*sin(q[0]) + L3*cos(q[2] - 1.57)*cos(q[1])*sin(q[0]) + L3*sin(q[2] - 1.57)*sin(q[0])*sin(q[1])))/2;
    Ft(1,0) = b + (1.414*(L1*cos(q[0]) + L2*cos(q[0])*cos(q[1]) + L3*cos(q[2] - 1.57)*cos(q[0])*cos(q[1]) + L3*sin(q[2] - 1.57)*cos(q[0])*sin(q[1])))/2 + (1.414*(L1*sin(q[0]) + L2*cos(q[1])*sin(q[0]) + L3*cos(q[2] - 1.57)*cos(q[1])*sin(q[0]) + L3*sin(q[2] - 1.57)*sin(q[0])*sin(q[1])))/2;
    Ft(2,0) = L3*sin(q[2] - 1.57)*cos(q[1]) - L3*cos(q[2] - 1.57)*sin(q[1]) - L2*sin(q[1]);

    Ft(3,0) = a + (1.414*(L1*cos(q[3]) + L2*cos(q[3])*cos(q[4]) + L3*cos(q[5] + 1.57)*cos(q[3])*cos(q[4]) + L3*sin(q[5] + 1.57)*cos(q[3])*sin(q[4])))/2 + (1.414*(L1*sin(q[3]) + L2*cos(q[4])*sin(q[3]) + L3*cos(q[5] + 1.57)*cos(q[4])*sin(q[3]) + L3*sin(q[5] + 1.57)*sin(q[3])*sin(q[4])))/2;
    Ft(4,0) = (1.414*(L1*sin(q[3]) + L2*cos(q[4])*sin(q[3]) + L3*cos(q[5] + 1.57)*cos(q[4])*sin(q[3]) + L3*sin(q[5] + 1.57)*sin(q[3])*sin(q[4])))/2 - (1.414*(L1*cos(q[3]) + L2*cos(q[3])*cos(q[4]) + L3*cos(q[5] + 1.57)*cos(q[3])*cos(q[4]) + L3*sin(q[5] + 1.57)*cos(q[3])*sin(q[4])))/2 - b;
    Ft(5,0) = L2*sin(q[4]) + L3*cos(q[5] + 1.57)*sin(q[4]) - L3*sin(q[5] + 1.57)*cos(q[4]);

    Ft(6,0) = - L1*sin(q[6]) - L2*cos(q[7])*sin(q[6]) - L3*cos(q[8] - 1.57)*cos(q[7])*sin(q[6]) - L3*sin(q[8] - 1.57)*sin(q[6])*sin(q[7]);
    Ft(7,0) = c + L1*cos(q[6]) + L2*cos(q[6])*cos(q[7]) + L3*cos(q[8] - 1.57)*cos(q[6])*cos(q[7]) + L3*sin(q[8] - 1.57)*cos(q[6])*sin(q[7]);
    Ft(8,0) = L3*sin(q[8] - 1.57)*cos(q[7]) - L3*cos(q[8] - 1.57)*sin(q[7]) - L2*sin(q[7]);

    Ft(9,0) = L1*sin(q[9]) + L2*cos(q[10])*sin(q[9]) + L3*cos(q[11] + 1.57)*cos(q[10])*sin(q[9]) + L3*sin(q[11] + 1.57)*sin(q[9])*sin(q[10]);
    Ft(10,0) = - c - L1*cos(q[9]) - L2*cos(q[9])*cos(q[10]) - L3*cos(q[11] + 1.57)*cos(q[9])*cos(q[10]) - L3*sin(q[11] + 1.57)*cos(q[9])*sin(q[10]);
    Ft(11,0) = L2*sin(q[10]) + L3*cos(q[11] + 1.57)*sin(q[10]) - L3*sin(q[11] + 1.57)*cos(q[10]);

    Ft(12,0) = - a - (1.414*(L1*cos(q[12]) + L2*cos(q[12])*cos(q[13]) + L3*cos(q[14] - 1.57)*cos(q[12])*cos(q[13]) + L3*sin(q[14] - 1.57)*cos(q[12])*sin(q[13])))/2 - (1.414*(L1*sin(q[12]) + L2*cos(q[13])*sin(q[12]) + L3*cos(q[14] - 1.57)*cos(q[13])*sin(q[12]) + L3*sin(q[14] - 1.57)*sin(q[12])*sin(q[13])))/2;
    Ft(13,0) = b + (1.414*(L1*cos(q[12]) + L2*cos(q[12])*cos(q[13]) + L3*cos(q[14] - 1.57)*cos(q[12])*cos(q[13]) + L3*sin(q[14] - 1.57)*cos(q[12])*sin(q[13])))/2 - (1.414*(L1*sin(q[12]) + L2*cos(q[13])*sin(q[12]) + L3*cos(q[14] - 1.57)*cos(q[13])*sin(q[12]) + L3*sin(q[14] - 1.57)*sin(q[12])*sin(q[13])))/2;
    Ft(14,0) = L3*sin(q[14] - 1.57)*cos(q[13]) - L3*cos(q[14] - 1.57)*sin(q[13]) - L2*sin(q[13]);

    Ft(15,0) = (1.414*(L1*sin(q[15]) + L2*cos(q[16])*sin(q[15]) + L3*cos(q[17] + 1.57)*cos(q[16])*sin(q[15]) + L3*sin(q[17] + 1.57)*sin(q[15])*sin(q[16])))/2 - (1.414*(L1*cos(q[15]) + L2*cos(q[15])*cos(q[16]) + L3*cos(q[17] + 1.57)*cos(q[15])*cos(q[16]) + L3*sin(q[17] + 1.57)*cos(q[15])*sin(q[16])))/2 - a;
    Ft(16,0) = - b - (1.414*(L1*cos(q[15]) + L2*cos(q[15])*cos(q[16]) + L3*cos(q[17] + 1.57)*cos(q[15])*cos(q[16]) + L3*sin(q[17] + 1.57)*cos(q[15])*sin(q[16])))/2 - (1.414*(L1*sin(q[15]) + L2*cos(q[16])*sin(q[15]) + L3*cos(q[17] + 1.57)*cos(q[16])*sin(q[15]) + L3*sin(q[17] + 1.57)*sin(q[15])*sin(q[16])))/2;
    Ft(17,0) = L2*sin(q[16]) + L3*cos(q[17] + 1.57)*sin(q[16]) - L3*sin(q[17] + 1.57)*cos(q[16]);

    // std::cout << "Ft matrix" << std::endl;
    // std::cout << Ft << std::endl;

}

void StateEstimator::calculateMeasurementJacobian(){

    double q1, q2, q3;

    J_fk(0,0) = - (1.414*(L1*cos(q1) + L2*cos(q1)*cos(q2) + L3*cos(q3 - 1.57)*cos(q1)*cos(q2) + L3*sin(q3 - 1.57)*cos(q1)*sin(q2)))/2 - (1.414*(L1*sin(q1) + L2*cos(q2)*sin(q1) + L3*cos(q3 - 1.57)*cos(q2)*sin(q1) + L3*sin(q3 - 1.57)*sin(q1)*sin(q2)))/2;
    J_fk(0,1) = (1.414*(L2*sin(q1)*sin(q2) + L3*cos(q3 - 1.57)*sin(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q2)*sin(q1)))/2 - (1.414*(L2*cos(q1)*sin(q2) + L3*cos(q3 - 1.57)*cos(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q1)*cos(q2)))/2;
    J_fk(0,2) = (1.414*(L3*cos(q3 - 1.57)*cos(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q1)*cos(q2)))/2 - (1.414*(L3*cos(q3 - 1.57)*sin(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q2)*sin(q1)))/2;
    
    J_fk(1,0) = (1.414*(L1*cos(q1) + L2*cos(q1)*cos(q2) + L3*cos(q3 - 1.57)*cos(q1)*cos(q2) + L3*sin(q3 - 1.57)*cos(q1)*sin(q2)))/2 - (1.414*(L1*sin(q1) + L2*cos(q2)*sin(q1) + L3*cos(q3 - 1.57)*cos(q2)*sin(q1) + L3*sin(q3 - 1.57)*sin(q1)*sin(q2)))/2;
    J_fk(1,1) = - (1.414*(L2*cos(q1)*sin(q2) + L3*cos(q3 - 1.57)*cos(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q1)*cos(q2)))/2 - (1.414*(L2*sin(q1)*sin(q2) + L3*cos(q3 - 1.57)*sin(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q2)*sin(q1)))/2;
    J_fk(1,2) = (1.414*(L3*cos(q3 - 1.57)*cos(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q1)*cos(q2)))/2 + (1.414*(L3*cos(q3 - 1.57)*sin(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q2)*sin(q1)))/2;
    
    J_fk(2,0) = 0;
    J_fk(2,1) = - L2*cos(q2) - L3*cos(q3 - 1.57)*cos(q2) - L3*sin(q3 - 1.57)*sin(q2);
    J_fk(2,2) = L3*cos(q3 - 1.57)*cos(q2) + L3*sin(q3 - 1.57)*sin(q2);
 
    J_fk(3,3) = (1.414*(L1*cos(q1) + L2*cos(q1)*cos(q2) + L3*cos(q3 + 1.57)*cos(q1)*cos(q2) + L3*sin(q3 + 1.57)*cos(q1)*sin(q2)))/2 - (1.414*(L1*sin(q1) + L2*cos(q2)*sin(q1) + L3*cos(q3 + 1.57)*cos(q2)*sin(q1) + L3*sin(q3 + 1.57)*sin(q1)*sin(q2)))/2;
    J_fk(3,4) = - (1.414*(L2*cos(q1)*sin(q2) + L3*cos(q3 + 1.57)*cos(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q1)*cos(q2)))/2 - (1.414*(L2*sin(q1)*sin(q2) + L3*cos(q3 + 1.57)*sin(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q2)*sin(q1)))/2;
    J_fk(3,5) = (1.414*(L3*cos(q3 + 1.57)*cos(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q1)*cos(q2)))/2 + (1.414*(L3*cos(q3 + 1.57)*sin(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q2)*sin(q1)))/2;
 
    J_fk(4,3) = (1.414*(L1*cos(q1) + L2*cos(q1)*cos(q2) + L3*cos(q3 + 1.57)*cos(q1)*cos(q2) + L3*sin(q3 + 1.57)*cos(q1)*sin(q2)))/2 + (1.414*(L1*sin(q1) + L2*cos(q2)*sin(q1) + L3*cos(q3 + 1.57)*cos(q2)*sin(q1) + L3*sin(q3 + 1.57)*sin(q1)*sin(q2)))/2;
    J_fk(4,4) = (1.414*(L2*cos(q1)*sin(q2) + L3*cos(q3 + 1.57)*cos(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q1)*cos(q2)))/2 - (1.414*(L2*sin(q1)*sin(q2) + L3*cos(q3 + 1.57)*sin(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q2)*sin(q1)))/2;
    J_fk(4,5) = (1.414*(L3*cos(q3 + 1.57)*sin(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q2)*sin(q1)))/2 - (1.414*(L3*cos(q3 + 1.57)*cos(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q1)*cos(q2)))/2;
 
    J_fk(5,3) = 0;
    J_fk(5,4) = L2*cos(q2) + L3*cos(q3 + 1.57)*cos(q2) + L3*sin(q3 + 1.57)*sin(q2);
    J_fk(5,5) = - L3*cos(q3 + 1.57)*cos(q2) - L3*sin(q3 + 1.57)*sin(q2);
 
    J_fk(6,6) = - L1*cos(q1) - L2*cos(q1)*cos(q2) - L3*cos(q3 - 1.57)*cos(q1)*cos(q2) - L3*sin(q3 - 1.57)*cos(q1)*sin(q2);
    J_fk(6,7) = L2*sin(q1)*sin(q2) + L3*cos(q3 - 1.57)*sin(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q2)*sin(q1);
    J_fk(6,8) = L3*sin(q3 - 1.57)*cos(q2)*sin(q1) - L3*cos(q3 - 1.57)*sin(q1)*sin(q2);
 
    J_fk(7,6) = - L1*sin(q1) - L2*cos(q2)*sin(q1) - L3*cos(q3 - 1.57)*cos(q2)*sin(q1) - L3*sin(q3 - 1.57)*sin(q1)*sin(q2);
    J_fk(7,7) = L3*sin(q3 - 1.57)*cos(q1)*cos(q2) - L3*cos(q3 - 1.57)*cos(q1)*sin(q2) - L2*cos(q1)*sin(q2);
    J_fk(7,8) = L3*cos(q3 - 1.57)*cos(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q1)*cos(q2);
 
    J_fk(8,6) = 0;
    J_fk(8,7) = - L2*cos(q2) - L3*cos(q3 - 1.57)*cos(q2) - L3*sin(q3 - 1.57)*sin(q2);
    J_fk(8,8) = L3*cos(q3 - 1.57)*cos(q2) + L3*sin(q3 - 1.57)*sin(q2);
 
    J_fk(9,9) = L1*cos(q1) + L2*cos(q1)*cos(q2) + L3*cos(q3 + 1.57)*cos(q1)*cos(q2) + L3*sin(q3 + 1.57)*cos(q1)*sin(q2);
    J_fk(9,10) = L3*sin(q3 + 1.57)*cos(q2)*sin(q1) - L3*cos(q3 + 1.57)*sin(q1)*sin(q2) - L2*sin(q1)*sin(q2);
    J_fk(9,11) = L3*cos(q3 + 1.57)*sin(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q2)*sin(q1);
 
    J_fk(10,9) = L1*sin(q1) + L2*cos(q2)*sin(q1) + L3*cos(q3 + 1.57)*cos(q2)*sin(q1) + L3*sin(q3 + 1.57)*sin(q1)*sin(q2);
    J_fk(10,10) = L2*cos(q1)*sin(q2) + L3*cos(q3 + 1.57)*cos(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q1)*cos(q2);
    J_fk(10,11) = L3*sin(q3 + 1.57)*cos(q1)*cos(q2) - L3*cos(q3 + 1.57)*cos(q1)*sin(q2);
 
    J_fk(11,9) = 0;
    J_fk(11,10) = L2*cos(q2) + L3*cos(q3 + 1.57)*cos(q2) + L3*sin(q3 + 1.57)*sin(q2);
    J_fk(11,11) = - L3*cos(q3 + 1.57)*cos(q2) - L3*sin(q3 + 1.57)*sin(q2);
 
    J_fk(12,12) = (1.414*(L1*sin(q1) + L2*cos(q2)*sin(q1) + L3*cos(q3 - 1.57)*cos(q2)*sin(q1) + L3*sin(q3 - 1.57)*sin(q1)*sin(q2)))/2 - (1.414*(L1*cos(q1) + L2*cos(q1)*cos(q2) + L3*cos(q3 - 1.57)*cos(q1)*cos(q2) + L3*sin(q3 - 1.57)*cos(q1)*sin(q2)))/2;
    J_fk(12,13) = (1.414*(L2*cos(q1)*sin(q2) + L3*cos(q3 - 1.57)*cos(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q1)*cos(q2)))/2 + (1.414*(L2*sin(q1)*sin(q2) + L3*cos(q3 - 1.57)*sin(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q2)*sin(q1)))/2;
    J_fk(12,14) = - (1.414*(L3*cos(q3 - 1.57)*cos(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q1)*cos(q2)))/2 - (1.414*(L3*cos(q3 - 1.57)*sin(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q2)*sin(q1)))/2;
 
    J_fk(13,12) = - (1.414*(L1*cos(q1) + L2*cos(q1)*cos(q2) + L3*cos(q3 - 1.57)*cos(q1)*cos(q2) + L3*sin(q3 - 1.57)*cos(q1)*sin(q2)))/2 - (1.414*(L1*sin(q1) + L2*cos(q2)*sin(q1) + L3*cos(q3 - 1.57)*cos(q2)*sin(q1) + L3*sin(q3 - 1.57)*sin(q1)*sin(q2)))/2;
    J_fk(13,13) = (1.414*(L2*sin(q1)*sin(q2) + L3*cos(q3 - 1.57)*sin(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q2)*sin(q1)))/2 - (1.414*(L2*cos(q1)*sin(q2) + L3*cos(q3 - 1.57)*cos(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q1)*cos(q2)))/2;
    J_fk(13,14) = (1.414*(L3*cos(q3 - 1.57)*cos(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q1)*cos(q2)))/2 - (1.414*(L3*cos(q3 - 1.57)*sin(q1)*sin(q2) - L3*sin(q3 - 1.57)*cos(q2)*sin(q1)))/2;
 
    J_fk(14,12) = 0;
    J_fk(14,13) = - L2*cos(q2) - L3*cos(q3 - 1.57)*cos(q2) - L3*sin(q3 - 1.57)*sin(q2);
    J_fk(14,14) = L3*cos(q3 - 1.57)*cos(q2) + L3*sin(q3 - 1.57)*sin(q2);
 
    J_fk(15,15) = (1.414*(L1*cos(q1) + L2*cos(q1)*cos(q2) + L3*cos(q3 + 1.57)*cos(q1)*cos(q2) + L3*sin(q3 + 1.57)*cos(q1)*sin(q2)))/2 + (1.414*(L1*sin(q1) + L2*cos(q2)*sin(q1) + L3*cos(q3 + 1.57)*cos(q2)*sin(q1) + L3*sin(q3 + 1.57)*sin(q1)*sin(q2)))/2;
    J_fk(15,16) = (1.414*(L2*cos(q1)*sin(q2) + L3*cos(q3 + 1.57)*cos(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q1)*cos(q2)))/2 - (1.414*(L2*sin(q1)*sin(q2) + L3*cos(q3 + 1.57)*sin(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q2)*sin(q1)))/2;
    J_fk(15,17) = (1.414*(L3*cos(q3 + 1.57)*sin(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q2)*sin(q1)))/2 - (1.414*(L3*cos(q3 + 1.57)*cos(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q1)*cos(q2)))/2;
 
    J_fk(16,15) = (1.414*(L1*sin(q1) + L2*cos(q2)*sin(q1) + L3*cos(q3 + 1.57)*cos(q2)*sin(q1) + L3*sin(q3 + 1.57)*sin(q1)*sin(q2)))/2 - (1.414*(L1*cos(q1) + L2*cos(q1)*cos(q2) + L3*cos(q3 + 1.57)*cos(q1)*cos(q2) + L3*sin(q3 + 1.57)*cos(q1)*sin(q2)))/2;
    J_fk(16,16) = (1.414*(L2*cos(q1)*sin(q2) + L3*cos(q3 + 1.57)*cos(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q1)*cos(q2)))/2 + (1.414*(L2*sin(q1)*sin(q2) + L3*cos(q3 + 1.57)*sin(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q2)*sin(q1)))/2;
    J_fk(16,17) = - (1.414*(L3*cos(q3 + 1.57)*cos(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q1)*cos(q2)))/2 - (1.414*(L3*cos(q3 + 1.57)*sin(q1)*sin(q2) - L3*sin(q3 + 1.57)*cos(q2)*sin(q1)))/2;
 
    J_fk(17,15) = 0;
    J_fk(17,16) = L2*cos(q2) + L3*cos(q3 + 1.57)*cos(q2) + L3*sin(q3 + 1.57)*sin(q2);
    J_fk(17,17) = - L3*cos(q3 + 1.57)*cos(q2) - L3*sin(q3 + 1.57)*sin(q2);

}

void StateEstimator::IMUCallback(const geometry_msgs::Quaternion::ConstPtr& msg){

    // ROS_INFO("Quaternion: %f %f %f %f", msg->x, msg->y, msg->z, msg->w);
    quat[0] = msg->x;
    quat[1] = msg->y;
    quat[2] = msg->z;
    quat[3] = msg->w;

    // update rotation matrix Ct
    Eigen::Quaterniond q(msg->w, msg->x, msg->y, msg->z);    
    Ct = q.normalized().toRotationMatrix();

    // update observation matrix jacobian
    Ht.block<3, 3>(0,0) = -Ct;
    Ht.block<3, 3>(3,0) = -Ct;
    Ht.block<3, 3>(6,0) = -Ct;
    Ht.block<3, 3>(9,0) = -Ct;
    Ht.block<3, 3>(12,0) = -Ct;
    Ht.block<3, 3>(15,0) = -Ct;
    Ht.block<3, 3>(0,6) = Ct;
    Ht.block<3, 3>(3,9) = Ct;
    Ht.block<3, 3>(6,12) = Ct;
    Ht.block<3, 3>(9,15) = Ct;
    Ht.block<3, 3>(12,18) = Ct;
    Ht.block<3, 3>(15,21) = Ct;

    // auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    // std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;

}

void StateEstimator::ContactSensorCallback(const geometry_msgs::Twist::ConstPtr& msg){

    double touch[] = {
        msg->linear.x,
        msg->linear.y,
        msg->linear.z,
        msg->angular.x,
        msg->angular.y,
        msg->angular.z
        };

    Eigen::Matrix<double, 3, 3> inf_matrix;
    inf_matrix = INF * inf_matrix.setIdentity();

    for(int i=0; i < 6; i++){
        /** set process noise parameter to infinity if the foot is not 
            touching the ground. This enables the corresponding foothold 
            to relocate and reset its position estimate when it regains 
            ground contact. 
         **/
        if(!touch[i]){
            // std::cout << "leg " << i << " is up!" << std::endl;
            Rt.block<3, 3>(6+3*i, 6+3*i) = inf_matrix;
        } else {
            Rt.block<3, 3>(6+3*i, 6+3*i) = Q_foot;
        }
    }

    // std::cout << std::endl;
    // std::cout << Rt << std::endl;

}

void StateEstimator::publishTransform(){

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = -state(0,0);
    transformStamped.transform.translation.y = -state(1,0);
    transformStamped.transform.translation.z = state(2,0);
    
    transformStamped.transform.rotation.x = quat[0];
    transformStamped.transform.rotation.y = quat[1];
    transformStamped.transform.rotation.z = quat[2];
    transformStamped.transform.rotation.w = quat[3];

    br.sendTransform(transformStamped);

    geometry_msgs::PoseStamped poseStamped;

    poseStamped.header.stamp = ros::Time::now();
    poseStamped.header.frame_id = "world";
    poseStamped.pose.position.x = -state(0,0);
    poseStamped.pose.position.y = -state(1,0);
    poseStamped.pose.position.z = state(2,0);
    poseStamped.pose.orientation.x = quat[0];
    poseStamped.pose.orientation.y = quat[1];
    poseStamped.pose.orientation.z = quat[2];
    poseStamped.pose.orientation.w = quat[3];

    state_publisher_.publish(poseStamped);

}

void StateEstimator::timerCallback(const ros::TimerEvent&){

    KalmanPredict();
    KalmanUpdate();

    publishTransform();

    std::cout << "[STATE]\tX=" << state(0,0)*100 << "\tY=" << state(1,0)*100 << "\tZ=" << state(2,0)*100 <<std::endl;
    // std::cout << covariance << std::endl;

    // std::cout << "[P1]\tX=" << state(6,0)*100 << "\tY=" << state(7,0)*100 << "\tZ=" << state(8,0)*100 <<std::endl;

}