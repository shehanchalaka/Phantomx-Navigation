#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"

ros::Publisher pub;
ros::Publisher imu_pub;

geometry_msgs::Twist twist;
geometry_msgs::Quaternion quat;

void ALFootSensorCallback(const gazebo_msgs::ContactsState::ConstPtr& msg){

    if(msg->states.empty()){
        std::cout << "Not touching !" << std::endl;
        twist.linear.x = 0;
    } else {
        std::cout << "Touching !" << std::endl;
        twist.linear.x = 1;
    }

    pub.publish(twist);


}

void ARFootSensorCallback(const gazebo_msgs::ContactsState::ConstPtr& msg){

    if(msg->states.empty()){
        twist.linear.y = 0;
    } else {
        twist.linear.y = 1;
    }

    pub.publish(twist);

}

void BLFootSensorCallback(const gazebo_msgs::ContactsState::ConstPtr& msg){

    if(msg->states.empty()){
        twist.linear.z = 0;
    } else {
        twist.linear.z = 1;
    }

    pub.publish(twist);

}

void BRFootSensorCallback(const gazebo_msgs::ContactsState::ConstPtr& msg){

    if(msg->states.empty()){
        twist.angular.x = 0;
    } else {
        twist.angular.x = 1;
    }

    pub.publish(twist);

}

void CLFootSensorCallback(const gazebo_msgs::ContactsState::ConstPtr& msg){

    if(msg->states.empty()){
        twist.angular.y = 0;
    } else {
        twist.angular.y = 1;
    }

    pub.publish(twist);

}

void CRFootSensorCallback(const gazebo_msgs::ContactsState::ConstPtr& msg){

    if(msg->states.empty()){
        twist.angular.z = 0;
    } else {
        twist.angular.z = 1;
    }

    pub.publish(twist);

}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){

    quat.x = msg->orientation.x;
    quat.y = msg->orientation.y;
    quat.z = msg->orientation.z;
    quat.w = msg->orientation.w;

    imu_pub.publish(quat);

}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "gazebo_sensor_publisher");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("phantomx_state_estimator/contacts", 10);
    imu_pub = nh.advertise<geometry_msgs::Quaternion>("phantomx_state_estimator/mpu6050/orientation", 10);

    ros::Subscriber sub1 = nh.subscribe("foot_AL_contact_sensor_state", 10, ALFootSensorCallback);
    ros::Subscriber sub2 = nh.subscribe("foot_AR_contact_sensor_state", 10, ARFootSensorCallback);
    ros::Subscriber sub3 = nh.subscribe("foot_BL_contact_sensor_state", 10, BLFootSensorCallback);
    ros::Subscriber sub4 = nh.subscribe("foot_BR_contact_sensor_state", 10, BRFootSensorCallback);
    ros::Subscriber sub5 = nh.subscribe("foot_CL_contact_sensor_state", 10, CLFootSensorCallback);
    ros::Subscriber sub6 = nh.subscribe("foot_CR_contact_sensor_state", 10, CRFootSensorCallback);
    ros::Subscriber imu_sub = nh.subscribe("imu", 10, ImuCallback);

    ros::spin();

    return 0;
}