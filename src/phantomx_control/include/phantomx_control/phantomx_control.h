#pragma once

#include <ros/ros.h>
#include <string.h>
#include <dynamixel_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <phantomx_control/Command.h>


#define L1  0.048     // coxa length *48mm
#define L2  0.083     // femur length *83mm
#define L3  0.142     // tibia length *142mm

#define X_COXA  0.120   
#define Y_COXA  0.060 
#define M_COXA  0.100 

#define RIPPLE_SMOOTH   0
#define AMBLE_SMOOTH    1
#define TRIPOD          2

#define LEG_COUNT       6

#define LEFT_FRONT      0
#define RIGHT_FRONT     1
#define LEFT_MIDDLE     2
#define RIGHT_MIDDLE    3
#define LEFT_REAR       4
#define RIGHT_REAR      5

#define STD_TRANSITION  98

#define MOVING   ((Xspeed > 0.005 || Xspeed < -0.005) || (Yspeed > 0.005 || Yspeed < -0.005) || (Rspeed > 0.05 || Rspeed < -0.05))



struct ik_req_t {
    double x;
    double y;
    double z;
    double r;
};

struct ik_sol_t {
    double coxa;
    double femur;
    double tibia;
};

struct vec3_t {
    double x;
    double y;
    double z;
};


class PhantomxControl {

    public:
        PhantomxControl(ros::NodeHandle& nodeHandle);
        virtual ~PhantomxControl();


    private:
        vec3_t bodyIK(vec3_t pos, double z_rot, double x_offset, double y_offset);
        ik_sol_t legIK(double x, double y, double z);
        void gaitSelect(int GaitType);
        ik_req_t SmoothGaitGen(int leg);
        void doIK();
        void publishJointGoal();
        void legCommand(int leg_no, ik_sol_t leg);
        void setupPhantomx();
        void PhantomxCommandCallback(const phantomx_control::Command::ConstPtr& msg);
        void timerCallback(const ros::TimerEvent&);


        ros::NodeHandle nodeHandle_;

        ros::Publisher publisher[18];
        ros::Subscriber subscriber;
        ros::Timer timer1;


        double joint_goal[18] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        const int motor_dir[18] = {1,-1,1,1,1,-1,1,-1,1,1,1,-1,1,-1,1,1,1,-1};                                
        const std::string prefix[6] = {"AL", "AR", "BL", "BR", "CL", "CR"};


        vec3_t endpoints[LEG_COUNT];
        double Xspeed;       // forward speed (m/s)
        double Yspeed;       // sideward speed (m/s)
        double Rspeed;       // rotation speed (rad/s)
        struct vec3_t BODY_ROT, BODY_TRA;

        /* Gait Engine */
        int gaitLegNo[LEG_COUNT];       // order to step through legs
        ik_req_t gaits[LEG_COUNT];      // gait engine output
        double pushSteps;                // how much of the cycle we are on the ground
        int stepsInCycle;               // how many steps in this cycle
        int step;                       // current step
        int tranTime;
        double liftHeight;
        double cycleTime;                // cycle time in seconds (adjustment from speed to step-size)
        int gait_;
        double sec = 0; 
        double ik_rate;

        double endpoint_x, endpoint_y, endpoint_z;

};