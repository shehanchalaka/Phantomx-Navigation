#include <ros/ros.h>
#include <math.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <actionlib_msgs/GoalStatusArray.h>

struct ik_req_t {
    float x;
    float y;
    float z;
    float r;
};


struct vec3_t {
    float x;
    float y;
    float z;
};

#define LEG_COUNT 6

#define AL_FOOT 0
#define AR_FOOT 1
#define BL_FOOT 2
#define BR_FOOT 3
#define CL_FOOT 4
#define CR_FOOT 5

#define STD_TRANSITION  98

#define RIPPLE_SMOOTH   0
#define AMBLE_SMOOTH    1
#define TRIPOD          2

vec3_t endpoints[LEG_COUNT];
float Xspeed;       // forward speed (m/s)
float Yspeed;       // sideward speed (m/s)
float Rspeed;       // rotation speed (rad/s)

#define MOVING   ((Xspeed > 0.005 || Xspeed < -0.005) || (Yspeed > 0.005 || Yspeed < -0.005) || (Rspeed > 0.05 || Rspeed < -0.05))

/* Gait Engine */
int gaitLegNo[LEG_COUNT];       // order to step through legs
ik_req_t gaits[LEG_COUNT];      // gait engine output
float pushSteps;                // how much of the cycle we are on the ground
int stepsInCycle;               // how many steps in this cycle
int step;                       // current step
int tranTime;
float liftHeight;
float cycleTime;                // cycle time in seconds (adjustment from speed to step-size)

bool moving = false;
double sec = 0;

const std::string end_effector_link[] = {"AL_foot_link", "AR_foot_link", "BL_foot_link", "BR_foot_link", "CL_foot_link", "CR_foot_link"};

void gaitSelect(int GaitType){

   if(GaitType == RIPPLE_SMOOTH){    
        gaitLegNo[AR_FOOT] = 0;
        gaitLegNo[CL_FOOT] = 4;
        gaitLegNo[BL_FOOT] = 8;
        gaitLegNo[AL_FOOT] = 12;
        gaitLegNo[CR_FOOT] = 16;
        gaitLegNo[BR_FOOT] = 20;
        pushSteps = 20;
        stepsInCycle = 24;
        tranTime = 65;
    }else if(GaitType == AMBLE_SMOOTH){  
        gaitLegNo[AR_FOOT] = 0;
        gaitLegNo[CL_FOOT] = 0;
        gaitLegNo[AL_FOOT] = 4;
        gaitLegNo[CR_FOOT] = 4;
        gaitLegNo[BR_FOOT] = 8;
        gaitLegNo[BL_FOOT] = 8;
        pushSteps = 8;
        stepsInCycle = 12;
        tranTime = 65;
    }else if(GaitType == TRIPOD){
        gaitLegNo[AR_FOOT] = 0;
        gaitLegNo[BL_FOOT] = 0;
        gaitLegNo[CR_FOOT] = 0;
        gaitLegNo[AL_FOOT] = 4;
        gaitLegNo[BR_FOOT] = 4;
        gaitLegNo[CL_FOOT] = 4;
        pushSteps = 4;
        stepsInCycle = 8;
        tranTime = 65;
    }

    tranTime = STD_TRANSITION;
    cycleTime = (stepsInCycle*tranTime)/1000.0;
    step = 0;

}

void initPhantomX(){

    endpoints[AL_FOOT].x = 0.15;
    endpoints[AL_FOOT].y = 0.20;
    endpoints[AL_FOOT].z = -0.10;

    endpoints[AR_FOOT].x = 0.15;
    endpoints[AR_FOOT].y = -0.20;
    endpoints[AR_FOOT].z = -0.10;

    endpoints[BL_FOOT].x = 0.0;
    endpoints[BL_FOOT].y = 0.24;
    endpoints[BL_FOOT].z = -0.10;

    endpoints[BR_FOOT].x = 0.0;
    endpoints[BR_FOOT].y = -0.24;
    endpoints[BR_FOOT].z = -0.10;

    endpoints[CL_FOOT].x = -0.15;
    endpoints[CL_FOOT].y = 0.20;
    endpoints[CL_FOOT].z = -0.10;

    endpoints[CR_FOOT].x = -0.15;
    endpoints[CR_FOOT].y = -0.20;
    endpoints[CR_FOOT].z = -0.10;

    liftHeight = -0.06;
    stepsInCycle = 1;
    step = 0;

    Xspeed = 0.0;
    Yspeed = 0.0;
    Rspeed = 0.0;

    gaitSelect(TRIPOD);
}


ik_req_t SmoothGaitGen(int leg){
  if( MOVING ){
    // are we moving?
    if(step == gaitLegNo[leg]){
        // leg up, halfway to middle
        gaits[leg].x = gaits[leg].x/2;
        gaits[leg].y = gaits[leg].y/2;
        gaits[leg].z = -liftHeight/2;
        gaits[leg].r = gaits[leg].r/2;
    }else if((step == gaitLegNo[leg]+1) && (gaits[leg].z < 0)){
        // leg up position
        gaits[leg].x = 0;
        gaits[leg].y = 0;
        gaits[leg].z = -liftHeight;
        gaits[leg].r = 0;
    }else if((step == gaitLegNo[leg] + 2) && (gaits[leg].z < 0)){
        // leg halfway down                                
        gaits[leg].x = (Xspeed*cycleTime*pushSteps)/(4*stepsInCycle);
        gaits[leg].y = (Yspeed*cycleTime*pushSteps)/(4*stepsInCycle);  
        gaits[leg].z = -liftHeight/2;                                             
        gaits[leg].r = (Rspeed*cycleTime*pushSteps)/(4*stepsInCycle); 
    }else if((step == gaitLegNo[leg]+3) && (gaits[leg].z < 0)){
        // leg down position                                           NOTE: dutyFactor = pushSteps/StepsInCycle
        gaits[leg].x = (Xspeed*cycleTime*pushSteps)/(2*stepsInCycle);     // travel/Cycle = speed*cycleTime
        gaits[leg].y = (Yspeed*cycleTime*pushSteps)/(2*stepsInCycle);     // Stride = travel/Cycle * dutyFactor
        gaits[leg].z = 0;                                                 //   = speed*cycleTime*pushSteps/stepsInCycle
        gaits[leg].r = (Rspeed*cycleTime*pushSteps)/(2*stepsInCycle);     //   we move Stride/2 here
    }else{
        // move body forward
        gaits[leg].x = gaits[leg].x - (Xspeed*cycleTime)/stepsInCycle;    // note calculations for Stride above
        gaits[leg].y = gaits[leg].y - (Yspeed*cycleTime)/stepsInCycle;    // we have to move Stride/pushSteps here
        gaits[leg].z = 0;                                                 //   = speed*cycleTime*pushSteps/stepsInCycle*pushSteps
        gaits[leg].r = gaits[leg].r - (Rspeed*cycleTime)/stepsInCycle;    //   = speed*cycleTime/stepsInCycle
    }
    sec = ros::Time::now().toSec();
  }else{ 
    // stopped, move leg into default position
    if(ros::Time::now().toSec() - sec < 2){
        return gaits[leg];
    }
    
    if(step == gaitLegNo[leg] && (gaits[leg].x != 0 || gaits[leg].y != 0) && gaits[leg].z == 0){
        gaits[leg].x = gaits[leg].x/2;
        gaits[leg].y = gaits[leg].y/2;
        gaits[leg].z = -liftHeight/2;
        gaits[leg].r = gaits[leg].r/2;
    }else if(step == gaitLegNo[leg]+1 && (gaits[leg].x != 0 || gaits[leg].y != 0) && gaits[leg].z != 0){
        gaits[leg].x = 0;
        gaits[leg].y = 0;
        gaits[leg].z = -liftHeight;
        gaits[leg].r = 0;
    }else{
        gaits[leg].z = 0;
    }    
  }

  return gaits[leg];

}


bool doIK(){

    ik_req_t gait;
    vec3_t pos;

    moveit::planning_interface::MoveGroupInterface robot("robot_group");

    for(int leg = 0; leg < LEG_COUNT; leg++){

        gait = SmoothGaitGen(leg);
        pos.x = endpoints[leg].x + gait.x;
        pos.y = endpoints[leg].y + gait.y;
        pos.z = endpoints[leg].z + gait.z;
        robot.setPositionTarget(pos.x, pos.y, pos.z, end_effector_link[leg]);
    }

    step = (step+1) % stepsInCycle;
    
    return (robot.asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

}

bool moveRobot(){

    moveit::planning_interface::MoveGroupInterface robot("robot_group");

    for(int i = 0; i < LEG_COUNT; i++){
        robot.setPositionTarget(endpoints[i].x, endpoints[i].y, endpoints[i].z, end_effector_link[i]);
    }

    return (robot.asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

}

bool translateRobotBase(float dx, float dy, float dz){

    moveit::planning_interface::MoveGroupInterface robot("robot_group");

    for(int i = 0; i < LEG_COUNT; i++){
        float x = endpoints[i].x + dx;
        float y = endpoints[i].y + dy;
        float z = endpoints[i].z - dz;
        robot.setPositionTarget(x, y, z, end_effector_link[i]);
    }

    return (robot.asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

}

void AL_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){     
    // if(msg->status_list[0].status == 3){
    //     ROS_INFO("yeah");
    // }
    ROS_INFO("%d", msg->status_list[0].status);
}

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "phantomx_control_moveit");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Subscriber sub1 = node_handle.subscribe("/AL_joint_trajectory_controller/follow_joint_trajectory/state", 100, AL_callback);

    // initialize variables
    initPhantomX();

    moveRobot();

    ros::Rate rate(0.5);
    rate.sleep();

    Xspeed = 0.06;

    while(ros::ok()){
        doIK();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}