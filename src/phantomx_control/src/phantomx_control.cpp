#include "phantomx_control/phantomx_control.h"

PhantomxControl::PhantomxControl(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle)
{
    ROS_INFO("Initializing PhantomX Control Node");

    if(!nodeHandle.getParam("ik_rate", ik_rate)){
        ROS_ERROR("Could not find ik_rate parameter!");
    }

    nodeHandle_.getParam("gait", gait_);
    nodeHandle_.getParam("legLiftHeight", liftHeight);
    nodeHandle_.getParam("speed/x", Xspeed);
    nodeHandle_.getParam("speed/y", Yspeed);
    nodeHandle_.getParam("speed/z", Rspeed);
    nodeHandle_.getParam("translation/x", BODY_TRA.x);
    nodeHandle_.getParam("translation/y", BODY_TRA.y);
    nodeHandle_.getParam("translation/z", BODY_TRA.z);
    nodeHandle_.getParam("rotation/x", BODY_ROT.x);
    nodeHandle_.getParam("rotation/y", BODY_ROT.y);
    nodeHandle_.getParam("rotation/z", BODY_ROT.z);
    nodeHandle_.getParam("endpoints/x", endpoint_x);
    nodeHandle_.getParam("endpoints/y", endpoint_y);
    nodeHandle_.getParam("endpoints/z", endpoint_z);

    
    for(int i = 0; i < LEG_COUNT; i++){
        int j = i * 3;
        std::string ns = "";
        publisher[j] = nodeHandle_.advertise<std_msgs::Float64>(ns + "/" + prefix[i] + "_coxa_joint/command", 10);
        publisher[j+1] = nodeHandle_.advertise<std_msgs::Float64>(ns + "/" + prefix[i] + "_femur_joint/command", 10);
        publisher[j+2] = nodeHandle_.advertise<std_msgs::Float64>(ns + "/" + prefix[i] + "_tibia_joint/command", 10);
    }

    subscriber = nodeHandle_.subscribe("/phantomx_command", 1, &PhantomxControl::PhantomxCommandCallback, this);

    timer1 = nodeHandle_.createTimer(ros::Duration((double)(1/ik_rate)), &PhantomxControl::timerCallback, this);

    setupPhantomx();

    ROS_INFO("Listening for incoming commands.");

}

PhantomxControl::~PhantomxControl(){

}

vec3_t PhantomxControl::bodyIK(vec3_t pos, double z_rot, double x_offset, double y_offset){
    
    // BODY_ROT : body rotayion
    // BODY_TRA : body translation
    // (x_offset, y_offset) : distance to coxa joint from base coordinate system in meters
    
    vec3_t ans;
    double c1 = cos(BODY_ROT.x);
    double s1 = sin(BODY_ROT.x);
    double c2 = cos(BODY_ROT.y);
    double s2 = sin(BODY_ROT.y);
    double c3 = cos(BODY_ROT.z + z_rot);
    double s3 = sin(BODY_ROT.z + z_rot);

    double X = pos.x + x_offset;
    double Y = pos.y + y_offset;
    double Z = pos.z;

    ans.x = X - (c2*c3*X - c1*s3*Y + c3*s1*s2*Y + c1*c3*s2*Z + s1*s3*Z) + BODY_TRA.x;
    ans.y = Y - (c2*s3*X + c1*c3*Y + s1*s2*s3*Y + c1*s2*s3*Z - c3*s1*Z) + BODY_TRA.y;
    ans.z = Z - (-s2*X + c2*s1*Y + c1*c2*Z) + BODY_TRA.z;

    return ans;

}

ik_sol_t PhantomxControl::legIK(double x, double y, double z){

    ik_sol_t ans;
    double truex, im, q1, q2, q3, d1, d2;

    ans.coxa = atan2(y, x);

    truex = sqrt( pow(y,2) + pow(x,2) ) - L1;
    im = sqrt( pow(truex,2) + pow(z,2) );

    q1 = atan2(z, truex);
    d1 = pow(L2,2) + pow(im,2) - pow(L3,2);
    d2 = 2 * L2 * im;
    q2 = acos(d1/d2);
    ans.femur = q2 - q1;

    d1 = pow(L2,2) + pow(L3,2) - pow(im,2);
    d2 = 2 * L2 * L3;
    q3 = acos(d1/d2);
    ans.tibia = q3 - 3.141592652;
    
    return ans;

}

void PhantomxControl::gaitSelect(int GaitType){

   if(GaitType == RIPPLE_SMOOTH){    
        gaitLegNo[RIGHT_FRONT] = 0;
        gaitLegNo[LEFT_REAR] = 4;
        gaitLegNo[LEFT_MIDDLE] = 8;
        gaitLegNo[LEFT_FRONT] = 12;
        gaitLegNo[RIGHT_REAR] = 16;
        gaitLegNo[RIGHT_MIDDLE] = 20;
        pushSteps = 20;
        stepsInCycle = 24;
        tranTime = 65;
    }else if(GaitType == AMBLE_SMOOTH){  
        gaitLegNo[RIGHT_FRONT] = 0;
        gaitLegNo[LEFT_REAR] = 0;
        gaitLegNo[LEFT_FRONT] = 4;
        gaitLegNo[RIGHT_REAR] = 4;
        gaitLegNo[RIGHT_MIDDLE] = 8;
        gaitLegNo[LEFT_MIDDLE] = 8;
        pushSteps = 8;
        stepsInCycle = 12;
        tranTime = 65;
    }else if(GaitType == TRIPOD){
        gaitLegNo[RIGHT_FRONT] = 0;
        gaitLegNo[LEFT_MIDDLE] = 0;
        gaitLegNo[RIGHT_REAR] = 0;
        gaitLegNo[LEFT_FRONT] = 4;
        gaitLegNo[RIGHT_MIDDLE] = 4;
        gaitLegNo[LEFT_REAR] = 4;
        pushSteps = 4;
        stepsInCycle = 8;
        tranTime = 65;
    }

    tranTime = STD_TRANSITION;
    cycleTime = (stepsInCycle*tranTime)/1000.0;
    step = 0;

}

ik_req_t PhantomxControl::SmoothGaitGen(int leg){
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

void PhantomxControl::doIK(){

    ik_req_t gait;
    ik_sol_t sol;
    vec3_t req, pos;

    double x_off[] = {X_COXA, X_COXA, 0, 0, -X_COXA, -X_COXA};
    double y_off[] = {Y_COXA, -Y_COXA, M_COXA, -M_COXA, Y_COXA, -Y_COXA}; 

    for(int leg = 0; leg < LEG_COUNT; leg++){

        gait = SmoothGaitGen(leg);
        pos.x = endpoints[leg].x + gait.x;
        pos.y = endpoints[leg].y + gait.y;
        pos.z = endpoints[leg].z + gait.z;
        req = bodyIK(pos, gait.r, x_off[leg], y_off[leg]);
        sol = legIK(pos.x+req.x, pos.y+req.y, pos.z+req.z);
        legCommand(leg, sol);
    }

    step = (step+1) % stepsInCycle;  

}

void PhantomxControl::publishJointGoal(){ 
    
    for(int i = 0; i < 18; i++){
        std_msgs::Float64 msg;
        msg.data = joint_goal[i];
        publisher[i].publish(msg);
    }    
}

void PhantomxControl::legCommand(int leg_no, ik_sol_t leg){

    int i = 3 * leg_no;
    double coxa_offset[] = {-0.7853, 0.7853, -1.5707, 1.5707, -2.3561, 2.3561};
    double tibia_offset = 1.5707;
    
    joint_goal[i] = motor_dir[i] * (leg.coxa + coxa_offset[leg_no]);
    joint_goal[i+1] = motor_dir[i+1] * (leg.femur);
    joint_goal[i+2] = motor_dir[i+2] * (leg.tibia + tibia_offset);

}

void PhantomxControl::setupPhantomx(){

    ROS_INFO("Setting up PhantomX Hexapod");\

    endpoints[RIGHT_FRONT].x = endpoint_x;  
    endpoints[RIGHT_FRONT].y = -endpoint_y;
    endpoints[RIGHT_FRONT].z = endpoint_z;

    endpoints[RIGHT_REAR].x = -endpoint_x;
    endpoints[RIGHT_REAR].y = -endpoint_y;
    endpoints[RIGHT_REAR].z = endpoint_z;

    endpoints[RIGHT_MIDDLE].x = 0.0;
    endpoints[RIGHT_MIDDLE].y = -endpoint_y;
    endpoints[RIGHT_MIDDLE].z = endpoint_z;

    endpoints[LEFT_MIDDLE].x = 0.0;
    endpoints[LEFT_MIDDLE].y = endpoint_y;
    endpoints[LEFT_MIDDLE].z = endpoint_z;

    endpoints[LEFT_FRONT].x = endpoint_x;
    endpoints[LEFT_FRONT].y = endpoint_y;
    endpoints[LEFT_FRONT].z = endpoint_z;

    endpoints[LEFT_REAR].x = -endpoint_x;
    endpoints[LEFT_REAR].y = endpoint_y;
    endpoints[LEFT_REAR].z = endpoint_z;

    gaitSelect(gait_);

    ik_sol_t sol;
    for(int leg = 0; leg < LEG_COUNT; leg++){
        sol = legIK(endpoints[leg].x, endpoints[leg].y, endpoints[leg].z);  
        legCommand(leg, sol); 
    }

}

void PhantomxControl::PhantomxCommandCallback(const phantomx_control::Command::ConstPtr& msg){

    std::cout << *msg << std::endl;
    
    Xspeed = msg->speed.x;
    Yspeed = msg->speed.y;
    Rspeed = msg->speed.z;

    BODY_TRA.x = msg->translation.x;
    BODY_TRA.y = msg->translation.y;
    BODY_TRA.z = msg->translation.z;

    BODY_ROT.x = msg->rotation.x;
    BODY_ROT.y = msg->rotation.y;
    BODY_ROT.z = msg->rotation.z;

    gait_ = (int) msg->gait;
    gaitSelect(gait_);

}

void PhantomxControl::timerCallback(const ros::TimerEvent&){

    publishJointGoal();
    doIK();

}