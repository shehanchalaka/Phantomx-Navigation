#include "ros/ros.h"
#include "phantomx_control/Command.h"
#include "sensor_msgs/Joy.h"


#define MOVE        0
#define TRANSLATE   1
#define ROTATE      2

#define RIPPLE      0
#define AMBLE       1
#define TRIPOD      2

int JOY_MODE;
int JOY_GAIT;

ros::Publisher command_pub;
phantomx_control::Command command;


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){

    if(joy->buttons[0]){

        if(JOY_MODE == MOVE){
            JOY_MODE = TRANSLATE;
        } else if(JOY_MODE == TRANSLATE){
            JOY_MODE = ROTATE;
        } else if(JOY_MODE == ROTATE){
            JOY_MODE = MOVE;
        }

        command.speed.x = 0.0;
        command.speed.y = 0.0;
        command.speed.z = 0.0;       
    }
    
    if(joy->buttons[1]){
        
        if(JOY_GAIT == RIPPLE){
            command.gait = AMBLE;
        } else if(JOY_GAIT == AMBLE){
            command.gait = TRIPOD;
        } else if(JOY_GAIT == TRIPOD){
            command.gait = RIPPLE;
        }
    }

    float gs[] = {0.05, 0.15, 0.2};

    if(JOY_MODE == MOVE){
        command.speed.x = gs[JOY_GAIT] * joy->axes[1];
        command.speed.y = gs[JOY_GAIT] * joy->axes[0];
        command.speed.z = gs[JOY_GAIT] * -3.5 * joy->axes[3];  

    } else if(JOY_MODE == TRANSLATE){
        command.translation.x = -0.08 * joy->axes[1];
        command.translation.y = -0.08 * joy->axes[0];
        command.translation.z = 0.08 * joy->axes[4];
          

    } else if(JOY_MODE == ROTATE){
        command.rotation.x = 0.3 * joy->axes[0];
        command.rotation.y = 0.3 * joy->axes[1];
        command.rotation.z = 0.3 * joy->axes[3];
    }

    command_pub.publish(command);

}

int main(int argc, char **argv){

  ros::init(argc, argv, "manual_control_node");
  ros::NodeHandle nh;

  ros::Subscriber joy_sub = nh.subscribe("/joy", 10, joyCallback);
  command_pub = nh.advertise<phantomx_control::Command>("/phantomx_command", 10);

  JOY_MODE = MOVE;
  JOY_GAIT = AMBLE;

  ros::spin();

  return 0;
}