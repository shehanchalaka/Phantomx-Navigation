#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Joint:
        def __init__(self, motor_name):
            #arm_name should be b_arm or f_arm
            self.name = motor_name
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')


        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()
            prefix = self.name[0:2];
            rospy.loginfo(prefix);
            goal.trajectory.joint_names = [prefix+'_coxa_joint', prefix+'_femur_joint',prefix+'_tibia_joint']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(0.5)
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)


def main():
            arm = Joint('AL_joint_trajectory')
            arm2 = Joint('AR_joint_trajectory')
            arm.move_joint([0.0,0.0,0.0])
            arm.move_joint([1.0, 1.0, 0.0])
            arm2.move_joint([0.0,0.0,0.0])
            arm2.move_joint([1.0, 1.0, 0.0])

if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()