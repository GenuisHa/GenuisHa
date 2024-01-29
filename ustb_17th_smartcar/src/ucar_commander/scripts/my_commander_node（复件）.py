#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
import time
import math
import tf
import actionlib
from copy import deepcopy
from std_srvs.srv import SetBool
from move_base_msgs.msg import MoveBaseGoal
from enum import Enum
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID

class NavStage(Enum):
    IDLE=0
    TO_MID_POINT=1
    ENROUTE_TO_MID_POINT=2
    STOP=3


class AccumTimer:
    timers=()

    def __init__(self,name=None):
        self.name=name
        self._start_time=None
        self._accum_time=0.0
        self.elapsed_time=0.0

    def start(self):
        if self._start_time is not None:
            return
            #raise RuntimeError(f"Timer{self.name} is running")
         
        self._start_time=time.time()

    def stop(self):
        if self._start_time is None:
            return -1
        self.elapsed_time=time.time() - self._start_time
        self._accum_time += self.elapsed_time
        self._start_time=None

        if self.name:
            self.timers[self.name]+=self._accum_time
        return self.elapsed_time
    

class UcarCommander:
    def __init__(self):
        rospy.init_node("my_commander_node")
        self.tf_listener=tf.TransformListener()
        self.srv_nav_start=rospy.Service(
            "nav_start",SetBool,self.nav_start)
        self.move_base_client=actionlib.SimpleActionClient(
            "move_base",MoveBaseGoal)
        self.pub_movebase_goal=rospy.Publisher(
            "move_base/cancel",GoalID,queue_size=1)
        
        self.pub_move_base_abort=rospy.Publisher(
            "move_base/cancel",GoalID,queue_size=1)#学长给的1
        self,nav_timer=AccumTimer("navgation")
        self.nav_stage=NavStage.IDLE
        self.is_nav_start=False

        self.odom_frame_id=rospy.get_param("~odom_frame_id","odom")
        self.global_frame_id=rospy.get_param("~global_frame_id","map")
        self.robot_frame_id=rospy.get_param("~robot_frame_id","base_link")

        self.position=MoveBaseGoal()
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id=self.global_frame_id
        self.goal.target_pose.pose.position.x=rospy.get_param("~goal_position",[0.0,0.0,0.0])[0]
        self.goal.target_pose.pose.position.y=rospy.get_param("~goal_position",[0.0,0.0,0.0])[1]
        self.goal.target_pose.pose.position.z=rospy.get_param("~goal_position",[0.0,0.0,0.0])[2]
        self.goal.target_pose.pose.position.orientation.x=rospy.get_param("~goal_position",[0.0,0.0,0.0])[2]
        self.goal.target_pose.pose.position.orientation.y=rospy.get_param("~goal_position",[0.0,0.0,0.0])[2]
        self.goal.target_pose.pose.position.orientation.z=rospy.get_param("~goal_position",[0.0,0.0,0.0])[2]
        self.goal.target_pose.pose.position.orientation.w=rospy.get_param("~goal_position",[0.0,0.0,0.0])[2]
        self.goal_tolerance=rospy.get_param("~goal_tolerance",0.2)
    
    def calculate_distance(self,point1,point2):
        distance=math.sqrt(
            (point1.target_pose.pose.position.x -
             point2.target_pose.pose.position.x
             )**2+
             (point1.target_pose.pose.position.y-
             point2.target_pose.pose.position.y
                 
             )**2
        )
        return distance

    # Link Start!!!
    #这个方法是"/nav_start"服务的回调函数。处理启动或停止导航过程的请求。
    def nav_start(self, req):
        if not self.is_nav_start:
            if req.data:
                # while not self.is_ready
                print("Action Stations!!!")
                self.nav_stage = NavStage.TO_MID_POINT
                self.is_nav_start = True
                self.goal_tolerance = deepcopy(
                    self.mid_point_tolerance[0])
                self.goal = deepcopy(self.mid_point[0])
                return True, "AccumviaTimer started, good luck."
            else:
                print("Ignoring request as the \"data\" field is set to false")
                return False, "False received"
        else:
            if req.data:
                print("Starting failed: already running")
                return False, "Already running"
            else:
                print("Abrot navigation is not implemented yet, doing nothing")
                return False, "Abrot navigation is not implemented yet, doing nothing"


    def _get_robot_position(self):
        try:
            (trans,rot)=self.tf_listener.lookupTransform(
                self.global_frame_id,self.robot_frame_id,rospy.Time(0))
            self.position.target_pose.pose.position.x=trans(0)
            self.position.target_pose.pose.position.y=trans(1)
        except (tf.LookupExceptionm,tf.ConnectivityException,tf.ExtrapolationException):
            rospy.logerr("td error")

    def send_movebase_goal(self,goal):
        self.move_base_client.send_goal(goal)
        simple_goal = PoseStamped()
        simple_goal.header.frame_id=self.global_frame_id
        simple_goal.pose.position.x=goal.target_pose.pose.position.x
        simple_goal.pose.position.y=goal.target_pose.pose.position.y
        simple_goal.pose.position.z=goal.target_pose.pose.position.z
        simple_goal.pose.orientation.x=goal.target_pose.pose.orientation.x
        simple_goal.pose.orientation.y=goal.target_pose.pose.orientation.y
        simple_goal.pose.orientation.z=goal.target_pose.pose.orientation.z
        simple_goal.pose.orientation.w=goal.target_pose.pose.orientation.w
        self.pub_movebase_goal.publish(simple_goal)

    def abort_move_base(self):
        goal_id=GoalID()
        goal_id.stamp=rospy.Time.now()
        self.pub_move_base_abort.Publish(goal_id)


    def folw(self):

        self._get_robot_position(


        )
        if self.nav_stage ==NavStage.IDLE:
            pass
        
        if self.nav_stage ==NavStage.TO_MID_POINT:
            self.send_movebase_goal(self.goal)
            self.nav_stage=NavStage.ENROUTE_TO_MID_POINT

        if self.nav_stage ==NavStage.ENROUTE_TO_MID_POINT:
            if self.calculate_distance(self.position,self.goal)<=self.goal_tolerance:
                
                self.abort_move_base()
                self.nav_stage=NavStage.STOP
          

        if self.nav_stage ==NavStage.STOP:
            final_time = self.nav_timer.stop()
            self.nav_stage=NavStage.TO_IDLE
            
            print("final_time:%.4"% final_time)

    def run(self):
        rate=rospy.Rate(20)
        self.folw()
        while not rospy.is_shutdown():
            self.folw()
            rate.sleep()

def main():
    node = UcarCommander()
    node.run()

if __name__=="__main__":
    try:
        main
    except KeyboardInterrupt:
        exit(0)