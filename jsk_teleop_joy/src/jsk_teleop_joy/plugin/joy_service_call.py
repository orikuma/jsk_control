import rospy
from joy_plugin import JSKJoyPlugin

import imp
try:
  imp.find_module("actionlib")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')

import actionlib
from jsk_rviz_plugins.msg import OverlayMenu
from std_srvs.srv import Empty
import copy
import time
import tf

class JoyServiceCall(JSKJoyPlugin):
  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)

    # make publisher
    self.pub = rospy.Publisher("joy_service_menu", OverlayMenu)
    self.menu = None

    # get args
    self.run_carry_task = self.getArg('carry', 'false')
    self.run_door_task = self.getArg('door', 'false')
    self.run_valve_task = self.getArg('valve', 'false')
  
    # make service proxy
    rospy.wait_for_service('/footstep_demo/reset_pose')
    self.reset_pose_srv = rospy.ServiceProxy('/footstep_demo/reset_pose', Empty)
    rospy.wait_for_service('/footstep_demo/reset_manip_pose')
    self.reset_manip_pose_srv = rospy.ServiceProxy('/footstep_demo/reset_manip_pose', Empty)
    
    if self.run_carry_task:
      rospy.wait_for_service('/carry_object_task/hold_box')
      self.hold_box_srv = rospy.ServiceProxy('/carry_object_task/hold_box', Empty)
      rospy.wait_for_service('/carry_object_task/put_down_box')
      self.put_down_box_srv = rospy.ServiceProxy('/carry_object_task/put_down_box', Empty)

    if self.run_door_task:
      rospy.wait_for_service('/slam_sequential_task/execute_door')
      self.execute_door_srv = rospy.ServiceProxy('/slam_sequential_task/execute_door', Empty)
    if self.run_valve_task:      
      rospy.wait_for_service('/slam_sequential_task/execute_valve')
      self.execute_valve_srv = rospy.ServiceProxy('/slam_sequential_task/execute_valve', Empty)
          
  def joyCB(self, status, history):
    now = rospy.Time.from_sec(time.time())
    latest = history.latest()
    if not latest:
      return

    # menu mode
    if self.menu != None:
      if status.up and not latest.up:
        self.menu.current_index = (self.menu.current_index - 1) % len(self.menu.menus)
        self.pub.publish(self.menu)
      elif status.down and not latest.down:
        self.menu.current_index = (self.menu.current_index + 1) % len(self.menu.menus)
        self.pub.publish(self.menu)        
      elif status.circle and not latest.circle:
        self.menu.action = OverlayMenu.ACTION_CLOSE
        # task service call
        if self.menu.title == "Call Task Service":
          if self.run_carry_task and self.menu.current_index == self.menu.menus.index("Hold Box"):
            self.hold_box_srv()
          elif self.run_carry_task and  self.menu.current_index == self.menu.menus.index("Put Down Box"):
            self.put_down_box_srv()
          elif self.run_door_task and self.menu.current_index == self.menu.menus.index("Open Door"):
            self.execute_door_srv()
          elif self.run_valve_task and self.menu.current_index == self.menu.menus.index("Turn Valve"):
            self.execute_valve_srv()
        elif self.menu.title == "Change Robot Pose":
          if self.menu.current_index == self.menu.menus.index("Reset Pose"):
            self.reset_pose_srv()
          elif self.menu.current_index == self.menu.menus.index("Reset Manip Pose"):
            self.reset_manip_pose_srv()
        self.pub.publish(self.menu)
        self.menu = None
      elif status.cross and not latest.cross:
        self.menu.action = OverlayMenu.ACTION_CLOSE
        self.pub.publish(self.menu)
        self.menu = None
      else:
        self.pub.publish(self.menu)
      return

    # control mode    
    if status.start and not latest.start:
      self.menu = OverlayMenu()
      self.menu.title = "Call Task Service"
      self.menu.menus = []
      if self.run_carry_task:
        self.menu.menus.append("Hold Box")
        self.menu.menus.append("Put Down Box")
      if self.run_door_task:
        self.menu.menus.append("Open Door")
      if self.run_valve_task:
        self.menu.menus.append("Turn Valve")
      self.menu.current_index = 0
      self.pub.publish(self.menu)
    elif status.triangle and not latest.triangle:
      self.menu = OverlayMenu()
      self.menu.title = "Change Robot Pose"
      self.menu.menus = ["Reset Pose", "Reset Manip Pose"]
      self.menu.current_index = 0
      self.pub.publish(self.menu)
