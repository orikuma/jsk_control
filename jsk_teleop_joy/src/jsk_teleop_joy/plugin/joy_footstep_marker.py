import rospy

import imp
try:
  imp.find_module("actionlib")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')

import actionlib
from joy_pose_6d import JoyPose6D
from jsk_interactive_marker.srv import GetTransformableMarkerPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import copy
import time
import tf

def isSamePose(pose1, pose2):
  return all([getattr(pose1.position, attr) == getattr(pose2.position, attr) for attr in ["x", "y", "z"]]) and all ([getattr(pose1.orientation, attr) == getattr(pose2.orientation, attr) for attr in ["x", "y", "z", "w"]])

class JoyFootstepMarker(JoyPose6D):
  def __init__(self, name, args):
    args['publish_pose'] = False # supress publishing pose of joy_pose_6d
    JoyPose6D.__init__(self, name, args)
    self.pose_pub = rospy.Publisher(self.getArg('pose', 'pose'), PoseStamped)
    self.current_goal_pose = None
    self.use_tf = self.getArg('use_tf', True)
  
    # make service proxy
    rospy.wait_for_service('/footstep_marker/reset_marker')
    self.reset_marker_srv = rospy.ServiceProxy('/footstep_marker/reset_marker', Empty)
    rospy.wait_for_service('/footstep_marker/execute_footstep')
    self.execute_footstep_srv = rospy.ServiceProxy('/footstep_marker/execute_footstep', Empty)
    rospy.wait_for_service('/footstep_marker/get_footstep_marker_pose')
    self.get_footstep_marker_pose_srv = rospy.ServiceProxy('/footstep_marker/get_footstep_marker_pose', GetTransformableMarkerPose)

    # initialize maker pose
    marker_pose = self.getCurrentMarkerPose()
    if marker_pose != None:
      self.pre_pose = marker_pose 
  def joyCB(self, status, history):
    # synchronize marker_pose before publish pre_pose
    now = rospy.Time.from_sec(time.time())    
    if (now - self.prev_time).to_sec() > 1.0 / 10.0:
      marker_pose = self.getCurrentMarkerPose()
      if marker_pose != None and not isSamePose(self.pre_pose.pose, marker_pose.pose):
        self.pre_pose = marker_pose
    
    JoyPose6D.joyCB(self, status, history)
    latest = history.latest()
    if not latest:
      return
    if status.circle and not latest.circle:
      self.execute_footstep_srv()
    elif status.cross and not latest.cross:
      self.reset_marker_srv()
      self.pre_pose = PoseStamped()
      self.pre_pose.pose.orientation.w = 1

    if self.current_goal_pose == None or not isSamePose(self.current_goal_pose.pose, self.pre_pose.pose):      
      if (now - self.prev_time).to_sec() > 1 / 30.0:
        self.pose_pub.publish(self.pre_pose)
        self.prev_time = now
        self.current_goal_pose = copy.deepcopy(self.pre_pose)
  def getCurrentMarkerPose(self):
    try:      
      marker_pose = self.get_footstep_marker_pose_srv("movable_footstep_marker").pose_stamped
      if self.use_tf:
        marker_pose = self.tf_listener.transformPose(self.frame_id, marker_pose)
      return marker_pose
    except rospy.ServiceException, e:
      rospy.logwarn("Failed to get initial marker pose: %s", e)
      return None
    except tf.LookupException, e:
      rospy.logwarn("Failed to lookup tf: %s", e)
      return None