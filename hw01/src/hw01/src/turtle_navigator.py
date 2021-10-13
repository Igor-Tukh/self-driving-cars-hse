#! /usr/bin/python3

import rospy
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

eps = 1e-9

class Navigator:
  def __init__(self):
    self.agent_pose = None
    self.destination_pose = None
    self.last_message = None

    rospy.Subscriber('/destination/pose', Pose, self.callback_destination)
    rospy.Subscriber('/turtle1/pose', Pose, self.callback_agent)
    self.agent_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

  def callback_agent(self, msg):
    self.agent_pose = (msg.x, msg.y)

  def callback_destination(self, msg):
    self.destination_pose = (msg.x, msg.y)
    if self.agent_pose is not None and not self.destination_reached():
      self.initiate_agent_movement()

  def destination_reached(self):
    if self.agent_pose is None or self.destination_pose is None:
      return False
    return abs(self.agent_pose[0] - self.destination_pose[0]) < eps and abs(self.agent_pose[1] - self.destination_pose[1]) < eps

  def initiate_agent_movement(self):
    if self.last_message is not None:
      if time.time() - self.last_message + eps < 1:
        return
    msg = Twist()
    msg.linear.x = self.destination_pose[0] - self.agent_pose[0]
    msg.linear.y = self.destination_pose[1] - self.agent_pose[0]
    self.agent_publisher.publish(msg)
    time.last_message = time.time()

rospy.init_node('turtle_navigator')
Navigator()
rospy.spin()
