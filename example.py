import AbstractRobot
import RobotRaconteur as RR

class Stretch_Robot(AbstractRobot.AbstractRobot):
  def __init__(self, robot_info,default_joint_count):
    super().__init__(self, robot_info,default_joint_count)



Stretch=Stretch_Robot()