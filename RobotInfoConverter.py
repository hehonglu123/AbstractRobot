import GeneralRoboticsToolbox as Rox
import numpy as np

def ToToolboxRobot(robot_info):


    num_joints=len(robot.robot_info.joint_info)
    P=np.array(robot.robot_info.chains[0].P.tolist())
    H=np.transpose(np.array(robot.robot_info.chains[0].H.tolist()))
    rox_robot=Rox.Robot(H,np.transpose(P),np.zeros(num_joints))

    return rox_robot

