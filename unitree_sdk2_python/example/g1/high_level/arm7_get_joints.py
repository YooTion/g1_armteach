import time
import sys

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from armteach import action_logger,action_replay

i = 0
class G1JointIndex:
    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21  

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28  

class ArmAngleReader:
    def __init__(self):
        self.left_arm_angles = []
        self.right_arm_angles = []
        self.low_state = None

    def Init(self):
        # Create subscriber for low state (which contains joint angles)
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def LowStateHandler(self, msg: LowState_):
        """ This function handles the low state message and stores the joint angles """
        self.low_state = msg

    def ReadJointAngles(self):
        """ Reads the joint angles and stores them in separate lists for left and right arms """
        if self.low_state:
            left_angles = [
                self.low_state.motor_state[G1JointIndex.LeftShoulderPitch].q,
                self.low_state.motor_state[G1JointIndex.LeftShoulderRoll].q,
                self.low_state.motor_state[G1JointIndex.LeftShoulderYaw].q,
                self.low_state.motor_state[G1JointIndex.LeftElbow].q,
                self.low_state.motor_state[G1JointIndex.LeftWristRoll].q,
                self.low_state.motor_state[G1JointIndex.LeftWristPitch].q,
                self.low_state.motor_state[G1JointIndex.LeftWristYaw].q
            ]
            right_angles = [
                self.low_state.motor_state[G1JointIndex.RightShoulderPitch].q,
                self.low_state.motor_state[G1JointIndex.RightShoulderRoll].q,
                self.low_state.motor_state[G1JointIndex.RightShoulderYaw].q,
                self.low_state.motor_state[G1JointIndex.RightElbow].q,
                self.low_state.motor_state[G1JointIndex.RightWristRoll].q,
                self.low_state.motor_state[G1JointIndex.RightWristPitch].q,
                self.low_state.motor_state[G1JointIndex.RightWristYaw].q
            ]
            
            # Append the current angles to the respective lists
            arm_logger.log_frame(left_angles, right_angles)
            global i 
            i += 1
            print(i)
            self.left_arm_angles.append(left_angles)
            self.right_arm_angles.append(right_angles)
            

    def StartReading(self):
        """ Starts reading joint angles every 0.02 seconds """
        while True:
            time.sleep(0.02)  # Delay for 0.02 seconds
            self.ReadJointAngles()
            # Optionally, print the angles to see them in real-time
            print(f"Left Arm Angles: {self.left_arm_angles[-1]}")
            print(f"Right Arm Angles: {self.right_arm_angles[-1]}")

if __name__ == '__main__':
    # Initialize ChannelFactory with or without command line arguments
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    arm_logger = action_logger.ArmDataLogger("test.act",0.02)

    arm_reader = ArmAngleReader()
    arm_reader.Init()
    arm_reader.StartReading()
