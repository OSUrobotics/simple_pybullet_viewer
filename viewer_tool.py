#!/usr/bin/python3
"""Script for testing the generated robot manipulators in a simply pybullet enviroment."""

# Author: Josh Campbell, campbjos@oregonstate.edu
# Date: 8-12-2022


import pybullet as p
import time
import pybullet_data
# import os
import json
from numpy import pi
import pathlib


class sim_tester():
    """Simulator class to test different hands in."""

    def __init__(self, robot_name, second_mesh_name=None, gripper_loc=None):
        """Initialize the sim_tester class.
        Args:
            robot_name (str): The name of the gripper to be pulled into the simulator enviroment
            gripper_loc (str): The location of the top hand directory in the output directory
        """
        self.robot_name = robot_name
        self.second_mesh_name = second_mesh_name
        self.gripper_loc = gripper_loc
        
        self.directory = str(pathlib.Path(__file__).parent.resolve())


    def main(self):
        """Run the simulator."""           
        physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setGravity(0, 0, -10)
        LinkId = []
        cubeStartPos = [0, 0, 1]
        cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        plane_id = p.loadURDF("plane.urdf")
        hand_id = p.loadURDF(self.robot_name, useFixedBase=1, basePosition=[0,0,0.0], flags=p.URDF_USE_SELF_COLLISION|p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT)#, baseOrientation=p.getQuaternionFromEuler([0, pi/2, pi/2]))


        p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9999,
                                cameraTargetPosition=[0, 0.1, 0.5])

        if self.second_mesh_name != None:
            second_mesh_id = p.loadURDF(self.second_mesh_name[0], basePosition=self.second_mesh_name[1], baseOrientation=p.getQuaternionFromEuler(self.second_mesh_name[2]))


        while p.isConnected():

            p.stepSimulation()
            time.sleep(1. / 240.)

            for i in range(0, len(LinkId)):
                if LinkId[i] != "skip":
                    linkPos = p.readUserDebugParameter(LinkId[i])
                    p.setJointMotorControl2(hand_id, i, p.POSITION_CONTROL, targetPosition=linkPos)


        p.disconnect()
    
def read_json(file_loc):
    """Read contents of a given json file.
    Args:
        file_loc (str): Full path to the json file including the file name.
    Returns:
        dictionary: dictionary that contains the content from the json.
    """
    with open(file_loc, "r") as read_file:
        file_contents = json.load(read_file)
    return file_contents


def urdf_finder(directory):
    folders = sorted(pathlib.Path(f'{directory}/').glob('**/*.urdf'))
    if len(folders) == 0:
        print("\033[91m\nNo URDF files found!!!\nPlease make sure the files are in the same or a lower directory as the script\033[0m\n")
        return None
    else:
        return folders

def select_meshes(folders):
    hand_names = []
    for i, hand in enumerate(folders):
            temp_hand = str(hand).split('/')
            hand_names.append(str(hand))
            print(f'{i}:   {temp_hand[-1][:-5]}')
    
    input_num = input("\n\033[92mEnter the number of the hand you want loaded:   \033[0m")
    num = int(input_num)
    hand_name = hand_names[num]
    if input("\033[92mDo you want to load a second model? (y/n) \033[0m" ) == "y":
        
        num2 = int(input("\033[91mEnter the number of the second model you want loaded:   \033[0m"))
        print("Enter the position and orientation of the second model: ")
        pose_list = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        pose_model2 = []
        for i in pose_list:
            pose_model2.append(float(input(f'\033[91mEnter value for {i}:   \033[0m')))

        return [hand_name, [hand_names[num2], pose_model2[:3], pose_model2[3:]]]
    else:
        return [hand_name]


if __name__ == '__main__':

    directory = str(pathlib.Path(__file__).parent.resolve())

    folders = urdf_finder(directory=directory)
    
    if folders != None:
        results = select_meshes(folders=folders)
        print(results)
        if len(results) == 1:
            sim_test = sim_tester(results[0])
        elif len(results) == 2:
            sim_test = sim_tester(results[0], results[1])
        else:
            exit
        sim_test.main()