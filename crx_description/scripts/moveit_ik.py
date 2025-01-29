#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
# from moveit_msgs.srv import GetPositionIKRequest
# from moveit_msgs.srv import GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
import moveit
import matplotlib
from moveit_msgs.msg import RobotState
import rclpy.exceptions
from moveit.core.robot_state import robotStateToRobotStateMsg

"""
Class to make IK calls using the /compute_ik service.
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class GetIK(object):
    def __init__(self, group, ik_timeout=1.0, ik_attempts=0,
                 avoid_collisions=True, args=None):
        rclpy.init()
        self.node = rclpy.create_node('GetIK')
        # self.node.declare_parameter('robot_description_kinematics.manipulator.kinematics_solver', 'pick_ik/PickIkPlugin')
        self.logger = self.node.get_logger()
        """
        A class to do IK calls thru the MoveIt!'s /compute_ik service.
        :param str group: MoveIt! group name
        :param float ik_timeout: default timeout for IK
        :param int ik_attempts: default number of attempts
        :param bool avoid_collisions: if to ask for IKs that take
        into account collisions
        """
        self.logger.info("Initalizing GetIK...")
        self.group_name = group
        self.ik_timeout = ik_timeout
        self.ik_attempts = ik_attempts
        self.avoid_collisions = avoid_collisions
        self.logger.info("Computing IKs for group: " + self.group_name)
        self.logger.info("With IK timeout: " + str(self.ik_timeout))
        self.logger.info("And IK attempts: " + str(self.ik_attempts))
        self.logger.info("Setting avoid collisions to: " +
                      str(self.avoid_collisions))
        
        self.mp = moveit.MoveItPy(node_name="moveit_py")
        self.logger.info("Connected to MoveIt!")
        
        # self.ik_srv = self.node.create_client(GetPositionIK, '/compute_ik')
        # self.logger.info("Waiting for /compute_ik service...")
        # self.ik_srv.wait_for_service()
        # self.logger.info("Connected!")
        
        self.pc = self.mp.get_planning_component("manipulator")
        self.logger.info("Got planning component")
        robot_model = self.mp.get_robot_model()
        self.robot_state = moveit.core.robot_state.RobotState(robot_model)
        # self.cmd = moveit_commander.RobotCommander()
        
    def get_current_robot_state(self):
        # This is a placeholder for getting the current robot state
        # You need to implement this based on your specific setup
        # For example, you might subscribe to a topic or call a service
        current_state = self.pc.get_start_state()
        # Fill in the current_state with actual data
        return current_state

    def get_ik(self, pose_stamped,
               group=None,
               ik_timeout=None,
               ik_attempts=None,
               avoid_collisions=None):
        """
        Do an IK call to pose_stamped pose.
        :param geometry_msgs/PoseStamped pose_stamped: The 3D pose
            (with header.frame_id)
            to which compute the IK.
        :param str group: The MoveIt! group.
        :param float ik_timeout: The timeout for the IK call.
        :param int ik_attemps: The maximum # of attemps for the IK.
        :param bool avoid_collisions: If to compute collision aware IK.
        """
        if group is None:
            group = self.group_name
        if ik_timeout is None:
            ik_timeout = self.ik_timeout
        if ik_attempts is None:
            ik_attempts = self.ik_attempts
        if avoid_collisions is None:
            avoid_collisions = self.avoid_collisions

        req = GetPositionIK.Request()
        req.ik_request.group_name = group
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = Duration(seconds=ik_timeout).to_msg()
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.ik_link_name = "tcp"
        
        
        success = self.robot_state.set_from_ik("manipulator", pose_stamped.pose, "tcp", ik_timeout)
        
        req.ik_request.timeout = rclpy.duration.Duration(seconds=ik_timeout).to_msg()
        # self.logger.info(str(type(self.get_current_robot_state())))
        # req.ik_request.robot_state = robotStateToRobotStateMsg(self.get_current_robot_state())

        if success:
            # self.logger.info("IK was successful")
            self.robot_state.update()
            return 0
        else:
            # self.logger.error("IK was not successful")
            return -1
        
        # try:
        #     resp = self.ik_srv.call(req)
        #     return resp
        # except Exception as e:
        #     self.logger.error("Service exception: " + str(e))
        #     resp = GetPositionIK.Response()
        #     resp.error_code = 99999  # Failure
        #     return resp

def main(args=None):
    c = GetIK('manipulator', ik_attempts=5, args=args)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "base_link"
    pose_stamped.pose.position.x = 0.0
    pose_stamped.pose.position.y = 0.0
    pose_stamped.pose.position.z = 1.6
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0

    import random

    successes = []
    x = []
    y = []
    z = []
    for n in range(10):
        c.logger.info("Getting IK for pose: " + str(pose_stamped))
        pose_stamped.pose.position.x = random.uniform(-1, 1)
        pose_stamped.pose.position.y = random.uniform(-1, 1)
        pose_stamped.pose.position.z = random.uniform(-0.6, 1.9)

        resp = c.get_ik(pose_stamped)
        if resp == 0:#.error_code.val == 1:
            x.append(pose_stamped.pose.position.x)
            y.append(pose_stamped.pose.position.y)
            z.append(pose_stamped.pose.position.z)

            successes.append([x[-1], y[-1], z[-1]])
            print(successes[-1])
    for i in successes:
        print(i)
    import matplotlib.pyplot as plt
    from mpl_toolkits import mplot3d

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.scatter3D(x,y,z)
    plt.show()
    

if __name__ == '__main__':
    main()
    
    
# https://github.com/AndrejOrsula/pymoveit2/blob/master/pymoveit2/moveit2.py#L60