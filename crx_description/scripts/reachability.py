#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import os
from math import degrees, radians
import numpy as np
import datetime
import pickle
import ik_funcs as ik
import moveit_ik
from geometry_msgs.msg import PoseStamped
# from tf import TransformListener

class Reachability(object):
    def __init__(self, create=True, collisions=False):
        src_fldr = os.path.join(
            os.path.dirname(os.path.abspath(os.path.join(os.path.realpath(__file__), os.pardir))), "config"
        )


        if collisions:
            self.reach_file = os.path.join(src_fldr, "ReachmapCollisions")
        else:
            self.reach_file = os.path.join(src_fldr, "Reachmap")

        self.collisions = collisions

        if create:
            #self.tf = TransformListener()
            self.compute_ik = moveit_ik.GetIK('IKfastGroup', ik_timeout=0.05)


    """def get_link_pos(self, link):
        if self.tf.frameExists(link) and self.tf.frameExists(link):
            t = self.tf_listener_.getLatestCommonTime("/world", link)
            p1 = PoseStamped()
            p1.header.frame_id = link
            p1.pose.orientation.w = 1.0    # Neutral orientation
            p_in_base = self.tf_listener_.transformPose("/world", p1)
            print("Position of {} in the world frame:".format(link))
            print(p_in_base)"""

    def load_params(self): 
        import yaml

        file_name = os.path.join(os.path.dirname(os.path.abspath(os.path.join(os.path.realpath(__file__), os.pardir))), "config", "reach_params.yaml")
        with open(file_name) as file:
            params = yaml.full_load(file)
        return params

    def generate_map(self):
        print("Building reachability map...")
        params = self.load_params()
        workspace_min = params['workspace_min'] #=-1.0
        workspace_max = params['workspace_max'] # 1.0
        z_min = params['z_min'] #-0.1
        z_max = params['z_max'] # 2.0
        oris_per_position = params['oris_per_position']
        discretisation = params['discretisation'] * (1e-3)  # 50 (mm)
        self.data = []
        self.verbose_data = {}
        numVoxels = 0.0
        gm_start = time.time()
        sec_timer = time.time()
        numPoses = 0.0
        total_successes = 0 
        cum_successes_cnt = 0
        
        
        N = int((workspace_max - workspace_min) / discretisation)
        N_z = int((z_max - z_min) / discretisation)
        # spacing = ((workspace_max-workspace_min) * 1.0 / np.floor(np.sqrt(N)))
        self.compute_ik.logger.info(str(N*N*N_z) + " positions, estimated " + str(N*N*N_z*oris_per_position) + " total poses to test.")
        for x in np.linspace(workspace_min, workspace_max, N):
            for y in np.linspace(workspace_min, workspace_max, N):
                for z in np.linspace(z_min, z_max, N_z):
                    numVoxels += 1
                    successes, num_poses = self.newVoxel(
                        x_c=x,
                        y_c=y,
                        z_c=z,
                        r=discretisation / 2,
                        N=oris_per_position
                    )
                    numPoses += num_poses
                    cum_successes_cnt += successes
                    if successes > 0:
                        total_successes += successes
                        self.data.append([x, y, z, float(successes) / num_poses])
                        if cum_successes_cnt > 100:
                            self.compute_ik.logger.info(
                                "Num poses tested: "
                                + str(numPoses)
                                + ", "
                                + str(total_successes) 
                                + " total successes."
                            )
                            cum_successes_cnt = 0

                    if time.time() - sec_timer >= 5.0:
                        pc_done = numVoxels / (N * N * N_z)
                        elapsed_time = time.time() - gm_start
                        projected_time = elapsed_time / max(pc_done, 0.0001)
                        remaining_time = projected_time - elapsed_time

                        elapsed_time_str = str(
                            datetime.timedelta(seconds=int(elapsed_time))
                        )
                        projected_time_str = str(
                            datetime.timedelta(seconds=int(projected_time))
                        )
                        remaining_time_str = str(
                            datetime.timedelta(seconds=int(remaining_time))
                        )

                        self.compute_ik.logger.info("{:.2%} done".format(pc_done))
                        self.compute_ik.logger.info(
                            "{} / {}. {} remaining".format(
                                elapsed_time_str, projected_time_str, remaining_time_str
                            )
                        )
                        sec_timer = time.time()
        self.compute_ik.logger.info(
            "Total {} voxels, {} poses tested in {}. ".format(
                numVoxels, numPoses, elapsed_time_str
            )   
        )

    def newVoxel(
        self, x_c=0.0, y_c=0.0, z_c=0.0, r=1.0, N=16
    ):
        """ Define a spherical voxel and test the poses around it """
        num_successes = 0
        num_poses = 0
        n = int(np.floor(np.sqrt(N)))

        # Set points distributed evenly. Could change to generalised spiral as J. Saff and A. Kuijlaars, “Distributing many points on a sphere"
        self.testPose(
            0, 0, r, x_c, y_c, z_c, collisions=self.collisions
        )
        self.testPose(
            np.pi, np.pi, r, x_c, y_c, z_c, collisions=self.collisions
        )

        for phi in np.linspace(0, 2 * np.pi, n, 1):
            for theta in np.linspace(0, np.pi, n, 0):
                num_poses += 1
                if self.testPose(
                    phi,
                    theta,
                    r,
                    x_c,
                    y_c,
                    z_c,
                    collisions=self.collisions,
                    time_limit=3.0
                ):
                    num_successes += 1
        return num_successes, num_poses

    def testPose(
        self,
        phi,
        theta,
        r=1.0,
        x_c=0.0,
        y_c=0.0,
        z_c=0.0,
        **kwargs
    ):
        """ Test a pose in spherical coordinate definition """
        st = np.sin(theta)
        x = r * np.cos(phi) * st + x_c
        y = r * np.sin(phi) * st + y_c
        z = r * np.cos(theta) + z_c

        roll = 0
        pitch = theta + np.pi
        yaw = phi

        qx, qy, qz, qw = ik.quat_from_euler(roll, pitch, yaw)
        pose = [[x, y, z], [qx, qy, qz, qw]]

        # self.poses[self.numPoses] = {self.numPoses: {'position': {'x': x, 'y': y, 'z': z}, 'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}}}
        # self.numPoses += 1

        success = self.test_ik(pose)
        
        #success = len(confs) > 0

        # self.verbose_data[len(self.verbose_data)] = {'position': {'x': x, 'y': y, 'z': z}, 'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}, 'success': success}

        return success

    def test_ik(self, pose):
        position = pose[0]
        orientation = pose[1]

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose.position.x = position[0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]
        pose_stamped.pose.orientation.x = orientation[0]
        pose_stamped.pose.orientation.y = orientation[1]
        pose_stamped.pose.orientation.z = orientation[2]
        pose_stamped.pose.orientation.w = orientation[3]

        resp = self.compute_ik.get_ik(pose_stamped, avoid_collisions=self.collisions)

        if resp == 0:#.error_code.val == 1:
            #print(resp.solution.joint_state.position)
            return True #[list(resp.solution.joint_state.position)]
        else:
            return False#[]

    def dump_data(self, file=None, data=None):
        if file == None:
            file = self.reach_file
        if data == None:
            data = self.data

        # with open(file + ".pickle", "wb") as handle:
        #     pickle.dump(data, handle, protocol=4)
        with open(file + ".pickle", "wb") as handle:
            pickle.dump(data, handle, protocol=2)
        print("Data dumped!")

    def load_data(self, file=None):
        if file == None:
            file = self.reach_file

        with open(file + ".pickle", mode="rb") as handle:
            self.data = pickle.load(handle)

    def plot(self):
        from mpl_toolkits.mplot3d import Axes3D
        from matplotlib import pyplot as plt
        import numpy as np

        fig = plt.figure()
        ax = fig.gca(projection='3d')

        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")

        x = []
        y = [] 
        z = []
        r = []

        for i in self.data:
            if i[1] >= 0:
                x.append(i[0])
                y.append(i[1])
                z.append(i[2])
                r.append(i[3])

        cmap = plt.cm.get_cmap("RdYlGn")
        ax.scatter(x,y, z, c=r, cmap=cmap)
        ax.axis('auto')
        plt.show()


def main():
    import sys
    import argparse

    parser = argparse.ArgumentParser()
    ns = argparse.Namespace()
    parser.add_argument(
        "-n",
        "--create",
        action="store_true",
        help="Create a new reachmap. If not then load from file",
        default=False
    )
    parser.add_argument(
        "-c",
        "--collisions",
        action="store_true",
        help="Factors for collisions in planning scene",
        default=True
    )
    parser.add_argument(
        "-p",
        "--plot",
        action="store_true",
        help="Plot graph",
        default=True
    )
    
    _args = vars(parser.parse_known_args()[0])
    print(_args)
    create = bool(_args['create'])
    collisions = bool(_args['collisions'])
    plot = bool(_args['plot'])

    rc = Reachability(create, collisions)

    if create:
        rc.generate_map()
        rc.dump_data(file=rc.reach_file)
    else:
        rc.load_data(rc.reach_file)
    
    if plot:
        rc.plot()

if __name__ == "__main__":
    main()