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
from scipy.spatial.transform import Rotation as R

## Helper functions

def fibonacci_sphere(samples=1):
    points = []
    phi = np.pi * (3. - np.sqrt(5.))  # golden angle in radians

    for i in range(samples):
        y = 1 - (i / float(samples - 1)) * 2  # y goes from 1 to -1
        radius = np.sqrt(1 - y * y)  # radius at y

        theta = phi * i  # golden angle increment

        x = np.cos(theta) * radius
        z = np.sin(theta) * radius

        points.append((x, y, z))

    return points

def vector_to_euler(target):
    # Normalize target to ensure it's on the unit sphere
    target = np.array(target) / np.linalg.norm(target)

    # Extract spherical coordinates
    theta = np.arctan2(target[1], target[0])  # Azimuth angle
    phi = np.arccos(target[2])  # Inclination angle from z-axis

    # Create rotation matrix
    R_z = R.from_euler('z', theta)  # Rotate around z
    R_y = R.from_euler('y', phi)  # Rotate around y

    # Combine rotations
    R_total = R_y * R_z

    # Convert to Euler angles (ZYX convention)
    euler_angles = R_total.as_euler('xyz', degrees=True)  # or degrees=False for radians
    return euler_angles


## Reachability class

class Reachability(object):
    def __init__(self, create=True, collisions=True):
        src_fldr = os.path.join(
            os.path.dirname(os.path.abspath(os.path.join(os.path.realpath(__file__), os.pardir))), "config"
        )
        self.src_fldr = src_fldr


        if collisions:
            self.reach_file = os.path.join(src_fldr, "ReachmapCollisions")
        else:
            self.reach_file = os.path.join(src_fldr, "Reachmap")

        self.collisions = collisions
        self.node = None
        if create:
            #self.tf = TransformListener()
            self.compute_ik = moveit_ik.GetIK('manipulator', ik_timeout=0.01, avoid_collisions=True)
            self.node = self.compute_ik.node
            
        self.sphere = None


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

        file_name = os.path.join(os.path.dirname(os.path.abspath(os.path.join(
                os.path.realpath(__file__), os.pardir))), "config", "reach_params.yaml")
        with open(file_name) as file:
            params = yaml.full_load(file)
        return params

    def generate_points(self, centre, radius, discretisation):
        points = []
        for x in np.linspace(centre[0] - radius, centre[0] + radius, int(2*radius/discretisation)+1):
            for y in np.linspace(centre[1] - radius, centre[1] + radius, int(2*radius/discretisation)+1):
                for z in np.linspace(centre[2] - radius, centre[2] + radius, int(2*radius/discretisation)+1):
                    # Check if point is within sphere
                    if np.linalg.norm(np.array([x, y, z]) - np.array(centre)) <= radius:                  
                        points.append([x, y, z])
        # Check here              
        # Plot points in 3D
        # from mpl_toolkits.mplot3d import Axes3D
        # from matplotlib import pyplot as plt
        # ll = np.linspace(centre[0] - radius, centre[0] + radius, int(2*radius/discretisation)+1)
        # self.compute_ik.logger.info(str(ll[1]-ll[0]))
        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        # ax.scatter([p[0] for p in points], [p[1] for p in points], [p[2] for p in points])
        # plt.show()
        return points

    def generate_map(self):
        print("Building reachability map...")
        params = self.load_params()
        # workspace_min = params['workspace_min'] #=-1.0
        # workspace_max = params['workspace_max'] # 1.0
        # z_min = params['z_min'] #-0.1
        # z_max = params['z_max'] # 2.0
        oris_per_position = params['oris_per_position']
        self.sphere = fibonacci_sphere(oris_per_position)
        radius = params['radius'] # 1.7
        centre_x = params['centre']['x']    # 0.0
        centre_y = params['centre']['y']    # 0.0
        centre_z = params['centre']['z']    # 0.0
        
        discretisation = params['discretisation'] * (1e-3)  # 50 (mm)
        self.data = []
        self.verbose_data = {}
        numVoxels = 0.0
        gm_start = time.time()
        sec_timer = time.time()
        numPoses = 0.0
        total_successes = 0 
        cum_successes_cnt = 0
        
        points = self.generate_points([centre_x, centre_y, centre_z], radius, discretisation)
        
        # N = int((workspace_max - workspace_min) / discretisation)
        # N_z = int((z_max - z_min) / discretisation)
        # spacing = ((workspace_max-workspace_min) * 1.0 / np.floor(np.sqrt(N)))
        self.compute_ik.logger.info(str(len(points)) + " positions, estimated " + str(len(points)*oris_per_position) + " total poses to test.")
        # for x in np.linspace(workspace_min, workspace_max, N):
        #     for y in np.linspace(workspace_min, workspace_max, N):
        #         for z in np.linspace(z_min, z_max, N_z):
        timings = []
        for point in points:
            numVoxels += 1
            successes, num_poses, successes_binary = self.newVoxel( 
                x_c=point[0],
                y_c=point[1],
                z_c=point[2],
                r=discretisation / 2,
                N=oris_per_position
            )
            elapsed_time = time.time() - gm_start
            timings.append(elapsed_time)
            numPoses += num_poses
            cum_successes_cnt += successes
            if successes > 0:
                total_successes += successes
                self.data.append([point[0], point[1], point[2], float(successes) / num_poses, successes_binary, oris_per_position])
                if cum_successes_cnt > 100:
                    self.compute_ik.logger.info(
                        str(self.data[-1]) +
                        ", Num poses tested: "
                        + str(numPoses)
                        + ", "
                        + str(total_successes) 
                        + " total successes."
                    )
                    cum_successes_cnt = 0

            if time.time() - sec_timer >= 5.0:
                pc_done = numVoxels / len(points)
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
        return timings

    def newVoxel(
        self, x_c=0.0, y_c=0.0, z_c=0.0, r=1.0, N=16
    ):
        """ Define a spherical voxel and test the poses around it """
        num_successes = 0
        num_poses = 0
        # Set points distributed evenly. Could change to generalised spiral as J. Saff and A. Kuijlaars, “Distributing many points on a sphere"
        successes_binary = ""
        for n in range(N):
            num_poses += 1
            if self.testPose(
                n,
                x_c,
                y_c,
                z_c,
                collisions=self.collisions,
                time_limit=3.0
            ):
                num_successes += 1
                successes_binary += "1"
            else:
                successes_binary += "0"
                
        # Reverse binary string to get the correct order
        successes_binary = successes_binary[::-1]
        # Convert binary string to integer
        successes_binary = int(successes_binary, 2)
        print(successes_binary)
        
        
        # successes_binary is a binary string that represents the successes of each pose, allowing for a more compact representation of the pose data
        # It should allow us using bin(successes_binary) and the number of test orientations, to rebuild the fibonacci sphere
        # And thus easily retrieve which exact orientations were successful for a given voxel
        
        
        return num_successes, num_poses, successes_binary

    def testPose(
        self,
        n,
        x_c=0.0,
        y_c=0.0,
        z_c=0.0,
        **kwargs
    ):
        """ Test a pose in spherical coordinate definition """
        x_s = -self.sphere[n][0]
        y_s = -self.sphere[n][1]
        z_s = -self.sphere[n][2]
        
        target_vector = [x_s, y_s, z_s]
        roll, pitch, yaw = vector_to_euler(target_vector)

        qx, qy, qz, qw = ik.quat_from_euler(roll, pitch, yaw)
        pose = [[x_c, y_c, z_c], [qx, qy, qz, qw]]

        # self.poses[self.numPoses] = {self.numPoses: {'position': {'x': x, 'y': y, 'z': z}, 'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}}}
        # self.numPoses += 1

        confs = self.test_ik(pose)
        
        success = len(confs) > 0

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

        if resp.error_code.val == 1:
            #print(resp.solution.joint_state.position)
            return [list(resp.solution.joint_state.position)]
        else:
            return []

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
            # if i[1] >= 0:
            x.append(i[0])
            y.append(i[1])
            z.append(i[2])
            r.append(i[3])
            
        print("X min: ", min(x), "X max: ", max(x))
        print("Y min: ", min(y), "Y max: ", max(y))
        print("Z min: ", min(z), "Z max: ", max(z))

        cmap = plt.cm.get_cmap("RdYlGn")
        ax.scatter(x,y, z, c=r, cmap=cmap)
        ax.axis('auto')
        plt.show()
        return x, y, z
    
    def plot_diff(self):
        params = self.load_params()
        oris_per_position = params['oris_per_position']
        self.sphere = fibonacci_sphere(oris_per_position)
        radius = params['radius'] # 1.7
        centre_x = params['centre']['x']    # 0.0
        centre_y = params['centre']['y']    # 0.0
        centre_z = params['centre']['z']    # 0.0
        
        discretisation = params['discretisation'] * (1e-3)  # 50 (mm)
        
        points = self.generate_points([centre_x, centre_y, centre_z], radius, discretisation)
        
        # Get set of points that are in the original reachability map
        points_reach = []
        print(len(self.data))
        for i in self.data:
            points_reach.append([i[0], i[1], i[2]])
        
        print(points)
        
        points_diff = [x for x in points if x not in points_reach]
        
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

        for i in points_diff:
            if i[1] >= 0:
                x.append(i[0])
                y.append(i[1])
                z.append(i[2])        

        cmap = plt.cm.get_cmap("RdYlGn")
        ax.scatter(x,y, z)
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
        default=True
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
        default=False
    )
    
    _args = vars(parser.parse_known_args()[0])
    print(_args)
    create = bool(_args['create'])
    collisions = bool(_args['collisions'])
    plot = bool(_args['plot'])

    rc = Reachability(create, collisions)

    if create:
        timings = rc.generate_map()
        rc.dump_data(file=rc.reach_file)
        np.savetxt(os.path.join(rc.src_fldr,"timings.csv"),
            timings,
            delimiter =", ",
            fmt ='% s')
        for i in range(10):
            print('\a')
            time.sleep(1)
    else:
        rc.load_data(rc.reach_file)
        
    # rc.plot_diff()
    if plot:
        x, y, z = rc.plot()
        
        if rc.node != None:
            node = rc.node
        else:
            import rclpy
            rclpy.init(args=sys.argv)
            node = rclpy.create_node("reachability")
        
        node.get_logger().info("X min: " + str(min(x)) + " X max: " + str(max(x)))
        node.get_logger().info("Y min: " + str(min(y)) + " Y max: " + str(max(y)))
        node.get_logger().info("Z min: " + str(min(z)) + " Z max: " + str(max(z)))
        
        Xmax_I = x.index(max(x))
        ZAtXmax = z[Xmax_I]
        node.get_logger().info("Z at X max: " + str(ZAtXmax))   
        node.get_logger().info("Max radius: " + str(max(z)-ZAtXmax) + " Min radius: " + str(ZAtXmax - min(z)))

if __name__ == "__main__":
    main()