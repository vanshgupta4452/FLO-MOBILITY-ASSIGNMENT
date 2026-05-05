#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan

import numpy as np
import math


class SimpleMPC(Node):

    def __init__(self):

        super().__init__("simple_mpc")


        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.current_path_index = 0


        self.path = []
        self.scan = None


        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.create_subscription(
            Path,
            "/path",
            self.path_callback,
            10
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )


        self.horizon = 30
        self.dt = 0.1

        self.max_v = 0.3
        self.max_w = 1.25

        self.goal_tolerance = 0.2


        self.v_samples = np.linspace(0.05, 0.3, 10)
        self.w_samples = np.linspace(-self.max_w, self.max_w, 15)

        self.dynamic_obstacles = []

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info("Simple MPC Controller Started")

    
    def path_callback(self, msg):

        self.path = []

        for pose in msg.poses:

            self.path.append([
                pose.pose.position.x,
                pose.pose.position.y
            ])

        self.get_logger().info(
            f"Received path with {len(self.path)} points"
        )

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)

        self.yaw = math.atan2(siny, cosy)


    def scan_callback(self, msg):

        self.scan = msg

    def simulate_trajectory(self, v, w):

        traj = []

        x = self.x
        y = self.y
        yaw = self.yaw

        for _ in range(self.horizon):

            x = x + v * math.cos(yaw) * self.dt
            y = y + v * math.sin(yaw) * self.dt
            yaw = yaw + w * self.dt

            traj.append((x, y, yaw))

        return traj


    def get_obstacle_points(self):

        points = []

        if self.scan is None:
            return points

        angle = self.scan.angle_min

        for r in self.scan.ranges:

            if np.isfinite(r):

                # ignore far points
                if r < 2.5:

                    ox = r * math.cos(angle)
                    oy = r * math.sin(angle)

                    # convert to world frame
                    wx = self.x + (
                        ox * math.cos(self.yaw)
                        - oy * math.sin(self.yaw)
                    )

                    wy = self.y + (
                        ox * math.sin(self.yaw)
                        + oy * math.cos(self.yaw)
                    )

                    points.append((wx, wy))

            angle += self.scan.angle_increment

        return points



    def predict_obstacle(self, obs):

        traj = []

        x = obs["x"]
        y = obs["y"]

        vx = obs["vx"]
        vy = obs["vy"]

        for i in range(self.horizon):

            t = i * self.dt

            fx = x + vx * t
            fy = y + vy * t

            traj.append((fx, fy))

        return traj 



    def dynamic_obstacle_cost(self, robot_traj):

        cost = 0.0

        for obs in self.dynamic_obstacles:

            obs_traj = self.predict_obstacle(obs)

            for rp, op in zip(robot_traj, obs_traj):

                d = math.hypot(rp[0]-op[0], rp[1]-op[1])

                if d < 0.6:
                  return 1e6

        return cost       


    def nearest_path_error(self, point):

        if len(self.path) == 0:
            return 999.0

        px, py = point

        min_dist = 1e9
        best_idx = self.current_path_index

        # SEARCH ONLY FORWARD PART OF PATH
        search_end = min(
            self.current_path_index + 2,
            len(self.path)
        )

        for i in range(
            self.current_path_index,
            search_end
        ):

            p = self.path[i]

            d = math.hypot(
                px - p[0],
                py - p[1]
            )

            if d < min_dist:

                min_dist = d
                best_idx = i

        self.current_path_index = best_idx

        return min_dist


    def obstacle_cost(self, traj):

        obstacle_points = self.get_obstacle_points()

        if len(obstacle_points) == 0:
            return 0.0

        cost = 0.0

        for pt in traj:

            rx = pt[0]
            ry = pt[1]

            for ox, oy in obstacle_points:

                d = math.hypot(rx - ox, ry - oy)

                # COLLISION
                if d < 0.35:
                    return 1e6

                # Near obstacle
                elif d < 0.8:
                    cost += 200.0 / d

        return cost



    def heading_cost(self, traj):

        if len(self.path) < 2:
            return 0.0

        final_pt = traj[-1]

        nearest_idx = 0
        min_d = 1e9

        for i, p in enumerate(self.path):

            d = math.hypot(
                final_pt[0] - p[0],
                final_pt[1] - p[1]
            )

            if d < min_d:
                min_d = d
                nearest_idx = i

        if nearest_idx >= len(self.path) - 1:
            return 0.0

        p1 = self.path[nearest_idx]
        p2 = self.path[nearest_idx + 1]

        path_yaw = math.atan2(
            p2[1] - p1[1],
            p2[0] - p1[0]
        )

        robot_yaw = traj[-1][2]

        yaw_error = abs(
            math.atan2(
                math.sin(path_yaw - robot_yaw),
                math.cos(path_yaw - robot_yaw)
            )
        )

        return 5.0 * yaw_error


   
    def goal_reached(self):

        if len(self.path) == 0:
            return False

        goal = self.path[-1]

        d = math.hypot(
            self.x - goal[0],
            self.y - goal[1]
        )

        return d < self.goal_tolerance


    def compute_cost(self, traj, v, w):

        cost = 0.0


        for pt in traj:

            path_error = self.nearest_path_error(
                (pt[0], pt[1])
            )

            cost += 3.0 * path_error

        goal = self.path[-1]

        final_pt = traj[-1]

        goal_error = math.hypot(
            final_pt[0] - goal[0],
            final_pt[1] - goal[1]
        )

        cost += 2.0 * goal_error


        cost += self.obstacle_cost(traj)


        cost += 0.2 * abs(w)


        cost -= 8.0 * v

        return cost

   
    def control_loop(self):

        try:

            self.get_logger().info("CONTROL LOOP")

            if len(self.path) == 0:
                self.get_logger().warn("No path")
                return

            if self.goal_reached():

                cmd = Twist()
                self.cmd_pub.publish(cmd)

                self.get_logger().info("Goal reached")

                return

            best_cost = 1e9

            best_v = 0.0
            best_w = 0.0

            for v in self.v_samples:

                for w in self.w_samples:

                    traj = self.simulate_trajectory(v, w)

                    if len(traj) == 0:
                        continue

                    cost = self.compute_cost(
                        traj,
                        v,
                        w
                    )

                    if cost < best_cost:

                        best_cost = cost
                        best_v = v
                        best_w = w

            cmd = Twist()

            cmd.linear.x = float(best_v)
            cmd.angular.z = float(best_w)

            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                f"PUBLISHED v={best_v:.2f}, "
                f"w={best_w:.2f}"
            )

        except Exception as e:

            self.get_logger().error(
                f"CONTROL LOOP ERROR: {str(e)}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = SimpleMPC()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

