#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""A/B evaluation logger for the social navigation experiment.

Samples robot-pedestrian proximity and path length during a run, then writes
a CSV and prints a summary on Ctrl-C. Run it once per condition:

    # Run A (baseline): social layer disabled in both costmaps
    ros2 run delivery_bot_bringup metrics_logger --ros-args -p label:=baseline

    # Run B (social): social layer enabled
    ros2 run delivery_bot_bringup metrics_logger --ros-args -p label:=social

Reported metrics:
  * run duration [s] (sim time)
  * robot path length [m]
  * minimum robot-pedestrian distance over the run [m]
  * personal-space intrusions: samples with min distance < `personal_space`
    (0.8 m default), as count and percentage

Design notes:
  * Distances are computed directly in the odom frame: the pedestrian
    simulator publishes in odom and /odom poses live there too, so no TF
    lookup is needed and the numbers are exact. If the pedestrian message
    ever arrives in a different frame, a one-time warning is printed.
  * Path length integrates /odom at full rate, with a per-step jump filter
    (> 0.5 m between consecutive 50 Hz samples = teleport/glitch, rejected)
    so a single bad reading can't corrupt the metric.
  * Stale pedestrian data (no message within `detection_timeout`) pauses
    sampling rather than freezing the last known distance into the stats.
"""

import csv
import math
import os
import time as wall_time
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from social_nav_msgs.msg import Pedestrians


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')

        # The "duration (sim)" metric is the basis of the A/B comparison, so
        # the node must run on the Gazebo clock even if the launch command
        # forgets -p use_sim_time:=true. Force it on unless explicitly set.
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        if self.get_parameter('use_sim_time').value is not True:
            self.set_parameters([rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.declare_parameter('label', 'run')
        self.declare_parameter('sample_rate', 10.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pedestrians_topic', '/pedestrians')
        self.declare_parameter('personal_space', 0.8)
        self.declare_parameter('detection_timeout', 1.0)
        self.declare_parameter('output_dir', '/tmp')

        self.label = str(self.get_parameter('label').value)
        rate = float(self.get_parameter('sample_rate').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        ped_topic = str(self.get_parameter('pedestrians_topic').value)
        self.personal_space = float(self.get_parameter('personal_space').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.output_dir = str(self.get_parameter('output_dir').value)

        # Live state
        self.odom = None
        self.peds = None
        self.peds_wall_stamp = 0.0
        self.frame_warned = False
        self.last_xy = None

        # Accumulated metrics
        self.rows = []                       # (t, x, y, min_dist, ped_id, path_len)
        self.path_length = 0.0
        self.min_dist_overall = float('inf')
        self.closest_ped_overall = ''
        self.intrusions = 0
        self.samples = 0
        self.t0 = None                       # sim time of first valid sample

        self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)
        self.create_subscription(Pedestrians, ped_topic, self.peds_cb, 20)
        self.create_timer(1.0 / max(rate, 1.0), self.sample)

        self.get_logger().info(
            f"metrics_logger '{self.label}': sampling {rate:.0f} Hz, "
            f'personal_space {self.personal_space:.2f} m. Ctrl-C to finish '
            'and write the CSV.')

    # ------------------------------------------------------------------
    def odom_cb(self, msg):
        self.odom = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_xy is not None:
            step = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            # Jump filter: at 50 Hz odom, 0.5 m/step = 25 m/s. Reject glitches
            # so one bad sample can't corrupt the path-length metric.
            if step < 0.5:
                self.path_length += step
        self.last_xy = (x, y)

    def peds_cb(self, msg):
        self.peds = msg
        self.peds_wall_stamp = wall_time.monotonic()
        if (not self.frame_warned and self.odom is not None and
                msg.header.frame_id != self.odom.header.frame_id):
            self.get_logger().warn(
                f"pedestrians frame '{msg.header.frame_id}' != odom frame "
                f"'{self.odom.header.frame_id}' -- distances assume a shared "
                'frame and may be biased by localization drift.')
            self.frame_warned = True

    # ------------------------------------------------------------------
    def sample(self):
        if self.odom is None or self.peds is None or not self.peds.pedestrians:
            return
        if wall_time.monotonic() - self.peds_wall_stamp > self.detection_timeout:
            return  # stale detections: pause rather than freeze stats

        rx = self.odom.pose.pose.position.x
        ry = self.odom.pose.pose.position.y

        min_dist = float('inf')
        closest = ''
        for ped in self.peds.pedestrians:
            d = math.hypot(ped.position.x - rx, ped.position.y - ry)
            if d < min_dist:
                min_dist = d
                closest = ped.id

        t = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = t

        self.samples += 1
        if min_dist < self.personal_space:
            self.intrusions += 1
        if min_dist < self.min_dist_overall:
            self.min_dist_overall = min_dist
            self.closest_ped_overall = closest

        self.rows.append(
            (t - self.t0, rx, ry, min_dist, closest, self.path_length))

    # ------------------------------------------------------------------
    def finish(self):
        """Write CSV + print the summary. Called once on shutdown."""
        if not self.rows:
            print('\n[metrics_logger] No samples recorded -- were /odom and '
                  '/pedestrians publishing?')
            return

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(
            self.output_dir, f'social_metrics_{self.label}_{stamp}.csv')
        try:
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['t_s', 'robot_x', 'robot_y',
                                 'min_ped_dist_m', 'closest_ped',
                                 'path_length_m'])
                writer.writerows(self.rows)
        except OSError as exc:
            csv_path = f'<write failed: {exc}>'

        duration = self.rows[-1][0]
        intrusion_pct = 100.0 * self.intrusions / self.samples

        print('\n' + '=' * 58)
        print(f"SOCIAL NAV METRICS -- '{self.label}'")
        print('=' * 58)
        print(f'  samples                 : {self.samples}')
        print(f'  duration (sim)          : {duration:8.1f} s')
        print(f'  path length             : {self.path_length:8.2f} m')
        print(f'  min pedestrian distance : {self.min_dist_overall:8.2f} m '
              f'({self.closest_ped_overall})')
        print(f'  personal-space (<{self.personal_space:.1f} m) '
              f'intrusions : {self.intrusions} ({intrusion_pct:.1f}% of samples)')
        print(f'  CSV                     : {csv_path}')
        print('=' * 58)


def main():
    rclpy.init()
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
