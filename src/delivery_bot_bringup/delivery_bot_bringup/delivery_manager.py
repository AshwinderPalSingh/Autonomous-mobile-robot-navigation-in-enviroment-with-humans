#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Delivery mission manager.

Drives the robot through a queue of delivery waypoints using the Nav2 Simple
Commander API (NavigateToPose action under the hood). This is the "delivery
system" layer of the project: mission sequencing, per-leg supervision with a
timeout guard, and an end-of-mission report.

Usage (Phase 6, with sim + nav already running):

    ros2 run delivery_bot_bringup delivery_manager --ros-args \
        --params-file $(ros2 pkg prefix delivery_bot_bringup)/share/delivery_bot_bringup/config/mission.yaml

A/B protocol: run the mission once with the social layer enabled and once
disabled (see nav2_params.yaml header), with metrics_logger running in a
separate terminal for each run.

Design notes:
  * `localizer` must match how nav.launch.py was started: 'amcl' for the
    default saved-map mode, 'slam_toolbox' for slam:=True. waitUntilNav2Active
    polls the localizer's lifecycle state and will wait forever on a mismatch.
  * Each leg has a hard timeout. A leg that can't finish (blocked corridor,
    replan loop) is canceled and reported instead of stalling the mission --
    no failed attempt is allowed to hold the system indefinitely.
  * AMCL already auto-initializes at (0,0,0) via nav2_params.yaml, so
    `set_initial_pose` defaults to False here to avoid double-setting.
"""

import math
import time

import rclpy
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

RESULT_LABELS = {
    TaskResult.SUCCEEDED: 'SUCCEEDED',
    TaskResult.CANCELED: 'CANCELED',
    TaskResult.FAILED: 'FAILED',
}


def make_pose(navigator, x, y, yaw):
    """PoseStamped in the map frame from (x, y, yaw)."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return pose


def run_leg(nav, log, leg_name, x, y, yaw, leg_timeout):
    """Execute one NavigateToPose leg with feedback logging + timeout guard.

    Returns (result_label, duration_s).
    """
    start = nav.get_clock().now()
    nav.goToPose(make_pose(nav, x, y, yaw))

    last_print = time.monotonic()
    timed_out = False

    # isTaskComplete() internally spins the node with a 100 ms timeout,
    # so this loop is naturally paced -- no explicit sleep needed.
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback is None:
            continue

        if time.monotonic() - last_print > 2.0:
            log.info(f'[{leg_name}] {feedback.distance_remaining:.2f} m remaining')
            last_print = time.monotonic()

        if not timed_out and Duration.from_msg(feedback.navigation_time) > Duration(seconds=leg_timeout):
            log.warn(f'[{leg_name}] exceeded {leg_timeout:.0f} s -- canceling leg')
            nav.cancelTask()
            timed_out = True

    duration = (nav.get_clock().now() - start).nanoseconds / 1e9
    label = 'TIMED_OUT' if timed_out else RESULT_LABELS.get(nav.getResult(), 'UNKNOWN')
    return label, duration


def main():
    rclpy.init()
    nav = BasicNavigator()
    log = nav.get_logger()

    # --- Parameters (see config/mission.yaml) ------------------------------
    nav.declare_parameter('waypoints', [
        -4.5, 3.5, 3.1416,
        2.0, 3.5, 0.0,
        3.5, -3.5, -1.5708,
        -4.5, -3.5, 3.1416,
    ])
    nav.declare_parameter('loops', 1)
    nav.declare_parameter('localizer', 'amcl')
    nav.declare_parameter('stop_on_failure', False)
    nav.declare_parameter('leg_timeout', 180.0)
    nav.declare_parameter('set_initial_pose', False)
    nav.declare_parameter('initial_pose', [0.0, 0.0, 0.0])

    flat = [float(v) for v in nav.get_parameter('waypoints').value]
    loops = int(nav.get_parameter('loops').value)
    localizer = str(nav.get_parameter('localizer').value)
    stop_on_failure = bool(nav.get_parameter('stop_on_failure').value)
    leg_timeout = float(nav.get_parameter('leg_timeout').value)

    if len(flat) < 3 or len(flat) % 3 != 0:
        log.error('waypoints must be a flat [x, y, yaw, x, y, yaw, ...] list '
                  f'(got {len(flat)} values). Aborting.')
        nav.destroy_node()
        rclpy.shutdown()
        return

    waypoints = [(flat[i], flat[i + 1], flat[i + 2])
                 for i in range(0, len(flat), 3)]

    # --- Bring-up handshake -------------------------------------------------
    if bool(nav.get_parameter('set_initial_pose').value):
        ip = [float(v) for v in nav.get_parameter('initial_pose').value]
        nav.setInitialPose(make_pose(nav, ip[0], ip[1], ip[2]))
        log.info(f'Initial pose set to ({ip[0]:.2f}, {ip[1]:.2f}, yaw {ip[2]:.2f})')

    log.info(f"Waiting for Nav2 to become active (localizer: '{localizer}')...")
    nav.waitUntilNav2Active(localizer=localizer)
    log.info(f'Nav2 active. Mission: {len(waypoints)} stops x {loops} loop(s), '
             f'leg timeout {leg_timeout:.0f} s.')

    # --- Mission loop --------------------------------------------------------
    results = []
    aborted = False
    for loop_idx in range(loops):
        for wp_idx, (x, y, yaw) in enumerate(waypoints):
            leg_name = f'loop {loop_idx + 1}/{loops} stop {wp_idx + 1}/{len(waypoints)}'
            log.info(f'[{leg_name}] -> ({x:.2f}, {y:.2f}), yaw {yaw:.2f}')

            label, duration = run_leg(nav, log, leg_name, x, y, yaw, leg_timeout)
            results.append((leg_name, x, y, label, duration))
            log.info(f'[{leg_name}] {label} in {duration:.1f} s')

            if label != 'SUCCEEDED' and stop_on_failure:
                log.error('stop_on_failure is set -- aborting mission')
                aborted = True
                break
        if aborted:
            break

    # --- Mission report ------------------------------------------------------
    ok = sum(1 for r in results if r[3] == 'SUCCEEDED')
    total_time = sum(r[4] for r in results)
    log.info('=' * 58)
    log.info(f'MISSION REPORT: {ok}/{len(results)} legs succeeded, '
             f'{total_time:.1f} s total navigation time')
    for leg_name, x, y, label, duration in results:
        log.info(f'  {label:10s} ({x:6.2f}, {y:6.2f}) {duration:6.1f} s  [{leg_name}]')
    log.info('=' * 58)

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
