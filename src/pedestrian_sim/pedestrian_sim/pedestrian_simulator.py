#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Kinematic pedestrian simulator with ORCA collision avoidance.

Simulates pedestrians as waypoint-following agents whose local collision
avoidance (of each other, of static warehouse obstacles, and of the robot)
is delegated to RVO2 (the reference ORCA -- Optimal Reciprocal Collision
Avoidance -- implementation). This replaces a hand-rolled repulsion-force
heuristic that could stall or oscillate when a pedestrian approached the
robot nearly head-on (the direction-to-dodge computation degenerates
exactly when the desired travel direction is anti-parallel to the
away-from-robot direction, i.e. the common "robot parked in the corridor"
case). ORCA's linear-programming formulation guarantees a collision-free
velocity each step given each agent's radius and time horizon, so no
teleport-style escape hatch is needed.

The robot is injected into the ORCA simulation as a ground-truth-driven
agent (its position/velocity are read from /odom every tick, never
computed by ORCA) -- pedestrians must react to it, but it never reacts to
them; Nav2/Gazebo remain the sole authority over the robot's motion.

Publishes:
  * social_nav_msgs/Pedestrians   on /pedestrians         -> consumed by SocialLayer
  * visualization_msgs/MarkerArray on /pedestrian_markers -> RViz visualization

Design intent: this node plays the role of a *perfect perception stack*.
The project's contribution is the navigation-side integration, so detections
are ground truth here. Swap this node for a real detector (leg tracker,
vision) later without touching the costmap plugin -- the interface is the
Pedestrians message.

Requires the Python-RVO2 package (not on PyPI under a pip-installable name;
build from source -- see pedestrian_sim/README or the top-level README's
Install section).
"""

import math
import random

import rclpy
import rvo2
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from social_nav_msgs.msg import Pedestrian, Pedestrians
from visualization_msgs.msg import Marker, MarkerArray


# ── ORCA tuning (scaled for this warehouse's ~12x9 m interior) ───────────
ORCA_NEIGHBOR_DIST = 4.0        # [m] how far an agent looks for other agents
ORCA_MAX_NEIGHBORS = 10
ORCA_TIME_HORIZON = 2.5         # [s] how far ahead to plan around other agents
ORCA_TIME_HORIZON_OBST = 1.2    # [s] how far ahead to plan around obstacles/walls
SPEED_HEADROOM = 1.3            # maxSpeed given to ORCA vs. nominal cruise speed,
                                 # so an avoidance maneuver can briefly hurry
ARRIVAL_TOL = 0.25              # [m] waypoint arrival tolerance

# Static obstacles as (cx, cy, half_sx, half_sy) rectangles -- true footprints
# from warehouse.sdf. No artificial margin: each agent's own radius plus
# ORCA_TIME_HORIZON_OBST already produce sensible clearance.
STATIC_OBSTACLES = [
    (-1.0, 2.0, 1.25, 0.30),    # shelf_a
    (-1.0, -2.0, 1.25, 0.30),   # shelf_b
    (4.8, 3.2, 0.40, 0.40),     # box_1
    (4.8, -3.2, 0.40, 0.40),    # box_2
    (-3.5, -2.5, 0.20, 0.20),   # pillar
    (0.0, 4.5, 6.15, 0.075),    # wall_north
    (0.0, -4.5, 6.15, 0.075),   # wall_south
    (6.0, 0.0, 0.075, 4.65),    # wall_east
    (-6.0, 0.0, 0.075, 4.65),   # wall_west
]

# Colors for each pedestrian's RViz marker (R, G, B)
PED_COLORS = [
    (0.1, 0.7, 0.9),   # cyan
    (0.9, 0.3, 0.3),   # red
    (0.3, 0.9, 0.3),   # green
    (0.9, 0.9, 0.2),   # yellow
    (0.9, 0.4, 0.9),   # magenta
    (0.3, 0.5, 0.9),   # blue
    (1.0, 0.6, 0.2),   # orange
    (0.6, 0.2, 0.8),   # purple
]


def _rect_corners_ccw(cx, cy, hx, hy):
    """Corners of an axis-aligned rectangle, counter-clockwise -- RVO2's
    required winding order for a solid (blocking) obstacle."""
    return [
        (cx - hx, cy - hy),
        (cx + hx, cy - hy),
        (cx + hx, cy + hy),
        (cx - hx, cy + hy),
    ]


class SimPed:
    """One simulated pedestrian's waypoint bookkeeping.

    Motion and collision avoidance are delegated to the shared RVO2
    simulation; this class only tracks which waypoint an agent is heading
    for and computes the *preferred* (unobstructed) velocity toward it.
    """

    def __init__(self, name, waypoints, speed, radius, mode, noise_std):
        self.name = name
        self.waypoints = waypoints          # list of (x, y)
        self.speed = float(speed)
        self.radius = float(radius)
        self.mode = mode                    # 'pingpong' or 'loop'
        self.noise_std = float(noise_std)
        self.x, self.y = waypoints[0]
        self.target_idx = 1 if len(waypoints) > 1 else 0
        self.direction = 1
        self.vx = 0.0
        self.vy = 0.0

    def preferred_velocity(self):
        """Desired velocity toward the current waypoint, ignoring
        obstacles/other agents -- ORCA resolves collisions against this."""
        if len(self.waypoints) < 2 or self.speed <= 0.0:
            return 0.0, 0.0

        tx, ty = self.waypoints[self.target_idx]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0, 0.0

        ux, uy = dx / dist, dy / dist
        pvx, pvy = ux * self.speed, uy * self.speed
        if self.noise_std > 0.0:
            pvx += random.gauss(0.0, self.noise_std)
            pvy += random.gauss(0.0, self.noise_std)
        return pvx, pvy

    def advance_if_arrived(self):
        """Move to the next waypoint once within tolerance of the current
        target. Called after the ORCA step updates self.x/self.y."""
        tx, ty = self.waypoints[self.target_idx]
        if math.hypot(tx - self.x, ty - self.y) <= ARRIVAL_TOL:
            self._advance()

    def _advance(self):
        n = len(self.waypoints)
        if self.mode == 'loop':
            self.target_idx = (self.target_idx + 1) % n
        else:  # pingpong
            nxt = self.target_idx + self.direction
            if nxt >= n or nxt < 0:
                self.direction *= -1
                nxt = self.target_idx + self.direction
            self.target_idx = max(0, min(n - 1, nxt))


class PedestrianSimulator(Node):

    def __init__(self):
        super().__init__('pedestrian_simulator')

        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('pedestrians', ['ped_1'])
        self.declare_parameter('robot_radius', 0.35)

        rate = float(self.get_parameter('update_rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        names = list(self.get_parameter('pedestrians').value)
        robot_radius = float(self.get_parameter('robot_radius').value)

        self.peds = []
        for name in names:
            self.declare_parameter(f'{name}.waypoints', [0.0, 0.0, 1.0, 0.0])
            self.declare_parameter(f'{name}.speed', 0.6)
            self.declare_parameter(f'{name}.radius', 0.30)
            self.declare_parameter(f'{name}.mode', 'pingpong')
            self.declare_parameter(f'{name}.noise_std', 0.0)

            flat = list(self.get_parameter(f'{name}.waypoints').value)
            if len(flat) < 4 or len(flat) % 2 != 0:
                self.get_logger().error(
                    f'{name}: waypoints must be a flat [x1,y1,x2,y2,...] list '
                    f'with >= 2 points; skipping this pedestrian')
                continue
            wps = [(float(flat[i]), float(flat[i + 1]))
                   for i in range(0, len(flat), 2)]

            ped = SimPed(
                name, wps,
                self.get_parameter(f'{name}.speed').value,
                self.get_parameter(f'{name}.radius').value,
                str(self.get_parameter(f'{name}.mode').value),
                self.get_parameter(f'{name}.noise_std').value)
            self.peds.append(ped)
            self.get_logger().info(
                f'{name}: {len(wps)} waypoints, speed {ped.speed:.2f} m/s, '
                f'mode {ped.mode}')

        if not self.peds:
            self.get_logger().warn('No valid pedestrians configured.')

        self.dt = 1.0 / max(rate, 1.0)

        # Robot ground truth (from /odom), injected into ORCA every tick as
        # a moving obstacle that pedestrians must avoid but that never
        # reacts to them -- Nav2/Gazebo remain the sole authority over it.
        self.robot_x, self.robot_y = 0.0, 0.0
        self.robot_vx, self.robot_vy = 0.0, 0.0
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # ── ORCA (RVO2) simulation set-up ───────────────────────────────
        self.sim = rvo2.PyRVOSimulator(
            self.dt,
            ORCA_NEIGHBOR_DIST, ORCA_MAX_NEIGHBORS,
            ORCA_TIME_HORIZON, ORCA_TIME_HORIZON_OBST,
            0.3, 1.0)

        self.ped_agents = []
        for ped in self.peds:
            agent_id = self.sim.addAgent(
                (ped.x, ped.y),
                ORCA_NEIGHBOR_DIST, ORCA_MAX_NEIGHBORS,
                ORCA_TIME_HORIZON, ORCA_TIME_HORIZON_OBST,
                ped.radius, ped.speed * SPEED_HEADROOM, (0.0, 0.0))
            self.ped_agents.append(agent_id)

        self.robot_agent = self.sim.addAgent(
            (self.robot_x, self.robot_y),
            ORCA_NEIGHBOR_DIST, ORCA_MAX_NEIGHBORS,
            ORCA_TIME_HORIZON, ORCA_TIME_HORIZON_OBST,
            robot_radius, 1.0, (0.0, 0.0))

        for cx, cy, hx, hy in STATIC_OBSTACLES:
            self.sim.addObstacle(_rect_corners_ccw(cx, cy, hx, hy))
        self.sim.processObstacles()

        self.ped_pub = self.create_publisher(Pedestrians, 'pedestrians', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, 'pedestrian_markers', 10)
        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'Publishing {len(self.peds)} pedestrian(s) in frame '
            f"'{self.frame_id}' at {rate:.0f} Hz (ORCA collision avoidance)")

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # twist is body-frame (REP103); rotate into the world/odom frame
        # ORCA operates in before feeding it in as the robot agent's velocity.
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        lvx = msg.twist.twist.linear.x
        lvy = msg.twist.twist.linear.y
        self.robot_vx = math.cos(yaw) * lvx - math.sin(yaw) * lvy
        self.robot_vy = math.sin(yaw) * lvx + math.cos(yaw) * lvy

    # ------------------------------------------------------------------
    def tick(self):
        stamp = self.get_clock().now().to_msg()

        # Inject the robot's ground truth *before* stepping, so this
        # step's ORCA constraints for the pedestrians see its real,
        # current position/velocity.
        self.sim.setAgentPosition(self.robot_agent, (self.robot_x, self.robot_y))
        self.sim.setAgentVelocity(self.robot_agent, (self.robot_vx, self.robot_vy))
        self.sim.setAgentPrefVelocity(self.robot_agent, (self.robot_vx, self.robot_vy))

        for ped, agent_id in zip(self.peds, self.ped_agents):
            pvx, pvy = ped.preferred_velocity()
            self.sim.setAgentPrefVelocity(agent_id, (pvx, pvy))

        self.sim.doStep()

        msg = Pedestrians()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        markers = MarkerArray()
        for k, (ped, agent_id) in enumerate(zip(self.peds, self.ped_agents)):
            ped.x, ped.y = self.sim.getAgentPosition(agent_id)
            ped.vx, ped.vy = self.sim.getAgentVelocity(agent_id)
            ped.advance_if_arrived()

            p = Pedestrian()
            p.id = ped.name
            p.position = Point(x=ped.x, y=ped.y, z=0.0)
            p.velocity = Vector3(x=ped.vx, y=ped.vy, z=0.0)
            msg.pedestrians.append(p)

            markers.markers.extend(self._make_markers(ped, k, stamp))

        self.ped_pub.publish(msg)
        self.marker_pub.publish(markers)

    # ------------------------------------------------------------------
    def _make_markers(self, ped, k, stamp):
        out = []
        lifetime = Duration(seconds=0.5).to_msg()
        base_id = k * 3
        r, g, b = PED_COLORS[k % len(PED_COLORS)]

        body = Marker()
        body.header.frame_id = self.frame_id
        body.header.stamp = stamp
        body.ns = 'pedestrians'
        body.id = base_id
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x = ped.x
        body.pose.position.y = ped.y
        body.pose.position.z = 0.85
        body.pose.orientation.w = 1.0
        body.scale.x = 2.0 * ped.radius
        body.scale.y = 2.0 * ped.radius
        body.scale.z = 1.7
        body.color.r, body.color.g, body.color.b, body.color.a = r, g, b, 0.9
        body.lifetime = lifetime
        out.append(body)

        speed = math.hypot(ped.vx, ped.vy)
        if speed > 0.02:
            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = stamp
            arrow.ns = 'pedestrians'
            arrow.id = base_id + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(x=ped.x, y=ped.y, z=1.0),
                Point(x=ped.x + ped.vx, y=ped.y + ped.vy, z=1.0),
            ]
            arrow.scale.x = 0.05   # shaft diameter
            arrow.scale.y = 0.12   # head diameter
            arrow.scale.z = 0.15   # head length
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.55, 0.0, 0.9
            arrow.lifetime = lifetime
            out.append(arrow)

        text = Marker()
        text.header.frame_id = self.frame_id
        text.header.stamp = stamp
        text.ns = 'pedestrians'
        text.id = base_id + 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = ped.x
        text.pose.position.y = ped.y
        text.pose.position.z = 2.0
        text.pose.orientation.w = 1.0
        text.scale.z = 0.25
        text.color.r = text.color.g = text.color.b = 1.0
        text.color.a = 0.9
        text.text = ped.name
        text.lifetime = lifetime
        out.append(text)

        return out


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
