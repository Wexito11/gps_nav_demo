#!/usr/bin/env python3
"""Drive the rover through GPS (lat/lon) or world (x/y) waypoints with Nav2.

Pipeline: waypoints.yaml -> /fromLL (robot_localization) -> map-frame poses
          -> Nav2 FollowWaypoints -> check that the robot stopped inside finish zone B.
"""
import math

import rclpy
import yaml
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.wait_for_message import wait_for_message
from robot_localization.srv import FromLL
from visualization_msgs.msg import Marker, MarkerArray


class GpsWaypointFollower(BasicNavigator):

    def __init__(self):
        super().__init__(node_name='gps_waypoint_follower')
        self.declare_parameter('waypoints_file', '')
        path = self.get_parameter('waypoints_file').value
        with open(path) as f:
            cfg = yaml.safe_load(f)
        self.wps = cfg['waypoints']
        self.finish_radius = float(cfg.get('finish_zone_radius', 0.7))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', latched)
        self.markers = MarkerArray()
        self.create_timer(1.0, lambda: self.marker_pub.publish(self.markers))
        self.fromll = self.create_client(FromLL, 'fromLL')

    # ---------- GPS -> map ----------
    def to_map_xy(self, wp):
        if 'x' in wp:                      # already in world/map frame
            return float(wp['x']), float(wp['y'])
        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=float(wp['lat']), longitude=float(wp['lon']), altitude=0.0)
        future = self.fromll.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        p = future.result().map_point
        return p.x, p.y

    def build_poses(self, xy):
        poses = []
        for i, (x, y) in enumerate(xy):
            # face the next waypoint, so the robot doesn't spin on every point
            nx, ny = xy[i + 1] if i + 1 < len(xy) else (2 * x - xy[i - 1][0], 2 * y - xy[i - 1][1])
            yaw = math.atan2(ny - y, nx - x)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            poses.append(ps)
        return poses

    def publish_markers(self, xy):
        arr = MarkerArray()
        for i, ((x, y), wp) in enumerate(zip(xy, self.wps)):
            last = i == len(xy) - 1
            for kind in (Marker.CYLINDER, Marker.TEXT_VIEW_FACING):
                m = Marker()
                m.header.frame_id = 'map'
                m.ns, m.id, m.type = 'waypoints', 2 * i + (kind == Marker.TEXT_VIEW_FACING), kind
                m.pose.position.x, m.pose.position.y = x, y
                m.pose.orientation.w = 1.0
                if kind == Marker.CYLINDER:
                    d = 2 * self.finish_radius if last else 0.3
                    m.scale.x, m.scale.y, m.scale.z = d, d, 0.02
                    m.color.r, m.color.g, m.color.b, m.color.a = (0.9, 0.1, 0.1, 0.5) if last \
                        else (0.1, 0.6, 1.0, 0.8)
                else:
                    m.pose.position.z = 0.6
                    m.scale.z = 0.3
                    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
                    m.text = wp.get('name', f'WP{i + 1}')
                arr.markers.append(m)
        self.markers = arr
        self.marker_pub.publish(arr)

    # ---------- main ----------
    def run(self):
        self.info('Waiting for GPS localisation (/odometry/gps)...')
        ok, _ = wait_for_message(Odometry, self, '/odometry/gps', time_to_wait=60.0)
        if not ok:
            self.error('No /odometry/gps after 60 s: is navsat_transform running?')
            return
        self.fromll.wait_for_service()

        xy = [self.to_map_xy(wp) for wp in self.wps]
        for wp, (x, y) in zip(self.wps, xy):
            self.info(f"{wp.get('name')}: lat/lon {wp.get('lat')}, {wp.get('lon')} -> map ({x:.2f}, {y:.2f})")
        self.publish_markers(xy)

        self.waitUntilNav2Active(localizer='robot_localization')
        self.clearAllCostmaps()   # drop anything marked before the EKF snapped to GPS
        self.followWaypoints(self.build_poses(xy))

        last = -1
        while not self.isTaskComplete():
            fb = self.getFeedback()
            if fb and fb.current_waypoint != last:
                last = fb.current_waypoint
                self.info(f'-> heading to {self.wps[last].get("name")} ({last + 1}/{len(xy)})')

        result = self.getResult()
        ok, odom = wait_for_message(Odometry, self, '/odometry/global', time_to_wait=5.0)
        bx, by = xy[-1]
        dist = math.hypot(odom.pose.pose.position.x - bx, odom.pose.pose.position.y - by) if ok else float('inf')
        if result == TaskResult.SUCCEEDED and dist <= self.finish_radius:
            self.info(f'FINISH: robot is in zone B ({dist:.2f} m from centre)')
        else:
            self.error(f'Route not completed: result={result}, distance to B={dist:.2f} m')


def main():
    rclpy.init()
    node = GpsWaypointFollower()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
