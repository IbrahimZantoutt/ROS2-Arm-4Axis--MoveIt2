#!/usr/bin/env python3
"""Throwaway: sweep candidate poses through /compute_ik to find reachable ones."""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
import math

# Orientations to try (qx,qy,qz,qw): identity(up), 180 about X, 90 about Y, 180 about Y
ORIENTS = {
    "up(identity)":   (0.0, 0.0, 0.0, 1.0),
    "flipX(down?)":   (1.0, 0.0, 0.0, 0.0),
    "rotY+90":        (0.0, math.sin(math.pi/4), 0.0, math.cos(math.pi/4)),
    "rotY-90":        (0.0, math.sin(-math.pi/4), 0.0, math.cos(-math.pi/4)),
}

def main():
    rclpy.init()
    node = Node("ik_probe")
    cli = node.create_client(GetPositionIK, "compute_ik")
    cli.wait_for_service(timeout_sec=5.0)

    def reachable(x, y, z, q, avoid=True):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.avoid_collisions = avoid
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout = Duration(sec=1)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        return fut.result().error_code.val if fut.result() else None

    # Sweep a grid of forward/side/height positions for each orientation.
    hits = []
    xs = [0.0, 0.15, 0.25, 0.35, 0.45]
    zs = [0.4, 0.6, 0.8, 1.0]
    for name, q in ORIENTS.items():
        for x in xs:
            for z in zs:
                code = reachable(x, 0.0, z, q)
                if code == 1:  # SUCCESS
                    hits.append((name, x, 0.0, z))
    print("\n=== REACHABLE (collision-free IK) poses ===")
    if not hits:
        print("  none in grid — retrying WITHOUT collision checking to see if it's reach vs collision")
        for name, q in ORIENTS.items():
            for x in xs:
                for z in zs:
                    code = reachable(x, 0.0, z, q, avoid=False)
                    if code == 1:
                        print(f"  [reach-OK but COLLIDES with avoid=on] {name:14s} ({x:.2f}, 0.00, {z:.2f})")
    else:
        for name, x, y, z in hits:
            print(f"  {name:14s} ({x:.2f}, {y:.2f}, {z:.2f})")
    node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
