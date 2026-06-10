#!/usr/bin/env python3
"""Find a reachable z+orientation for a fixed (x,y) via /compute_ik."""
import rclpy, math
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration

X, Y = 0.362, 0.142

def q_axis(ax, deg):
    a = math.radians(deg) / 2.0
    s = math.sin(a)
    return (ax[0]*s, ax[1]*s, ax[2]*s, math.cos(a))

ORIENTS = {
    "identity(up)": (0.0, 0.0, 0.0, 1.0),
    "rotY+45":  q_axis((0,1,0),  45),
    "rotY+90":  q_axis((0,1,0),  90),
    "rotY-45":  q_axis((0,1,0), -45),
    "rotX+45":  q_axis((1,0,0),  45),
    "rotX-45":  q_axis((1,0,0), -45),
    # yaw toward the target so the arm faces (x,y), then pitch down 45
    "yaw+pitch": None,
}
yaw = math.atan2(Y, X)
# compose yaw(Z) then pitch(Y) 45deg as quaternion
def quat_mult(a, b):
    ax,ay,az,aw=a; bx,by,bz,bw=b
    return (aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,
            aw*bw-ax*bx-ay*by-az*bz)
ORIENTS["yaw+pitch"] = quat_mult(q_axis((0,0,1), math.degrees(yaw)), q_axis((0,1,0), 45))

def main():
    rclpy.init(); node = Node("ik_probe_xy")
    cli = node.create_client(GetPositionIK, "compute_ik"); cli.wait_for_service(5.0)
    def code_for(z, q, avoid=True):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"; req.ik_request.avoid_collisions = avoid
        ps = PoseStamped(); ps.header.frame_id = "world"
        ps.pose.position.x=X; ps.pose.position.y=Y; ps.pose.position.z=z
        ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w=q
        req.ik_request.pose_stamped = ps; req.ik_request.timeout = Duration(sec=1)
        fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        return fut.result().error_code.val if fut.result() else None
    print(f"\n=== (x={X}, y={Y}), horizontal reach = {math.hypot(X,Y):.3f} m ===")
    zs = [round(z,2) for z in [0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1]]
    any_hit = False
    for name, q in ORIENTS.items():
        ok = [z for z in zs if code_for(z, q) == 1]
        if ok:
            any_hit = True
            print(f"  {name:12s} reachable z: {ok}   quat={tuple(round(v,4) for v in q)}")
    if not any_hit:
        print("  No collision-free IK at any z/orientation tried. Checking reach-only (avoid_collisions=False):")
        for name, q in ORIENTS.items():
            ok = [z for z in zs if code_for(z, q, avoid=False) == 1]
            if ok:
                print(f"  [reach-OK, blocked only by collision] {name:12s} z: {ok}")
    node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
