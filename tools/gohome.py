"""アームを初期姿勢へ 6 秒かけて戻す(コンテナ open_manipulator 内で実行)。"""
import json,time,rclpy,numpy as np
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
J=['joint1','joint2','joint3','joint4','joint5','gripper_joint_1']
# 初期姿勢(学習データの開始姿勢)
home=np.array([0.129,-2.049,1.741,1.443,0.06,0.693])
rclpy.init(); n=Node("gohome"); pub=n.create_publisher(JointTrajectory,"/leader/joint_trajectory",10); cur={}
n.create_subscription(JointState,"/joint_states",lambda m:cur.update(zip(m.name,m.position)),10)
while not all(j in cur for j in J): rclpy.spin_once(n,timeout_sec=0.1)
st=np.array([cur[j] for j in J]); T=6.0; t0=time.time(); nxt=t0
while time.time()-t0<T+1.5:
    f=min((time.time()-t0)/T,1.0); f=f*f*(3-2*f); p=st+(home-st)*f
    if time.time()>=nxt:
        m=JointTrajectory(); m.joint_names=J; pt=JointTrajectoryPoint(); pt.positions=list(map(float,p)); pt.time_from_start=Duration(sec=0,nanosec=0); m.points=[pt]; pub.publish(m); nxt+=1/30
    rclpy.spin_once(n,timeout_sec=0.002)
print("final",[round(cur[j],2) for j in J])
