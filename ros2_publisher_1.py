#!/usr/bin/env python3

# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
import pinocchio as pin
from numpy.linalg  import norm,solve

def inverse_kinematics(current_q,target_dir,target_pos):
    urdf_filename = "/home/liman/ros2/mercury_x1_ros2/src/mercury_robot_urdf/mercury_robot_urdf/urdf/mercury_x1/mercury_x1_turing_stack.urdf"
    #从urdf文件中构建机器人模型
    model = pin.buildModelFromUrdf(urdf_filename)
    #为模型创建数据对象，用于存储计算过程的中间结果
    data = model.createData()
    #指定要控制的关节ID
    JOINT_ID = 7
    #定义期望的位姿，使用目标姿态的旋转矩阵和目标位置创建SE3对象
    oMdes = pin.SE3(target_dir , np.array(target_pos))
    #将当前关节角度赋值给变量q,作为迭代的初始值
    q = current_q
    #定义收敛的阈值
    eps = 1e-4
    #定义最大的迭代次数，防止算法进入无限循环
    IT_MAX = 1000
    #定义积分步长，用于更新关节角度
    DT = 1e-2
    #定义阻尼因子，用于避免矩阵奇异
    damp = 1e-12
    #初始化迭代次数
    i = 0

    while True:
        #进行正运动学计算，得到当前关节角度下机器人各关节的位置和姿态
        pin.forwardKinematics(model,data,q)
        #计算目标位姿到当前位姿之间的变换
        iMd = data.oMi[JOINT_ID].actInv(oMdes)
        #通过李群对数映射将变换矩阵转换为 6 维误差向量（包含位置误差和方向误差），用于量化当前位姿与目标位姿的差异
        err = pin.log(iMd).vector
        #判断误差是否小于收敛阈值，如果是则认为算法收敛
        if norm(err) < eps:
            success = True
            break
        #判断迭代次数是否超过最大迭代次数，如果是则认为算法未收敛
        if i>= IT_MAX:
            success = False
            break
        #计算当前关节角度下的雅可比矩阵，关节速度与末端速度的映射关系
        J =  pin.computeJointJacobian(model,data,q)
        #对雅可比矩阵进行变换，转换到李代数空间，以匹配误差向量的坐标系，同时取反以调整误差方向
        J = -np.dot(pin.Jlog6(iMd.inverse()),J)
        #使用阻尼最小二乘法求解关节速度
        v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6),err))
        #根据关节速度更新关节角度
        q = pin.integrare(model,q, v * DT)
        #每迭代 10 次打印一次当前的误差信息
        if not i % 10 :
            print(f"{i}:error = {err.T}")
        i += 1

        if success:
            print("Convergence achieved!")
        else:
            print(
            "\n"
            "Warning: the iterative algorithm has not reached convergence "
            "to the desired precision")

        print(f"\nresult: {q.flatten().tolist()}")
        print(f"\nfinal error: {err.T}")
        #返回最终的关节角度向量（以列表形式）
        return q.flatten().tolist()

def limit_angle(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle 

class TestROS2Bridge(Node):
    def __init__(self):

        super().__init__("test_ros2bridge")

        # Create the publisher. This publisher will publish a JointState message to the /joint_command topic.
        self.publisher_ = self.create_publisher(JointState, "joint_command_2", 10)
        
        # Create a JointState message
        self.joint_state = JointState()

        self.joint_state.name = [
            "joint1_L",
            "joint2_L",
            "joint3_L",
            "joint4_L",
            "joint5_L",
            "joint6_L",
            
        ]

        num_joints = len(self.joint_state.name)

        # make sure kit's editor is playing for receiving messages
        self.joint_state.position = np.array([0.0] * num_joints, dtype=np.float64).tolist()
        self.default_joints = [0, 0, 0, 0, 0, 0]
        # position control the robot to wiggle around each joint
        self.time_start = time.time()
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        theta = np.pi
        self.R_x = np.array([
            [1,0,0],
            [0,np.cos(theta),-np.sin(theta)],
            [0,np.sin(theta),-np.cos(theta)]
        ])
        self.x = 0.3

    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        if self.x < 6:
            self.x += 0.001
            new_q = inverse_kinematics(self.default_joints,self.R_x,[0.5,0,0])
        self.current_q = new_q
        joint_position = self.current_q
        self.joint_state.position = joint_position.tolist()

        # Publish the message to the topic
        self.publisher_.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)

    ros2_publisher = TestROS2Bridge()

    rclpy.spin(ros2_publisher)

    # Destroy the node explicitly
    ros2_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()