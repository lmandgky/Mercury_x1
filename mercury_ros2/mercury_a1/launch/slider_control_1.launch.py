import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import pinocchio as pin
import numpy as np
from scipy.optimize import minimize
from numpy.linalg  import norm,solve
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


def inverse_kinematics(current_q,target_dir,target_pos):
    urdf_filename = "/home/liman/ros_ws/src/mercury_ros2/mercury_description/urdf/mercury_a1/mercury_a1.urdf"
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
        J =  pin.computeJointJacobian(model,data,q,JOINT_ID)
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

class CustomNode(Node):
    def __init__(self,model,data):
        super.__init__("custom_node")
        self.joint_pub = self.create_publisher(JointState,"/joint_states",20)
        self.timer = self.create_timer(0.1,self.timer_callback)
        self._initial_q = data.qpos[:7].copy()
        print( print(f"Initial joint positions: {self.initial_q}"))
        theta = np.pi
        self.R_x = np.array([
            [1,0,0],
            [0,np.cos(theta),-np.sin(theta)],
            [0,np.sin(theta),-np.cos(theta)]
        ])
        self.new_q = self._initial_q
    
    def timer_callback(self):
        status = 0
        if self.x < 6:
            self.x += 0.001
            new_q = inverse_kinematics(self._initial_q,self.R_x,[0.5,0,0])
        self.current_q = new_q
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1','joint2','joint3','joint4','joint5','joint6','joint7']
        msg.position = self.current_q
        self.joint_pub.publish(msg)

def launch_setup(context):
    custom_node = CustomNode()
    robot_state_publish_node = Node(
        package = 'robot_state_publish',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ',
                    os.path.join(
                        get_package_share_directory('mercury_description'),
                        'urdf/mercury_a1/mercury_a1.urdf'
                    )
                ]),
                value_type=str
            )
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('mercury_a1'),
            'config/mercury_a1.rviz'
        )]

    )
    return [custom_node,robot_state_publish_node,rviz_node]

def generate_launch_description():
    res = []

    model_launch_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("mercury_description"),
            "urdf/mercury_a1/mercury_a1.urdf"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mercury_a1"),
            "config/mercury_a1.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    gui_launch_arg = DeclareLaunchArgument(
        "gui",
        default_value="true"
    )
    res.append(gui_launch_arg)
    
    # serial_port_arg = DeclareLaunchArgument(
    #     'port',
    #     default_value='/dev/ttyUSB0',
    #     description='Serial port to use'
    # )
    # res.append(serial_port_arg)
    # baud_rate_arg = DeclareLaunchArgument(
    #     'baud',
    #     default_value='115200',
    #     description='Baud rate to use'
    # )
    # res.append(baud_rate_arg)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
    res.append(robot_state_publisher_node)

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    res.append(joint_state_publisher_gui_node)

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)
    
    slider_control_node = Node(
        package="mercury_a1",
        executable="slider_control",
        # parameters=[
        #     {'port': LaunchConfiguration('port')},
        #     {'baud': LaunchConfiguration('baud')}
        # ],
        name="slider_control",
        output="screen"
    )
    res.append(slider_control_node)

    return LaunchDescription(res)
