#! /usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import rospy
import copy

# Intera SDK Imports
import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION

# ROS Message Imports
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class IKJoystickControl(object):
    
    def __init__(self, limb="right", joystick_type="xbox"):
        # 初始化 ROS 节点
        rospy.init_node("rsdk_ik_joystick_control")
        
        self.limb_name = limb
        self.limb = intera_interface.Limb(limb)
        
        # 初始化夹爪
        try:
            self.gripper = intera_interface.Gripper(limb + '_gripper')
        except:
            self.gripper = None
            rospy.logwarn("未检测到夹爪")

        # 初始化 IK 服务代理
        self.ik_service_name = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        rospy.loginfo("等待 IK 服务: %s" % self.ik_service_name)
        rospy.wait_for_service(self.ik_service_name)
        self.ik_svc = rospy.ServiceProxy(self.ik_service_name, SolvePositionIK)
        rospy.loginfo("IK 服务已连接！")

        # 初始化手柄
        if joystick_type == 'xbox':
            self.joystick = intera_external_devices.joystick.XboxController()
        elif joystick_type == 'ps3':
            self.joystick = intera_external_devices.joystick.PS3Controller()
        elif joystick_type == 'logitech':
            self.joystick = intera_external_devices.joystick.LogitechController()
        else:
            raise ValueError("不支持的手柄类型")

        # 控制参数
        self.rate = rospy.Rate(10)  # 10Hz，IK解算比较重，不要太快
        self.linear_speed = 0.05    # 移动步长 (米/每周期)，根据需要调整灵敏度
        self.deadzone = 0.1         # 摇杆死区，防止漂移

    def get_ik_solution(self, target_pose, current_joints):
        """
        核心函数：请求 Advanced IK 解算
        """
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        
        # 1. 填充目标 Pose
        pose_stamped = PoseStamped(header=hdr, pose=target_pose)
        ikreq.pose_stamp.append(pose_stamped)
        ikreq.tip_names.append(self.limb_name + '_hand')
        
        # 2. 【重要】设置 Seed (种子) 为当前关节角度
        # 这确保了解算器寻找离当前状态最近的解，保证运动平滑
        ikreq.seed_mode = ikreq.SEED_USER
        seed = JointState()
        seed.name = self.limb.joint_names()
        seed.position = [current_joints[n] for n in seed.name]
        ikreq.seed_angles.append(seed)
        
        # 发送请求
        try:
            resp = self.ik_svc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("IK 服务调用失败: %s" % (e,))
            return None

        # 检查结果
        if (resp.result_type[0] > 0):
            # 格式化结果为字典 {name: angle}
            return dict(zip(resp.joints[0].name, resp.joints[0].position))
        else:
            return None

    def run(self):
        # 启用机器人
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        rospy.loginfo("正在启用机器人...")
        rs.enable()
        
        rospy.loginfo("控制开始！按 Ctrl-C 退出。")
        rospy.loginfo("映射说明：")
        rospy.loginfo("  左摇杆 (前后左右): X/Y 平面移动")
        rospy.loginfo("  右摇杆 (上下): Z 轴垂直升降")
        rospy.loginfo("  A键 (Xbox) / X键 (PS): 闭合夹爪")
        rospy.loginfo("  B键 (Xbox) / O键 (PS): 松开夹爪")

        while not rospy.is_shutdown():
            # 1. 获取当前末端状态
            # endpoint_pose 返回字典: {'position': Point(x,y,z), 'orientation': Quaternion(x,y,z,w)}
            current_pose_dict = self.limb.endpoint_pose()
            curr_pos = current_pose_dict['position']
            curr_ori = current_pose_dict['orientation']
            
            # 2. 读取手柄输入并处理死区
            # 左摇杆控制 X (前后) 和 Y (左右)
            raw_x = -self.joystick.stick_value('leftStickVert') # 注意：很多手柄上前推是负值，视情况取反
            raw_y = -self.joystick.stick_value('leftStickHorz')
            # 右摇杆控制 Z (上下)
            raw_z = -self.joystick.stick_value('rightStickVert')

            # 死区过滤
            val_x = raw_x if abs(raw_x) > self.deadzone else 0.0
            val_y = raw_y if abs(raw_y) > self.deadzone else 0.0
            val_z = raw_z if abs(raw_z) > self.deadzone else 0.0

            # 3. 处理夹爪
            if self.gripper:
                if self.joystick.button_down('btnDown'): # Xbox A键
                    self.gripper.close()
                elif self.joystick.button_down('btnRight'): # Xbox B键
                    self.gripper.open()

            # 4. 如果没有移动指令，跳过 IK 运算，节省资源
            if val_x == 0 and val_y == 0 and val_z == 0:
                self.rate.sleep()
                continue

            # 5. 计算新的目标位置
            # 注意：这里是在 Base 坐标系下移动
            target_pos = Point(
                x = curr_pos.x + (val_x * self.linear_speed),
                y = curr_pos.y + (val_y * self.linear_speed),
                z = curr_pos.z + (val_z * self.linear_speed)
            )
            
            # 保持 Orientation 不变 (锁定姿态)
            target_pose = Pose(position=target_pos, orientation=curr_ori)
            
            # 6. 获取当前关节角度作为 Seed
            current_joints = self.limb.joint_angles()
            
            # 7. 调用 IK 解算
            joint_solution = self.get_ik_solution(target_pose, current_joints)
            
            if joint_solution:
                # 8. 执行运动 (使用位置控制模式)
                self.limb.set_joint_positions(joint_solution)
            else:
                # IK 无解（通常是因为到了工作空间边缘，或者奇异点）
                rospy.logwarn_throttle(1.0, "IK 无法到达目标位置 (超出范围或奇异点)")

            self.rate.sleep()

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description="Sawyer Joystick Control via IK")
    parser.add_argument(
        '-j', '--joystick', required=True,
        choices=['xbox', 'logitech', 'ps3'],
        help='specify the type of joystick to use'
    )
    parser.add_argument(
        "-l", "--limb", dest="limb", default="right",
        choices=["right"],
        help="Limb to control"
    )
    
    # 过滤掉 ROS 自动添加的参数
    args = parser.parse_args(rospy.myargv()[1:])
    
    try:
        controller = IKJoystickControl(limb=args.limb, joystick_type=args.joystick)
        controller.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()