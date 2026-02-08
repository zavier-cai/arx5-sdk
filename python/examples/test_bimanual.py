"""
双机械臂关节空间同步运动示例

用法:
  python test_bimanual.py <model0> <interface0> <model1> <interface1>
  例如: python test_bimanual.py X5 can0 L5 can1

两台机械臂从 home 位置出发，用 easeInOutQuad 缓动曲线同步运动到目标关节角，
再同步回到 home；关节命令与夹爪命令共用同一套缓动系数。
"""
import time
import numpy as np
import os
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5
import click


def easeInOutQuad(t):
    """二次方 ease-in-out 缓动 t∈[0,1] 返回 [0,1] 先慢后快再慢"""
    t *= 2
    if t < 1:
        return t * t / 2
    else:
        t -= 1
        return -(t * (t - 2) - 1) / 2


@click.command()
@click.argument("model0")  # 左臂型号: X5 或 L5
@click.argument("interface0")  # 左臂 CAN 接口名 如 can0
@click.argument("model1")  # 右臂型号
@click.argument("interface1")  # 右臂 CAN 接口名
def main(model0: str, interface0: str, model1: str, interface1: str):
    """双臂关节控制：先同步到目标关节角+夹爪开合 再同步回 home"""
    np.set_printoptions(precision=3, suppress=True)
    assert interface0 != interface1, "两台机械臂必须使用不同 CAN 接口"

    # 创建两台关节空间控制器
    arx5_0 = arx5.Arx5JointController(model0, interface0)
    arx5_1 = arx5.Arx5JointController(model1, interface1)
    robot_config = arx5_0.get_robot_config()
    controller_config = arx5_0.get_controller_config()

    # 先回 home
    arx5_0.reset_to_home()
    arx5_1.reset_to_home()

    # 目标关节角 (弧度) 前 4 个关节参与运动 后 2 个保持
    target_joint_poses = np.array([1.0, 2.0, 2.0, 1.5, 1.5, -1.57])
    step_num = 1500  # 约 3s (取决于 controller_dt)

    # 第一段：从 home 缓动到目标
    for i in range(step_num):
        cmd = arx5.JointState(robot_config.joint_dof)
        cmd.pos()[0:4] = easeInOutQuad(float(i) / step_num) * target_joint_poses[0:4]
        cmd.gripper_pos = easeInOutQuad((i / (step_num - 1))) * 0.08  # 夹爪开合 0~0.08
        arx5_0.set_joint_cmd(cmd)
        arx5_1.set_joint_cmd(cmd)
        joint_state = arx5_0.get_joint_state()
        joint_state = arx5_1.get_joint_state()
        arm_dof_pos = joint_state.pos().copy()
        arm_dof_vel = joint_state.vel().copy()
        time.sleep(controller_config.controller_dt)

    # 第二段：从目标缓动回 home
    for i in range(step_num):
        cmd = arx5.JointState(robot_config.joint_dof)
        cmd.pos()[0:4] = (
            easeInOutQuad((1 - float(i) / step_num)) * target_joint_poses[0:4]
        )
        cmd.gripper_pos = easeInOutQuad((1 - i / (step_num - 1))) * 0.08
        arx5_0.set_joint_cmd(cmd)
        arx5_1.set_joint_cmd(cmd)
        time.sleep(controller_config.controller_dt)
        joint_state = arx5_0.get_joint_state()
        joint_state = arx5_1.get_joint_state()


main()
