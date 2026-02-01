from queue import Queue
import os
import sys

import numpy as np

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
from arx5_interface import (
    Arx5CartesianController,
    ControllerConfig,
    ControllerConfigFactory,
    EEFState,
    Gain,
    LogLevel,
    RobotConfigFactory,
)
from peripherals.spacemouse_shared_memory import Spacemouse
from multiprocessing.managers import SharedMemoryManager

import time
import click


def start_teleop_recording(controller: Arx5CartesianController):
    """启动 Spacemouse 遥操作控制"""
    
    ori_speed = 1.5  # 姿态速度增益
    pos_speed = 0.8  # 位置速度增益
    gripper_speed = 0.04  # 夹爪速度增益
    # 对于早期有线版 Spacemouse，松开后读数可能不为零
    # 如果使用无线 3Dconnexion Spacemouse，可以将 deadzone_threshold 设为 0.0 以提高灵敏度
    deadzone_threshold = 0.1  # 死区阈值
    target_pose_6d = controller.get_home_pose()  # 目标末端6D位姿

    target_gripper_pos = 0.0  # 目标夹爪位置

    window_size = 3  # 滑动平均窗口大小
    cmd_dt = 0.01  # 控制周期 (10ms)
    preview_time = 0.05  # 轨迹命令的预览时间（提前量）

    UPDATE_TRAJ = True
    # False: 仅发送单点位置命令
    # True: 每个周期发送轨迹命令，包含速度信息

    pose_x_min = target_pose_6d[0]
    spacemouse_queue = Queue(window_size)
    robot_config = controller.get_robot_config()

    avg_error = np.zeros(6)
    avg_cnt = 0
    prev_eef_cmd = EEFState()
    eef_cmd = EEFState()

    with SharedMemoryManager() as shm_manager:
        with Spacemouse(
            shm_manager=shm_manager, deadzone=deadzone_threshold, max_value=500
        ) as sm:

            def get_filtered_spacemouse_output(sm: Spacemouse):
                """获取过滤后的 Spacemouse 输出（去除死区 + 滑动平均）"""
                state = sm.get_motion_state_transformed()
                # 去除死区并归一化输出
                positive_idx = state >= deadzone_threshold
                negative_idx = state <= -deadzone_threshold
                state[positive_idx] = (state[positive_idx] - deadzone_threshold) / (
                    1 - deadzone_threshold
                )
                state[negative_idx] = (state[negative_idx] + deadzone_threshold) / (
                    1 - deadzone_threshold
                )

                # 滑动窗口平均滤波
                if (
                    spacemouse_queue.maxsize > 0
                    and spacemouse_queue._qsize() == spacemouse_queue.maxsize
                ):
                    spacemouse_queue._get()
                spacemouse_queue.put_nowait(state)
                return np.mean(np.array(list(spacemouse_queue.queue)), axis=0)

            print("Teleop tracking ready. Waiting for spacemouse movement to start.")

            # 等待 Spacemouse 有输入后才开始
            while True:
                button_left = sm.is_button_pressed(0)
                button_right = sm.is_button_pressed(1)
                state = get_filtered_spacemouse_output(sm)
                if state.any() or button_left or button_right:
                    print(f"Start tracking!")
                    break
                eef_cmd = controller.get_eef_cmd()
                prev_eef_cmd = eef_cmd
            start_time = time.monotonic()
            loop_cnt = 0
            # 主控制循环
            while True:

                print(
                    f"Time elapsed: {time.monotonic() - start_time:.03f}s",
                    end="\r",
                )
                # Spacemouse 状态格式: (x y z roll pitch yaw)
                state = get_filtered_spacemouse_output(sm)
                button_left = sm.is_button_pressed(0)  # 左键
                button_right = sm.is_button_pressed(1)  # 右键
                
                # 双键同时按下: 复位到 home 位置
                if button_left and button_right:
                    print(f"Avg 6D pose error: {avg_error / avg_cnt}")
                    # 带速度的轨迹命令误差:    [ 0.0004  0.0002 -0.0016  0.0002  0.0032  0.0005]
                    # 不带速度的单点命令误差:  [-0.0002 -0.0006 -0.0026  0.0027  0.0042 -0.0017]
                    # 不带速度的轨迹命令误差:  [ 0.0005  0.0008 -0.005  -0.0024  0.0073 -0.0001]

                    controller.reset_to_home()
                    config = controller.get_robot_config()
                    target_pose_6d = controller.get_home_pose()
                    target_gripper_pos = 0.0
                    loop_cnt = 0
                    start_time = time.monotonic()

                    continue
                elif button_left and not button_right:
                    gripper_cmd = 1  # 左键: 打开夹爪
                elif button_right and not button_left:
                    gripper_cmd = -1  # 右键: 关闭夹爪
                else:
                    gripper_cmd = 0  # 无按键: 保持
                # 根据 Spacemouse 输入更新目标位姿
                target_pose_6d[:3] += state[:3] * pos_speed * cmd_dt  # 更新位置 (xyz)
                target_pose_6d[3:] += state[3:] * ori_speed * cmd_dt  # 更新姿态 (rpy)
                target_gripper_pos += gripper_cmd * gripper_speed * cmd_dt  # 更新夹爪
                
                # 限制夹爪位置范围
                if target_gripper_pos >= robot_config.gripper_width:
                    target_gripper_pos = robot_config.gripper_width
                elif target_gripper_pos <= 0:
                    target_gripper_pos = 0
                    
                loop_cnt += 1
                # 等待下一个控制周期
                while time.monotonic() < start_time + loop_cnt * cmd_dt:
                    pass
                    
                current_timestamp = controller.get_timestamp()
                prev_eef_cmd = eef_cmd
                # 限制 x 方向最小值（可选）
                # if target_pose_6d[0] < pose_x_min:
                #     target_pose_6d[0] = pose_x_min
                
                # 构建末端命令
                eef_cmd.pose_6d()[:] = target_pose_6d
                eef_cmd.gripper_pos = target_gripper_pos
                eef_cmd.timestamp = current_timestamp + preview_time

                if UPDATE_TRAJ:
                    # 发送轨迹命令（自动计算速度）
                    controller.set_eef_traj([eef_cmd])
                else:
                    # 发送单点命令（仅位置控制）
                    controller.set_eef_cmd(eef_cmd)

                # 计算跟踪误差
                output_eef_cmd = controller.get_eef_cmd()
                eef_state = controller.get_eef_state()
                avg_error += output_eef_cmd.pose_6d() - eef_state.pose_6d()
                avg_cnt += 1

                print(f"6DPose Error: {output_eef_cmd.pose_6d() - eef_state.pose_6d()}")


@click.command()
@click.argument("model")  # ARX 机械臂型号: X5, L5 等
@click.argument("interface")  # CAN 总线名称 (can0 等)
def main(model: str, interface: str):
    
    robot_config = RobotConfigFactory.get_instance().get_config(model)
    controller_config = ControllerConfigFactory.get_instance().get_config(
        "cartesian_controller", robot_config.joint_dof
    )
    # controller_config.interpolation_method = "cubic"  # 可选: 三次插值
    controller_config.default_kp = controller_config.default_kp
    
    # 创建笛卡尔控制器并复位到 home 位置
    controller = Arx5CartesianController(robot_config, controller_config, interface)
    controller.reset_to_home()

    robot_config = controller.get_robot_config()
    gain = Gain(robot_config.joint_dof)
    controller.set_log_level(LogLevel.DEBUG)
    np.set_printoptions(precision=4, suppress=True)
    
    try:
        start_teleop_recording(controller)
    except KeyboardInterrupt:
        print(f"Teleop recording is terminated. Resetting to home.")
        controller.reset_to_home()
        controller.set_to_damping()


if __name__ == "__main__":
    main()
