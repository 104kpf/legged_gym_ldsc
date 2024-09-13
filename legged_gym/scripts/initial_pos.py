import os
from isaacgym import gymapi
from isaacgym import gymutil
import numpy as np

# 初始化 gym
gym = gymapi.acquire_gym()

# 创建仿真
sim_params = gymapi.SimParams()
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

# 加载机器人URDF
asset_root = "/home/ldsc/chuweihuang/src/legged_gym/resources/robots/ldsc_bipedal/urdf"
asset_file = "ldsc_bipedal.urdf"
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
asset_options.disable_gravity = True  # 禁用重力以保持初始姿态
asset_options.default_dof_drive_mode = gymapi.DOF_MODE_NONE  # 禁用默认驱动模式
robot_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

# 创建环境
env = gym.create_env(sim, gymapi.Vec3(-1, -1, 0), gymapi.Vec3(1, 1, 1), 1)

plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)  # 设置地面法线方向为z轴正方向
gym.add_ground(sim, plane_params)

# 创建机器人
pose = gymapi.Transform()
pose.p = gymapi.Vec3(0.0, 0.0, -0.02)  
robot_handle = gym.create_actor(env, robot_asset, pose, "robot", 0, 1)

# 设置初始关节角度
initial_joint_positions = {
    'L_Hip_Yaw': -0.0537,
    'L_Hip_Roll': -0.0322,
    'L_Hip_Pitch': -0.3383,
    'L_Knee_Pitch': 0.6350,
    'L_Ankle_Pitch': -0.2906,
    'L_Ankle_Roll': 0.0269,
    'R_Hip_Yaw': 0.0537,
    'R_Hip_Roll': 0.0322,
    'R_Hip_Pitch': -0.3383,
    'R_Knee_Pitch': 0.6350,
    'R_Ankle_Pitch': -0.2906,
    'R_Ankle_Roll': -0.0269
}

# 直接设置关节角度
dof_names = gym.get_actor_dof_names(env, robot_handle)
dof_states = gym.get_actor_dof_states(env, robot_handle, gymapi.STATE_ALL)
dof_props = gym.get_actor_dof_properties(env, robot_handle)

for i, name in enumerate(dof_names):
    if name in initial_joint_positions:
        dof_states['pos'][i] = initial_joint_positions[name]
        dof_states['vel'][i] = 0  # 设置初始速度为0
        dof_props['driveMode'][i] = gymapi.DOF_MODE_NONE  # 禁用驱动模式

gym.set_actor_dof_states(env, robot_handle, dof_states, gymapi.STATE_ALL)
gym.set_actor_dof_properties(env, robot_handle, dof_props)

# 创建查看器
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    print("*** Failed to create viewer")
    quit()

# 设置相机位置
cam_pos = gymapi.Vec3(4, 3, 2)
cam_target = gymapi.Vec3(0, 0, 1)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

# 主循环
while not gym.query_viewer_has_closed(viewer):
    # 步进仿真
    gym.simulate(sim)
    gym.fetch_results(sim, True)
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # 打印当前关节角度
    dof_states = gym.get_actor_dof_states(env, robot_handle, gymapi.STATE_ALL)
    for i, name in enumerate(dof_names):
        if name in initial_joint_positions:
            print(f"{name}: Current: {dof_states['pos'][i]:.4f}, Target: {initial_joint_positions[name]:.4f}, Velocity: {dof_states['vel'][i]:.4f}")
    print("\n")

    gym.sync_frame_time(sim)

# 清理
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)