import os
from isaacgym import gymapi
from isaacgym import gymutil
import numpy as np
import time

# 初始化 gym
gym = gymapi.acquire_gym()

# 创建仿真
sim_params = gymapi.SimParams()
sim_params.up_axis = gymapi.UP_AXIS_Z

sim_params.dt = 1.0 / 60.0  # 设置时间步长
sim_params.substeps = 2  # 增加子步骤数

sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

# 加载机器人URDF
asset_root = "/home/ldsc/chuweihuang/src/legged_gym/resources/robots/ldsc_bipedal/urdf"
asset_file = "ldsc_bipedal.urdf"
asset_options = gymapi.AssetOptions()

#asset_options.disable_gravity = True  # 暂时禁用重力
asset_options.default_dof_drive_mode = int(gymapi.DOF_MODE_POS)
print("loading...")
robot_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)


# 创建环境
print("create env...")
env = gym.create_env(sim, gymapi.Vec3(-1, -1, 0), gymapi.Vec3(1, 1, 1), 1)

# 添加地面平面
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)
plane_params.static_friction = 0.5  # 增加静摩擦力
plane_params.dynamic_friction = 0.4 # 增加动摩擦力
plane_params.restitution = 1.0  # 设置恢复系数
gym.add_ground(sim, plane_params)

# 创建机器人

pose = gymapi.Transform()
pose.p = gymapi.Vec3(0.0, 0.0, 0.5)  
robot_handle = gym.create_actor(env, robot_asset, pose, "robot", 0, 1)

# 设置初始关节角度和控制参数
initial_joint_positions = {
    'L_Hip_Yaw': {'pos': -0.05, 'stiffness': 100.0, 'damping': 3.0},
    'L_Hip_Roll': {'pos': -0.0, 'stiffness': 100.0, 'damping': 3.0},
    'L_Hip_Pitch': {'pos': -0.37, 'stiffness': 10.0, 'damping': 6.0},
    'L_Knee_Pitch': {'pos': 0.74, 'stiffness': 30.0, 'damping': 6.0},
    'L_Ankle_Pitch': {'pos': -0.37, 'stiffness': 20.0, 'damping': 5.0},
    'L_Ankle_Roll': {'pos': 0.0, 'stiffness': 140.0, 'damping': 1.0},
    'R_Hip_Yaw': {'pos': 0.05, 'stiffness': 100.0, 'damping': 3.0},
    'R_Hip_Roll': {'pos': 0.0, 'stiffness': 100.0, 'damping': 3.0},
    'R_Hip_Pitch': {'pos': -0.37, 'stiffness': 10.0, 'damping': 6.0},
    'R_Knee_Pitch': {'pos': 0.74, 'stiffness': 30.0, 'damping': 6.0},
    'R_Ankle_Pitch': {'pos': -0.37, 'stiffness': 20.0, 'damping': 5.0},
    'R_Ankle_Roll': {'pos': 0.0, 'stiffness': 140.0, 'damping': 1.0}
}

# 设置关节属性和初始状态
print("set dof...")
dof_names = gym.get_actor_dof_names(env, robot_handle)
dof_props = gym.get_actor_dof_properties(env, robot_handle)
dof_states = gym.get_actor_dof_states(env, robot_handle, gymapi.STATE_ALL)

# for i, name in enumerate(dof_names):
#     if name in initial_joint_positions:
#         joint_data = initial_joint_positions[name]
#         dof_props['driveMode'][i] = int(gymapi.DOF_MODE_POS)
#         dof_props['stiffness'][i] = joint_data['stiffness']
#         dof_props['damping'][i] = joint_data['damping']
#         dof_states['pos'][i] = joint_data['pos']
#         dof_states['vel'][i] = 0

# gym.set_actor_dof_properties(env, robot_handle, dof_props)
# gym.set_actor_dof_states(env, robot_handle, dof_states, gymapi.STATE_ALL)

# 创建查看器

viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    print("*** false")
    quit()

# 设置相机位置
cam_pos = gymapi.Vec3(4, 3, 2)
cam_target = gymapi.Vec3(0, 0, 1)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

print("init done, start main loop")

# 主循环
step = 0
while not gym.query_viewer_has_closed(viewer):
    # 步进仿真
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # 更新关节目标位置
    for i, name in enumerate(dof_names):
        if name in initial_joint_positions:
            gym.set_dof_target_position(env, i, initial_joint_positions[name]['pos'])

    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # 打印当前关节角度
    dof_states = gym.get_actor_dof_states(env, robot_handle, gymapi.STATE_ALL)
    for i, name in enumerate(dof_names):
        if name in initial_joint_positions:
            print(f"{name}: current: {dof_states['pos'][i]:.4f}, goal: {initial_joint_positions[name]['pos']:.4f}, speed: {dof_states['vel'][i]:.4f}")
    print("\n")

    gym.sync_frame_time(sim)
    
    step += 1
    if step > 10:  # 在100步后启用重力和解除固定
        print("enable gravity and unfix")
        asset_options.fix_base_link = False
        gym.set_asset_rigid_shape_properties(robot_asset, [gymapi.RigidShapeProperties()])
        
        # 启用重力
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
        gym.set_sim_params(sim, sim_params)

        gravity_enabled = True

    time.sleep(0.01)  # 添加小延迟以便观察

# 清理

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
