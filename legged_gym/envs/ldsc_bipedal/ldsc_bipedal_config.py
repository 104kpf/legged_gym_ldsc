# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class LdscRoughCfg( LeggedRobotCfg ):
    class env( LeggedRobotCfg.env):
        num_envs = 1024  
        num_observations = 235
        num_actions = 12

    
    class terrain( LeggedRobotCfg.terrain):
        # measured_points_x = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5] # 1mx1m rectangle (without center line)
        # measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        mesh_type = 'plane' # "heightfield" # none, plane,

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, -0.02] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'L_Hip_Yaw': 0.0,
            'L_Hip_Roll': 0.0,
            'L_Hip_Pitch': -0.3383,
            'L_Knee_Pitch': 0.6350,
            'L_Ankle_Pitch': -0.3312,
            'L_Ankle_Roll': 0.0,

            'R_Hip_Yaw': 0.0,
            'R_Hip_Roll': 0.0,
            'R_Hip_Pitch': -0.3383,
            'R_Knee_Pitch': 0.6350,
            'R_Ankle_Pitch': -0.3312,
            'R_Ankle_Roll': 0.0
        }
        dof_pos_range = {
            'L_Hip_Yaw': [-0.2, 0.2],
            'L_Hip_Roll': [-0.2, 0.2],
            'L_Hip_Pitch': [-0.25, 0.25],
            'L_Knee_Pitch': [-0.25, 0.25],
            'L_Ankle_Pitch': [-0.25, 0.25],
            'L_Ankle_Roll': [-0.25, 0.25],
            'R_Hip_Yaw': [-0.2, 0.2],
            'R_Hip_Roll': [-0.2, 0.2],
            'R_Hip_Pitch': [-0.25, 0.25],
            'R_Knee_Pitch': [-0.25, 0.25],
            'R_Ankle_Pitch': [-0.25, 0.25],
            'R_Ankle_Roll': [-0.25, 0.25],
        }

        dof_vel_range = {
            'L_Hip_Yaw': [-0.05, 0.05],
            'L_Hip_Roll': [-0.05, 0.05],
            'L_Hip_Pitch': [-0.05, 0.05],
            'L_Knee_Pitch': [-0.1, 0.1],
            'L_Ankle_Pitch': [-0.1, 0.1],
            'L_Ankle_Roll': [-0.1, 0.1],
            'R_Hip_Yaw': [-0.05, 0.05],
            'R_Hip_Roll': [-0.05, 0.05],
            'R_Hip_Pitch': [-0.05, 0.05],
            'R_Knee_Pitch': [-0.1, 0.1],
            'R_Ankle_Pitch': [-0.1, 0.1],
            'R_Ankle_Roll': [-0.1, 0.1],
        }
    class commands:
        curriculum = False
        max_curriculum = 1.
        num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 10. # time before command are changed[s]
        heading_command = True # if true: compute ang vel command from heading error
        class ranges:
            lin_vel_x = [-0.0, 0.3] # min max [m/s]
            lin_vel_y = [-0.0, 0.0]   # min max [m/s]
            ang_vel_yaw = [-0, 0]    # min max [rad/s]
            heading = [0, 0]
    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        stiffness = {'Hip_Yaw': 30.0, 'Hip_Roll': 30.0,
             'Hip_Pitch': 10.0, 'Knee_Pitch': 10.0, 'Ankle_Pitch': 10.0,
             'Ankle_Roll': 10.0}  # [N*m/rad]
        damping = {'Hip_Yaw': 0.5, 'Hip_Roll': 1.5,
             'Hip_Pitch': 0.5, 'Knee_Pitch': 1.0, 'Ankle_Pitch': 0.5,
             'Ankle_Roll': 0.5}  # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.4
        # decimation: Number of control action updates @ sim DT per spolicy DT
        decimation = 12
        
    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/ldsc_bipedal/urdf/ldsc_bipedal.urdf'
        name = "ldsc_bipedal"
        end_effectors = ['l_foot_1', 'r_foot_1']
        foot_name = 'foot'
        terminate_after_contacts_on = ['base_link',
                                       'l_hip_yaw_1',
                                       'r_hip_yaw_1',
                                       'l_hip_pitch_1',
                                       'r_hip_pitch_1',
                                       'l_thigh_1',
                                       'r_thigh_1',
                                       'l_shank_1',
                                       'r_shank_1',
                                       'l_ankle_1',
                                       'r_ankle_1',
                                       "pelvis_link",
                                       "pelvis_joint"]
        flip_visual_attachments = False
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
  
    class rewards( LeggedRobotCfg.rewards ):
        base_height_target = 0.43
        soft_dof_pos_limit = 0.9
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 0.8
        only_positive_rewards = False
        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -100.
            tracking_lin_vel = 10.
            tracking_ang_vel = -10.
            torques = -1.e-6
            dof_acc = -2.e-7
            lin_vel_z = -0.9
            feet_air_time = 5.
            dof_pos_limits = -1.
            no_fly = 0.25
            dof_vel = -0.0
            ang_vel_xy = -0.0
            feet_contact_forces = -0.
            

class LdscRoughCfgPPO( LeggedRobotCfgPPO ):
    
    class runner( LeggedRobotCfgPPO.runner ):
        resume = True
        load_run = "Sep15_19-39-34_ldsc_bipedal"  # 你想继续训练的运行名称
        checkpoint = "20"  # 你想加载的检查点文件名
        experiment_name = "rough_ldsc_bipedal"  # 实验名称
        run_name = "continued_training"  # 添加这行
        max_iterations = 1000  # 设置最大迭代次数
    # class runner( LeggedRobotCfgPPO.runner ):
    #     num_steps_per_env = 24
    #     max_iterations = 20
    #     run_name = 'ldsc_bipedal'
    #     experiment_name = 'rough_ldsc_bipedal'
    #     save_interval = 50
    #     plot_input_gradients = False
    #     plot_parameter_gradients = False

    class algorithm( LeggedRobotCfgPPO.algorithm):
        value_loss_coef = 1.0
        use_clipped_value_loss = True
        clip_param = 0.2
        entropy_coef = 0.01
        num_learning_epochs = 5
        num_mini_batches = 4    # minibatch size = num_envs*nsteps/nminibatches
        learning_rate = 1.e-5
        schedule = 'adaptive'   # could be adaptive, fixed
        gamma = 0.99
        lam = 0.95
        desired_kl = 0.01
        max_grad_norm = 1.
        


