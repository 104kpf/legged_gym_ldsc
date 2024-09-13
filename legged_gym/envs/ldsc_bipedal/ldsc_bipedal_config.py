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
        num_envs = 512  
        num_observations = 169
        num_actions = 12

    
    class terrain( LeggedRobotCfg.terrain):
        measured_points_x = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5] # 1mx1m rectangle (without center line)
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, -0.05] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'L_Hip_Yaw': 0.0537,
            'L_Hip_Roll': 0.0322,
            'L_Hip_Pitch': -0.3383,
            'L_Knee_Pitch': 0.6350,
            'L_Ankle_Pitch': -0.3312,
            'L_Ankle_Roll': 0.0,

            'R_Hip_Yaw': 0.0537,
            'R_Hip_Roll': 0.0322,
            'R_Hip_Pitch': -0.3383,
            'R_Knee_Pitch': 0.6350,
            'R_Ankle_Pitch': -0.3312,
            'R_Ankle_Roll': 0.0
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
       
        stiffness = {'Hip_Yaw': 50.0, 'Hip_Roll': 100.0,
             'Hip_Pitch': 100.0, 'Knee_Pitch': 10.0, 'Ankle_Pitch': 50.0,
             'Ankle_Roll': 50.0}  # [N*m/rad]
        damping = {'Hip_Yaw': 0.5, 'Hip_Roll': 1.5,
             'Hip_Pitch': 0.5, 'Knee_Pitch': 1.0, 'Ankle_Pitch': 0.5,
             'Ankle_Roll': 0.5}  # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.4
        # decimation: Number of control action updates @ sim DT per spolicy DT
        decimation = 6
        
    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/ldsc_bipedal/urdf/ldsc_bipedal.urdf'
        name = "ldsc_bipedal"
        foot_name = 'foot'
        terminate_after_contacts_on = ['base_link']
        flip_visual_attachments = False
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.95
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 0.9
        max_contact_force = 100.
        only_positive_rewards = False
        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -100.
            tracking_ang_vel = 0.5
            torques = -1.e-6
            dof_acc = -2.e-7
            lin_vel_z = -0.5
            feet_air_time = 5.
            dof_pos_limits = -1.
            no_fly = 0.25
            dof_vel = -0.0
            ang_vel_xy = -0.0
            feet_contact_forces = -0.
            

class LdscRoughCfgPPO( LeggedRobotCfgPPO ):
    
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_ldsc_bipedal'

    class algorithm( LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01



  
