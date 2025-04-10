#!/usr/bin/env python3

import numpy as np
import math
from typing import Any, List, Tuple, Optional, Dict
from scipy.spatial.transform import Rotation
from genesis.utils.geom import quat_to_xyz, quat_to_R, xyz_to_quat, R_to_quat

from genesis_drones.controllers.base_controller import BaseController

# URDFから取得した定数
KF = 3.16e-10  # 推力係数 [N/RPM^2]
KM = 7.94e-12  # トルク係数 [Nm/RPM^2]
ARM = 0.0397   # アーム長 [m]
MASS = 0.027   # 質量 [kg]

# RPM関連の定数
BASE_RPM = 14468.429183500699
MIN_RPM = 0.9 * BASE_RPM
MAX_RPM = 1.5 * BASE_RPM
GRAVITY = 9.81

# ホバリング用のRPM計算
# F = m*g = 4*kf*rpm^2 => rpm = sqrt(m*g/(4*kf))
HOVER_RPM = np.sqrt(MASS * GRAVITY / (4 * KF))

class DSLPIDController(BaseController):
    """
    DSL PID Controller
    
    UTIASのDSLで開発されたPIDコントローラをベースにしたコントローラ。
    gym_pybullet_dronesのDSLPIDControlをもとに実装。
    """
    
    def __init__(self, drone: Any, dt: float, base_rpm: float = BASE_RPM):
        """
        DSLPIDコントローラの初期化
        
        Args:
            drone (Any): 制御対象のドローン
            dt (float): 時間ステップ
            base_rpm (float): 基準RPM
        """
        # 基準RPMを先に設定（resetメソッドで使用されるため）
        self.__base_rpm = base_rpm
        
        # ホバリング用のRPM
        self.hover_rpm = HOVER_RPM
        
        # 親クラスの初期化
        super().__init__(drone, dt)
        
        # 制御カウンタ
        self.control_counter = 0
        
        # 位置制御用PID制御のゲイン（DSLPIDControlから取得）
        self.P_COEFF_FOR = np.array([.1, .1, .1])
        self.I_COEFF_FOR = np.array([.15, .15, .15])
        self.D_COEFF_FOR = np.array([.2, .2, .5])
        
        # 速度制御用PID制御のゲイン
        self.P_COEFF_VEL = np.array([.07, .07, .07])
        self.I_COEFF_VEL = np.array([.05, .05, .05])
        self.D_COEFF_VEL = np.array([.002, .002, .002])
        
        # 姿勢制御用PID制御のゲイン
        self.P_COEFF_TOR = np.array([50000., 50000., 50000.])
        self.I_COEFF_TOR = np.array([500., 500., 500.])
        self.D_COEFF_TOR = np.array([100, 100, 100])
        
        # PWM関連の定数
        self.PWM2RPM_SCALE = 0.2685
        self.PWM2RPM_CONST = 4070.3
        self.MIN_PWM = 20000
        self.MAX_PWM = 65535
        
        # X型のミキサーマトリックス（元のDSLPIDControlと同じ）
        self.MIXER_MATRIX = np.array([
            [-.5, -.5, -1],
            [-.5,  .5,  1],
            [.5, .5, -1],
            [.5, -.5,  1]
        ])
        
        # 重力加速度
        self.GRAVITY = GRAVITY

        # ドローンの質量
        self.MASS = MASS
        
        # 最後の制御入力
        self.last_rpms = np.array([self.hover_rpm] * 4)
    
    def update(self, target, target_att=np.zeros(3), vel_target=False) -> np.ndarray:
        """
        目標位置または目標速度に基づいて制御入力を計算
        
        Args:
            target: 位置制御モードでは目標位置 (x, y, z)、速度制御モードでは目標速度 (vx, vy, vz)
            target_att: 位置制御モードでは目標姿勢 (roll, pitch, yaw)、速度制御モードでは目標角速度 (wx, wy, wz)
            vel_target: 速度制御モードかどうか
            
        Returns:
            np.ndarray: 4つのプロペラRPM
        """
        # ドローンの状態を取得
        curr_pos = self.get_drone_pos().cpu().numpy()
        curr_vel = self.get_drone_vel().cpu().numpy()
        curr_quat = self.get_drone_quat().cpu().numpy()
        curr_ang_vel = (quat_to_xyz(curr_quat) - self.last_rpy) / self.dt
        
        # 目標姿勢（デフォルトは0）
        target_pos = np.zeros(3)
        target_rpy = np.zeros(3)
        target_vel = np.zeros(3)
        target_ang_vel = np.zeros(3)
        
        # 速度制御モードかどうかを判断
        if vel_target:
            # 速度制御モード
            target_vel = np.array(target)
            target_ang_vel = np.array(target_att)
        else:
            # 位置制御モード
            target_pos = np.array(target)
            target_rpy = np.array(target_att)

        # DSLPIDControlのcomputeControlに相当する処理
        rpms, pos_e, _ = self._compute_control(
            self.dt,
            curr_pos,
            curr_quat,
            curr_vel,
            curr_ang_vel,
            target_pos,
            target_rpy,
            target_vel,
            target_ang_vel,  # 変数名を統一
            vel_target  # 速度制御モードかどうかを渡す
        )
        
        # RPMを制限
        rpms = self.clamp_rpms(rpms)
        
        # 最後の制御入力を保存
        self.last_rpms = rpms
        self.last_rpy = quat_to_xyz(curr_quat)
        
        return rpms
    
    def _compute_control(self,
                         control_timestep,
                         cur_pos,
                         cur_quat,
                         cur_vel,
                         cur_ang_vel,
                         target_pos,
                         target_rpy=np.zeros(3),
                         target_vel=np.zeros(3),
                         target_ang_vel=np.zeros(3),  # 変数名を統一
                         vel_target=False  # 速度制御モードかどうか
                        ):
        """
        PID制御入力（RPM）を計算
        
        DSLPIDControlのcomputeControlに相当するメソッド。
        
        Args:
            control_timestep (float): 制御の時間ステップ
            cur_pos (ndarray): 現在位置
            cur_quat (ndarray): 現在の姿勢（四元数）
            cur_vel (ndarray): 現在速度
            cur_ang_vel (ndarray): 現在の角速度
            target_pos (ndarray): 目標位置
            target_rpy (ndarray): 目標姿勢（ロール、ピッチ、ヨー）
            target_vel (ndarray): 目標速度
            target_ang_vel (ndarray): 目標角速度
            vel_target (bool): 速度制御モードかどうか
            
        Returns:
            ndarray: 4つのモーターに適用するRPM
            ndarray: 現在のXYZ位置誤差または速度誤差
            float: 現在のヨー誤差
        """
        self.control_counter += 1
        thrust, computed_target_rpy, pos_e = self._dsl_pid_position_control(
            control_timestep,
            cur_pos,
            cur_quat,
            cur_vel,
            target_pos,
            target_rpy,
            target_vel,
            vel_target  # 速度制御モードかどうかを渡す
        )
        rpm = self._dsl_pid_attitude_control(
            control_timestep,
            thrust,
            cur_quat,
            cur_ang_vel,
            computed_target_rpy,
            target_ang_vel  # 変数名を統一
        )
        
        # 現在のロール、ピッチ、ヨーを計算
        cur_rpy = quat_to_xyz(cur_quat)
        
        return rpm, pos_e, computed_target_rpy[2] - cur_rpy[2]
    
    def _dsl_pid_position_control(self,
                                 control_timestep,
                                 cur_pos,
                                 cur_quat,
                                 cur_vel,
                                 target_pos,
                                 target_rpy,
                                 target_vel,
                                 vel_target=False  # 速度制御モードかどうか
                                ):
        """
        DSLのPID位置制御または速度制御
        
        DSLPIDControlの_dslPIDPositionControlに相当するメソッド。
        速度制御モードの場合は、位置誤差の代わりに速度誤差を使用します。
        
        Args:
            control_timestep (float): 制御の時間ステップ
            cur_pos (ndarray): 現在位置
            cur_quat (ndarray): 現在の姿勢（四元数）
            cur_vel (ndarray): 現在速度
            target_pos (ndarray): 目標位置
            target_rpy (ndarray): 目標姿勢（ロール、ピッチ、ヨー）
            target_vel (ndarray): 目標速度
            vel_target (bool): 速度制御モードかどうか
            
        Returns:
            float: ドローンのz軸に沿った目標推力
            ndarray: 目標ロール、ピッチ、ヨーを含む配列
            float: 現在の位置誤差または速度誤差
        """
        # 現在の回転行列を計算
        cur_rotation = quat_to_R(cur_quat)
        
        # 位置と速度の誤差
        pos_e = target_pos - cur_pos
        vel_e = target_vel - cur_vel
        
        # 速度制御モードかどうかで処理を分岐
        if vel_target:
            # 速度制御モード
            # 速度誤差の積分を更新
            self.integral_vel_e = self.integral_vel_e + vel_e * control_timestep
            self.integral_vel_e = np.clip(self.integral_vel_e, -2., 2.)
            self.integral_vel_e[2] = np.clip(self.integral_vel_e[2], -0.15, .15)
            
            # 速度誤差の変化率（加速度誤差）を計算
            acc_e = (vel_e - self.last_vel_e) / control_timestep
            self.last_vel_e = vel_e.copy()
            
            # PID目標推力（速度制御モード）
            target_thrust = np.multiply(self.P_COEFF_VEL, vel_e) \
                          + np.multiply(self.I_COEFF_VEL, self.integral_vel_e) \
                          + np.multiply(self.D_COEFF_VEL, acc_e) \
                          + np.array([0, 0, self.GRAVITY*self.MASS])
            
            # 返り値の誤差は速度誤差
            error = vel_e
        else:
            # 位置制御モード
            # 積分誤差の更新
            self.integral_pos_e = self.integral_pos_e + pos_e * control_timestep
            self.integral_pos_e = np.clip(self.integral_pos_e, -2., 2.)
            self.integral_pos_e[2] = np.clip(self.integral_pos_e[2], -0.15, .15)
            
            # PID目標推力（位置制御モード）
            target_thrust = np.multiply(self.P_COEFF_FOR, pos_e) \
                          + np.multiply(self.I_COEFF_FOR, self.integral_pos_e) \
                          + np.multiply(self.D_COEFF_FOR, vel_e) \
                          + np.array([0, 0, self.GRAVITY*self.MASS])
            
            # 返り値の誤差は位置誤差
            error = pos_e
        
        # スカラー推力
        scalar_thrust = max(0., np.dot(target_thrust, cur_rotation[:,2]))
        # PWMへの変換を行う（元のDSLPIDControlと同様）
        thrust = (math.sqrt(scalar_thrust / (4*KF)) - self.PWM2RPM_CONST) / self.PWM2RPM_SCALE
        
        # 目標z軸
        target_z_ax = target_thrust / np.linalg.norm(target_thrust)
        
        # 目標x軸の計算
        target_x_c = np.array([math.cos(target_rpy[2]), math.sin(target_rpy[2]), 0])
        
        # 目標y軸の計算
        target_y_ax = np.cross(target_z_ax, target_x_c) / np.linalg.norm(np.cross(target_z_ax, target_x_c))
        
        # 目標x軸の計算
        target_x_ax = np.cross(target_y_ax, target_z_ax)
        
        # 目標回転行列
        target_rotation = (np.vstack([target_x_ax, target_y_ax, target_z_ax])).transpose()
        
        # 目標オイラー角
        target_euler = quat_to_xyz(R_to_quat(target_rotation))
        
        return thrust, target_euler, error
    
    def _dsl_pid_attitude_control(self,
                                 control_timestep,
                                 thrust,
                                 cur_quat,
                                 cur_ang_vel,
                                 target_euler,
                                 target_ang_vel  # 変数名を統一
                                ):
        """
        DSLのPID姿勢制御
        
        DSLPIDControlの_dslPIDAttitudeControlに相当するメソッド。
        
        Args:
            control_timestep (float): 制御の時間ステップ
            thrust (float): ドローンのz軸に沿った目標推力
            cur_quat (ndarray): 現在の姿勢（四元数）
            target_euler (ndarray): 計算された目標オイラー角
            target_ang_vel (ndarray): 目標角速度
            
        Returns:
            ndarray: 4つのモーターに適用するRPM
        """
        # 現在の回転行列を計算
        cur_rotation = quat_to_R(cur_quat)
        
        # 目標四元数を計算
        target_quat = xyz_to_quat(target_euler)
        
        # 目標回転行列を計算
        target_rotation = quat_to_R(target_quat)
        
        # 回転行列の誤差を計算
        rot_matrix_e = np.dot((target_rotation.transpose()), cur_rotation) - np.dot(cur_rotation.transpose(), target_rotation)
        rot_e = np.array([rot_matrix_e[2, 1], rot_matrix_e[0, 2], rot_matrix_e[1, 0]])
        
        # 角速度の誤差を計算
        ang_vel_e = target_ang_vel - cur_ang_vel

        # 積分誤差の更新
        self.integral_rpy_e = self.integral_rpy_e - rot_e * control_timestep
        self.integral_rpy_e = np.clip(self.integral_rpy_e, -1500., 1500.)
        self.integral_rpy_e[0:2] = np.clip(self.integral_rpy_e[0:2], -1., 1.)
        
        # PID目標トルク
        target_torques = - np.multiply(self.P_COEFF_TOR, rot_e) \
                        + np.multiply(self.D_COEFF_TOR, ang_vel_e) \
                        + np.multiply(self.I_COEFF_TOR, self.integral_rpy_e)
        
        # トルクの制限
        target_torques = np.clip(target_torques, -3200, 3200)

        
        # PWMを計算（元のDSLPIDControlと同様）
        pwm = thrust + np.dot(self.MIXER_MATRIX, target_torques)
        
        # PWM制限
        pwm = np.clip(pwm, self.MIN_PWM, self.MAX_PWM)
        
        # PWMからRPMに変換
        rpms = self.PWM2RPM_SCALE * pwm + self.PWM2RPM_CONST
        rpms = np.clip(rpms, MIN_RPM, MAX_RPM)
        return rpms
    
    def reset(self) -> None:
        """
        コントローラの状態をリセット
        """
        super().reset()
        
        # 最後のロール、ピッチ、ヨー
        self.last_rpy = np.zeros(3)
        
        # PID制御変数の初期化
        self.last_pos_e = np.zeros(3)
        self.integral_pos_e = np.zeros(3)
        self.last_rpy_e = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)
        
        # 速度制御用の変数
        self.last_vel_e = np.zeros(3)
        self.integral_vel_e = np.zeros(3)
        
        # 最後の制御入力
        self.last_rpms = np.array([self.hover_rpm] * 4)
    
    def hover(self) -> List[float]:
        """
        ドローンをホバリングさせる
        
        Returns:
            list: ホバリング用のRPM設定
        """
        hover_rpms = [self.hover_rpm] * 4
        self.set_propellers_rpm(hover_rpms)
        return hover_rpms
    
    def clamp_rpms(self, rpms: np.ndarray) -> np.ndarray:
        """
        RPMを最小値と最大値の間に制限する
        
        Args:
            rpms (np.ndarray): RPM値の配列
            
        Returns:
            np.ndarray: 制限されたRPM値の配列
        """
        # モジュールレベルの定数を使用
        return np.clip(rpms, MIN_RPM, MAX_RPM)
    
    def get_parameters(self) -> Dict[str, Any]:
        """
        コントローラのパラメータを取得
        
        Returns:
            dict: パラメータの辞書
        """
        params = super().get_parameters()
        params.update({
            "base_rpm": self.__base_rpm,
            "hover_rpm": self.hover_rpm,
            "P_COEFF_FOR": self.P_COEFF_FOR.tolist(),
            "I_COEFF_FOR": self.I_COEFF_FOR.tolist(),
            "D_COEFF_FOR": self.D_COEFF_FOR.tolist(),
            "P_COEFF_VEL": self.P_COEFF_VEL.tolist(),
            "I_COEFF_VEL": self.I_COEFF_VEL.tolist(),
            "D_COEFF_VEL": self.D_COEFF_VEL.tolist(),
            "P_COEFF_TOR": self.P_COEFF_TOR.tolist(),
            "I_COEFF_TOR": self.I_COEFF_TOR.tolist(),
            "D_COEFF_TOR": self.D_COEFF_TOR.tolist(),
            "gravity": self.GRAVITY,
            "kf": KF,
            "km": KM,
            "arm": ARM,
            "mass": MASS
        })
        return params
    
    def set_parameters(self, params: Dict[str, Any]) -> None:
        """
        コントローラのパラメータを設定
        
        Args:
            params (dict): パラメータの辞書
        """
        super().set_parameters(params)
        
        if "base_rpm" in params:
            self.__base_rpm = params["base_rpm"]
        
        if "P_COEFF_FOR" in params:
            self.P_COEFF_FOR = np.array(params["P_COEFF_FOR"])
        
        if "I_COEFF_FOR" in params:
            self.I_COEFF_FOR = np.array(params["I_COEFF_FOR"])
        
        if "D_COEFF_FOR" in params:
            self.D_COEFF_FOR = np.array(params["D_COEFF_FOR"])
            
        if "P_COEFF_VEL" in params:
            self.P_COEFF_VEL = np.array(params["P_COEFF_VEL"])
        
        if "I_COEFF_VEL" in params:
            self.I_COEFF_VEL = np.array(params["I_COEFF_VEL"])
        
        if "D_COEFF_VEL" in params:
            self.D_COEFF_VEL = np.array(params["D_COEFF_VEL"])
        
        if "P_COEFF_TOR" in params:
            self.P_COEFF_TOR = np.array(params["P_COEFF_TOR"])
        
        if "I_COEFF_TOR" in params:
            self.I_COEFF_TOR = np.array(params["I_COEFF_TOR"])
        
        if "D_COEFF_TOR" in params:
            self.D_COEFF_TOR = np.array(params["D_COEFF_TOR"])
        
        if "gravity" in params:
            self.GRAVITY = params["gravity"]
