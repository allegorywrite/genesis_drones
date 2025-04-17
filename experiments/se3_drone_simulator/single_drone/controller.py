"""
ドローン制御モジュール
"""
import numpy as np
import cvxopt
import sys
import os

# 親ディレクトリをパスに追加
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.cbf_se3 import compute_position_tracking_control, compute_single_drone_cbf_coefficients, compute_single_point_cbf_coefficients, compute_single_prob_cbf_coefficients, compute_position_tracking_acceleration_control, compute_hocbf_single_point_coefficients
from utils.solver import compute_objective_coefficients, compute_dynamic_objective_coefficients, solve_dynamic_qp
from utils.drone import DynamicDrone


def create_dynamic_drone_input_func(drone, feature_points, target_trajectory, use_cbf=False, cbf_method='point', hocbf_visualizer=None, dt=0.01):
    """
    2次系ドローンの入力関数を作成
    
    Parameters:
    -----------
    drone : DynamicDrone
        2次系ドローン
    feature_points : list of FeaturePoint
        特徴点のリスト
    target_trajectory : array_like, shape (n, 3) or array_like, shape (3,)
        目標軌道または固定目標位置
    use_cbf : bool, optional
        CBF制約を使用するかどうか（デフォルトはFalse）
    cbf_method : str, optional
        CBF制約の方法: 'pcl'（複数特徴点に対するCBF）または
        'point'（単一特徴点に対するCBF）（デフォルトは'point'）
    hocbf_visualizer : HOCBFVisualizer, optional
        HOCBFの可視化クラス（デフォルトはNone）
    
    Returns:
    --------
    input_func : callable
        ドローンの入力関数
    """
    # 目標軌道かどうかを判定
    is_trajectory = isinstance(target_trajectory, np.ndarray) and len(target_trajectory.shape) == 2
    
    # 入力制約の設定
    if drone.dynamics_model == 'holonomic_dynamics':
        # ホロノミック系の場合
        # u_min = np.array([-100.0, -100.0, -100.0, -100.0, -100.0, -100.0])  # [f_min, tau_min]
        # u_max = np.array([100.0, 100.0, 100.0, 100.0, 100.0, 100.0])  # [f_max, tau_max]
        u_min = np.array([-1000.0, -1000.0, -1000.0, -5000.0, -5000.0, -5000.0])  # [f_min, tau_min]
        u_max = np.array([1000.0, 1000.0, 1000.0, 5000.0, 5000.0, 5000.0])  # [f_max, tau_max]
    else:
        # 非ホロノミック系の場合
        u_min = np.array([-1000.0, -5000.0, -5000.0, -5000.0])  # [f_min, tau_min]
        u_max = np.array([1000.0, 5000.0, 5000.0, 5000.0])  # [f_max, tau_max]
    
    def drone_input_func(drone, frame):
        # 現在のフレームに対応する目標位置を取得
        if is_trajectory:
            current_target = target_trajectory[frame % len(target_trajectory)]
        else:
            current_target = target_trajectory
        
        # 返却値の初期化
        gamma_val = 0.0
        constraint_margin = 0.0
        ax_value_float = 0.0
        h_val = 0.0
        h_dot_val = 0.0
        h_ddot_val = 0.0
        
        # CBF制約を使用する場合
        if use_cbf:
            # HOCBFのゲイン
            gamma0 = 1.0
            gamma1 = 1.0
            Q1 = np.eye(3)
            Q2 = np.eye(6)
            Q2[0:3, 0:3] = np.eye(3)*0.0000001 # 推力
            Q2[3:6, 3:6] = np.eye(3)*0.000005 # トルク
            
            # QP問題を解く
            u, constraint_value, B_val, B_dot_val, B_ddot_val = solve_dynamic_qp(
                drone, feature_points, current_target, use_cbf=True, cbf_method=cbf_method,
                gamma0=gamma0, gamma1=gamma1, Q1=Q1, Q2=Q2,
                u_min=u_min, u_max=u_max
            )
            # print(f"制御入力: {u}")
            if constraint_value is not None:
                constraint_margin = constraint_value
            
            if hocbf_visualizer is not None and B_val is not None and B_dot_val is not None and B_ddot_val is not None:
                _, B_dot_prev, _ = hocbf_visualizer.get_previous_values()
                ref1_value = (B_dot_val - B_dot_prev) / dt
                ref2_value = B_ddot_val
                # ref2_value = B_ddot_minus_val
                hocbf_visualizer.update(B_val, B_dot_val, B_ddot_val, gamma0, gamma1)
                return u, ref1_value, constraint_margin, ref2_value, B_val, B_dot_val, B_ddot_val
            
            # HOCBFの可視化がない場合、またはB_val, B_dot_val, B_ddot_valがNoneの場合
            return u, 0.0, constraint_margin, 0.0, 0.0, 0.0, 0.0
        else:
            # 制約なしの場合も入力制約付きQPを解く
            u, _, _, _, _ = solve_dynamic_qp(
                drone, feature_points, current_target, use_cbf=False, h=dt,
                u_min=u_min, u_max=u_max
            )
            return u, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    
    return drone_input_func


def create_drone_input_func(drone, feature_points, target_trajectory, use_cbf=False, cbf_type='no-decomp', cbf_method='pcl', dynamics_model='kinematics', hocbf_visualizer=None, dt=0.01):
    """
    ドローンの入力関数を作成
    
    Parameters:
    -----------
    drone : Drone or DynamicDrone
        ドローン
    feature_points : list of FeaturePoint
        特徴点のリスト
    target_trajectory : array_like, shape (n, 3) or array_like, shape (3,)
        目標軌道または固定目標位置
    use_cbf : bool, optional
        CBF制約を使用するかどうか（デフォルトはFalse）
    cbf_type : str, optional
        CBF制約のタイプ: 'no-decomp'（速度入力及び角速度入力について分解しない場合）または
        'decomp'（分解する場合）（デフォルトは'no-decomp'）
    cbf_method : str, optional
        CBF制約の方法: 'pcl'（複数特徴点に対するCBF）または
        'point'（単一特徴点に対するCBF）（デフォルトは'pcl'）
    dynamics_model : str, optional
        動力学モデル: 'kinematics'（1次系）または'dynamics'（2次系）または
        'holonomic_dynamics'（ホロノミック2次系）（デフォルトは'kinematics'）
    hocbf_visualizer : HOCBFVisualizer, optional
        HOCBFの可視化クラス（デフォルトはNone）
    
    Returns:
    --------
    input_func : callable
        ドローンの入力関数
    """
    # 動力学モデルに応じて適切な入力関数を返す
    if (dynamics_model == 'dynamics' or dynamics_model == 'holonomic_dynamics') and isinstance(drone, DynamicDrone):
        # ドローンの動力学モデルを設定
        drone.dynamics_model = dynamics_model
        return create_dynamic_drone_input_func(drone, feature_points, target_trajectory, use_cbf, cbf_method, hocbf_visualizer, dt=dt)
    # 目標軌道かどうかを判定
    is_trajectory = isinstance(target_trajectory, np.ndarray) and len(target_trajectory.shape) == 2
    
    def drone_input_func(drone, frame):
        # 現在のフレームに対応する目標位置を取得
        if is_trajectory:
            current_target = target_trajectory[frame % len(target_trajectory)]
        else:
            current_target = target_trajectory
        
        # 目標制御入力
        xi_des = compute_position_tracking_control(drone, current_target, K_p=1.0, K_r=0.5)
        
        # 返却値の初期化
        gamma_val = 0.0
        constraint_margin = 0.0
        ax_value_float = 0.0
        
        # CBF制約を使用する場合
        if use_cbf:
            # 特徴点の中で視野内にあるものを取得
            visible_features = []
            for fp in feature_points:
                if drone.is_point_visible(fp.position):
                    visible_features.append(fp)
            
            # 視野内の特徴点がない場合は制約なしQPを解く
            if not visible_features:
                print("警告: 視野内に特徴点がありません")
                H, f = compute_objective_coefficients(drone, current_target)
                # cvxoptの形式に変換
                P = cvxopt.matrix(H)
                q = cvxopt.matrix(f)
                # 制約なしQP問題を解く
                try:
                    sol = cvxopt.solvers.qp(P, q)
                    # 解が見つかった場合
                    if sol['status'] == 'optimal':
                        xi = np.array(sol['x']).flatten()
                    else:
                        print(f"警告: QP問題の解決に失敗しました: {sol['status']}")
                        xi = xi_des
                except Exception as e:
                    print(f"警告: QP問題の解決中にエラーが発生しました: {e}")
                    xi = xi_des
            else:
                # CBF制約の係数を計算
                if cbf_method == 'pcl':
                    # 複数特徴点に対するCBF
                    alpha_omega, alpha_v, gamma = compute_single_drone_cbf_coefficients(drone, visible_features, q=0.5)
                else:  # cbf_method == 'point'
                    # 単一特徴点に対するCBF
                    feature = visible_features[0]
                    alpha_omega, alpha_v, gamma = compute_single_point_cbf_coefficients(drone, feature, d=1.0)
                    # print(f"CBF: point, gamma={gamma}")
                
                # gamma値を保存
                gamma_val = gamma
                
                # 目的関数の係数を計算
                H, f = compute_objective_coefficients(drone, current_target)
                
                # 安全集合の値が既に負の場合は、制約なしQPを解く
                if gamma <= 0:
                    print(f"警告: 安全集合の値が負です (gamma={gamma:.4f})")
                    # cvxoptの形式に変換
                    P = cvxopt.matrix(H)
                    q = cvxopt.matrix(f)
                    # 制約なしQP問題を解く
                    try:
                        sol = cvxopt.solvers.qp(P, q)
                        # 解が見つかった場合
                        if sol['status'] == 'optimal':
                            xi = np.array(sol['x']).flatten()
                        else:
                            print(f"警告: QP問題の解決に失敗しました: {sol['status']}")
                            xi = xi_des
                    except Exception as e:
                        print(f"警告: QP問題の解決中にエラーが発生しました: {e}")
                        xi = xi_des
                else:
                    # CBFのゲイン
                    gamma0 = 5.0
                    
                    # CBF制約のタイプに応じて処理を分岐
                    if cbf_type == 'no-decomp':
                        # 速度入力及び角速度入力について分解しない場合
                        # CBF制約行列の構築
                        # [alpha_omega, alpha_v]^T * [omega, v] <= gamma0 * gamma
                        A = np.zeros((1, 6))
                        A[0, :3] = alpha_omega
                        A[0, 3:6] = alpha_v
                        
                        # CBF制約の右辺
                        b = np.array([gamma0 * gamma])
                        
                        # cvxoptの形式に変換
                        P = cvxopt.matrix(H)
                        q = cvxopt.matrix(f)
                        G = cvxopt.matrix(A)
                        h = cvxopt.matrix(b)
                        
                        # 制約付きQP問題を解く
                        try:
                            sol = cvxopt.solvers.qp(P, q, G, h)
                            # 解が見つかった場合
                            if sol['status'] == 'optimal':
                                xi = np.array(sol['x']).flatten()
                                # 制約値と制約余裕を計算
                                ax_value = np.dot(A, xi)
                                constraint_value = b - ax_value
                                constraint_margin = float(constraint_value[0])
                                ax_value_float = float(-ax_value[0])
                            
                                # print(f"制約余裕 (no-decomp): {constraint_margin}")
                                # print(f"Ax値 (no-decomp): {ax_value_float}")
                            else:
                                print(f"警告: QP問題の解決に失敗しました: {sol['status']}")
                                xi = xi_des
                        except Exception as e:
                            print(f"警告: QP問題の解決中にエラーが発生しました: {e}")
                            xi = xi_des
                    
                    else:  # cbf_type == 'decomp'
                        # 速度入力及び角速度入力について分解する場合
                        # c_1の初期値（前回の値がある場合はそれを使用）
                        if not hasattr(drone, 'c1'):
                            drone.c1 = 0.5  # デフォルト値
                        
                        # 距離パラメータ（ここでは仮の値を設定）
                        d_m, d_M = 0.5, 15
                        
                        # 拡張された決定変数 [omega, v, c1]
                        # 拡張された目的関数の係数
                        H_ext = np.zeros((7, 7))
                        H_ext[:6, :6] = H
                        f_ext = np.zeros(7)
                        f_ext[:6] = f
                        
                        # CBF制約行列の構築
                        # [alpha_omega, 0, -gamma; 0, alpha_v, -gamma; 0, 0, 1; 0, 0, -1] * [omega; v; c1] <= [0; 0; gamma0/(1-d_m); -gamma0/(1-d_M)]
                        A = np.zeros((4, 7))
                        A[0, :3] = alpha_omega
                        A[0, 6] = -gamma
                        A[1, 3:6] = alpha_v
                        A[1, 6] = -gamma
                        A[2, 6] = 1
                        A[3, 6] = -1
                        
                        # CBF制約の右辺
                        b = np.zeros(4)
                        b[0] = 0
                        b[1] = gamma0 * gamma
                        b[2] = gamma0 / (1 - d_m)
                        b[3] = gamma0 / (1 - d_M)
                        
                        # cvxoptの形式に変換
                        P = cvxopt.matrix(H_ext)
                        q = cvxopt.matrix(f_ext)
                        G = cvxopt.matrix(A)
                        h = cvxopt.matrix(b)
                        
                        # 制約付きQP問題を解く
                        try:
                            sol = cvxopt.solvers.qp(P, q, G, h)
                            # 解が見つかった場合
                            if sol['status'] == 'optimal':
                                x_opt = np.array(sol['x']).flatten()
                                xi = x_opt[:6]
                                print(f"解: {x_opt}")
                                drone.c1 = x_opt[6]  # c1の値を更新
                                # 制約余裕を計算
                                constraint_values = b - np.dot(A, x_opt)
                                # 最小の制約余裕を取得
                                constraint_margin = float(min(constraint_values))
                                print(f"制約余裕 (decomp): {constraint_margin}")
                                print(f"c1: {drone.c1}, c2: {gamma0 - drone.c1}")
                            else:
                                print(f"警告: QP問題の解決に失敗しました: {sol['status']}")
                                xi = xi_des
                        except Exception as e:
                            print(f"警告: QP問題の解決中にエラーが発生しました: {e}")
                            xi = xi_des
        else:
            H, f = compute_objective_coefficients(drone, current_target)
            # cvxoptの形式に変換
            P = cvxopt.matrix(H)
            q = cvxopt.matrix(f)
            # 制約なしQP問題を解く
            try:
                sol = cvxopt.solvers.qp(P, q)
                # 解が見つかった場合
                if sol['status'] == 'optimal':
                    xi = np.array(sol['x']).flatten()
                else:
                    print(f"警告: QP問題の解決に失敗しました: {sol['status']}")
                    xi = xi_des
            except Exception as e:
                print(f"警告: QP問題の解決中にエラーが発生しました: {e}")
                xi = xi_des
        
        # 入力と安全集合の値、制約余裕、Ax値を返す
        # Ax値が計算されていない場合は0を返す
        return xi, gamma_val, constraint_margin, ax_value_float
    
    return drone_input_func
