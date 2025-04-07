"""
ドローン制御モジュール
"""
import numpy as np
import cvxopt
import sys
import os

# 親ディレクトリをパスに追加
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.cbf_se3 import compute_position_tracking_control, compute_single_drone_cbf_coefficients, compute_single_point_cbf_coefficients, compute_single_prob_cbf_coefficients
from utils.solver import compute_objective_coefficients


def create_drone_input_func(drone, feature_points, target_position, use_cbf=False, cbf_type='no-decomp', cbf_method='pcl'):
    """
    ドローンの入力関数を作成
    
    Parameters:
    -----------
    drone : Drone
        ドローン
    feature_points : list of FeaturePoint
        特徴点のリスト
    target_position : array_like, shape (3,)
        目標位置
    use_cbf : bool, optional
        CBF制約を使用するかどうか（デフォルトはFalse）
    cbf_type : str, optional
        CBF制約のタイプ: 'no-decomp'（速度入力及び角速度入力について分解しない場合）または
        'decomp'（分解する場合）（デフォルトは'no-decomp'）
    cbf_method : str, optional
        CBF制約の方法: 'pcl'（複数特徴点に対するCBF）または
        'point'（単一特徴点に対するCBF）（デフォルトは'pcl'）
    
    Returns:
    --------
    input_func : callable
        ドローンの入力関数
    """
    def drone_input_func(drone, frame):
        # 目標制御入力
        xi_des = compute_position_tracking_control(drone, target_position, K_p=1.0, K_r=0.5)
        
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
                H, f = compute_objective_coefficients(drone, target_position)
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
                    # alpha_omega, alpha_v, gamma = compute_single_prob_cbf_coefficients(drone, visible_features[0], q=0.5)
                else:  # cbf_method == 'point'
                    # 単一特徴点に対するCBF
                    feature = visible_features[0]
                    alpha_omega, alpha_v, gamma = compute_single_point_cbf_coefficients(drone, feature, d=1.0)
                    print(f"CBF: point, gamma={gamma}")
                
                # gamma値を保存
                gamma_val = gamma
                
                # デバッグ出力
                # print(f"alpha_omega: {alpha_omega}")
                # print(f"alpha_v: {alpha_v}")
                # print(f"gamma: {gamma}")
                
                # 目的関数の係数を計算
                H, f = compute_objective_coefficients(drone, target_position)
                
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
                    gamma0 = 0.01
                    
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
            H, f = compute_objective_coefficients(drone, target_position)
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
