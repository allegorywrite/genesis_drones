"""
SE(3)上の制約付き最適化問題（Control Barrier Function）のソルバーを実装するモジュール
"""
import numpy as np
from scipy import optimize
import cvxopt
import cvxopt.solvers
from .se3 import SE3, skew
from .drone import Drone, FeaturePoint

# cvxoptのソルバーの設定（出力を抑制）
cvxopt.solvers.options['show_progress'] = False

def compute_objective_coefficients(drone, p_des, Q1=None, Q2=None, h=0.01):
    """
    目的関数の係数を計算
    
    Parameters:
    -----------
    drone : Drone
        ドローン
    p_des : array_like, shape (3,)
        目標位置
    Q1 : array_like, optional
        位置誤差の重み行列（デフォルトは単位行列）
    Q2 : array_like, optional
        制御入力の重み行列（デフォルトは単位行列）
    h : float, optional
        時間ステップ（デフォルトは0.01）
        
    Returns:
    --------
    H : ndarray, shape (6, 6)
        目的関数のヘッセ行列
    f : ndarray, shape (6,)
        目的関数の一次項
    """
    # デフォルトの重み行列
    if Q1 is None:
        Q1 = np.eye(3)
    if Q2 is None:
        Q2 = np.eye(6)*0.01
        # Q2 = np.zeros((6, 6))
        # Q2[:3, :3] = np.eye(3)*0.0001
        # Q2[3:, 3:] = np.eye(3)*0.01
    
    # 位置誤差
    e_i = p_des - drone.T.p
    
    # ヘッセ行列の計算
    # H = 2 * [Q2_omega, Q2_omega_v; Q2_omega_v^T, Q2_v + h^2 * R_i^T * Q1 * R_i]
    H = np.zeros((6, 6))
    
    # 角速度部分
    H[:3, :3] = 2 * Q2[:3, :3]
    
    # 角速度と速度の相互項
    H[:3, 3:] = 2 * Q2[:3, 3:]
    H[3:, :3] = 2 * Q2[3:, :3]
    
    # 速度部分
    H[3:, 3:] = 2 * (Q2[3:, 3:] + h**2 * drone.T.R.T @ Q1 @ drone.T.R)
    
    # 一次項の計算
    # f = [0; -2h * R_i^T * Q1 * e_i]
    f = np.zeros(6)
    f[3:] = -2 * h * drone.T.R.T @ Q1 @ e_i
    
    return H, f

def solve_direct_qp(drone1, drone2, feature_points, p1_des, p2_des, Q1=None, Q2=None, h=0.01):
    """
    目標位置から直接制約なしQP問題を解く
    
    Parameters:
    -----------
    drone1 : Drone
        1つ目のドローン
    drone2 : Drone
        2つ目のドローン
    feature_points : list of FeaturePoint
        環境内の特徴点のリスト（この関数では使用しないが、インターフェースの一貫性のために含める）
    p1_des : array_like, shape (3,)
        ドローン1の目標位置
    p2_des : array_like, shape (3,)
        ドローン2の目標位置
    Q1 : array_like, optional
        位置誤差の重み行列（デフォルトは単位行列）
    Q2 : array_like, optional
        制御入力の重み行列（デフォルトは単位行列）
    h : float, optional
        時間ステップ（デフォルトは0.01）
        
    Returns:
    --------
    xi1 : ndarray, shape (6,)
        ドローン1の制御入力
    xi2 : ndarray, shape (6,)
        ドローン2の制御入力
    None : NoneType
        制約値（CBF制約がないためNone）
    """
    # デフォルトの重み行列
    if Q1 is None:
        Q1 = np.eye(3)
    if Q2 is None:
        Q2 = np.eye(6)*0.001
    
    # ドローン1の目的関数の係数を計算
    H1, f1 = compute_objective_coefficients(drone1, p1_des, Q1, Q2, h)
    
    # ドローン2の目的関数の係数を計算
    H2, f2 = compute_objective_coefficients(drone2, p2_des, Q1, Q2, h)
    
    # 目的関数の係数を結合
    H = np.zeros((12, 12))
    H[:6, :6] = H1
    H[6:, 6:] = H2
    
    f = np.zeros(12)
    f[:6] = f1
    f[6:] = f2
    
    # cvxoptの形式に変換
    P = cvxopt.matrix(H)
    q = cvxopt.matrix(f)
    
    # 制約なしQP問題を解く
    try:
        sol = cvxopt.solvers.qp(P, q)
        
        # 解が見つかった場合
        if sol['status'] == 'optimal':
            x_opt = np.array(sol['x']).flatten()
            xi1 = x_opt[:6]
            xi2 = x_opt[6:]
            return xi1, xi2, None
        else:
            print(f"警告: QP問題の解決に失敗しました: {sol['status']}")
            # 目標位置追従のための制御入力を計算
            from .cbf_se3 import compute_position_tracking_control
            xi1_des = compute_position_tracking_control(drone1, p1_des, K_p=1.0)
            xi2_des = compute_position_tracking_control(drone2, p2_des, K_p=1.0)
            return xi1_des, xi2_des, None
    except Exception as e:
        print(f"警告: QP問題の解決中にエラーが発生しました: {e}")
        # 目標位置追従のための制御入力を計算
        from .cbf_se3 import compute_position_tracking_control
        xi1_des = compute_position_tracking_control(drone1, p1_des, K_p=1.0)
        xi2_des = compute_position_tracking_control(drone2, p2_des, K_p=1.0)
        return xi1_des, xi2_des, None


def solve_direct_cbf_qp(drone1, drone2, feature_points, p1_des, p2_des, q=0.5, gamma0=0.1, c1=0.5, c2=0.5, Q1=None, Q2=None, h=0.01):
    """
    目標位置から直接CBF制約付きQP問題を解く（pcl_ccbf.mdのアルゴリズムに基づく実装）
    
    Parameters:
    -----------
    drone1 : Drone
        1つ目のドローン
    drone2 : Drone
        2つ目のドローン
    feature_points : list of FeaturePoint
        環境内の特徴点のリスト
    p1_des : array_like, shape (3,)
        ドローン1の目標位置
    p2_des : array_like, shape (3,)
        ドローン2の目標位置
    q : float, optional
        確率の閾値（デフォルトは0.5）
    gamma0 : float, optional
        CBFのゲイン（デフォルトは0.1）
    c1, c2 : float, optional
        CBF制約の重み（デフォルトはそれぞれ0.5）
        速度入力及び角速度入力を分解する場合に使用
    Q1 : array_like, optional
        位置誤差の重み行列（デフォルトは単位行列）
    Q2 : array_like, optional
        制御入力の重み行列（デフォルトは単位行列）
    h : float, optional
        時間ステップ（デフォルトは0.01）
        
    Returns:
    --------
    xi1_safe : ndarray, shape (6,)
        ドローン1の安全な速度入力
    xi2_safe : ndarray, shape (6,)
        ドローン2の安全な速度入力
    constraint_values : tuple
        制約値 (alpha_omega, beta_omega, alpha_v, beta_v, gamma_val, constraint_value)
        constraint_value: 制約余裕の値
    """
    # デフォルトの重み行列
    if Q1 is None:
        Q1 = np.eye(3)
    if Q2 is None:
        Q2 = np.eye(6)*0.01
    
    # CBF制約の係数を計算
    from .cbf_se3 import compute_cbf_constraint_coefficients, compute_gamma_predict
    alpha_omega, beta_omega, alpha_v, beta_v, gamma_val = compute_cbf_constraint_coefficients(
        drone1, drone2, feature_points, q)
    
    # ドローン1, 2の目的関数の係数を計算
    H1, f1 = compute_objective_coefficients(drone1, p1_des, Q1, Q2, h)
    H2, f2 = compute_objective_coefficients(drone2, p2_des, Q1, Q2, h)
    
    # 目的関数の係数を結合
    H = np.zeros((12, 12))
    H[:6, :6] = H1
    H[6:, 6:] = H2
    
    f = np.zeros(12)
    f[:6] = f1
    f[6:] = f2
    
    # cvxoptの形式に変換
    P_cvx = cvxopt.matrix(H)
    q_cvx = cvxopt.matrix(f)
    
    # 制約行列の構築
    # 速度入力及び角速度入力について分解しない場合（pcl_ccbf.mdの式(40)-(43)に基づく）
    # [alpha_omega, alpha_v, beta_omega, beta_v] * [omega1; v1; omega2; v2] <= gamma0 * gamma
    A = np.zeros((1, 12))
    A[0, :3] = alpha_omega
    A[0, 3:6] = alpha_v
    A[0, 6:9] = beta_omega
    A[0, 9:] = beta_v
    
    # 制約の右辺
    b = np.array([gamma0 * gamma_val])

    # 安全集合の値が既に負の場合は、安全な入力を見つけるのが難しいため、
    # 目標入力をそのまま返す
    if gamma_val < 0:
        print(f"警告: 安全集合の値が負です (gamma={gamma_val:.4f})")
        # 目標位置追従のための制御入力を計算
        from .cbf_se3 import compute_position_tracking_control
        xi1_des = compute_position_tracking_control(drone1, p1_des, K_p=1.0)
        xi2_des = compute_position_tracking_control(drone2, p2_des, K_p=1.0)
        x_des = np.concatenate([xi1_des, xi2_des])
        constraint_value = np.dot(A, x_des) - b
        print(f"制約値: {constraint_value}")
        return xi1_des, xi2_des, (alpha_omega, beta_omega, alpha_v, beta_v, gamma_val, constraint_value)
    
    # cvxoptの形式に変換
    G = cvxopt.matrix(A)
    h_cvx = cvxopt.matrix(b)
    
    # QP問題を解く
    try:
        sol = cvxopt.solvers.qp(P_cvx, q_cvx, G, h_cvx)
        # 解が見つかった場合
        if sol['status'] == 'optimal':
            x_opt = np.array(sol['x']).flatten()
            xi1_safe = x_opt[:6]
            xi2_safe = x_opt[6:]
            constraint_value = np.dot(A, x_opt) - b
            return xi1_safe, xi2_safe, (alpha_omega, beta_omega, alpha_v, beta_v, gamma_val, constraint_value)
        else:
            print(f"警告: QP問題の解決に失敗しました: {sol['status']}")
            # 目標位置追従のための制御入力を計算
            from .cbf_se3 import compute_position_tracking_control
            xi1_des = compute_position_tracking_control(drone1, p1_des, K_p=1.0)
            xi2_des = compute_position_tracking_control(drone2, p2_des, K_p=1.0)
            return xi1_des, xi2_des, (alpha_omega, beta_omega, alpha_v, beta_v, gamma_val, None)
    except Exception as e:
        print(f"警告: QP問題の解決中にエラーが発生しました: {e}")
        # 目標位置追従のための制御入力を計算
        from .cbf_se3 import compute_position_tracking_control
        xi1_des = compute_position_tracking_control(drone1, p1_des, K_p=1.0)
        xi2_des = compute_position_tracking_control(drone2, p2_des, K_p=1.0)
        return xi1_des, xi2_des, (alpha_omega, beta_omega, alpha_v, beta_v, gamma_val, None)
