"""
SE(3)上の制約付き最適化問題（Control Barrier Function）のソルバーを実装するモジュール
集中型最適化と分散型最適化（IEQ-PDMM）の両方をサポート
"""
import numpy as np
from scipy import optimize
import cvxopt
import cvxopt.solvers
from .se3 import SE3, skew
from .drone import Drone, DynamicDrone, FeaturePoint
from .cbf_se3 import compute_hocbf_single_point_coefficients
from .constants import get_gravity

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
        Q2 = np.eye(6)*0.0005
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


def solve_distributed_ieq_pdmm_qp(drone1, drone2, feature_points, p1_des, p2_des, q=0.5, gamma0=0.1, c=1.0, max_iter=10, Q1=None, Q2=None, h=0.01):
    """
    IEQ-PDMMを用いた分散型最適化によりCBF制約付きQP問題を解く（develop.mdのアルゴリズムに基づく実装）
    
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
    c : float, optional
        ペナルティパラメータ（デフォルトは1.0）
    max_iter : int, optional
        最大反復回数（デフォルトは10）
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
        Q1 = np.eye(3)*0.5
    if Q2 is None:
        Q2 = np.eye(6)*0.00001
    
    # CBF制約の係数を計算
    from .cbf_se3 import compute_cbf_constraint_coefficients, compute_gamma_predict
    alpha_omega, beta_omega, alpha_v, beta_v, gamma_val = compute_cbf_constraint_coefficients(
        drone1, drone2, feature_points, q)
    
    # 安全集合の値が既に負の場合は、安全な入力を見つけるのが難しいため、
    # 目標入力をそのまま返す
    if gamma_val < 0:
        print(f"警告: 安全集合の値が負です (gamma={gamma_val:.4f})")
        # 目標位置追従のための制御入力を計算
        from .cbf_se3 import compute_position_tracking_control
        xi1_des = compute_position_tracking_control(drone1, p1_des, K_p=1.0)
        xi2_des = compute_position_tracking_control(drone2, p2_des, K_p=1.0)
        
        # 制約行列の構築
        A = np.zeros((1, 12))
        A[0, :3] = alpha_omega
        A[0, 3:6] = alpha_v
        A[0, 6:9] = beta_omega
        A[0, 9:] = beta_v
        
        # 制約の右辺
        b = np.array([gamma0 * gamma_val])
        
        x_des = np.concatenate([xi1_des, xi2_des])
        constraint_value = np.dot(A, x_des) - b
        print(f"制約値: {constraint_value}")
        return xi1_des, xi2_des, (alpha_omega, beta_omega, alpha_v, beta_v, gamma_val, constraint_value)
    
    # ドローン1, 2の目的関数の係数を計算
    H1, f1 = compute_objective_coefficients(drone1, p1_des, Q1, Q2, h)
    H2, f2 = compute_objective_coefficients(drone2, p2_des, Q1, Q2, h)
    
    # 制約行列の構築
    # ドローン1の制約行列 A_12
    A_12 = np.zeros((1, 6))
    A_12[0, :3] = alpha_omega
    A_12[0, 3:6] = alpha_v
    
    # ドローン2の制約行列 A_21
    A_21 = np.zeros((1, 6))
    A_21[0, :3] = beta_omega
    A_21[0, 3:6] = beta_v
    
    # 制約の右辺
    b = gamma0 * gamma_val / 2  # 各ドローンで半分ずつ
    
    # 双対変数の初期化
    z_12 = np.zeros(1)  # ドローン1からドローン2への双対変数
    z_21 = np.zeros(1)  # ドローン2からドローン1への双対変数
    
    # IEQ-PDMMの反復
    for iter_idx in range(max_iter):
        # ドローン1のQP問題を解く
        # min J_1(ξ_1) + z_1|2^T A_12 ξ_1 + (c/2)||A_12 ξ_1 - (1/2)γ_0 γ||^2
        H1_pdmm = H1.copy()
        f1_pdmm = f1.copy()
        
        # 双対項とペナルティ項の追加
        H1_pdmm += c * A_12.T @ A_12
        f1_pdmm += A_12.T @ (z_12 - c * b)
        
        # cvxoptの形式に変換
        P1 = cvxopt.matrix(H1_pdmm)
        q1 = cvxopt.matrix(f1_pdmm)
        
        # ドローン1のQP問題を解く
        try:
            sol1 = cvxopt.solvers.qp(P1, q1)
            if sol1['status'] == 'optimal':
                xi1 = np.array(sol1['x']).flatten()
            else:
                print(f"警告: ドローン1のQP問題の解決に失敗しました: {sol1['status']}")
                # 目標位置追従のための制御入力を計算
                xi1 = compute_position_tracking_control(drone1, p1_des, K_p=1.0)
        except Exception as e:
            print(f"警告: ドローン1のQP問題の解決中にエラーが発生しました: {e}")
            # 目標位置追従のための制御入力を計算
            xi1 = compute_position_tracking_control(drone1, p1_des, K_p=1.0)
        
        # ドローン2のQP問題を解く
        # min J_2(ξ_2) + z_2|1^T A_21 ξ_2 + (c/2)||A_21 ξ_2 - (1/2)γ_0 γ||^2
        H2_pdmm = H2.copy()
        f2_pdmm = f2.copy()
        
        # 双対項とペナルティ項の追加
        H2_pdmm += c * A_21.T @ A_21
        f2_pdmm += A_21.T @ (z_21 - c * b)
        
        # cvxoptの形式に変換
        P2 = cvxopt.matrix(H2_pdmm)
        q2 = cvxopt.matrix(f2_pdmm)
        
        # ドローン2のQP問題を解く
        try:
            sol2 = cvxopt.solvers.qp(P2, q2)
            if sol2['status'] == 'optimal':
                xi2 = np.array(sol2['x']).flatten()
            else:
                print(f"警告: ドローン2のQP問題の解決に失敗しました: {sol2['status']}")
                # 目標位置追従のための制御入力を計算
                xi2 = compute_position_tracking_control(drone2, p2_des, K_p=1.0)
        except Exception as e:
            print(f"警告: ドローン2のQP問題の解決中にエラーが発生しました: {e}")
            # 目標位置追従のための制御入力を計算
            xi2 = compute_position_tracking_control(drone2, p2_des, K_p=1.0)
        
        # 双対変数の更新
        # y_1|2 = z_1|2 + 2c(A_12 ξ_1 - (1/2)γ_0 γ)
        y_12 = z_12 + 2 * c * (A_12 @ xi1 - b)
        
        # y_2|1 = z_2|1 + 2c(A_21 ξ_2 - (1/2)γ_0 γ)
        y_21 = z_21 + 2 * c * (A_21 @ xi2 - b)
        
        # 双対変数の交換と更新
        # if y_1|2 + y_2|1 > 0: z_1|2 = y_2|1, z_2|1 = y_1|2
        # else: z_1|2 = -y_1|2, z_2|1 = -y_2|1
        if y_12 + y_21 > 0:
            z_12 = y_21
            z_21 = y_12
        else:
            z_12 = -y_12
            z_21 = -y_21
        
        # 収束判定（オプション）
        # 例: 双対変数の変化が小さくなったら収束したと判断
        if iter_idx > 0 and np.linalg.norm(y_12 - z_12) < 1e-6 and np.linalg.norm(y_21 - z_21) < 1e-6:
            print(f"IEQ-PDMM: {iter_idx+1}回目の反復で収束しました")
            break
    
    # 最終的な制約値の計算
    A = np.zeros((1, 12))
    A[0, :3] = alpha_omega
    A[0, 3:6] = alpha_v
    A[0, 6:9] = beta_omega
    A[0, 9:] = beta_v
    
    # 制約の右辺
    b_full = np.array([gamma0 * gamma_val])
    
    x_opt = np.concatenate([xi1, xi2])
    constraint_value = b_full - np.dot(A, x_opt)
    
    return xi1, xi2, (alpha_omega, beta_omega, alpha_v, beta_v, gamma_val, constraint_value)


def compute_dynamic_objective_coefficients(drone, p_des, Q1=None, Q2=None, h=0.01):
    """
    2次系モデルの目的関数の係数を計算
    
    Parameters:
    -----------
    drone : DynamicDrone
        2次系ドローン
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
    H : ndarray, shape (4, 4) または shape (6, 6)
        目的関数のヘッセ行列
    f : ndarray, shape (4,) または shape (6,)
        目的関数の一次項
    """
    # 動力学モデルに応じて処理を分岐
    if drone.dynamics_model == 'holonomic_dynamics':
        # ホロノミック系の場合
        # デフォルトの重み行列
        if Q1 is None:
            Q1 = np.eye(3)
        if Q2 is None:
            Q2 = np.eye(6)
            Q2[0:3, 0:3] = np.eye(3)*0.0000005 # 推力
            Q2[3:6, 3:6] = np.eye(3)*0.00001 # トルク
        
        # 位置誤差
        e_i = p_des - drone.T.p - h*drone.T.R @ drone.v
        
        # 行列Aの計算（develop_dynamics.mdの式に基づく）
        # A = [h^2 * M^{-1} * R_k, h^3 * v_k × J^{-1}]
        A = np.zeros((3, 6))
        # 推力に関する項（3次元ベクトル）
        A[:, 0:3] = -h**2 * np.linalg.inv(drone.M)[0, 0] * drone.T.R
        
        # トルクに関する項
        for i in range(3):
            A[:, i+3] = h**3 * np.cross(drone.v, np.linalg.inv(drone.J)[i, :])
        
        # 修正された誤差項
        tilde_e_k = e_i + h**2 * get_gravity()  # 重力補正
        
        # B'_1とB'_2の計算（develop_dynamics.mdの式に基づく）
        # B'_1 = [M^{-2}B_1, 0; 0, (J^{-1})^T B_2 J^{-1}]
        B_prime_1 = np.zeros((6, 6))
        B_1 = Q2[0:3, 0:3]
        B_2 = Q2[3:6, 3:6]
        
        # スラストに関する項
        M_inv = np.linalg.inv(drone.M)[0, 0]
        B_prime_1[0:3, 0:3] = M_inv**2 * B_1
        
        # トルクに関する項
        J_inv = np.linalg.inv(drone.J)
        B_prime_1[3:6, 3:6] = J_inv.T @ B_2 @ J_inv
        
        # B'_2 = [M^{-1}B_1 A_g; 0]
        B_prime_2 = np.zeros(6)
        A_g = np.cross(drone.v, drone.omega) - drone.T.R.T @ get_gravity()
        B_prime_2[0:3] = M_inv * B_1 @ A_g
        
        # ヘッセ行列の計算
        H = A.T @ A + B_prime_1
        
        # 一次項の計算
        f = A.T @ tilde_e_k + B_prime_2
    else:
        # 非ホロノミック系の場合
        # デフォルトの重み行列
        if Q1 is None:
            Q1 = np.eye(3)
        if Q2 is None:
            Q2 = np.eye(4)*0.00001
            Q2[0, 0] = 0.0001
        
        # 位置誤差
        e_i = p_des - drone.T.p - h*drone.T.R @ drone.v
        
        # 行列Aの計算（develop_dynamics.mdの式に基づく）
        # A = [h^2 * M^{-1} * R_k * e_z, h^3 * v_k × J^{-1}]
        A = np.zeros((3, 4))
        e_z = np.array([0, 0, 1])
        A[:, 0] = -h**2 * np.linalg.inv(drone.M)[0, 0] * drone.T.R @ e_z
        
        # トルクに関する項
        for i in range(3):
            # A[:, i+1] = h**3 * np.cross(drone.v, np.linalg.inv(drone.J)[i, :])
            A[:, i+1] = h**2 * np.cross(drone.v, np.linalg.inv(drone.J)[i, :])
        
        # 修正された誤差項
        tilde_e_k = e_i + h**2 * get_gravity()  # 重力補正
        
        # B'_1とB'_2の計算（develop_dynamics.mdの式に基づく）
        # B'_1 = [M^{-2}b_1, 0; 0, (J^{-1})^T B_2 J^{-1}]
        B_prime_1 = np.zeros((4, 4))
        b_1 = Q2[0, 0]
        B_2 = Q2[1:4, 1:4]
        
        # スラストに関する項
        B_prime_1[0, 0] = np.linalg.inv(drone.M)[0, 0]**2 * b_1
        
        # トルクに関する項
        J_inv = np.linalg.inv(drone.J)
        B_prime_1[1:4, 1:4] = J_inv.T @ B_2 @ J_inv
        
        # B'_2 = [M^{-1}b_1 A_g^T e_z; 0]
        B_prime_2 = np.zeros(4)
        A_g = np.cross(drone.v, drone.omega) - drone.T.R.T @ get_gravity()
        B_prime_2[0] = np.linalg.inv(drone.M)[0, 0] * b_1 * np.dot(A_g, e_z)
        
        # ヘッセ行列の計算
        H = A.T @ A + B_prime_1
        
        # 一次項の計算
        f = A.T @ tilde_e_k + B_prime_2
    
    return H, f


def solve_dynamic_qp(drone, feature_point, p_des, use_cbf=False, gamma0=0.1, gamma1=0.1, Q1=None, Q2=None, h=0.01):
    """
    2次系モデルのQP問題を解く
    
    Parameters:
    -----------
    drone : DynamicDrone
        2次系ドローン
    feature_point : FeaturePoint
        視野内の特徴点（CBF制約を使用する場合）
    p_des : array_like, shape (3,)
        目標位置
    use_cbf : bool, optional
        CBF制約を使用するかどうか（デフォルトはFalse）
    gamma0 : float, optional
        CBFのゲイン0（デフォルトは0.1）
    gamma1 : float, optional
        CBFのゲイン1（デフォルトは0.1）
    Q1 : array_like, optional
        位置誤差の重み行列（デフォルトは単位行列）
    Q2 : array_like, optional
        制御入力の重み行列（デフォルトは単位行列）
    h : float, optional
        時間ステップ（デフォルトは0.01）
        
    Returns:
    --------
    u : ndarray, shape (4,) または shape (6,)
        制御入力 [f, tau]
    constraint_value : float or None
        制約余裕の値（CBF制約を使用する場合）
    B, B_dot, B_ddot : float or None
        CBF値、その1階微分、2階微分（CBF制約を使用する場合）
    """
    # 目的関数の係数を計算
    H, f = compute_dynamic_objective_coefficients(drone, p_des, Q1, Q2, h)
    
    # CBF制約を使用する場合
    if use_cbf and feature_point is not None:
        # HOCBFの係数を計算
        C, b, B, B_dot, B_ddot_minus = compute_hocbf_single_point_coefficients(drone, feature_point, gamma0, gamma1)
        
        # 制約行列の構築
        if drone.dynamics_model == 'holonomic_dynamics':
            # ホロノミック系の場合
            A = np.zeros((1, 6))
            A[0, :] = C
        else:
            # 非ホロノミック系の場合
            A = np.zeros((1, 4))
            A[0, :] = C
        
        # 制約の右辺
        b_arr = np.array([b])
        
        # cvxoptの形式に変換
        P = cvxopt.matrix(H)
        q = cvxopt.matrix(f)
        G = cvxopt.matrix(A)
        h_cvx = cvxopt.matrix(b_arr)
        
        # 制約付きQP問題を解く
        try:
            sol = cvxopt.solvers.qp(P, q, G, h_cvx)
            # 解が見つかった場合
            if sol['status'] == 'optimal':
                u = np.array(sol['x']).flatten()
                # 制約値と制約余裕を計算
                Cu_value = np.dot(A, u)
                constraint_value = float(b - Cu_value)
                # for debug
                # u = np.array([9.81, 0.0, 1.0, 0.0])
                B_ddot = B_ddot_minus - C @ u
                return u, constraint_value, B, B_dot, B_ddot
            else:
                print(f"警告: QP問題の解決に失敗しました: {sol['status']}")
                # 目標位置追従のための制御入力を計算
                from .cbf_se3 import compute_position_tracking_acceleration_control
                u_des = compute_position_tracking_acceleration_control(drone, p_des)
                return u_des, None, None, None, None
        except Exception as e:
            print(f"警告: QP問題の解決中にエラーが発生しました: {e}")
            # 目標位置追従のための制御入力を計算
            from .cbf_se3 import compute_position_tracking_acceleration_control
            u_des = compute_position_tracking_acceleration_control(drone, p_des)
            return u_des, None, None, None, None
    else:
        # 制約なしQP問題を解く
        P = cvxopt.matrix(H)
        q = cvxopt.matrix(f)
        try:
            sol = cvxopt.solvers.qp(P, q)
            # 解が見つかった場合
            if sol['status'] == 'optimal':
                u = np.array(sol['x']).flatten()
                return u, None, None, None, None
            else:
                print(f"警告: QP問題の解決に失敗しました: {sol['status']}")
                # 目標位置追従のための制御入力を計算
                from .cbf_se3 import compute_position_tracking_acceleration_control
                u_des = compute_position_tracking_acceleration_control(drone, p_des)
                return u_des, None, None, None, None
        except Exception as e:
            print(f"警告: QP問題の解決中にエラーが発生しました: {e}")
            # 目標位置追従のための制御入力を計算
            from .cbf_se3 import compute_position_tracking_acceleration_control
            u_des = compute_position_tracking_acceleration_control(drone, p_des)
            return u_des, None, None, None, None


def solve_direct_cbf_qp(drone1, drone2, feature_points, p1_des, p2_des, q=0.5, gamma0=0.1, c1=0.5, c2=0.5, Q1=None, Q2=None, h=0.01):
    """
    目標位置から直接CBF制約付きQP問題を解く（集中型最適化、pcl_ccbf.mdのアルゴリズムに基づく実装）
    
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
        Q2 = np.eye(6)*0.0005
    
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
        constraint_value = b - np.dot(A, x_des)
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
            constraint_value = b - np.dot(A, x_opt)
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
