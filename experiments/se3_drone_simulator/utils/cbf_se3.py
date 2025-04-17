"""
SE(3)上の制約付き最適化問題（Control Barrier Function）を実装するモジュール
"""
import math
import numpy as np
from .se3 import SE3, skew
from .drone import Drone, DynamicDrone, FeaturePoint
from .constants import get_gravity

def sample_safe_configuration(simulator, fov_angle, min_barrier_value=0.0, max_attempts=1000, q=0.5):
    """
    安全集合の値が指定値以上になるようにドローン2の位置と姿勢をランダムサンプリング
    
    Parameters:
    -----------
    simulator : Simulator
        シミュレータ（ドローン1と特徴点が既に追加されている必要がある）
    fov_angle : float
        視野角（ラジアン）
    min_barrier_value : float, optional
        最小安全集合値（デフォルトは0.0）
    max_attempts : int, optional
        最大試行回数（デフォルトは1000）
        
    Returns:
    --------
    success : bool
        サンプリングが成功したかどうか
    attempts : int
        試行回数
    barrier_value : float
        最終的な安全集合の値
    """
    from scipy.spatial.transform import Rotation as R
    import random
    
    # ドローン1が存在することを確認
    if len(simulator.drones) == 0:
        raise ValueError("ドローン1がシミュレータに追加されていません")
    
    drone1 = simulator.drones[0]
    p_initial = np.array([4.0, 4.0, 4.0])
    R_initial = R.from_euler('xyz', [1.0, -1.0, 0.0]).as_matrix()

    drone1.T = SE3(R=R_initial, p=p_initial)
    
    # ランダムな回転行列を生成する関数
    def random_rotation_matrix():
        quat = R.random().as_quat()
        return R.from_quat(quat).as_matrix()
    
    # ランダムな位置ベクトルを生成する関数
    def random_position(min_val=-3.0, max_val=3.0):
        return np.array([
            random.uniform(min_val, max_val),
            random.uniform(min_val, max_val),
            random.uniform(min_val, max_val)
        ])
    
    print(f"ドローン2の位置と姿勢をランダムにサンプリングしています...")
    print(f"安全集合の値が{min_barrier_value}以上になるまで繰り返します...")
    
    for attempt in range(max_attempts):
        # ドローン2をシミュレータから削除（2回目以降のループ用）
        if len(simulator.drones) > 1:
            simulator.drones.pop()
        
        # ドローン2: ランダムな位置と姿勢、指定された視野角
        rot_matrix = random_rotation_matrix()
        position = random_position(min_val=-5.0, max_val=-3.0)
        
        # 位置が原点に近すぎる場合は調整（ドローン1と重ならないように）
        if np.linalg.norm(position) < 1.0:
            position = position / np.linalg.norm(position) * 1.0
        
        drone2 = Drone(SE3(rot_matrix, position), fov_angle=fov_angle)
        simulator.add_drone(drone2)
        
        # 安全集合の値を計算
        # barrier_value = compute_barrier_function(drone1, drone2, simulator.feature_points)
        barrier_value = compute_gamma(drone1, drone2, simulator.feature_points, q)
        
        # 進捗表示（100回ごと）
        if attempt % 100 == 0 and attempt > 0:
            print(f"  {attempt}回試行: 安全集合の値 = {barrier_value:.4f}")
        
        # 安全集合の値が指定値以上であれば終了
        if barrier_value >= min_barrier_value:
            # print(f"成功! {attempt + 1}回目の試行で安全集合の値が{barrier_value:.4f}になりました")
            # print(f"ドローン2の位置: {position}")
            # print(f"ドローン2の回転行列:\n{rot_matrix}")
            return True, attempt + 1, barrier_value
    
    # 最大試行回数に達しても見つからなかった場合
    print(f"警告: {max_attempts}回の試行で安全集合の値が{min_barrier_value}以上になりませんでした")
    print(f"最後の試行での安全集合の値: {barrier_value:.4f}")
    return False, max_attempts, barrier_value

def compute_single_point_cbf_coefficients(drone, feature_point, d=1.0):
    """
    単一特徴点のCBF制約の係数を計算（point_cbf.mdに基づく実装）
    
    Parameters:
    -----------
    drone : Drone
        ドローン
    feature_point : FeaturePoint
        視野内の特徴点
    d : float, optional
        距離パラメータ（デフォルトは1.0）
        
    Returns:
    --------
    alpha_omega : ndarray, shape (3,)
        角速度に関する制約係数
    alpha_v : ndarray, shape (3,)
        速度に関する制約係数
    gamma : float
        CBF制約の右辺値
    """
    # カメラの方向ベクトル
    e_c = drone.camera_direction
    
    # 視野角の余弦
    cos_psi_F = np.cos(drone.fov_angle)
    
    # 特徴点の位置
    q_l = feature_point.position
    
    # βベクトル（特徴点の方向を表す単位ベクトル）
    beta_l = drone.get_beta_vector(q_l)
    
    # 安全集合の値（gamma）の計算
    # γ = β_l^T(p_i)R_ie_c - cos(Ψ_F)
    gamma_val = beta_l.T @ drone.T.R @ e_c - cos_psi_F
    
    # 制約係数の計算
    # α_ω = β_l^T(p_i) R_i [e_c]_×
    alpha_omega = beta_l.T @ drone.T.R @ skew(e_c)

    # 距離パラメータ
    # d = np.linalg.norm(q_l - drone.T.p)
    
    # α_v = e_c^T R_i^T P_{β_l} / d
    # P_{β_l} = I - β_l β_l^T（投影行列）
    P_beta_l = np.eye(3) - np.outer(beta_l, beta_l)
    # alpha_v = e_c.T @ drone.T.R.T @ P_beta_l / d
    alpha_v_b = e_c.T @ drone.T.R.T @ P_beta_l @ drone.T.R / d
    
    return alpha_omega, alpha_v_b, gamma_val


def compute_single_drone_cbf_coefficients(drone, feature_points, q=0.5):
    """
    単一ドローンのCBF制約の係数を計算
    
    Parameters:
    -----------
    drone : Drone
        ドローン
    feature_points : list of FeaturePoint
        視野内の特徴点のリスト
    q : float, optional
        確率の閾値（デフォルトは0.5）
    d : float, optional
        距離パラメータ（デフォルトは1.0）
        
    Returns:
    --------
    alpha_omega : ndarray, shape (3,)
        角速度に関する制約係数
    alpha_v : ndarray, shape (3,)
        速度に関する制約係数
    gamma : float
        CBF制約の右辺値
    """
    # カメラの方向ベクトル
    e_c = drone.camera_direction
    
    # 視野角の余弦
    cos_psi_F = np.cos(drone.fov_angle)

    # ドローンから見える特徴点を取得
    visible_points = [fp for fp in feature_points if drone.is_point_visible(fp.position)]

    prod_term_all = 1.0
    alpha_omega = np.zeros(3)
    alpha_v = np.zeros(3)

    for i, fp in enumerate(visible_points):
        prod_other = 1.0
        for j, fp2 in enumerate(visible_points):
            if i != j:
                q2_l = fp2.position
                beta2_l = drone.get_beta_vector(q2_l)
                prod_other *= (1 - (beta2_l.T @ drone.T.R @ e_c - cos_psi_F) / (1 - cos_psi_F))
        
        q_l = fp.position
        beta_l = drone.get_beta_vector(q_l)

        prod_term_all *= (1 - (beta_l.T @ drone.T.R @ e_c - cos_psi_F) / (1 - cos_psi_F))
        
        alpha_omega += prod_other * (beta_l.T @ drone.T.R @ skew(e_c)) / (1 - cos_psi_F)

        d = np.linalg.norm(q_l - drone.T.p)

        P_beta_l = np.eye(3) - np.outer(beta_l, beta_l)
        alpha_v += prod_other * e_c.T @ drone.T.R.T @ P_beta_l @ drone.T.R / ((1 - cos_psi_F)*d)

    gamma_val = 1 - q - prod_term_all

    return alpha_omega, alpha_v, gamma_val


def compute_single_prob_cbf_coefficients(drone, feature_point, q=0.5):
    """
    単一特徴点のCBF制約の係数を計算（point_cbf.mdに基づく実装）
    
    Parameters:
    -----------
    drone : Drone
        ドローン
    feature_point : FeaturePoint
        視野内の特徴点
    d : float, optional
        距離パラメータ（デフォルトは1.0）
        
    Returns:
    --------
    alpha_omega : ndarray, shape (3,)
        角速度に関する制約係数
    alpha_v : ndarray, shape (3,)
        速度に関する制約係数
    gamma : float
        CBF制約の右辺値
    """
    # カメラの方向ベクトル
    e_c = drone.camera_direction
    
    # 視野角の余弦
    cos_psi_F = np.cos(drone.fov_angle)
    
    # 特徴点の位置
    q_l = feature_point.position
    
    # βベクトル（特徴点の方向を表す単位ベクトル）
    beta_l = drone.get_beta_vector(q_l)
    
    # 安全集合の値（gamma）の計算
    gamma_val = (beta_l.T @ drone.T.R @ e_c - cos_psi_F) / (1 - cos_psi_F) - q
    
    # 制約係数の計算
    alpha_omega = (beta_l.T @ drone.T.R @ skew(e_c)) / (1 - cos_psi_F)

    # 距離パラメータ
    d = np.linalg.norm(q_l - drone.T.p)
    
    P_beta_l = np.eye(3) - np.outer(beta_l, beta_l)
    alpha_v_b = e_c.T @ drone.T.R.T @ P_beta_l @ drone.T.R / ((1 - cos_psi_F)*d)
    
    return alpha_omega, alpha_v_b, gamma_val


def compute_position_tracking_control(drone, p_des, K_p=1.0, K_r=0.5):
    """
    目標位置に追従するための制御入力を計算
    
    Parameters:
    -----------
    drone : Drone
        ドローン
    p_des : array_like, shape (3,)
        目標位置
    K_p : float, optional
        位置制御のゲイン（デフォルトは1.0）
        
    Returns:
    --------
    xi : ndarray, shape (6,)
        制御入力 [omega, v]
    """
    # 現在位置と目標位置の誤差
    p_error = p_des - drone.T.p
    
    # 並進速度（ワールドフレーム）
    v = K_p * p_error
    vb = drone.T.R.T @ v
    
    # 制御入力
    xi = np.concatenate([np.zeros(3), vb])
    
    return xi

def compute_gamma_predict(drone1, drone2, feature_points, xi1, xi2, q=0.5, dt=0.01):
    """
    安全集合の値を予測する
    
    Parameters:
    -----------
    drone1 : Drone
        1つ目のドローン
    drone2 : Drone
        2つ目のドローン
    feature_points : list of FeaturePoint
        環境内の特徴点のリスト
    xi1 : array_like, shape (6,)
        ドローン1の速度入力 [omega, v]
    xi2 : array_like, shape (6,)
        ドローン2の速度入力 [omega, v]
    q : float, optional
        確率の閾値（デフォルトは0.5）
    dt : float, optional
        時間ステップ（デフォルトは0.01）
        
    Returns:
    --------
    gamma : float
        安全集合の値
    """    
    # ドローンを速度入力に基づいて更新
    drone1_next = Drone(SE3(drone1.T.R.copy(), drone1.T.p.copy()), 
                        drone1.camera_direction.copy(), drone1.fov_angle)
    drone2_next = Drone(SE3(drone2.T.R.copy(), drone2.T.p.copy()), 
                        drone2.camera_direction.copy(), drone2.fov_angle)
    
    drone1_next.update(xi1, dt)
    drone2_next.update(xi2, dt)
    
    # 更新後の値
    next_value = compute_gamma(drone1_next, drone2_next, feature_points, q, dt)
    
    return next_value

def compute_gamma(drone1, drone2, feature_points, q=0.5, d=1.0):
    """
    CBF制約の係数を計算
    
    Parameters:
    -----------
    drone1 : Drone
        1つ目のドローン
    drone2 : Drone
        2つ目のドローン
    feature_points : list of FeaturePoint
        環境内の特徴点のリスト
    q : float, optional
        確率の閾値（デフォルトは0.5）
    d : float, optional
        距離パラメータ（デフォルトは1.0）
        
    Returns:
    --------
    gamma : float
        CBF制約の右辺値
    """
    # カメラの方向ベクトル
    e_c = drone1.camera_direction  # 両方のドローンで同じと仮定
    
    # 視野角の余弦
    cos_psi_F = np.cos(drone1.fov_angle)
    
    # 各特徴点についての確率を計算
    probs = []
    visible_features = []
    
    for fp in feature_points:
        # 両方のドローンから見える特徴点のみを考慮
        if drone1.is_point_visible(fp.position) and drone2.is_point_visible(fp.position):
            prob = drone1.calculate_cofov_probability(drone2, fp.position)
            probs.append(prob)
            visible_features.append(fp)
    

    # 共有視野内の特徴点がない場合
    if not visible_features:
        return -q
    
    # 安全集合の値（gamma）の計算
    prod_term = 1.0
    for prob in probs:
        prod_term *= (1.0 - prob)
    
    gamma_val = 1.0 - q - prod_term

    return gamma_val

def compute_hocbf_single_point_coefficients(drone, feature_point, gamma0=0.1, gamma1=0.1):
    """
    単一特徴点のHOCBF制約の係数を計算（develop_dynamics.mdに基づく実装）
    
    Parameters:
    -----------
    drone : DynamicDrone
        2次系ドローン
    feature_point : FeaturePoint
        視野内の特徴点
    gamma0 : float, optional
        CBFのゲイン0（デフォルトは0.1）
    gamma1 : float, optional
        CBFのゲイン1（デフォルトは0.1）
        
    Returns:
    --------
    C : ndarray, shape (1, 4) または shape (1, 6)
        制約行列 [C_f, C_tau]
    b : float
        制約の右辺値
    """
    v = drone.v
    omega = drone.omega
    M = drone.M[0, 0]
    R = drone.T.R
    g = get_gravity()

    # カメラの方向ベクトル
    e_c = drone.camera_direction
    
    # 視野角の余弦
    cos_psi_F = np.cos(drone.fov_angle)
    
    # 特徴点の位置
    q_l = feature_point.position
    
    # βベクトル（特徴点の方向を表す単位ベクトル）
    beta_l = drone.get_beta_vector(q_l)
    
    # 安全集合の値（B）の計算
    B = beta_l.T @ drone.T.R @ e_c - cos_psi_F
    
    # 投影行列
    P_beta_l = np.eye(3) - np.outer(beta_l, beta_l)
    
    # 距離
    d_il = np.linalg.norm(q_l - drone.T.p)
    
    # C_tau: トルクに関する係数
    C_tau = beta_l.T @ drone.T.R @ skew(e_c) @ np.linalg.inv(drone.J)
    
    # 動力学モデルに応じて処理を分岐
    if drone.dynamics_model == 'holonomic_dynamics':
        # ホロノミック系の場合
        # C_f: 推力に関する係数（3次元ベクトル）
        C_f = e_c.T @ drone.T.R.T @ P_beta_l @ drone.T.R / (d_il*M)

        print(f"e_c: {e_c}")
        print(f"beta_l: {beta_l}")
        print(f"P_beta_l: {P_beta_l}")
        print(f"drone.T.R: {drone.T.R}")
        print(f"d_il: {d_il}")
        print(f"M: {M}")

        print(f"C_f: {C_f}")
        print(f"C_tau: {C_tau}")
        
        # 制約行列
        C = np.zeros(6)
        C[0:3] = C_f
        C[3:6] = C_tau
    else:
        # 非ホロノミック系の場合
        # C_f: 推力に関する係数（スカラー）
        C_f = e_c.T @ drone.T.R.T @ P_beta_l @ drone.T.R @ np.array([0, 0, 1]) / (d_il*M)
        
        # 制約行列
        C = np.zeros(4)
        C[0] = C_f
        C[1:4] = C_tau
    
    # 右辺の計算（develop_dynamics.mdの式に基づく）

    # 1階微分項の計算
    B_dot = -beta_l.T @ drone.T.R @ skew(e_c) @ drone.omega - e_c.T @ drone.T.R.T @ P_beta_l @ drone.T.R @ drone.v / d_il
    
    # 2階微分項の計算
    z = R @ e_c

    term1 = (beta_l @ (z.T @ P_beta_l) + (z.T @ beta_l)*P_beta_l + P_beta_l @ z @ beta_l.T) / d_il**2
    term2 = z.T @ P_beta_l @ R @ skew(omega) / d_il

    hess_ph_omega = v.T @ R.T @ P_beta_l @ R @ skew(e_c) @ omega / d_il
    hess_ph_v = -v.T @ R.T @ term1 @ R @ v - term2 @ v
    hess_Rh_v = hess_ph_omega
    hess_Rh_omega = - beta_l.T @ R @ skew(omega) @ skew(e_c) @ omega
    print("hess_Rh_omega:", hess_Rh_omega)
    print("hess_Rh_v:", hess_Rh_v)
    print("hess_ph_v:", hess_ph_v)
    print("hess_ph_omega:", hess_ph_omega)

    print("B:", B)
    print("B_dot:", B_dot)
    
    A_g = np.cross(v, omega) - np.dot(R.T, g)
    # A_g =  - np.dot(R.T, g)
    b_g = - e_c.T @ R.T @ P_beta_l @ R @ A_g / d_il

    B_ddot_minus = hess_ph_omega + hess_ph_v + hess_Rh_v + hess_Rh_omega + b_g
    b = B_ddot_minus + (gamma0 + gamma1)*B_dot + (gamma0*gamma1)*B
    
    return C, b, B, B_dot, B_ddot_minus

def compute_hocbf_multi_point_coefficients(drone, feature_points, q=0.5, gamma0=0.1, gamma1=0.1):
    """
    複数特徴点のHOCBF制約の係数を計算（develop_dynamics.mdに基づく実装）
    
    Parameters:
    -----------
    drone : DynamicDrone
        2次系ドローン
    feature_points : list of FeaturePoint
        視野内の特徴点のリスト
    q : float, optional
        確率の閾値（デフォルトは0.5）
    gamma0 : float, optional
        CBFのゲイン0（デフォルトは0.1）
    gamma1 : float, optional
        CBFのゲイン1（デフォルトは0.1）
        
    Returns:
    --------
    C : ndarray, shape (1, 4) または shape (1, 6)
        制約行列 [C_f, C_tau]
    b : float
        制約の右辺値
    B : float
        安全集合の値
    B_dot : float
        安全集合の1階微分
    B_ddot_minus : float
        制御入力を除いた安全集合の2階微分
    """
    v = drone.v
    omega = drone.omega
    M = drone.M[0, 0]
    R = drone.T.R
    g = get_gravity()

    # カメラの方向ベクトル
    e_c = drone.camera_direction
    
    # 視野角の余弦
    cos_psi_F = np.cos(drone.fov_angle)
    
    # ドローンから見える特徴点を取得
    visible_features = []
    for fp in feature_points:
        if drone.is_point_visible(fp.position):
            visible_features.append(fp)
    
    # 視野内の特徴点がない場合
    if not visible_features:
        print("警告: 視野内に特徴点がありません")
        # 制約なしの場合と同様に扱う
        if drone.dynamics_model == 'holonomic_dynamics':
            C = np.zeros(6)
        else:
            C = np.zeros(4)
        return C, 0.0, 0.0, 0.0, 0.0
    
    # 安全集合の値（B）の計算
    # B_i = 1 - q - η_i
    # η_i = Π_{l∈L}(1-φ_i^l)
    eta_i = 1.0
    phi_values = []
    
    for fp in visible_features:
        # 特徴点の位置
        q_l = fp.position
        
        # βベクトル（特徴点の方向を表す単位ベクトル）
        beta_l = drone.get_beta_vector(q_l)
        
        # P_i^l = (β_l^T(p_i) R_i e_c - cos(Ψ_F)) / (1 - cos(Ψ_F))
        P_i_l = (beta_l.T @ R @ e_c - cos_psi_F) / (1 - cos_psi_F)
        
        # φ_i^l = P_i^l（視野内の特徴点の場合）
        phi_i_l = P_i_l
        phi_values.append(phi_i_l)
        
        # η_i = Π_{l∈L}(1-φ_i^l)
        eta_i *= (1 - phi_i_l)
    
    # B_i = 1 - q - η_i
    B = 1 - q - eta_i
    
    # 安全集合の1階微分の計算
    # Ḃ_i = -η̇_i
    # η̇_i = Σ_{l∈L}(Π_{k≠l}(1-φ_i^k))φ̇_i^l
    B_dot = 0.0
    
    for i, fp in enumerate(visible_features):
        # Π_{k≠l}(1-φ_i^k)
        prod_other = 1.0
        for j, phi in enumerate(phi_values):
            if i != j:
                prod_other *= (1 - phi)
        
        # 特徴点の位置
        q_l = fp.position
        
        # βベクトル（特徴点の方向を表す単位ベクトル）
        beta_l = drone.get_beta_vector(q_l)
        
        # 投影行列
        P_beta_l = np.eye(3) - np.outer(beta_l, beta_l)
        
        # 距離
        d_il = np.linalg.norm(q_l - drone.T.p)
        
        # φ̇_i^l = ⟨grad P_i^l, ξ_W⟩
        # grad_R P_i^l = (1/(1-cos ψ_F))(-β_l^T(p_i) R_i [e_c]_×)
        grad_R_P_i_l = (1 / (1 - cos_psi_F)) * (-beta_l.T @ R @ skew(e_c))
        
        # grad_p P_i^l = (1/(1-cos ψ_F))(-e_c^T R_i^T P_β_l/d)
        grad_p_P_i_l = (1 / (1 - cos_psi_F)) * (-e_c.T @ R.T @ P_beta_l / d_il)
        
        # φ̇_i^l = ⟨grad_R P_i^l, ω⟩ + ⟨grad_p P_i^l, v⟩
        phi_dot_i_l = grad_R_P_i_l @ omega + grad_p_P_i_l @ v
        
        # Ḃ_i += (Π_{k≠l}(1-φ_i^k))φ̇_i^l
        B_dot += prod_other * phi_dot_i_l
    
    # 安全集合の2階微分の計算（制御入力を除いた部分）
    # B̈_i = Σ_{l∈L}(-Σ_{j≠l}(Π_{m≠j,l}(1-φ_i^m))φ̇_i^jφ̇_i^l + (Π_{k≠l}(1-φ_i^k))φ̈_i^l)
    B_ddot_minus = 0.0
    
    # 制約行列の初期化
    if drone.dynamics_model == 'holonomic_dynamics':
        C_f = np.zeros(3)
        C_tau = np.zeros(3)
    else:
        C_f = 0.0
        C_tau = np.zeros(3)
    
    for i, fp_i in enumerate(visible_features):
        # Π_{k≠l}(1-φ_i^k)
        prod_other_i = 1.0
        for j, phi in enumerate(phi_values):
            if i != j:
                prod_other_i *= (1 - phi)
        
        # 特徴点の位置
        q_l_i = fp_i.position
        
        # βベクトル（特徴点の方向を表す単位ベクトル）
        beta_l_i = drone.get_beta_vector(q_l_i)
        
        # 投影行列
        P_beta_l_i = np.eye(3) - np.outer(beta_l_i, beta_l_i)
        
        # 距離
        d_il_i = np.linalg.norm(q_l_i - drone.T.p)
        
        # grad_R P_i^l = (1/(1-cos ψ_F))(-β_l^T(p_i) R_i [e_c]_×)
        grad_R_P_i_l_i = (1 / (1 - cos_psi_F)) * (-beta_l_i.T @ R @ skew(e_c))
        
        # grad_p P_i^l = (1/(1-cos ψ_F))(-e_c^T R_i^T P_β_l/d)
        grad_p_P_i_l_i = (1 / (1 - cos_psi_F)) * (-e_c.T @ R.T @ P_beta_l_i / d_il_i)
        
        # φ̇_i^l = ⟨grad_R P_i^l, ω⟩ + ⟨grad_p P_i^l, v⟩
        phi_dot_i_l_i = grad_R_P_i_l_i @ omega + grad_p_P_i_l_i @ v
        
        # -Σ_{j≠l}(Π_{m≠j,l}(1-φ_i^m))φ̇_i^jφ̇_i^l
        term1 = 0.0
        for j, fp_j in enumerate(visible_features):
            if i != j:
                # Π_{m≠j,l}(1-φ_i^m)
                prod_other_j = 1.0
                for k, phi in enumerate(phi_values):
                    if k != i and k != j:
                        prod_other_j *= (1 - phi)
                
                # 特徴点の位置
                q_l_j = fp_j.position
                
                # βベクトル（特徴点の方向を表す単位ベクトル）
                beta_l_j = drone.get_beta_vector(q_l_j)
                
                # 投影行列
                P_beta_l_j = np.eye(3) - np.outer(beta_l_j, beta_l_j)
                
                # 距離
                d_il_j = np.linalg.norm(q_l_j - drone.T.p)
                
                # grad_R P_i^l = (1/(1-cos ψ_F))(-β_l^T(p_i) R_i [e_c]_×)
                grad_R_P_i_l_j = (1 / (1 - cos_psi_F)) * (-beta_l_j.T @ R @ skew(e_c))
                
                # grad_p P_i^l = (1/(1-cos ψ_F))(-e_c^T R_i^T P_β_l/d)
                grad_p_P_i_l_j = (1 / (1 - cos_psi_F)) * (-e_c.T @ R.T @ P_beta_l_j / d_il_j)
                
                # φ̇_i^l = ⟨grad_R P_i^l, ω⟩ + ⟨grad_p P_i^l, v⟩
                phi_dot_i_l_j = grad_R_P_i_l_j @ omega + grad_p_P_i_l_j @ v
                
                term1 -= prod_other_j * phi_dot_i_l_j * phi_dot_i_l_i
        
        # φ̈_i^l = ⟨Hess P_i^l[ξ_W], ξ_W⟩ + ⟨grad P_i^l, ξ̇_W⟩
        # ⟨Hess P_i^l[ξ_W], ξ_W⟩ = ⟨Hess_p P_i^l[v], v⟩ + ⟨Hess_p P_i^l[v], ω⟩ + ⟨Hess_R P_i^l[ω], v⟩ + ⟨Hess_R P_i^l[ω], ω⟩
        
        # ヘッシアン項の計算
        z = R @ e_c
        
        # ⟨Hess_p P_i^l[v], v⟩
        term_hess_p_v = (1 / (1 - cos_psi_F)) * (-v.T @ R.T @ ((beta_l_i @ (z.T @ P_beta_l_i) + (z.T @ beta_l_i) * P_beta_l_i + P_beta_l_i @ z @ beta_l_i.T) / d_il_i**2) @ R @ v - (z.T @ P_beta_l_i @ R @ skew(omega) / d_il_i) @ v)
        
        # ⟨Hess_p P_i^l[v], ω⟩ = ⟨Hess_R P_i^l[ω], v⟩
        term_hess_p_omega = (1 / (1 - cos_psi_F)) * (v.T @ R.T @ P_beta_l_i @ R @ skew(e_c) @ omega / d_il_i)
        
        # ⟨Hess_R P_i^l[ω], ω⟩
        term_hess_R_omega = (1 / (1 - cos_psi_F)) * (-beta_l_i.T @ (R @ skew(omega)) @ skew(e_c) @ omega)
        
        # ⟨Hess P_i^l[ξ_W], ξ_W⟩
        term_hess = term_hess_p_v + term_hess_p_omega + term_hess_p_omega + term_hess_R_omega
        
        # ⟨grad P_i^l, ξ̇_W⟩ = ⟨grad_p P_i^l, v̇⟩ + ⟨grad_R P_i^l, ω̇⟩
        # v̇ = v×ω - gR^Te_z + M^{-1}f
        # ω̇ = J^{-1}τ
        
        # 重力項
        A_g = np.cross(v, omega) - np.dot(R.T, g)
        
        # ⟨grad_p P_i^l, A_g⟩
        term_grad_p_Ag = grad_p_P_i_l_i @ A_g
        
        # 制約行列の計算
        # C_f: 推力に関する係数
        if drone.dynamics_model == 'holonomic_dynamics':
            # ホロノミック系の場合（3次元ベクトル）
            C_f_i = prod_other_i * grad_p_P_i_l_i * (1 / M)
            C_f += C_f_i
        else:
            # 非ホロノミック系の場合（スカラー）
            C_f_i = prod_other_i * grad_p_P_i_l_i @ np.array([0, 0, 1]) / M
            C_f += C_f_i
        
        # C_tau: トルクに関する係数
        C_tau_i = prod_other_i * grad_R_P_i_l_i @ np.linalg.inv(drone.J)
        C_tau += C_tau_i
        
        # 項の合計
        B_ddot_minus += term1 + prod_other_i * term_hess + prod_other_i * term_grad_p_Ag
    
    # 制約行列の構築
    if drone.dynamics_model == 'holonomic_dynamics':
        # ホロノミック系の場合
        C = np.zeros(6)
        C[0:3] = C_f
        C[3:6] = C_tau
    else:
        # 非ホロノミック系の場合
        C = np.zeros(4)
        C[0] = C_f
        C[1:4] = C_tau
    
    # 右辺の計算
    b = B_ddot_minus + (gamma0 + gamma1) * B_dot + (gamma0 * gamma1) * B
    
    return C, b, B, B_dot, B_ddot_minus

def compute_position_tracking_acceleration_control(drone, p_des, K_p=1.0, K_d=0.5):
    """
    目標位置に追従するための加速度制御入力を計算
    
    Parameters:
    -----------
    drone : DynamicDrone
        2次系ドローン
    p_des : array_like, shape (3,)
        目標位置
    K_p : float, optional
        位置制御のゲイン（デフォルトは1.0）
    K_d : float, optional
        速度制御のゲイン（デフォルトは0.5）
        
    Returns:
    --------
    u : ndarray, shape (4,) または shape (6,)
        制御入力 [f, tau]
        dynamics_model='dynamics'の場合: shape (4,), [f, tau]
            f: 推力（スカラー）
            tau: トルク（3次元ベクトル）
        dynamics_model='holonomic_dynamics'の場合: shape (6,), [f, tau]
            f: 推力（3次元ベクトル）
            tau: トルク（3次元ベクトル）
    """
    # 現在位置と目標位置の誤差
    p_error = p_des - drone.T.p
    
    # 速度誤差（目標速度は0と仮定）
    v_error = -drone.v
    
    # PD制御則
    a_des = K_p * p_error + K_d * v_error
    
    # 重力補償
    g = get_gravity()
    
    # 必要な加速度（ワールド座標系）
    a_world = a_des + g
    
    # ボディ座標系に変換
    a_body = drone.T.R.T @ a_world
    
    # 姿勢制御（簡易的な実装）
    # 目標の姿勢は上向き（z軸）を維持
    R_des = np.eye(3)
    
    # 回転誤差
    R_error = R_des.T @ drone.T.R
    
    # 軸角表現に変換
    from scipy.spatial.transform import Rotation
    r = Rotation.from_matrix(R_error)
    rotvec = r.as_rotvec()
    
    # PD制御則（角速度フィードバック）
    tau = -K_p * rotvec - K_d * drone.omega
    
    # 動力学モデルに応じて処理を分岐
    if hasattr(drone, 'dynamics_model') and drone.dynamics_model == 'holonomic_dynamics':
        # ホロノミック系の場合
        # 推力（3次元ベクトル）
        f = drone.M[0, 0] * a_body
        
        # 制御入力
        u = np.zeros(6)
        u[0:3] = f
        u[3:6] = tau
    else:
        # 非ホロノミック系の場合
        # 推力（z軸方向のみ）
        f = drone.M[0, 0] * a_body[2]
        
        # 制御入力
        u = np.zeros(4)
        u[0] = f
        u[1:4] = tau
    
    return u




def compute_cbf_constraint_coefficients(drone1, drone2, feature_points, q=0.5, d=1.0):
    """
    CBF制約の係数を計算
    
    Parameters:
    -----------
    drone1 : Drone
        1つ目のドローン
    drone2 : Drone
        2つ目のドローン
    feature_points : list of FeaturePoint
        環境内の特徴点のリスト
    q : float, optional
        確率の閾値（デフォルトは0.5）
    d : float, optional
        距離パラメータ（デフォルトは1.0）
        
    Returns:
    --------
    alpha_omega : ndarray, shape (3,)
        ドローン1の角速度に関する制約係数
    beta_omega : ndarray, shape (3,)
        ドローン2の角速度に関する制約係数
    alpha_v : ndarray, shape (3,)
        ドローン1の速度に関する制約係数
    beta_v : ndarray, shape (3,)
        ドローン2の速度に関する制約係数
    gamma : float
        CBF制約の右辺値
    """
    # カメラの方向ベクトル
    e_c = drone1.camera_direction  # 両方のドローンで同じと仮定
    
    # 視野角の余弦
    cos_psi_F = np.cos(drone1.fov_angle)
    
    # ドローンから見える特徴点を取得
    visible_features = []
    for fp in feature_points:
        # 両方のドローンから見える特徴点のみを考慮
        if drone1.is_point_visible(fp.position) and drone2.is_point_visible(fp.position):
            visible_features.append(fp)
    
    # 共有視野内の特徴点がない場合
    if not visible_features:
        return np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), -q
    
    # 各特徴点の確率を計算
    phi_values = []
    for fp in visible_features:
        P_i_l = drone1.get_observation_probability(fp.position)
        P_j_l = drone2.get_observation_probability(fp.position)
        phi_values.append(P_i_l * P_j_l)
    
    # 安全集合の値（gamma）の計算
    prod_term_all = 1.0
    for phi in phi_values:
        prod_term_all *= (1.0 - phi)
    
    gamma_val = 1.0 - q - prod_term_all
    
    # 制約係数の初期化
    alpha_omega = np.zeros(3)
    beta_omega = np.zeros(3)
    alpha_v = np.zeros(3)
    beta_v = np.zeros(3)
    
    # 各特徴点について制約係数を計算
    for i, fp in enumerate(visible_features):
        # 他の特徴点の確率の積を計算
        prod_other = 1.0
        for j, phi in enumerate(phi_values):
            if i != j:
                prod_other *= (1.0 - phi)
        
        # 特徴点の位置
        q_l = fp.position
        
        # βベクトル（特徴点の方向を表す単位ベクトル）
        beta_l_i = drone1.get_beta_vector(q_l)
        beta_l_j = drone2.get_beta_vector(q_l)
        
        # 観測確率
        P_i_l = drone1.get_observation_probability(q_l)
        P_j_l = drone2.get_observation_probability(q_l)
        
        # P_βの計算（投影行列）
        P_beta_l_i = np.eye(3) - np.outer(beta_l_i, beta_l_i)
        P_beta_l_j = np.eye(3) - np.outer(beta_l_j, beta_l_j)
        
        # 係数の計算（pcl_ccbf.mdの式(43)に基づく）
        # alpha_omega
        alpha_omega += prod_other * P_j_l * beta_l_i.T @ drone1.T.R @ skew(e_c) / (1.0 - cos_psi_F)
        
        # beta_omega
        beta_omega += prod_other * P_i_l * beta_l_j.T @ drone2.T.R @ skew(e_c) / (1.0 - cos_psi_F)
        
        # alpha_v
        alpha_v += prod_other * P_j_l * e_c.T @ drone1.T.R.T @ P_beta_l_i @ drone1.T.R / ((1.0 - cos_psi_F) * d)
        
        # beta_v
        beta_v += prod_other * P_i_l * e_c.T @ drone2.T.R.T @ P_beta_l_j @ drone2.T.R / ((1.0 - cos_psi_F) * d)
    
    return alpha_omega, beta_omega, alpha_v, beta_v, gamma_val
