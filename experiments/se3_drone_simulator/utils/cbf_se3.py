"""
SE(3)上の制約付き最適化問題（Control Barrier Function）を実装するモジュール
"""
import numpy as np
from .se3 import SE3, skew
from .drone import Drone, FeaturePoint

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
        position = random_position(min_val=-3.0, max_val=3.0)
        
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
