"""
SE(3)上の制約付き最適化問題（Control Barrier Function）を実装するモジュール
"""
import numpy as np
from scipy import optimize
import cvxopt
import cvxopt.solvers
from se3 import SE3, skew
from drone import Drone, FeaturePoint

# cvxoptのソルバーの設定（出力を抑制）
cvxopt.solvers.options['show_progress'] = False


def compute_barrier_function(drone1, drone2, feature_points, q=0.5):
    """
    2つのドローン間の安全集合（B_{ij}）を計算
    
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
        
    Returns:
    --------
    barrier_value : float
        安全集合の値（B_{ij}）
        正の値は安全条件を満たす
        負の値は安全条件を満たさない
    """
    # 各特徴点についての確率を計算
    probs = []
    for fp in feature_points:
        prob = drone1.calculate_cofov_probability(drone2, fp.position)
        probs.append(prob)
    
    # 安全集合の計算
    # B_{ij} = 1 - q - \prod_{l \in \mathcal{L}} (1 - \phi_{ij}^l)
    if not probs:
        return -q  # 特徴点がない場合
    
    prod_term = 1.0
    for prob in probs:
        prod_term *= (1.0 - prob)
    
    return 1.0 - q - prod_term


def compute_barrier_function_gradient(drone1, drone2, feature_points, q=0.5, epsilon=1e-6):
    """
    安全集合（B_{ij}）の勾配を数値的に計算
    
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
    epsilon : float, optional
        数値微分の刻み幅（デフォルトは1e-6）
        
    Returns:
    --------
    gradient : ndarray, shape (12,)
        安全集合の勾配
        [∂B/∂R1_vec, ∂B/∂p1, ∂B/∂R2_vec, ∂B/∂p2]
        R1_vecとR2_vecは回転行列を9次元ベクトルに変換したもの
    """
    # 現在の値
    current_value = compute_barrier_function(drone1, drone2, feature_points, q)
    
    # 勾配を格納する配列
    gradient = np.zeros(12)
    
    # ドローン1の回転行列の勾配（簡易的に各要素について計算）
    for i in range(3):
        for j in range(3):
            # 回転行列の(i,j)要素を微小変化
            R1_perturbed = drone1.T.R.copy()
            R1_perturbed[i, j] += epsilon
            
            # 変化後のドローン
            drone1_perturbed = Drone(SE3(R1_perturbed, drone1.T.p), 
                                    drone1.camera_direction, 
                                    drone1.fov_angle)
            
            # 変化後の値
            perturbed_value = compute_barrier_function(drone1_perturbed, drone2, feature_points, q)
            
            # 勾配の計算
            gradient[i*3 + j] = (perturbed_value - current_value) / epsilon
    
    # ドローン1の位置の勾配
    for i in range(3):
        # 位置の第i成分を微小変化
        p1_perturbed = drone1.T.p.copy()
        p1_perturbed[i] += epsilon
        
        # 変化後のドローン
        drone1_perturbed = Drone(SE3(drone1.T.R, p1_perturbed), 
                                drone1.camera_direction, 
                                drone1.fov_angle)
        
        # 変化後の値
        perturbed_value = compute_barrier_function(drone1_perturbed, drone2, feature_points, q)
        
        # 勾配の計算
        gradient[9 + i] = (perturbed_value - current_value) / epsilon
    
    return gradient


def compute_barrier_function_derivative(drone1, drone2, feature_points, xi1, xi2, q=0.5, dt=0.01):
    """
    安全集合（B_{ij}）の時間微分を計算
    
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
    derivative : float
        安全集合の時間微分
    """
    # 現在の値
    current_value = compute_barrier_function(drone1, drone2, feature_points, q)
    
    # ドローンを速度入力に基づいて更新
    drone1_next = Drone(SE3(drone1.T.R.copy(), drone1.T.p.copy()), 
                        drone1.camera_direction.copy(), drone1.fov_angle)
    drone2_next = Drone(SE3(drone2.T.R.copy(), drone2.T.p.copy()), 
                        drone2.camera_direction.copy(), drone2.fov_angle)
    
    drone1_next.update(xi1, dt)
    drone2_next.update(xi2, dt)
    
    # 更新後の値
    next_value = compute_barrier_function(drone1_next, drone2_next, feature_points, q)
    
    # 時間微分の計算
    return (next_value - current_value) / dt


def solve_cbf_qp(drone1, drone2, feature_points, xi1_des, xi2_des, q=0.5, gamma=0.1):
    """
    Control Barrier Function (CBF)に基づく二次計画問題を解く
    
    Parameters:
    -----------
    drone1 : Drone
        1つ目のドローン
    drone2 : Drone
        2つ目のドローン
    feature_points : list of FeaturePoint
        環境内の特徴点のリスト
    xi1_des : array_like, shape (6,)
        ドローン1の目標速度入力 [omega, v]
    xi2_des : array_like, shape (6,)
        ドローン2の目標速度入力 [omega, v]
    q : float, optional
        確率の閾値（デフォルトは0.5）
    gamma : float, optional
        CBFのゲイン（デフォルトは0.1）
        
    Returns:
    --------
    xi1_safe : ndarray, shape (6,)
        ドローン1の安全な速度入力
    xi2_safe : ndarray, shape (6,)
        ドローン2の安全な速度入力
    """
    # 現在の安全集合の値
    b = compute_barrier_function(drone1, drone2, feature_points, q)
    
    # 安全集合の値が既に負の場合は、安全な入力を見つけるのが難しいため、
    # 目標入力をそのまま返す（実際のアプリケーションではより洗練された対応が必要）
    if b < 0:
        print(f"警告: 安全集合の値が負です (B={b:.4f})")
        return xi1_des, xi2_des
    
    # 最適化問題の定式化
    def objective(x):
        """
        目標入力からの偏差を最小化する目的関数
        """
        xi1 = x[:6]
        xi2 = x[6:]
        
        # L2ノルムの二乗
        return np.sum((xi1 - xi1_des)**2) + np.sum((xi2 - xi2_des)**2)
    
    def constraint(x):
        """
        CBF制約: ドット{B} + γB ≥ 0
        """
        xi1 = x[:6]
        xi2 = x[6:]
        
        # 安全集合の時間微分
        b_dot = compute_barrier_function_derivative(drone1, drone2, feature_points, xi1, xi2, q)
        
        # CBF制約
        return b_dot + gamma * b
    
    # 初期値（目標入力）
    x0 = np.concatenate([xi1_des, xi2_des])
    
    # 制約条件
    constraints = [{'type': 'ineq', 'fun': constraint}]
    
    # 最適化問題を解く
    result = optimize.minimize(objective, x0, constraints=constraints, method='SLSQP')
    
    # 最適解
    if result.success:
        xi1_safe = result.x[:6]
        xi2_safe = result.x[6:]
        return xi1_safe, xi2_safe
    else:
        print(f"警告: 最適化に失敗しました: {result.message}")
        return xi1_des, xi2_des


def sample_safe_configuration(simulator, fov_angle, min_barrier_value=0.0, max_attempts=1000):
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
        barrier_value = compute_barrier_function(drone1, drone2, simulator.feature_points)
        
        # 進捗表示（100回ごと）
        if attempt % 100 == 0 and attempt > 0:
            print(f"  {attempt}回試行: 安全集合の値 = {barrier_value:.4f}")
        
        # 安全集合の値が指定値以上であれば終了
        if barrier_value >= min_barrier_value:
            print(f"成功! {attempt + 1}回目の試行で安全集合の値が{barrier_value:.4f}になりました")
            print(f"ドローン2の位置: {position}")
            print(f"ドローン2の回転行列:\n{rot_matrix}")
            return True, attempt + 1, barrier_value
    
    # 最大試行回数に達しても見つからなかった場合
    print(f"警告: {max_attempts}回の試行で安全集合の値が{min_barrier_value}以上になりませんでした")
    print(f"最後の試行での安全集合の値: {barrier_value:.4f}")
    return False, max_attempts, barrier_value


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
    K_r : float, optional
        姿勢制御のゲイン（デフォルトは0.5）
        
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
    
    # 目標方向ベクトル（目標位置への方向）
    # direction_to_target = p_error / (np.linalg.norm(p_error) + 1e-6)
    
    # 現在のカメラ方向（ボディフレーム）
    # current_direction = drone.camera_direction
    
    # ワールドフレームでの現在のカメラ方向
    # world_direction = drone.T.R @ current_direction
    
    # 目標方向と現在方向の外積で回転軸を計算
    # rotation_axis = np.cross(world_direction, direction_to_target)
    
    # 回転の大きさ（内積から計算）
    # rotation_magnitude = np.arccos(np.clip(
    #     np.dot(world_direction, direction_to_target), -1.0, 1.0))
    
    # 角速度（回転軸と大きさから）
    # omega = K_r * rotation_axis * rotation_magnitude
    omega = np.zeros(3)
    
    # 制御入力
    xi = np.concatenate([omega, vb])
    
    return xi
    

def solve_position_tracking_cbf_qp(drone1, drone2, feature_points, p1_des, p2_des, K_p=1.0, q=0.5, gamma=0.1):
    """
    目標位置に追従するためのCBF制約付きQPを解く
    
    Parameters:
    -----------
    drone1, drone2 : Drone
        ドローン
    feature_points : list of FeaturePoint
        特徴点のリスト
    p1_des, p2_des : array_like, shape (3,)
        ドローン1,2の目標位置
    K_p : float, optional
        位置制御のゲイン
    q, gamma : float, optional
        CBFのパラメータ
        
    Returns:
    --------
    xi1_safe, xi2_safe : ndarray, shape (6,)
        安全な制御入力
    """
    # 位置追従のための目標制御入力を計算
    xi1_des = compute_position_tracking_control(drone1, p1_des, K_p)
    xi2_des = compute_position_tracking_control(drone2, p2_des, K_p)
    
    # 既存のCBF-QPを使用して安全な制御入力を計算
    xi1_safe, xi2_safe = solve_cbf_qp(drone1, drone2, feature_points, xi1_des, xi2_des, q, gamma)
    
    return xi1_safe, xi2_safe


def generate_position_tracking_control_inputs(simulator, p1_des, p2_des, K_p=1.0, q=0.5, gamma=0.1):
    """
    目標位置追従のための安全な制御入力を生成
    
    Parameters:
    -----------
    simulator : Simulator
        シミュレータ（2機のドローンが追加されている必要がある）
    p1_des, p2_des : array_like, shape (3,)
        ドローン1,2の目標位置
    K_p : float, optional
        位置制御のゲイン
    q, gamma : float, optional
        CBFのパラメータ
        
    Returns:
    --------
    xi1_safe, xi2_safe : ndarray, shape (6,)
        安全な制御入力
    """
    # 2機のドローンが存在することを確認
    if len(simulator.drones) < 2:
        raise ValueError("2機のドローンがシミュレータに追加されていません")
    
    drone1 = simulator.drones[0]
    drone2 = simulator.drones[1]
    
    # 目標位置追従のためのCBF制約付きQPを解く
    xi1_safe, xi2_safe = solve_position_tracking_cbf_qp(
        drone1, drone2, simulator.feature_points, p1_des, p2_des, K_p, q, gamma)
    
    return xi1_safe, xi2_safe


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
        return np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), -q
    
    # 安全集合の値（gamma）の計算
    prod_term = 1.0
    for prob in probs:
        prod_term *= (1.0 - prob)
    
    gamma_val = 1.0 - q - prod_term
    
    # 制約係数の初期化
    alpha_omega = np.zeros(3)
    beta_omega = np.zeros(3)
    alpha_v = np.zeros(3)
    beta_v = np.zeros(3)
    
    # 各特徴点について制約係数を計算
    for i, fp in enumerate(visible_features):
        # 特徴点の位置
        q_l = fp.position
        
        # βベクトル（特徴点の方向を表す単位ベクトル）
        beta_l_i = drone1.get_beta_vector(q_l)
        beta_l_j = drone2.get_beta_vector(q_l)
        
        # 観測確率
        P_i_l = drone1.get_observation_probability(q_l)
        P_j_l = drone2.get_observation_probability(q_l)
        
        # 他の特徴点の確率の積
        prod_other = 1.0
        for j, prob in enumerate(probs):
            if j != i:
                prod_other *= (1.0 - prob)
        
        # P_βの計算（投影行列）
        P_beta_l_i = np.eye(3) - np.outer(beta_l_i, beta_l_i)
        P_beta_l_j = np.eye(3) - np.outer(beta_l_j, beta_l_j)
        
        # 係数の計算
        # alpha_omega
        alpha_omega_term = prod_other * P_j_l * beta_l_i.T @ drone1.T.R @ skew(e_c) / (1.0 - cos_psi_F)
        alpha_omega += alpha_omega_term
        
        # beta_omega
        beta_omega_term = prod_other * P_i_l * beta_l_j.T @ drone2.T.R @ skew(e_c) / (1.0 - cos_psi_F)
        beta_omega += beta_omega_term
        
        # alpha_v
        alpha_v_term = prod_other * P_j_l * e_c.T @ drone1.T.R.T @ P_beta_l_i / ((1.0 - cos_psi_F) * d)
        alpha_v += alpha_v_term
        
        # beta_v
        beta_v_term = prod_other * P_i_l * e_c.T @ drone2.T.R.T @ P_beta_l_j / ((1.0 - cos_psi_F) * d)
        beta_v += beta_v_term
    
    return alpha_omega, beta_omega, alpha_v, beta_v, gamma_val


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
        Q2 = np.eye(6)
    
    # 位置誤差
    e_i = p_des - drone.T.p - h * drone.T.R @ np.zeros(3)  # 現在の速度を0と仮定
    
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


def solve_single_drone_qp(drone, p_des, xi_des=None, use_cbf=True, Q1=None, Q2=None, h=0.01, cbf_constraints=None):
    """
    単一ドローンのQP問題を解く
    
    Parameters:
    -----------
    drone : Drone
        ドローン
    p_des : array_like, shape (3,)
        目標位置
    xi_des : array_like, shape (6,), optional
        目標制御入力（指定しない場合は位置追従制御から計算）
    use_cbf : bool, optional
        CBF制約を使用するかどうか（デフォルトはTrue）
    Q1 : array_like, optional
        位置誤差の重み行列
    Q2 : array_like, optional
        制御入力の重み行列
    h : float, optional
        時間ステップ
    cbf_constraints : tuple, optional
        CBF制約の係数 (G, h)
        G: 制約行列
        h: 制約の右辺
        
    Returns:
    --------
    xi_safe : ndarray, shape (6,)
        安全な制御入力
    """
    # 目標制御入力が指定されていない場合は位置追従制御から計算
    if xi_des is None:
        xi_des = compute_position_tracking_control(drone, p_des)
    
    # 目的関数の係数を計算
    H, f = compute_objective_coefficients(drone, p_des, Q1, Q2, h)
    
    # 目標入力からの偏差を最小化する目的関数
    # J = 1/2 * (xi - xi_des)^T * (xi - xi_des)
    # = 1/2 * xi^T * xi - xi_des^T * xi + 1/2 * xi_des^T * xi_des
    # = 1/2 * xi^T * I * xi - xi_des^T * xi + const
    # cvxoptの形式: 1/2 * x^T * P * x + q^T * x
    P = cvxopt.matrix(np.eye(6))
    q = cvxopt.matrix(-xi_des)
    
    # 制約条件がない場合は解析解を使用
    if not use_cbf or cbf_constraints is None:
        # 解析解: xi = xi_des
        return xi_des
    
    # CBF制約を使用する場合は、制約条件を設定
    G, h_val = cbf_constraints
    G_cvx = cvxopt.matrix(G)
    h_cvx = cvxopt.matrix(h_val)
    
    # QP問題を解く
    try:
        sol = cvxopt.solvers.qp(P, q, G_cvx, h_cvx)
        
        # 解が見つかった場合
        if sol['status'] == 'optimal':
            xi_safe = np.array(sol['x']).flatten()
            return xi_safe
        else:
            print(f"警告: QP問題の解決に失敗しました: {sol['status']}")
            return xi_des
    except Exception as e:
        print(f"警告: QP問題の解決中にエラーが発生しました: {e}")
        return xi_des


def compute_single_drone_cbf_constraints(drone, obstacles, safety_distance=0.5, gamma=0.1):
    """
    単一ドローンのCBF制約を計算
    
    Parameters:
    -----------
    drone : Drone
        ドローン
    obstacles : list of ndarray
        障害物の位置のリスト
    safety_distance : float, optional
        安全距離（デフォルトは0.5）
    gamma : float, optional
        CBFのゲイン（デフォルトは0.1）
        
    Returns:
    --------
    G : ndarray
        制約行列
    h : ndarray
        制約の右辺
    """
    if not obstacles:
        # 障害物がない場合は制約なし
        return None
    
    num_obstacles = len(obstacles)
    
    # 制約行列と右辺の初期化
    G = np.zeros((num_obstacles, 6))
    h = np.zeros(num_obstacles)
    
    for i, obstacle_pos in enumerate(obstacles):
        # ドローンと障害物の距離
        diff = drone.T.p - obstacle_pos
        dist = np.linalg.norm(diff)
        
        # 安全集合: B(x) = ||p - p_obs||^2 - d_safe^2
        b = dist**2 - safety_distance**2
        
        # 勾配: ∇B(x) = 2 * (p - p_obs)
        grad_b = 2 * diff
        
        # CBF制約: ∇B(x)^T * v >= -γ * B(x)
        # ボディフレームに変換
        grad_b_body = drone.T.R.T @ grad_b
        
        # 制約行列: [0, grad_b_body]
        G[i, 3:] = -grad_b_body  # 負の符号は制約の向きによる
        
        # 制約の右辺: -γ * B(x)
        h[i] = gamma * b
    
    return G, h


def solve_multi_drone_cbf_qp(drone1, drone2, feature_points, xi1_des, xi2_des, q=0.5, gamma0=0.1, c1=0.5, c2=0.5):
    """
    複数ドローンのCBF制約付きQP問題を解く（ccbf.mdの数式に基づく実装）
    
    Parameters:
    -----------
    drone1 : Drone
        1つ目のドローン
    drone2 : Drone
        2つ目のドローン
    feature_points : list of FeaturePoint
        環境内の特徴点のリスト
    xi1_des : array_like, shape (6,)
        ドローン1の目標速度入力 [omega, v]
    xi2_des : array_like, shape (6,)
        ドローン2の目標速度入力 [omega, v]
    q : float, optional
        確率の閾値（デフォルトは0.5）
    gamma0 : float, optional
        CBFのゲイン（デフォルトは0.1）
    c1, c2 : float, optional
        CBF制約の重み（デフォルトはそれぞれ0.5）
        
    Returns:
    --------
    xi1_safe : ndarray, shape (6,)
        ドローン1の安全な速度入力
    xi2_safe : ndarray, shape (6,)
        ドローン2の安全な速度入力
    """
    # CBF制約の係数を計算
    alpha_omega, beta_omega, alpha_v, beta_v, gamma_val = compute_cbf_constraint_coefficients(
        drone1, drone2, feature_points, q)
    
    # 安全集合の値が既に負の場合は、安全な入力を見つけるのが難しいため、
    # 目標入力をそのまま返す
    if gamma_val < 0:
        print(f"警告: 安全集合の値が負です (gamma={gamma_val:.4f})")
        return xi1_des, xi2_des
    
    # 目的関数の係数
    # 簡単のため、目標入力からの偏差を最小化する二次形式を使用
    P = cvxopt.matrix(np.eye(12))
    q = cvxopt.matrix(np.concatenate([-xi1_des, -xi2_des]))
    
    # 制約行列の構築
    # [alpha_omega, 0, beta_omega, 0; 0, alpha_v, 0, beta_v] * [omega1; v1; omega2; v2] <= [c1; c2] * gamma0 * gamma_val
    A = np.zeros((2, 12))
    A[0, :3] = alpha_omega
    A[0, 6:9] = beta_omega
    A[1, 3:6] = alpha_v
    A[1, 9:] = beta_v
    
    # 制約の右辺
    b = np.array([c1, c2]) * gamma0 * gamma_val
    
    # cvxoptの形式に変換
    G = cvxopt.matrix(-A)
    h = cvxopt.matrix(b)
    
    # QP問題を解く
    sol = cvxopt.solvers.qp(P, q, G, h)
    
    # 解が見つかった場合
    if sol['status'] == 'optimal':
        x_opt = np.array(sol['x']).flatten()
        xi1_safe = x_opt[:6]
        xi2_safe = x_opt[6:]
        return xi1_safe, xi2_safe
    else:
        print(f"警告: QP問題の解決に失敗しました: {sol['status']}")
        return xi1_des, xi2_des


def generate_safe_control_inputs(simulator, xi1_des, xi2_des, q=0.5, gamma=0.1, use_new_cbf=True):
    """
    CBFに基づいて安全な制御入力を生成
    
    Parameters:
    -----------
    simulator : Simulator
        シミュレータ（2機のドローンが追加されている必要がある）
    xi1_des : array_like, shape (6,)
        ドローン1の目標速度入力 [omega, v]
    xi2_des : array_like, shape (6,)
        ドローン2の目標速度入力 [omega, v]
    q : float, optional
        確率の閾値（デフォルトは0.5）
    gamma : float, optional
        CBFのゲイン（デフォルトは0.1）
    use_new_cbf : bool, optional
        新しいCBF実装を使用するかどうか（デフォルトはTrue）
        
    Returns:
    --------
    xi1_safe : ndarray, shape (6,)
        ドローン1の安全な速度入力
    xi2_safe : ndarray, shape (6,)
        ドローン2の安全な速度入力
    """
    # 2機のドローンが存在することを確認
    if len(simulator.drones) < 2:
        raise ValueError("2機のドローンがシミュレータに追加されていません")
    
    drone1 = simulator.drones[0]
    drone2 = simulator.drones[1]
    
    # CBFに基づく二次計画問題を解く
    if use_new_cbf:
        # 新しいCBF実装を使用
        xi1_safe, xi2_safe = solve_multi_drone_cbf_qp(
            drone1, drone2, simulator.feature_points, xi1_des, xi2_des, q, gamma)
    else:
        # 既存のCBF実装を使用
        xi1_safe, xi2_safe = solve_cbf_qp(
            drone1, drone2, simulator.feature_points, xi1_des, xi2_des, q, gamma)
    
    return xi1_safe, xi2_safe
