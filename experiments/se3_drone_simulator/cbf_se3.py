"""
SE(3)上の制約付き最適化問題（Control Barrier Function）を実装するモジュール
"""
import numpy as np
from scipy import optimize
from se3 import SE3, skew
from drone import Drone, FeaturePoint


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


def generate_safe_control_inputs(simulator, xi1_des, xi2_des, q=0.5, gamma=0.1):
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
    xi1_safe, xi2_safe = solve_cbf_qp(drone1, drone2, simulator.feature_points, 
                                      xi1_des, xi2_des, q, gamma)
    
    return xi1_safe, xi2_safe
