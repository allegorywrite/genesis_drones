"""
cbf_se3.pyのアルゴリズム（特に制約付き最適化）をテストするスクリプト
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils.se3 import SE3
from utils.drone import Drone, FeaturePoint
from utils.simulator import Simulator
from utils.cbf_se3 import (
    compute_barrier_function,
    compute_barrier_function_gradient,
    compute_barrier_function_derivative,
    solve_cbf_qp,
    sample_safe_configuration,
    generate_safe_control_inputs
)


def test_compute_barrier_function():
    """
    compute_barrier_function関数のテスト
    """
    print("=== compute_barrier_function関数のテスト ===")
    
    # シミュレータの初期化
    simulator = Simulator(dt=0.1)
    
    # ドローン1: 原点、単位姿勢、視野角60度
    drone1 = Drone(fov_angle=np.pi/6)
    simulator.add_drone(drone1)
    
    # 特徴点の追加（格子状）
    n_points = 3
    for x in np.linspace(-3, 3, n_points):
        for y in np.linspace(-3, 3, n_points):
            for z in np.linspace(-3, 3, n_points):
                if abs(x) > 1 or abs(y) > 1 or abs(z) > 1:  # 中心付近は除外
                    fp = FeaturePoint([x, y, z])
                    simulator.add_feature_point(fp)
    
    # ドローン2の位置を変えながら安全集合の値を計算
    positions = []
    barrier_values = []
    
    # x軸に沿って位置を変える
    for x in np.linspace(1.0, 3.0, 10):
        # ドローン2: x軸上の位置、単位姿勢、視野角60度
        drone2 = Drone(SE3(p=np.array([x, 0.0, 0.0])), fov_angle=np.pi/6)
        
        # 安全集合の値を計算
        barrier_value = compute_barrier_function(drone1, drone2, simulator.feature_points)
        
        positions.append(x)
        barrier_values.append(barrier_value)
        
        print(f"ドローン2の位置: [{x}, 0, 0], 安全集合の値: {barrier_value:.4f}")
    
    # 結果をプロット
    plt.figure(figsize=(10, 6))
    plt.plot(positions, barrier_values, 'b-', marker='o')
    plt.axhline(y=0, color='r', linestyle='--', label='Safety Boundary')
    plt.xlabel('Drone 2 X Position')
    plt.ylabel('Barrier Function Value B_{ij}')
    plt.title('Barrier Function Value vs. Drone 2 Position')
    plt.grid(True)
    plt.legend()
    plt.savefig('barrier_function_test.png')
    plt.close()
    
    print("Results saved to barrier_function_test.png")
    print()


def test_compute_barrier_function_gradient():
    """
    compute_barrier_function_gradient関数のテスト
    """
    print("=== compute_barrier_function_gradient関数のテスト ===")
    
    # シミュレータの初期化
    simulator = Simulator(dt=0.1)
    
    # ドローン1: 原点、単位姿勢、視野角60度
    drone1 = Drone(fov_angle=np.pi/6)
    
    # ドローン2: x軸上の位置、単位姿勢、視野角60度
    drone2 = Drone(SE3(p=np.array([1.5, 0.0, 0.0])), fov_angle=np.pi/6)
    
    # 特徴点の追加（ドローン1とドローン2の間に配置して共有視野に入るようにする）
    feature_points = [
        FeaturePoint([0.75, 0.0, 0.0]),  # ドローン1とドローン2の中間
        FeaturePoint([0.75, 0.5, 0.0]),  # 少し上
        FeaturePoint([0.75, -0.5, 0.0])  # 少し下
    ]
    
    # 勾配を計算
    gradient = compute_barrier_function_gradient(drone1, drone2, feature_points)
    
    print("安全集合の勾配:")
    print(f"ドローン1の回転行列の勾配: {gradient[:9]}")
    print(f"ドローン1の位置の勾配: {gradient[9:12]}")
    
    # 数値的に勾配を検証
    epsilon = 1e-4
    
    # ドローン1のx座標を微小変化させる
    drone1_perturbed = Drone(SE3(p=np.array([epsilon, 0.0, 0.0])), fov_angle=np.pi/6)
    
    # 変化前と変化後の安全集合の値
    value_before = compute_barrier_function(drone1, drone2, feature_points)
    value_after = compute_barrier_function(drone1_perturbed, drone2, feature_points)
    
    # 数値微分
    numerical_gradient_x = (value_after - value_before) / epsilon
    
    print(f"x方向の数値勾配: {numerical_gradient_x:.6f}")
    print(f"x方向の計算勾配: {gradient[9]:.6f}")
    print(f"差: {abs(numerical_gradient_x - gradient[9]):.6f}")
    print()


def test_compute_barrier_function_derivative():
    """
    compute_barrier_function_derivative関数のテスト
    """
    print("=== compute_barrier_function_derivative関数のテスト ===")
    
    # シミュレータの初期化
    simulator = Simulator(dt=0.1)
    
    # ドローン1: 原点、単位姿勢、視野角60度
    drone1 = Drone(fov_angle=np.pi/6)
    
    # ドローン2: x軸上の位置、単位姿勢、視野角60度
    drone2 = Drone(SE3(p=np.array([2.0, 0.0, 0.0])), fov_angle=np.pi/6)
    
    # 特徴点の追加
    feature_points = [
        FeaturePoint([1.0, 0.0, 0.0]),  # ドローン1とドローン2の中間
        FeaturePoint([1.5, 0.5, 0.0]),  # 少し右上
        FeaturePoint([1.5, -0.5, 0.0])  # 少し右下
    ]
    
    # 速度入力
    xi1 = np.array([0.0, 0.0, 0.0, 0.1, 0.0, 0.0])  # ドローン1: x方向に前進
    xi2 = np.array([0.0, 0.0, 0.0, -0.1, 0.0, 0.0])  # ドローン2: x方向に後退
    
    # 時間微分を計算
    derivative = compute_barrier_function_derivative(drone1, drone2, feature_points, xi1, xi2)
    
    print(f"安全集合の時間微分: {derivative:.6f}")
    
    # 数値的に時間微分を検証
    dt = 0.01
    
    # ドローンを速度入力に基づいて更新
    drone1_next = Drone(SE3(drone1.T.R.copy(), drone1.T.p.copy()), 
                        drone1.camera_direction.copy(), drone1.fov_angle)
    drone2_next = Drone(SE3(drone2.T.R.copy(), drone2.T.p.copy()), 
                        drone2.camera_direction.copy(), drone2.fov_angle)
    
    drone1_next.update(xi1, dt)
    drone2_next.update(xi2, dt)
    
    # 変化前と変化後の安全集合の値
    value_before = compute_barrier_function(drone1, drone2, feature_points)
    value_after = compute_barrier_function(drone1_next, drone2_next, feature_points)
    
    # 数値微分
    numerical_derivative = (value_after - value_before) / dt
    
    print(f"数値的な時間微分: {numerical_derivative:.6f}")
    print(f"計算された時間微分: {derivative:.6f}")
    print(f"差: {abs(numerical_derivative - derivative):.6f}")
    print()


def test_solve_cbf_qp():
    """
    solve_cbf_qp関数のテスト（制約付き最適化問題）
    """
    print("=== solve_cbf_qp関数のテスト（制約付き最適化問題） ===")
    
    # シミュレータの初期化
    simulator = Simulator(dt=0.1)
    
    # ドローン1: 原点、単位姿勢、視野角60度
    drone1 = Drone(fov_angle=np.pi/6)
    simulator.add_drone(drone1)
    
    # 特徴点の追加（格子状）
    n_points = 3
    for x in np.linspace(-3, 3, n_points):
        for y in np.linspace(-3, 3, n_points):
            for z in np.linspace(-3, 3, n_points):
                if abs(x) > 1 or abs(y) > 1 or abs(z) > 1:  # 中心付近は除外
                    fp = FeaturePoint([x, y, z])
                    simulator.add_feature_point(fp)
    
    # ランダムサンプリングで安全集合の値が正になる初期状態を見つける
    print("ランダムサンプリングで安全集合の値が正になる初期状態を探索中...")
    success, attempts, barrier_value = sample_safe_configuration(
        simulator, np.pi/6, min_barrier_value=0.1, max_attempts=100)
    
    if not success:
        print("安全集合の値が正になる初期状態が見つかりませんでした。テストをスキップします。")
        return
    
    print(f"安全集合の値が正になる初期状態を{attempts}回目の試行で見つけました（B={barrier_value:.4f}）")
    
    # ドローン1とドローン2を取得
    drone1 = simulator.drones[0]
    drone2 = simulator.drones[1]
    feature_points = simulator.feature_points
    
    # 目標速度入力（安全でない）
    xi1_des = np.array([0.0, 0.0, 0.0, 0.5, 0.0, 0.0])  # ドローン1: x方向に前進
    xi2_des = np.array([0.0, 0.0, 0.0, -0.5, 0.0, 0.0])  # ドローン2: x方向に後退
    
    # 現在の安全集合の値
    barrier_value = compute_barrier_function(drone1, drone2, feature_points)
    print(f"現在の安全集合の値: {barrier_value:.6f}")
    
    # 目標入力による安全集合の時間微分
    derivative = compute_barrier_function_derivative(drone1, drone2, feature_points, xi1_des, xi2_des)
    print(f"目標入力による安全集合の時間微分: {derivative:.6f}")
    
    # CBF制約: ドット{B} + γB ≥ 0
    gamma = 0.1
    cbf_constraint = derivative + gamma * barrier_value
    print(f"CBF制約（目標入力）: {cbf_constraint:.6f}")
    
    # 制約付き最適化問題を解く
    xi1_safe, xi2_safe = solve_cbf_qp(drone1, drone2, feature_points, xi1_des, xi2_des, gamma=gamma)
    
    print("安全な速度入力:")
    print(f"ドローン1（目標）: {xi1_des}")
    print(f"ドローン1（安全）: {xi1_safe}")
    print(f"ドローン2（目標）: {xi2_des}")
    print(f"ドローン2（安全）: {xi2_safe}")
    
    # 安全な入力による安全集合の時間微分
    derivative_safe = compute_barrier_function_derivative(drone1, drone2, feature_points, xi1_safe, xi2_safe)
    print(f"安全な入力による安全集合の時間微分: {derivative_safe:.6f}")
    
    # CBF制約（安全な入力）
    cbf_constraint_safe = derivative_safe + gamma * barrier_value
    print(f"CBF制約（安全な入力）: {cbf_constraint_safe:.6f}")
    
    # 目標入力と安全な入力の差
    xi1_diff = np.linalg.norm(xi1_safe - xi1_des)
    xi2_diff = np.linalg.norm(xi2_safe - xi2_des)
    print(f"ドローン1の入力の差: {xi1_diff:.6f}")
    print(f"ドローン2の入力の差: {xi2_diff:.6f}")
    print()


def test_sample_safe_configuration():
    """
    sample_safe_configuration関数のテスト
    """
    print("=== sample_safe_configuration関数のテスト ===")
    
    # シミュレータの初期化
    simulator = Simulator(dt=0.1)
    
    # ドローン1: 原点、単位姿勢、視野角60度
    drone1 = Drone(fov_angle=np.pi/6)
    simulator.add_drone(drone1)
    
    # 特徴点の追加（格子状）
    n_points = 3
    for x in np.linspace(-3, 3, n_points):
        for y in np.linspace(-3, 3, n_points):
            for z in np.linspace(-3, 3, n_points):
                if abs(x) > 1 or abs(y) > 1 or abs(z) > 1:  # 中心付近は除外
                    fp = FeaturePoint([x, y, z])
                    simulator.add_feature_point(fp)
    
    # 安全な配置をサンプリング
    min_barrier_value = 0.1
    max_attempts = 100
    
    success, attempts, barrier_value = sample_safe_configuration(
        simulator, np.pi/6, min_barrier_value, max_attempts)
    
    if success:
        print(f"安全な配置を{attempts}回目の試行で見つけました")
        print(f"安全集合の値: {barrier_value:.6f}")
        
        # ドローン2の位置と姿勢
        drone2 = simulator.drones[1]
        print(f"ドローン2の位置: {drone2.T.p}")
        print(f"ドローン2の回転行列:\n{drone2.T.R}")
        
        # 共有視野内の特徴点
        cofov_indices = []
        for i, fp in enumerate(simulator.feature_points):
            if (drone1.is_point_visible(fp.position) and 
                drone2.is_point_visible(fp.position)):
                cofov_indices.append(i)
        
        print(f"共有視野内の特徴点数: {len(cofov_indices)}")
    else:
        print(f"安全な配置を{max_attempts}回の試行で見つけられませんでした")
        print(f"最後の試行での安全集合の値: {barrier_value:.6f}")
    
    print()


def test_generate_safe_control_inputs():
    """
    generate_safe_control_inputs関数のテスト
    """
    print("=== generate_safe_control_inputs関数のテスト ===")
    
    # シミュレータの初期化
    simulator = Simulator(dt=0.1)
    
    # ドローン1: 原点、単位姿勢、視野角60度
    drone1 = Drone(fov_angle=np.pi/6)
    simulator.add_drone(drone1)
    
    # 特徴点の追加（格子状）
    n_points = 3
    for x in np.linspace(-3, 3, n_points):
        for y in np.linspace(-3, 3, n_points):
            for z in np.linspace(-3, 3, n_points):
                if abs(x) > 1 or abs(y) > 1 or abs(z) > 1:  # 中心付近は除外
                    fp = FeaturePoint([x, y, z])
                    simulator.add_feature_point(fp)
    
    # ランダムサンプリングで安全集合の値が正になる初期状態を見つける
    print("ランダムサンプリングで安全集合の値が正になる初期状態を探索中...")
    success, attempts, barrier_value = sample_safe_configuration(
        simulator, np.pi/6, min_barrier_value=0.1, max_attempts=100)
    
    if not success:
        print("安全集合の値が正になる初期状態が見つかりませんでした。テストをスキップします。")
        return
    
    print(f"安全集合の値が正になる初期状態を{attempts}回目の試行で見つけました（B={barrier_value:.4f}）")
    
    # 目標速度入力（安全でない）
    xi1_des = np.array([0.0, 0.0, 0.0, 0.5, 0.0, 0.0])  # ドローン1: x方向に前進
    xi2_des = np.array([0.0, 0.0, 0.0, -0.5, 0.0, 0.0])  # ドローン2: x方向に後退
    
    # 安全な制御入力を生成
    xi1_safe, xi2_safe = generate_safe_control_inputs(simulator, xi1_des, xi2_des)
    
    print("安全な速度入力:")
    print(f"ドローン1（目標）: {xi1_des}")
    print(f"ドローン1（安全）: {xi1_safe}")
    print(f"ドローン2（目標）: {xi2_des}")
    print(f"ドローン2（安全）: {xi2_safe}")
    
    # ドローン1とドローン2を取得
    drone1 = simulator.drones[0]
    drone2 = simulator.drones[1]
    feature_points = simulator.feature_points
    
    # 安全集合の値と時間微分
    barrier_value = compute_barrier_function(drone1, drone2, feature_points)
    derivative_safe = compute_barrier_function_derivative(
        drone1, drone2, feature_points, xi1_safe, xi2_safe)
    
    print(f"安全集合の値: {barrier_value:.6f}")
    print(f"安全な入力による安全集合の時間微分: {derivative_safe:.6f}")
    print()


def test_simulation_with_cbf():
    """
    CBFを使用したシミュレーションのテスト
    制約なしの場合とありの場合での比較、制約式における制約余裕の出力を行い
    結果から制約付き最適化が正常に動作し、それによって安全集合が正不変になっているかvalidation
    """
    print("=== CBFを使用したシミュレーションのテスト ===")
    
    # シミュレータの初期化
    dt = 0.1
    
    # CBFありとなしの両方のケースをシミュレーション
    for use_cbf in [False, True]:
        print(f"\n--- CBF {'あり' if use_cbf else 'なし'} のシミュレーション ---")
        
        # シミュレータの初期化（毎回新しいシミュレータを作成）
        simulator = Simulator(dt)
        
        # ドローン1: 原点、単位姿勢、視野角60度
        drone1 = Drone(fov_angle=np.pi/6)
        simulator.add_drone(drone1)
        
        # 特徴点の追加（格子状）
        n_points = 3
        for x in np.linspace(-3, 3, n_points):
            for y in np.linspace(-3, 3, n_points):
                for z in np.linspace(-3, 3, n_points):
                    if abs(x) > 1 or abs(y) > 1 or abs(z) > 1:  # 中心付近は除外
                        fp = FeaturePoint([x, y, z])
                        simulator.add_feature_point(fp)
        
        # ランダムサンプリングで安全集合の値が正になる初期状態を見つける
        print("ランダムサンプリングで安全集合の値が正になる初期状態を探索中...")
        success, attempts, barrier_value = sample_safe_configuration(
            simulator, np.pi/6, min_barrier_value=0.2, max_attempts=100)
        
        if not success:
            print("安全集合の値が正になる初期状態が見つかりませんでした。テストをスキップします。")
            continue
        
        print(f"安全集合の値が正になる初期状態を{attempts}回目の試行で見つけました（B={barrier_value:.4f}）")
        
        # ドローン1とドローン2を取得
        drone1 = simulator.drones[0]
        drone2 = simulator.drones[1]
        feature_points = simulator.feature_points
        
        # シミュレーションのパラメータ
        num_steps = 50
        gamma = 0.1  # CBF制約のゲイン
        
        # 結果を格納する配列
        times = np.arange(0, num_steps * dt, dt)
        positions1 = np.zeros((num_steps, 3))
        positions2 = np.zeros((num_steps, 3))
        barrier_values = np.zeros(num_steps)
        cbf_constraints = np.zeros(num_steps)  # CBF制約の値（制約余裕）
        
        # 初期状態を記録
        positions1[0] = drone1.T.p
        positions2[0] = drone2.T.p
        barrier_values[0] = compute_barrier_function(drone1, drone2, feature_points)
        
        # シミュレーション
        for i in range(1, num_steps):
            # 目標速度入力（安全でない）
            xi1_des = np.array([0.0, 0.0, 0.0, 0.2, 0.0, 0.0])  # ドローン1: x方向に前進
            xi2_des = np.array([0.0, 0.0, 0.0, -0.2, 0.0, 0.0])  # ドローン2: x方向に後退
            
            # 目標入力による安全集合の時間微分
            derivative_des = compute_barrier_function_derivative(
                simulator.drones[0], simulator.drones[1], simulator.feature_points, 
                xi1_des, xi2_des)
            
            # CBF制約: ドット{B} + γB ≥ 0
            cbf_constraint_des = derivative_des + gamma * barrier_values[i-1]
            
            if use_cbf:
                # CBFを使用して安全な制御入力を生成
                xi1, xi2 = generate_safe_control_inputs(simulator, xi1_des, xi2_des, gamma=gamma)
            else:
                # CBFを使用しない場合は目標入力をそのまま使用
                xi1, xi2 = xi1_des, xi2_des
            
            # 実際の入力による安全集合の時間微分
            derivative = compute_barrier_function_derivative(
                simulator.drones[0], simulator.drones[1], simulator.feature_points, 
                xi1, xi2)
            
            # CBF制約の値（制約余裕）
            cbf_constraints[i] = derivative + gamma * barrier_values[i-1]
            
            # シミュレーションを1ステップ進める
            simulator.step([xi1, xi2])
            
            # 状態を記録
            positions1[i] = simulator.drones[0].T.p
            positions2[i] = simulator.drones[1].T.p
            barrier_values[i] = compute_barrier_function(
                simulator.drones[0], simulator.drones[1], simulator.feature_points)
            
            # 各ステップでの情報を出力
            if i % 10 == 0 or i == 1:
                print(f"ステップ {i}:")
                print(f"  安全集合の値: {barrier_values[i]:.6f}")
                print(f"  目標入力によるCBF制約: {cbf_constraint_des:.6f}")
                print(f"  実際の入力によるCBF制約: {cbf_constraints[i]:.6f}")
                print(f"  ドローン1の位置: {positions1[i]}")
                print(f"  ドローン2の位置: {positions2[i]}")
        
        # 結果をプロット
        plt.figure(figsize=(12, 15))
        
        # ドローンの軌道
        plt.subplot(3, 1, 1)
        plt.plot(times, positions1[:, 0], 'b-', label='Drone 1 X Position')
        plt.plot(times, positions2[:, 0], 'r-', label='Drone 2 X Position')
        plt.xlabel('Time [s]')
        plt.ylabel('X Position')
        plt.title(f'Drone Trajectories (CBF {"Enabled" if use_cbf else "Disabled"})')
        plt.grid(True)
        plt.legend()
        
        # 安全集合の値
        plt.subplot(3, 1, 2)
        plt.plot(times, barrier_values, 'g-')
        plt.axhline(y=0, color='r', linestyle='--', label='Safety Boundary')
        plt.xlabel('Time [s]')
        plt.ylabel('Barrier Function Value B_{ij}')
        plt.title('Barrier Function Value Over Time')
        plt.grid(True)
        plt.legend()
        
        # CBF制約の値（制約余裕）
        plt.subplot(3, 1, 3)
        plt.plot(times[1:], cbf_constraints[1:], 'm-')
        plt.axhline(y=0, color='r', linestyle='--', label='Constraint Boundary')
        plt.xlabel('Time [s]')
        plt.ylabel('CBF Constraint Value')
        plt.title('CBF Constraint Value (Constraint Margin) Over Time')
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        plt.savefig(f'cbf_simulation_{"with" if use_cbf else "without"}_cbf.png')
        plt.close()
        
        print(f"\nシミュレーション結果（CBF {'あり' if use_cbf else 'なし'}）:")
        print(f"  最終的な安全集合の値: {barrier_values[-1]:.6f}")
        print(f"  最終的なCBF制約の値: {cbf_constraints[-1]:.6f}")
        print(f"  ドローン1の最終位置: {positions1[-1]}")
        print(f"  ドローン2の最終位置: {positions2[-1]}")
        print(f"  結果を cbf_simulation_{'with' if use_cbf else 'without'}_cbf.png に保存しました")
        
        # 安全集合が正不変かどうかを検証
        is_safe = np.all(barrier_values >= 0)
        print(f"  安全集合は正不変か: {is_safe}")
        if not is_safe:
            # 安全集合が負になったステップを特定
            unsafe_steps = np.where(barrier_values < 0)[0]
            print(f"  安全集合が負になったステップ: {unsafe_steps}")
            print(f"  最小の安全集合の値: {np.min(barrier_values):.6f}")
    
    # CBFありとなしの比較結果
    print("\n=== CBFありとなしの比較 ===")
    print("CBFを使用することで、安全集合が正不変（常に正の値を保つ）になることが期待されます。")
    print("CBFなしの場合、ドローン同士が近づきすぎると安全集合の値が負になる可能性があります。")
    print("CBF制約の値（制約余裕）が常に正であれば、制約付き最適化が正常に動作していることを示します。")
    print()


def main():
    """
    メイン関数
    """
    print("cbf_se3.pyのテストを開始します\n")
    
    # 各関数のテスト
    test_compute_barrier_function()
    test_compute_barrier_function_gradient()
    test_compute_barrier_function_derivative()
    test_solve_cbf_qp()
    test_sample_safe_configuration()
    test_generate_safe_control_inputs()
    test_simulation_with_cbf()
    
    print("すべてのテストが完了しました")


if __name__ == "__main__":
    main()
