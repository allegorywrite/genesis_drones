#!/usr/bin/env python3

"""
Parameter Optimizer Module

このモジュールはA*アルゴリズムを使用したパラメータ最適化機能を提供します。
"""

import numpy as np
import heapq
from typing import Dict, List, Tuple, Any, Callable, Optional
import itertools

class AStarParameterOptimizer:
    """A*アルゴリズムを使用したパラメータ最適化クラス"""
    
    def __init__(self, param_ranges: Dict[str, np.ndarray], 
                 evaluate_func: Callable, 
                 heuristic_func: Optional[Callable] = None,
                 logger=None):
        """
        A*パラメータ最適化の初期化
        
        Args:
            param_ranges: パラメータの範囲（辞書）
            evaluate_func: パラメータを評価する関数
            heuristic_func: ヒューリスティック関数（オプション）
            logger: ロガー（オプション）
        """
        self.param_ranges = param_ranges
        self.evaluate_func = evaluate_func
        self.heuristic_func = heuristic_func or (lambda x: 0)
        self.logger = logger
        
        # 探索履歴
        self.search_history = {
            'visited_states': [],  # 訪問した状態
            'scores': [],          # 各状態のスコア
            'paths': [],           # 探索経路
            'grid_scores': {},     # グリッド上のスコア（可視化用）
            'best_path': []        # 最良経路
        }
        
        # パラメータの名前と次元
        self.param_names = list(param_ranges.keys())
        self.param_dims = len(self.param_names)
        
        # 各パラメータの値の数
        self.param_sizes = {name: len(values) for name, values in param_ranges.items()}
        
        # 評価済みの状態を記録
        self.evaluated_states = {}
    
    def _state_to_params(self, state: Tuple[int, ...]) -> Dict[str, float]:
        """
        状態インデックスからパラメータ値への変換
        
        Args:
            state: 状態インデックス（各パラメータのインデックス）
            
        Returns:
            dict: パラメータ値の辞書
        """
        params = {}
        for i, name in enumerate(self.param_names):
            params[name] = self.param_ranges[name][state[i]]
        return params
    
    def _get_neighbors(self, state: Tuple[int, ...]) -> List[Tuple[int, ...]]:
        """
        現在の状態の近傍状態を取得
        
        Args:
            state: 現在の状態
            
        Returns:
            list: 近傍状態のリスト
        """
        neighbors = []
        
        # 各パラメータ次元について
        for dim in range(self.param_dims):
            # 現在の値
            current_idx = state[dim]
            
            # 増加方向の近傍
            if current_idx + 1 < len(self.param_ranges[self.param_names[dim]]):
                new_state = list(state)
                new_state[dim] = current_idx + 1
                neighbors.append(tuple(new_state))
            
            # 減少方向の近傍
            if current_idx - 1 >= 0:
                new_state = list(state)
                new_state[dim] = current_idx - 1
                neighbors.append(tuple(new_state))
        
        return neighbors
    
    def _evaluate_state(self, state: Tuple[int, ...]) -> float:
        """
        状態を評価
        
        Args:
            state: 評価する状態
            
        Returns:
            float: 評価スコア
        """
        # 既に評価済みの場合はキャッシュから返す
        if state in self.evaluated_states:
            return self.evaluated_states[state]
        
        # パラメータ値に変換
        params = self._state_to_params(state)
        
        # 評価関数を呼び出し
        score = self.evaluate_func(params)
        
        # 結果をキャッシュ
        self.evaluated_states[state] = score
        
        # グリッドスコアを記録（可視化用）
        for i in range(self.param_dims):
            for j in range(i+1, self.param_dims):
                # パラメータのペアを取得
                param_i = self.param_names[i]
                param_j = self.param_names[j]
                
                # グリッドキーを作成
                grid_key = (param_i, param_j)
                
                # グリッド座標を取得
                grid_coord = (state[i], state[j])
                
                # グリッドスコアを記録
                if grid_key not in self.search_history['grid_scores']:
                    self.search_history['grid_scores'][grid_key] = {}
                
                self.search_history['grid_scores'][grid_key][grid_coord] = score
        
        return score
    
    def optimize(self, max_iterations: int = 100, start_state: Optional[Tuple[int, ...]] = None, 
                callback: Optional[Callable] = None) -> Tuple[Dict[str, float], float, Dict]:
        """
        A*アルゴリズムによるパラメータ最適化
        
        Args:
            max_iterations: 最大反復回数
            start_state: 開始状態（指定しない場合は各パラメータの中央値）
            callback: 各イテレーション後に呼び出されるコールバック関数
                     callback(iteration, current_state, current_score, search_history, best_state, best_score)
            
        Returns:
            tuple: (最適なパラメータ, 最適なスコア, 探索履歴)
        """
        if self.logger:
            self.logger.info("A*アルゴリズムによるパラメータ最適化を開始")
        
        # 開始状態が指定されていない場合は各パラメータの中央値を使用
        if start_state is None:
            start_state = tuple(len(self.param_ranges[name]) // 2 for name in self.param_names)
        
        # 優先度キュー（f値, 状態, 親状態, g値）
        open_set = []
        
        # 開始状態の評価
        start_score = self._evaluate_state(start_state)
        start_f = start_score + self.heuristic_func(start_state)
        
        # 開始状態をオープンセットに追加
        heapq.heappush(open_set, (start_f, start_state, None, 0))
        
        # クローズドセット（訪問済みの状態）
        closed_set = set()
        
        # 各状態の親状態を記録
        came_from = {}
        
        # 各状態のg値（開始状態からのコスト）
        g_score = {start_state: 0}
        
        # 各状態のf値（g値 + ヒューリスティック）
        f_score = {start_state: start_f}
        
        # 最良の状態と評価値
        best_state = start_state
        best_score = start_score
        
        # 探索履歴の初期化
        self.search_history['visited_states'] = [start_state]
        self.search_history['scores'] = [start_score]
        self.search_history['paths'] = [(None, start_state)]
        
        # 反復回数
        iterations = 0
        
        while open_set and iterations < max_iterations:
            # 優先度キューから最もf値の小さい状態を取得
            current_f, current_state, parent_state, current_g = heapq.heappop(open_set)
            
            # 既に訪問済みの場合はスキップ
            if current_state in closed_set:
                continue
            
            # 訪問済みに追加
            closed_set.add(current_state)
            
            # 親状態を記録
            if parent_state is not None:
                came_from[current_state] = parent_state
            
            # 現在の状態を評価
            current_score = self._evaluate_state(current_state)
            
            # 探索履歴に追加
            self.search_history['visited_states'].append(current_state)
            self.search_history['scores'].append(current_score)
            self.search_history['paths'].append((parent_state, current_state))
            
            # より良いスコアが見つかった場合
            if current_score < best_score:
                best_state = current_state
                best_score = current_score
                
                if self.logger:
                    self.logger.info(f"新しい最良スコア: {best_score}, パラメータ: {self._state_to_params(best_state)}")
            
            # 近傍状態を探索
            for neighbor in self._get_neighbors(current_state):
                # 既に訪問済みの場合はスキップ
                if neighbor in closed_set:
                    continue
                
                # 近傍へのg値を計算
                tentative_g = current_g + 1
                
                # より良いパスが見つかった場合、または初めて訪問する場合
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # パスを更新
                    g_score[neighbor] = tentative_g
                    
                    # 近傍の評価
                    neighbor_score = self._evaluate_state(neighbor)
                    
                    # f値を計算
                    f_value = tentative_g + self.heuristic_func(neighbor)
                    f_score[neighbor] = f_value
                    
                    # オープンセットに追加
                    heapq.heappush(open_set, (f_value, neighbor, current_state, tentative_g))
            
            iterations += 1
            
            # コールバック関数の呼び出し
            if callback is not None:
                callback(
                    iterations,
                    current_state,
                    current_score,
                    self.search_history,
                    best_state,
                    best_score
                )
            
            if self.logger and iterations % 10 == 0:
                self.logger.info(f"反復回数: {iterations}/{max_iterations}, 評価済み状態数: {len(self.evaluated_states)}")
        
        # 最良経路を再構築
        path = []
        current = best_state
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(start_state)
        path.reverse()
        
        self.search_history['best_path'] = path
        
        # 最適なパラメータを返す
        best_params = self._state_to_params(best_state)
        
        if self.logger:
            self.logger.info(f"最適化完了: 反復回数={iterations}, 評価済み状態数={len(self.evaluated_states)}")
            self.logger.info(f"最適なパラメータ: {best_params}")
            self.logger.info(f"最適なスコア: {best_score}")
        
        return best_params, best_score, self.search_history

class GridSearchOptimizer:
    """グリッドサーチによるパラメータ最適化クラス"""
    
    def __init__(self, param_ranges: Dict[str, np.ndarray], 
                 evaluate_func: Callable,
                 logger=None):
        """
        グリッドサーチパラメータ最適化の初期化
        
        Args:
            param_ranges: パラメータの範囲（辞書）
            evaluate_func: パラメータを評価する関数
            logger: ロガー（オプション）
        """
        self.param_ranges = param_ranges
        self.evaluate_func = evaluate_func
        self.logger = logger
        
        # 探索履歴
        self.search_history = {
            'visited_states': [],  # 訪問した状態
            'scores': [],          # 各状態のスコア
            'grid_scores': {},     # グリッド上のスコア（可視化用）
        }
        
        # パラメータの名前と次元
        self.param_names = list(param_ranges.keys())
        self.param_dims = len(self.param_names)
        
        # 各パラメータの値の数
        self.param_sizes = {name: len(values) for name, values in param_ranges.items()}
    
    def _state_to_params(self, state: Tuple[int, ...]) -> Dict[str, float]:
        """
        状態インデックスからパラメータ値への変換
        
        Args:
            state: 状態インデックス（各パラメータのインデックス）
            
        Returns:
            dict: パラメータ値の辞書
        """
        params = {}
        for i, name in enumerate(self.param_names):
            params[name] = self.param_ranges[name][state[i]]
        return params
    
    def optimize(self, max_iterations: int = 100, callback: Optional[Callable] = None) -> Tuple[Dict[str, float], float, Dict]:
        """
        グリッドサーチによるパラメータ最適化
        
        Args:
            max_iterations: 最大評価回数
            callback: 各評価後に呼び出されるコールバック関数
                     callback(iteration, current_state, current_score, search_history, best_state, best_score)
            
        Returns:
            tuple: (最適なパラメータ, 最適なスコア, 探索履歴)
        """
        if self.logger:
            self.logger.info("グリッドサーチによるパラメータ最適化を開始")
        
        # 各パラメータのインデックスの範囲
        param_indices = [range(len(self.param_ranges[name])) for name in self.param_names]
        
        # 全ての組み合わせを生成
        all_combinations = list(itertools.product(*param_indices))
        
        # 組み合わせ数が多い場合は、均等にサンプリング
        if len(all_combinations) > max_iterations:
            if self.logger:
                self.logger.info(f"組み合わせ数({len(all_combinations)})が最大評価回数({max_iterations})を超えるため、サンプリングします")
            
            # 均等にサンプリング
            indices = np.linspace(0, len(all_combinations) - 1, max_iterations, dtype=int)
            selected_combinations = [all_combinations[idx] for idx in indices]
        else:
            selected_combinations = all_combinations
        
        # 最良の状態と評価値
        best_state = None
        best_score = float('inf')
        
        # 各組み合わせを評価
        for i, state in enumerate(selected_combinations):
            # パラメータ値に変換
            params = self._state_to_params(state)
            
            # 評価関数を呼び出し
            score = self.evaluate_func(params)
            
            # 探索履歴に追加
            self.search_history['visited_states'].append(state)
            self.search_history['scores'].append(score)
            
            # グリッドスコアを記録（可視化用）
            for i in range(self.param_dims):
                for j in range(i+1, self.param_dims):
                    # パラメータのペアを取得
                    param_i = self.param_names[i]
                    param_j = self.param_names[j]
                    
                    # グリッドキーを作成
                    grid_key = (param_i, param_j)
                    
                    # グリッド座標を取得
                    grid_coord = (state[i], state[j])
                    
                    # グリッドスコアを記録
                    if grid_key not in self.search_history['grid_scores']:
                        self.search_history['grid_scores'][grid_key] = {}
                    
                    self.search_history['grid_scores'][grid_key][grid_coord] = score
            
            # より良いスコアが見つかった場合
            if score < best_score:
                best_state = state
                best_score = score
                
                if self.logger:
                    self.logger.info(f"新しい最良スコア: {best_score}, パラメータ: {params}")
            
            # コールバック関数の呼び出し
            if callback is not None:
                callback(
                    i+1,
                    state,
                    score,
                    self.search_history,
                    best_state,
                    best_score
                )
            
            if self.logger and (i+1) % 10 == 0:
                self.logger.info(f"進捗: {i+1}/{len(selected_combinations)}")
        
        # 最適なパラメータを返す
        best_params = self._state_to_params(best_state)
        
        if self.logger:
            self.logger.info(f"最適化完了: 評価回数={len(selected_combinations)}")
            self.logger.info(f"最適なパラメータ: {best_params}")
            self.logger.info(f"最適なスコア: {best_score}")
        
        return best_params, best_score, self.search_history
