#!/usr/bin/env python3

import torch
import numpy as np
from typing import Any, List, Tuple, Optional, Dict
from abc import ABC, abstractmethod

class BaseController(ABC):
    """
    ドローンコントローラの基底クラス
    
    すべてのコントローラはこのクラスを継承し、必要なメソッドを実装する必要があります。
    """
    
    def __init__(self, drone: Any, dt: float):
        """
        コントローラの初期化
        
        Args:
            drone (Any): 制御対象のドローン
            dt (float): 時間ステップ
        """
        self.drone = drone
        self.dt = dt
        self.reset()
    
    @abstractmethod
    def update(self, target: Tuple[float, float, float]) -> np.ndarray:
        """
        目標位置に基づいて制御入力を計算
        
        Args:
            target (tuple): 目標位置 (x, y, z)
            
        Returns:
            np.ndarray: 制御入力（モーターのRPM値など）
        """
        pass
    
    @abstractmethod
    def reset(self) -> None:
        """
        コントローラの状態をリセット
        """
        pass
    
    def get_drone_pos(self) -> torch.Tensor:
        """
        ドローンの位置を取得
        
        Returns:
            torch.Tensor: ドローンの位置
        """
        return self.drone.get_pos()
    
    def get_drone_vel(self) -> torch.Tensor:
        """
        ドローンの速度を取得
        
        Returns:
            torch.Tensor: ドローンの速度
        """
        return self.drone.get_vel()
    
    def get_drone_quat(self) -> torch.Tensor:
        """
        ドローンの姿勢（クォータニオン）を取得
        
        Returns:
            torch.Tensor: ドローンの姿勢（クォータニオン）
        """
        return self.drone.get_quat()
    
    def set_propellers_rpm(self, rpms: List[float]) -> None:
        """
        ドローンのプロペラRPMを設定
        
        Args:
            rpms (list): 各プロペラのRPM値
        """

        try:
            # set_propellels_rpmメソッドを試す（古いバージョンとの互換性のため）
            self.drone.set_propellels_rpm(rpms)
        except AttributeError:
            # set_propellers_rpmメソッドを試す
            try:
                self.drone.set_propellers_rpm(rpms)
            except AttributeError:
                print(f"Drone does not have a method to set propeller RPM")
    
    def get_parameters(self) -> Dict[str, Any]:
        """
        コントローラのパラメータを取得
        
        Returns:
            dict: パラメータの辞書
        """
        return {
            "dt": self.dt
        }
    
    def set_parameters(self, params: Dict[str, Any]) -> None:
        """
        コントローラのパラメータを設定
        
        Args:
            params (dict): パラメータの辞書
        """
        if "dt" in params:
            self.dt = params["dt"]
