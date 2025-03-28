#!/usr/bin/env python3

import time
import statistics
import math
from collections import defaultdict, deque
from typing import Dict, List, Deque, Any, Optional, Callable, Union, Tuple

class TimingLogger:
    """処理時間を計測・ロギングするためのユーティリティクラス"""
    
    # ASCIIバーチャート用の文字
    BAR_CHAR = "■"
    BAR_MAX_LENGTH = 30
    
    # カテゴリ定義
    CATEGORY_DRONE = "DRONE PROCESSING"
    CATEGORY_PHYSICS = "PHYSICS SIMULATION"
    CATEGORY_OTHER = "OTHER"
    
    # サブカテゴリ定義
    SUBCATEGORY_CONTROLLER = "Controller"
    SUBCATEGORY_CAMERA = "Camera"
    SUBCATEGORY_CAMERA_POSITION = "Position"
    SUBCATEGORY_CAMERA_RENDER = "Rendering"
    SUBCATEGORY_CAMERA_PROCESSING = "Processing"
    SUBCATEGORY_CAMERA_DISPLAY = "Display"
    SUBCATEGORY_CAMERA_PUBLISH = "Publish"
    SUBCATEGORY_MARKER = "Marker"
    SUBCATEGORY_TF = "TF"
    SUBCATEGORY_OTHER = "Other"
    
    def __init__(self, logger: Any, log_interval: int = 10):
        """
        初期化
        
        Args:
            logger: ロギング用のロガーオブジェクト（ROS 2のロガーなど）
            log_interval: 何ステップごとに統計情報をログ出力するか
        """
        self.logger = logger
        self.timing_stats = defaultdict(lambda: deque(maxlen=100))  # 直近100回分の処理時間を保存
        self.total_timing_stats = defaultdict(list)  # すべての処理時間を保存
        self.step_count = 0  # ステップ数のカウンター
        self.log_interval = log_interval  # 何ステップごとに統計情報をログ出力するか
        self.timing_enabled = True  # 時間計測を有効にするフラグ
    
    def start_timer(self) -> float:
        """タイマーを開始し、開始時間を返す"""
        if not self.timing_enabled:
            return 0.0
        return time.time()
    
    def stop_timer(self, start_time: float, key: str) -> float:
        """
        タイマーを停止し、経過時間を記録して返す
        
        Args:
            start_time: 開始時間
            key: 処理の識別子
            
        Returns:
            経過時間（秒）
        """
        if not self.timing_enabled or start_time == 0.0:
            return 0.0
            
        elapsed_time = time.time() - start_time
        self.record_timing(key, elapsed_time)
        return elapsed_time
    
    def record_timing(self, key: str, time_value: float) -> None:
        """
        処理時間を記録する
        
        Args:
            key: 処理の識別子
            time_value: 処理時間（秒）
        """
        if not self.timing_enabled:
            return
            
        self.timing_stats[key].append(time_value)
        self.total_timing_stats[key].append(time_value)
    
    def increment_step(self, dt: float = 0.033) -> None:
        """
        ステップカウントを増やし、必要に応じて統計情報をログ出力する
        
        Args:
            dt (float): シミュレーションの時間ステップ（秒）
        """
        if not self.timing_enabled:
            return
            
        self.step_count += 1
        
        # 定期的に統計情報をログ出力
        if self.step_count % self.log_interval == 0:
            self.log_timing_stats(dt)
    
    def _create_ascii_bar(self, percentage: float) -> str:
        """
        パーセンテージに基づいてASCIIバーを作成する
        
        Args:
            percentage (float): パーセンテージ値（0-100）
            
        Returns:
            str: ASCIIバー文字列
        """
        # バーの長さを計算（最大長に対する割合）
        bar_length = math.ceil(percentage / 100 * self.BAR_MAX_LENGTH)
        bar_length = min(bar_length, self.BAR_MAX_LENGTH)  # 最大長を超えないようにする
        
        # バーを作成
        bar = "[" + self.BAR_CHAR * bar_length + " " * (self.BAR_MAX_LENGTH - bar_length) + "]"
        
        return bar
    
    def _categorize_timing_keys(self, timing_dict: Dict[str, List[float]]) -> Dict[str, Dict[str, Dict[str, List[float]]]]:
        """
        タイミングキーをカテゴリ別に分類する
        
        Args:
            timing_dict: タイミング情報の辞書
            
        Returns:
            カテゴリ別に分類された辞書
        """
        categories = {
            self.CATEGORY_DRONE: {},
            self.CATEGORY_PHYSICS: {},
            self.CATEGORY_OTHER: {}
        }
        
        # ドローン関連のキーを処理
        drone_keys = defaultdict(list)
        for key, times in timing_dict.items():
            if not times:
                continue
                
            # ドローン関連のキーを処理
            if key.startswith("drone_") and "_" in key[6:]:
                # drone_X_YYY の形式からYYYの部分を抽出
                parts = key.split("_", 2)
                if len(parts) >= 3:
                    category = parts[2]  # YYYの部分
                    
                    # サブカテゴリに分類
                    if "controller_update" in category:
                        if self.SUBCATEGORY_CONTROLLER not in categories[self.CATEGORY_DRONE]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CONTROLLER] = []
                        categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CONTROLLER].extend(times)
                    elif "marker_update" in category:
                        if self.SUBCATEGORY_MARKER not in categories[self.CATEGORY_DRONE]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_MARKER] = []
                        categories[self.CATEGORY_DRONE][self.SUBCATEGORY_MARKER].extend(times)
                    elif "tf_publish" in category:
                        if self.SUBCATEGORY_TF not in categories[self.CATEGORY_DRONE]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_TF] = []
                        categories[self.CATEGORY_DRONE][self.SUBCATEGORY_TF].extend(times)
                    elif "camera_position_update" in category:
                        if self.SUBCATEGORY_CAMERA not in categories[self.CATEGORY_DRONE]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA] = {}
                        if self.SUBCATEGORY_CAMERA_POSITION not in categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA][self.SUBCATEGORY_CAMERA_POSITION] = []
                        categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA][self.SUBCATEGORY_CAMERA_POSITION].extend(times)
                    elif "camera_render" in category:
                        if self.SUBCATEGORY_CAMERA not in categories[self.CATEGORY_DRONE]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA] = {}
                        if self.SUBCATEGORY_CAMERA_RENDER not in categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA][self.SUBCATEGORY_CAMERA_RENDER] = []
                        categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA][self.SUBCATEGORY_CAMERA_RENDER].extend(times)
                    elif "image_processing" in category:
                        if self.SUBCATEGORY_CAMERA not in categories[self.CATEGORY_DRONE]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA] = {}
                        if self.SUBCATEGORY_CAMERA_PROCESSING not in categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA][self.SUBCATEGORY_CAMERA_PROCESSING] = []
                        categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA][self.SUBCATEGORY_CAMERA_PROCESSING].extend(times)
                    elif "display_image" in category:
                        if self.SUBCATEGORY_CAMERA not in categories[self.CATEGORY_DRONE]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA] = {}
                        if self.SUBCATEGORY_CAMERA_DISPLAY not in categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA][self.SUBCATEGORY_CAMERA_DISPLAY] = []
                        categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA][self.SUBCATEGORY_CAMERA_DISPLAY].extend(times)
                    elif "publish_image" in category:
                        if self.SUBCATEGORY_CAMERA not in categories[self.CATEGORY_DRONE]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA] = {}
                        if self.SUBCATEGORY_CAMERA_PUBLISH not in categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA][self.SUBCATEGORY_CAMERA_PUBLISH] = []
                        categories[self.CATEGORY_DRONE][self.SUBCATEGORY_CAMERA][self.SUBCATEGORY_CAMERA_PUBLISH].extend(times)
                    elif "camera_total" in category:
                        # camera_totalは集計に使用するが、表示はしない
                        pass
                    elif "total" in category:
                        # totalは集計に使用するが、表示はしない
                        pass
                    else:
                        # その他のドローン関連の処理
                        if self.SUBCATEGORY_OTHER not in categories[self.CATEGORY_DRONE]:
                            categories[self.CATEGORY_DRONE][self.SUBCATEGORY_OTHER] = []
                        categories[self.CATEGORY_DRONE][self.SUBCATEGORY_OTHER].extend(times)
            elif key == "scene_step":
                # 物理シミュレーション
                categories[self.CATEGORY_PHYSICS][key] = times
            elif key != "total_step":  # total_stepは全体の時間なので除外
                # その他の処理
                categories[self.CATEGORY_OTHER][key] = times
        
        return categories
    
    def _calculate_category_stats(self, categories: Dict[str, Dict[str, Dict[str, List[float]]]], total_time: float) -> Dict[str, Dict[str, Dict[str, Tuple[float, float]]]]:
        """
        各カテゴリの統計情報を計算する
        
        Args:
            categories: カテゴリ別に分類された辞書
            total_time: 全体の処理時間
            
        Returns:
            カテゴリ別の統計情報（平均時間、割合）
        """
        stats = {}
        
        # 各カテゴリの処理時間を計算
        raw_times = {}
        
        for category, subcategories in categories.items():
            stats[category] = {}
            raw_times[category] = {}
            
            if isinstance(subcategories, dict):
                for subcategory, items in subcategories.items():
                    if isinstance(items, dict):
                        # サブカテゴリがさらに階層を持つ場合
                        stats[category][subcategory] = {}
                        raw_times[category][subcategory] = {}
                        for item_name, times in items.items():
                            if times:
                                avg_time = statistics.mean(times)
                                # 一時的に時間のみを保存
                                raw_times[category][subcategory][item_name] = avg_time
                    else:
                        # サブカテゴリが直接時間のリストを持つ場合
                        if items:
                            avg_time = statistics.mean(items)
                            # 一時的に時間のみを保存
                            raw_times[category][subcategory] = avg_time
            else:
                # カテゴリが直接時間のリストを持つ場合
                if subcategories:
                    avg_time = statistics.mean(subcategories)
                    # 一時的に時間のみを保存
                    raw_times[category] = avg_time
        
        # 全カテゴリの合計時間を計算（total_stepを除く）
        total_raw_time = 0.0
        for category, subcategories in raw_times.items():
            if isinstance(subcategories, dict):
                for subcategory, items in subcategories.items():
                    if isinstance(items, dict):
                        # サブカテゴリがさらに階層を持つ場合
                        for item_name, time_value in items.items():
                            total_raw_time += time_value
                    else:
                        # サブカテゴリが直接時間を持つ場合
                        total_raw_time += items
            else:
                # カテゴリが直接時間を持つ場合
                total_raw_time += subcategories
        
        # 正規化係数を計算（合計が100%になるように）
        normalization_factor = 100.0 / total_raw_time if total_raw_time > 0 else 0
        
        # パーセンテージを計算
        for category, subcategories in raw_times.items():
            if isinstance(subcategories, dict):
                for subcategory, items in subcategories.items():
                    if isinstance(items, dict):
                        # サブカテゴリがさらに階層を持つ場合
                        for item_name, time_value in items.items():
                            percentage = time_value * normalization_factor
                            stats[category][subcategory][item_name] = (time_value, percentage)
                    else:
                        # サブカテゴリが直接時間を持つ場合
                        percentage = items * normalization_factor
                        stats[category][subcategory] = (items, percentage)
            else:
                # カテゴリが直接時間を持つ場合
                percentage = subcategories * normalization_factor
                stats[category] = (subcategories, percentage)
        
        return stats
    
    def _calculate_category_totals(self, stats: Dict[str, Dict[str, Dict[str, Tuple[float, float]]]]) -> Dict[str, Tuple[float, float]]:
        """
        各カテゴリの合計時間と割合を計算する
        
        Args:
            stats: カテゴリ別の統計情報
            
        Returns:
            カテゴリ別の合計時間と割合
        """
        totals = {}
        
        for category, subcategories in stats.items():
            total_time = 0.0
            total_percentage = 0.0
            
            if isinstance(subcategories, dict):
                for subcategory, items in subcategories.items():
                    if isinstance(items, dict):
                        # サブカテゴリがさらに階層を持つ場合
                        subcategory_time = 0.0
                        subcategory_percentage = 0.0
                        for item_name, (time_value, percentage) in items.items():
                            subcategory_time += time_value
                            subcategory_percentage += percentage
                        total_time += subcategory_time
                        total_percentage += subcategory_percentage
                    else:
                        # サブカテゴリが直接統計情報を持つ場合
                        time_value, percentage = items
                        total_time += time_value
                        total_percentage += percentage
            else:
                # カテゴリが直接統計情報を持つ場合
                time_value, percentage = subcategories
                total_time = time_value
                total_percentage = percentage
            
            totals[category] = (total_time, total_percentage)
        
        return totals
    
    def log_timing_stats(self, dt: float = 0.033) -> None:
        """
        処理時間の統計情報をログに出力する
        
        Args:
            dt (float): シミュレーションの時間ステップ（秒）
        """
        if not self.timing_enabled:
            return
            
        # 前書きなしでログを出力するためのカスタム関数
        def custom_log(message: str) -> None:
            print(message)
            
        custom_log(f"=== Timing Statistics (Step {self.step_count}) ===")
        
        # 全体の処理時間を取得
        total_time = 0.0
        if "total_step" in self.timing_stats and self.timing_stats["total_step"]:
            total_time = statistics.mean(self.timing_stats["total_step"])
        
        # タイミングキーをカテゴリ別に分類
        categories = self._categorize_timing_keys(self.timing_stats)
        
        # 各カテゴリの統計情報を計算
        stats = self._calculate_category_stats(categories, total_time)
        
        # 各カテゴリの合計時間と割合を計算
        category_totals = self._calculate_category_totals(stats)
        
        # 表示するサブカテゴリの数をカウント
        subcategory_counts = {}
        for category, subcategories in stats.items():
            if isinstance(subcategories, dict):
                subcategory_counts[category] = 0
                for subcategory, items in subcategories.items():
                    if isinstance(items, dict):
                        # サブカテゴリがさらに階層を持つ場合
                        subcat_percentage = sum(percentage for _, percentage in items.values())
                        if subcat_percentage > 0.5:  # 0.5%以上のみカウント
                            subcategory_counts[category] += 1
                    else:
                        # サブカテゴリが直接統計情報を持つ場合
                        _, percentage = items
                        if percentage > 0.5:  # 0.5%以上のみカウント
                            subcategory_counts[category] += 1
        
        # カテゴリごとに出力
        for category, (cat_time, cat_percentage) in sorted(category_totals.items(), key=lambda x: x[1][1], reverse=True):
            if cat_percentage > 0:
                # カテゴリ名を整形
                category_display = f"【{category}】".ljust(20)
                
                # ASCIIバーを作成
                bar = self._create_ascii_bar(cat_percentage)
                
                # カテゴリの合計時間と割合を出力
                custom_log(f"{category_display} {bar} {cat_percentage:.1f}%")
                
                # サブカテゴリを出力
                if category in stats and isinstance(stats[category], dict):
                    # 表示するサブカテゴリをソート
                    sorted_subcategories = sorted(
                        stats[category].items(),
                        key=lambda x: (isinstance(x[1], tuple) and x[1][1] or 0),
                        reverse=True
                    )
                    
                    # サブカテゴリの数をカウント
                    subcategory_count = subcategory_counts.get(category, 0)
                    
                    for i, (subcategory, items) in enumerate(sorted_subcategories):
                        is_last_subcategory = i == len(sorted_subcategories) - 1 or i == subcategory_count - 1
                        
                        if isinstance(items, dict):
                            # サブカテゴリがさらに階層を持つ場合
                            # サブカテゴリの合計時間と割合を計算
                            subcat_time = sum(time for time, _ in items.values())
                            subcat_percentage = sum(percentage for _, percentage in items.values())
                            
                            if subcat_percentage > 0.5:  # 0.5%以上のみ表示
                                # サブカテゴリ名を整形（最後のサブカテゴリは└─、それ以外は├─）
                                prefix = "  └─" if is_last_subcategory else "  ├─"
                                subcategory_display = f"{prefix}{subcategory}".ljust(20)
                                
                                # ASCIIバーを作成
                                bar = self._create_ascii_bar(subcat_percentage)
                                
                                # サブカテゴリの合計時間と割合を出力
                                custom_log(f"{subcategory_display} {bar} {subcat_percentage:.1f}%")
                                
                                # サブカテゴリの項目を出力
                                sorted_items = sorted(items.items(), key=lambda x: x[1][1], reverse=True)
                                visible_items = [item for item in sorted_items if item[1][1] > 0.5]  # 0.5%以上のみ表示
                                
                                for j, (item_name, (item_time, item_percentage)) in enumerate(visible_items):
                                    # 項目名を整形（最後の項目は└─、それ以外は├─）
                                    item_prefix = "  │  └─" if j == len(visible_items) - 1 else "  │  ├─"
                                    if is_last_subcategory:
                                        item_prefix = "     └─" if j == len(visible_items) - 1 else "     ├─"
                                    
                                    item_display = f"{item_prefix}{item_name}".ljust(20)
                                    
                                    # ASCIIバーを作成
                                    bar = self._create_ascii_bar(item_percentage)
                                    
                                    # 項目の時間と割合を出力
                                    custom_log(f"{item_display} {bar} {item_percentage:.1f}%")
                        else:
                            # サブカテゴリが直接統計情報を持つ場合
                            time_value, percentage = items
                            
                            if percentage > 0.5:  # 0.5%以上のみ表示
                                # サブカテゴリ名を整形（最後のサブカテゴリは└─、それ以外は├─）
                                prefix = "  └─" if is_last_subcategory else "  ├─"
                                subcategory_display = f"{prefix}{subcategory}".ljust(20)
                                
                                # ASCIIバーを作成
                                bar = self._create_ascii_bar(percentage)
                                
                                # サブカテゴリの時間と割合を出力
                                custom_log(f"{subcategory_display} {bar} {percentage:.1f}%")
        
        # ドローン数を計算
        drone_count = len([k for k in self.timing_stats.keys() if k.startswith("drone_") and k.endswith("_total")]) / 2
        if drone_count > 0:
            custom_log(f"Number of drones: {drone_count}")
        
        # 全体の処理時間（改行なし）
        if total_time > 0:
            custom_log(f"Simulation Total: {total_time:.6f}s ({1.0/total_time:.2f} FPS, {dt/total_time*100:.2f}% of realtime)")
    
    def log_final_stats(self, dt: float = 0.033) -> None:
        """
        最終的な統計情報をログに出力する
        
        Args:
            dt (float): シミュレーションの時間ステップ（秒）
        """
        if not self.timing_enabled or self.step_count == 0:
            return
            
        # 前書きなしでログを出力するためのカスタム関数
        def custom_log(message: str) -> None:
            print(message)
            
        custom_log("=== Final Timing Statistics ===")
        
        # 全体の処理時間を取得
        total_time = 0.0
        if "total_step" in self.total_timing_stats and self.total_timing_stats["total_step"]:
            total_time = statistics.mean(self.total_timing_stats["total_step"])
        
        # タイミングキーをカテゴリ別に分類
        categories = self._categorize_timing_keys(self.total_timing_stats)
        
        # 各カテゴリの統計情報を計算
        stats = self._calculate_category_stats(categories, total_time)
        
        # 各カテゴリの合計時間と割合を計算
        category_totals = self._calculate_category_totals(stats)
        
        # 表示するサブカテゴリの数をカウント
        subcategory_counts = {}
        for category, subcategories in stats.items():
            if isinstance(subcategories, dict):
                subcategory_counts[category] = 0
                for subcategory, items in subcategories.items():
                    if isinstance(items, dict):
                        # サブカテゴリがさらに階層を持つ場合
                        subcat_percentage = sum(percentage for _, percentage in items.values())
                        if subcat_percentage > 0.5:  # 0.5%以上のみカウント
                            subcategory_counts[category] += 1
                    else:
                        # サブカテゴリが直接統計情報を持つ場合
                        _, percentage = items
                        if percentage > 0.5:  # 0.5%以上のみカウント
                            subcategory_counts[category] += 1
        
        # カテゴリごとに出力
        for category, (cat_time, cat_percentage) in sorted(category_totals.items(), key=lambda x: x[1][1], reverse=True):
            if cat_percentage > 0:
                # カテゴリ名を整形
                category_display = f"【{category}】".ljust(20)
                
                # ASCIIバーを作成
                bar = self._create_ascii_bar(cat_percentage)
                
                # カテゴリの合計時間と割合を出力
                custom_log(f"{category_display} {bar} {cat_percentage:.1f}%")
                
                # サブカテゴリを出力
                if category in stats and isinstance(stats[category], dict):
                    # 表示するサブカテゴリをソート
                    sorted_subcategories = sorted(
                        stats[category].items(),
                        key=lambda x: (isinstance(x[1], tuple) and x[1][1] or 0),
                        reverse=True
                    )
                    
                    # サブカテゴリの数をカウント
                    subcategory_count = subcategory_counts.get(category, 0)
                    
                    for i, (subcategory, items) in enumerate(sorted_subcategories):
                        is_last_subcategory = i == len(sorted_subcategories) - 1 or i == subcategory_count - 1
                        
                        if isinstance(items, dict):
                            # サブカテゴリがさらに階層を持つ場合
                            # サブカテゴリの合計時間と割合を計算
                            subcat_time = sum(time for time, _ in items.values())
                            subcat_percentage = sum(percentage for _, percentage in items.values())
                            
                            if subcat_percentage > 0.5:  # 0.5%以上のみ表示
                                # サブカテゴリ名を整形（最後のサブカテゴリは└─、それ以外は├─）
                                prefix = "  └─" if is_last_subcategory else "  ├─"
                                subcategory_display = f"{prefix}{subcategory}".ljust(20)
                                
                                # ASCIIバーを作成
                                bar = self._create_ascii_bar(subcat_percentage)
                                
                                # サブカテゴリの合計時間と割合を出力
                                custom_log(f"{subcategory_display} {bar} {subcat_percentage:.1f}%")
                                
                                # サブカテゴリの項目を出力
                                sorted_items = sorted(items.items(), key=lambda x: x[1][1], reverse=True)
                                visible_items = [item for item in sorted_items if item[1][1] > 0.5]  # 0.5%以上のみ表示
                                
                                for j, (item_name, (item_time, item_percentage)) in enumerate(visible_items):
                                    # 項目名を整形（最後の項目は└─、それ以外は├─）
                                    item_prefix = "  │  └─" if j == len(visible_items) - 1 else "  │  ├─"
                                    if is_last_subcategory:
                                        item_prefix = "     └─" if j == len(visible_items) - 1 else "     ├─"
                                    
                                    item_display = f"{item_prefix}{item_name}".ljust(20)
                                    
                                    # ASCIIバーを作成
                                    bar = self._create_ascii_bar(item_percentage)
                                    
                                    # 項目の時間と割合を出力
                                    custom_log(f"{item_display} {bar} {item_percentage:.1f}%")
                        else:
                            # サブカテゴリが直接統計情報を持つ場合
                            time_value, percentage = items
                            
                            if percentage > 0.5:  # 0.5%以上のみ表示
                                # サブカテゴリ名を整形（最後のサブカテゴリは└─、それ以外は├─）
                                prefix = "  └─" if is_last_subcategory else "  ├─"
                                subcategory_display = f"{prefix}{subcategory}".ljust(20)
                                
                                # ASCIIバーを作成
                                bar = self._create_ascii_bar(percentage)
                                
                                # サブカテゴリの時間と割合を出力
                                custom_log(f"{subcategory_display} {bar} {percentage:.1f}%")
        
        # ドローン数を計算
        drone_count = len([k for k in self.total_timing_stats.keys() if k.startswith("drone_") and k.endswith("_total")])
        if drone_count > 0:
            custom_log(f"Number of drones: {drone_count}")
        
        # 全体の処理時間（改行なし）
        if total_time > 0:
            custom_log(f"Simulation Total: {total_time:.6f}s ({1.0/total_time:.2f} FPS, {dt/total_time*100:.2f}% of realtime)")

    def timed_function(self, key: str):
        """
        関数の実行時間を計測するデコレータ
        
        Args:
            key: 処理の識別子
            
        Returns:
            デコレータ関数
        """
        def decorator(func):
            def wrapper(*args, **kwargs):
                start_time = self.start_timer()
                result = func(*args, **kwargs)
                self.stop_timer(start_time, key)
                return result
            return wrapper
        return decorator
