#!/usr/bin/env python3

import math

# TF関連のインポート
try:
    from tf2_amber import (
        TransformStamped,
        Header,
        Transform,
        Time,
        Vector3,
        Quaternion,
    )
    from amber.importer.tf import TfImporter, TfImporterConfig
    HAS_TF = True
except ImportError:
    print("Warning: tf2_amber not found. TF functionality will be disabled.")
    HAS_TF = False

def create_tf_importer():
    """
    TFインポーターを作成する関数
    
    Returns:
        TfImporter or None: TFインポーターオブジェクト（TF機能が無効な場合はNone）
    """
    if HAS_TF:
        return TfImporter(TfImporterConfig())
    return None

def get_tf_from_link(cur_t, link, frame_id, child_frame_id):
    """
    リンクからTFメッセージを生成する関数
    
    Args:
        cur_t (float): 現在の時間
        link: リンクオブジェクト
        frame_id (str): フレームID
        child_frame_id (str): 子フレームID
        
    Returns:
        TransformStamped or None: TFメッセージ（TF機能が無効な場合はNone）
    """
    if not HAS_TF:
        return None
        
    return TransformStamped(
        Header(
            Time(
                int(math.floor(cur_t)),
                int((cur_t - float(math.floor(cur_t))) * pow(10, 9)),
            ),
            frame_id,
        ),
        child_frame_id,
        Transform(
            Vector3(link.get_pos()[0], link.get_pos()[1], link.get_pos()[2]),
            Quaternion(
                link.get_quat()[0],
                link.get_quat()[1],
                link.get_quat()[2],
                link.get_quat()[3],
            ),
        ),
    )

def finish_tf_importer(importer):
    """
    TFインポーターの終了処理を行う関数
    
    Args:
        importer: TFインポーターオブジェクト
    """
    if HAS_TF and importer:
        importer.finish()
