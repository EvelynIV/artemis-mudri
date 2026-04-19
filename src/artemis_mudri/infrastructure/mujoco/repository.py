from __future__ import annotations
"""MuJoCo XML 文件的落盘位置管理。"""

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[4]
MODELS_DIR = PROJECT_ROOT / "models"


def model_output_path() -> Path:
    """返回固定的车辆模型文件路径。"""
    return MODELS_DIR / "vehicel.xml"

def read_demo_model_xml_path() -> Path:
    """返回预置 MuJoCo XML 模型文件路径，并检查是否存在。"""
    model_path = model_output_path()
    if not model_path.exists():
        raise FileNotFoundError(f"Missing MuJoCo model file: {model_path}")
    return model_path
