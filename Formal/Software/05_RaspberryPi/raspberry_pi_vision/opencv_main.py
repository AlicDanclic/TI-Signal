# -*- coding: utf-8 -*-
"""单文件 OpenCV 示波器侧边拐点提取（固定机位版）。

摄像头、示波器和焦距固定后，程序直接使用预先标定的屏幕四角和电压标尺，
每帧只在左右窄带内提取高亮拐点，不再搜索屏幕边框或上下参考亮线。
按 q 或 ESC 退出，按 s 保存当前帧结果。
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np


# ============================== 可调参数 ==============================

# 有效斜坡宽度（微秒），对应 FPGA 输出的 τ。运行时可用 --ramp-us
# 在 100 / 500 / 2000 / 5000 μs 四档中选择。
EFFECTIVE_RAMP_DURATION_US = 500.0
RAMP_DURATION_CHOICES_US = (100.0, 500.0, 2000.0, 5000.0)

# 拐点不需要很高的像素密度，640x512 可显著降低树莓派计算量。
DEFAULT_SCREEN_SIZE = (640, 512)

# 最多保留的侧边拐点数量。0.1 ms 档配合 100 kHz 输入时理论上会出现
# 20 个拐点，因此保留 24 个余量；仍只处理左右窄带，不会拟合整条波形。
DEFAULT_MAX_POINTS = 32

# 树莓派实拍图只需在每侧约 12% 的窄带中寻找正弦极值。搜索带过宽会把
# 人物胸口、衣服边缘和中央网格纳入候选，既增加误检也浪费计算时间。
SIDE_SEARCH_FRACTION = 0.12
SIDE_SCORE_PERCENTILE = 94.0
SIDE_GREEN_PERCENTILE = 65.0
HORIZONTAL_ROI_FRACTION = 0.85

# 每帧会先自动估计 X 正弦波当前的左右极值线，再保留其附近的候选。这里的
# 4% 是相对“本帧检测宽度”的容差，不再依赖固定的示波器水平位置或幅度。
TURNING_POINT_EDGE_TOLERANCE_FRACTION = 0.04

# 下面参数只影响人眼观察的结果窗口，不参与拐点识别和频率计算。
# 模拟示波器与摄像头不同步时，单帧会出现密集横向扫描带；低权重多帧平均
# 可以在不拖慢识别算法的前提下，让静止波形逐帧变清楚。
DISPLAY_HEADER_HEIGHT = 82
DISPLAY_TEMPORAL_ALPHA = 0.16

# 仅供文件中保留的离线自动标定工具函数使用；实时入口不会调用自动标定。
REFERENCE_SEARCH_FRACTION = 0.36

# -------------------------- 固定机位标定 --------------------------
# 树莓派实拍标定图：Camera_screenshot_31.07.2026.png（640 x 480）。
# 上边框位于画面外，下面四角由左右内边框、下边框及 10 x 8 方格比例拟合。
# 角点允许为负数，透视变换会把没有拍到的顶部保留为黑色区域。
# 若比赛现场移动了摄像头，只需重新填写下面四个角点，不要重新启用逐帧搜索。
FIXED_CALIBRATION_FRAME_SIZE = (640, 480)
FIXED_SCREEN_CORNERS = (
    (17.0, -27.0),
    (556.0, -15.0),
    (549.0, 418.0),
    (4.0, 407.0),
)

# 以下参数均对应矫正后的 640 x 512 屏幕。
# 上下二次曲线不是运行时检测结果，而是固定机位的一次性标定数据；它们只用于
# 把拐点 Y 像素换算为锯齿扫描时间，不要求 FPGA 再输出两条参考亮线。
# 左右值只是动态检测失败前的几何初值；正常处理时会被当前帧结果替换。
FIXED_REFERENCE_LEFT_X = 84.0
FIXED_REFERENCE_RIGHT_X = 552.0
FIXED_REFERENCE_CENTER_X = 318.0
FIXED_REFERENCE_SCALE_X = 234.0

# Measured calibration from TI_code_main.py. Keep these curves fixed: they
# define the proven point-search region and the pixel/time conversion.
FIXED_TOP_CURVE = (-1.4984422, 1.6279430, 104.05)
FIXED_BOTTOM_CURVE = (0.5968569, -0.3133652, 469.05)

# 有效锯齿仍为 -2 V 到 +2 V。空闲段改为 +/-3 V 后位于有效标尺之外，
# 因此这里只留少量边缘余量，直接在整个有效扫描高度内寻找拐点。
FIXED_TRACE_EDGE_MARGIN = 5

# Ignore the upper/lower 7.5% of the active ramp while retaining the full
# calibrated height for time normalization.
POINT_SEARCH_CENTER_FRACTION = 0.85

# 640 x 512 矫正图中，-2 V 到 +2 V 有效锯齿的完整高度。
# 频率单独使用这个标尺和同侧完整周期，不受左右纵向剪切影响。
FREQUENCY_RAMP_HEIGHT_PX = 469.05

# 标准周期占多数；漏检一个同侧点会产生接近 2 倍的长间距。
STANDARD_PERIOD_TOLERANCE = 0.20
LONG_PERIOD_RATIO_MIN = 1.70

# 对最近 9 帧的完整周期取中位数，抑制摄像头 1～3 px 的单帧定位抖动。
TEMPORAL_PERIOD_WINDOW = 9


@dataclass(frozen=True)
class ReferenceLines:
    """上下参考曲线及其有效水平范围。

    CRT 屏幕存在几何失真，摄像头透视矫正后参考线仍可能倾斜或弯曲。
    top_curve / bottom_curve 保存归一化 X 坐标下的二次曲线系数
    ``a*x*x + b*x + c``；top_y / bottom_y 仅表示屏幕中央的高度，
    供状态显示和基本几何检查使用。
    """

    top_y: float
    bottom_y: float
    top_curve: tuple[float, float, float]
    bottom_curve: tuple[float, float, float]
    curve_center_x: float
    curve_scale_x: float
    top_band: tuple[int, int]
    bottom_band: tuple[int, int]
    left_x: float
    right_x: float
    confidence: float

    def _curve_y(self, coefficients: tuple[float, float, float], x_px: float) -> float:
        """计算指定 X 位置的参考线中心 Y 坐标。"""

        normalized_x = (float(x_px) - self.curve_center_x) / max(
            self.curve_scale_x, 1.0)
        a, b, c = coefficients
        return float((a * normalized_x + b) * normalized_x + c)

    def top_y_at(self, x_px: float) -> float:
        """返回指定 X 位置的上参考线中心。"""

        return self._curve_y(self.top_curve, x_px)

    def bottom_y_at(self, x_px: float) -> float:
        """返回指定 X 位置的下参考线中心。"""

        return self._curve_y(self.bottom_curve, x_px)


@dataclass(frozen=True)
class WavePoint:
    """一个最终输出的波形采样点。"""

    x_px: float
    y_px: float
    x_normalized: float
    y_normalized: float
    y_volts: float
    time_normalized: float
    strength: float


@dataclass
class ProcessResult:
    """单帧处理结果。"""

    corners: np.ndarray
    rectified: np.ndarray
    trace_mask: np.ndarray
    overlay: np.ndarray
    points: list[WavePoint]
    references: ReferenceLines
    avg_phase_interval: float    # 稳健估计的完整周期归一化间隔
    phase_interval_std: float    # 标准差
    valid_interval_count: int    # 有效间隔数
    frequency_hz: float          # 估计的频率（Hz）


class TemporalPeriodFilter:
    """用短窗口中位数稳定连续帧的完整周期。"""

    def __init__(self, window_size: int = TEMPORAL_PERIOD_WINDOW) -> None:
        self._periods: deque[float] = deque(maxlen=max(1, int(window_size)))

    def update(
        self,
        period_normalized: float,
        valid_count: int,
        ramp_duration_us: float,
    ) -> tuple[float, float]:
        if (
            valid_count < 2
            or period_normalized <= 0.0
            or ramp_duration_us <= 0.0
        ):
            return period_normalized, 0.0

        self._periods.append(float(period_normalized))
        stable_period = float(np.median(np.asarray(self._periods, np.float64)))
        period_sec = stable_period * ramp_duration_us / 1_000_000.0
        stable_frequency_hz = 1.0 / period_sec if period_sec > 0.0 else 0.0
        return stable_period, stable_frequency_hz


def write_image(path: Path, image: np.ndarray) -> None:
    """兼容 Windows 中文路径保存图片。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix if path.suffix else ".png"
    ok, encoded = cv2.imencode(suffix, image)
    if not ok:
        raise RuntimeError(f"无法编码图片：{path}")
    encoded.tofile(path)


def order_corners(points: np.ndarray) -> np.ndarray:
    """把四个角点统一整理为：左上、右上、右下、左下。"""

    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    ordered = np.empty((4, 2), dtype=np.float32)
    sums = points.sum(axis=1)
    differences = np.diff(points, axis=1).reshape(-1)
    ordered[0] = points[np.argmin(sums)]
    ordered[2] = points[np.argmax(sums)]
    ordered[1] = points[np.argmin(differences)]
    ordered[3] = points[np.argmax(differences)]
    return ordered


def get_fixed_screen_corners(frame: np.ndarray) -> np.ndarray:
    """按当前帧尺寸缩放固定机位的四角标定值。

    标定基准为 780 x 564。摄像头驱动若只做等比例像素缩放，程序仍可适配
    其他输出分辨率；若改变了裁切范围、焦距或相机位置，则必须重新标定四角。
    """

    frame_height, frame_width = frame.shape[:2]
    calibration_width, calibration_height = FIXED_CALIBRATION_FRAME_SIZE
    if frame_width < 2 or frame_height < 2:
        raise ValueError("摄像头帧尺寸无效")

    scale_x = (frame_width - 1) / max(1, calibration_width - 1)
    scale_y = (frame_height - 1) / max(1, calibration_height - 1)
    corners = np.asarray(FIXED_SCREEN_CORNERS, np.float32).copy()
    corners[:, 0] *= scale_x
    corners[:, 1] *= scale_y
    return order_corners(corners)


def get_fixed_reference_calibration(
    screen_size: tuple[int, int],
) -> ReferenceLines:
    """返回固定的像素/电压标尺，不读取当前帧中的上下亮线。

    二次曲线补偿模拟示波器的轻微几何失真。所有系数随矫正图尺寸缩放，
    默认 640 x 512 时即为文件顶部记录的一次性标定值。
    """

    width, height = screen_size
    base_width, base_height = DEFAULT_SCREEN_SIZE
    scale_x = (width - 1) / max(1, base_width - 1)
    scale_y = (height - 1) / max(1, base_height - 1)

    top_curve = tuple(value * scale_y for value in FIXED_TOP_CURVE)
    bottom_curve = tuple(value * scale_y for value in FIXED_BOTTOM_CURVE)
    left_x = FIXED_REFERENCE_LEFT_X * scale_x
    right_x = FIXED_REFERENCE_RIGHT_X * scale_x
    center_x = FIXED_REFERENCE_CENTER_X * scale_x
    curve_scale_x = FIXED_REFERENCE_SCALE_X * scale_x

    # band 字段只保留数据结构兼容性；直接提点流程不会根据图像搜索它们。
    top_values = [
        (top_curve[0] * normalized_x + top_curve[1]) * normalized_x + top_curve[2]
        for normalized_x in (-1.0, 0.0, 1.0)
    ]
    bottom_values = [
        (bottom_curve[0] * normalized_x + bottom_curve[1]) * normalized_x
        + bottom_curve[2]
        for normalized_x in (-1.0, 0.0, 1.0)
    ]
    top_band = (
        max(0, int(math.floor(min(top_values)))),
        min(height - 1, int(math.ceil(max(top_values)))),
    )
    bottom_band = (
        max(0, int(math.floor(min(bottom_values)))),
        min(height - 1, int(math.ceil(max(bottom_values)))),
    )

    return ReferenceLines(
        top_y=float(top_curve[2]),
        bottom_y=float(bottom_curve[2]),
        top_curve=top_curve,
        bottom_curve=bottom_curve,
        curve_center_x=center_x,
        curve_scale_x=curve_scale_x,
        top_band=top_band,
        bottom_band=bottom_band,
        left_x=left_x,
        right_x=right_x,
        confidence=1.0,
    )


def line_equation(segment: tuple[int, int, int, int]) -> np.ndarray:
    """把线段转换为 ax + by + c = 0。"""

    x1, y1, x2, y2 = [float(value) for value in segment]
    return np.asarray([y1 - y2, x2 - x1, x1 * y2 - x2 * y1], np.float64)


def line_intersection(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """计算两条直线的交点。"""

    cross = np.cross(first, second)
    if abs(float(cross[2])) < 1e-8:
        raise ValueError("屏幕边界直线近似平行，无法求交点")
    return (cross[:2] / cross[2]).astype(np.float32)


def detect_dark_screen_corners(gray: np.ndarray) -> np.ndarray:
    """利用“暗色屏幕区域”的凸包优先定位内屏四角。

    老式模拟示波器的内屏通常明显暗于浅色机壳。先找暗区再取凸包，
    可以避免把左侧机壳斜边误认为屏幕边界。若现场光照不满足这一特征，
    调用者仍会继续使用后面的霍夫直线方法作为兜底。
    """

    height, width = gray.shape
    frame_area = float(height * width)

    # 使用亮度分位数而不是固定阈值，兼容不同曝光和摄像头。
    dark_limit = int(np.clip(np.percentile(gray, 18.0), 35, 115))
    dark_mask = cv2.threshold(
        gray, dark_limit, 255, cv2.THRESH_BINARY_INV)[1]

    # 补齐屏幕内部被亮参考线、波形和网格切断的小空洞。
    kernel_size = max(9, int(round(min(height, width) * 0.035)) | 1)
    dark_mask = cv2.morphologyEx(
        dark_mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(
            cv2.MORPH_RECT, (kernel_size, kernel_size)),
    )

    contours = cv2.findContours(
        dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    candidates: list[tuple[float, np.ndarray]] = []
    expected_ratio = DEFAULT_SCREEN_SIZE[0] / DEFAULT_SCREEN_SIZE[1]

    for contour in contours:
        x, y, box_width, box_height = cv2.boundingRect(contour)
        if not (width * 0.50 <= box_width <= width * 0.90):
            continue
        if not (height * 0.52 <= box_height <= height * 0.93):
            continue
        if x < width * 0.05:
            # 贴住画面左边的暗区通常是背景或机壳外部，不是内屏。
            continue
        if not (x <= width * 0.5 <= x + box_width):
            continue
        if not (y <= height * 0.5 <= y + box_height):
            continue

        hull = cv2.convexHull(contour)
        perimeter = cv2.arcLength(hull, True)
        if perimeter <= 0.0:
            continue

        quadrilateral: np.ndarray | None = None
        for epsilon_ratio in (0.012, 0.018, 0.025, 0.035, 0.050):
            approximation = cv2.approxPolyDP(
                hull, epsilon_ratio * perimeter, True)
            if len(approximation) == 4 and cv2.isContourConvex(approximation):
                quadrilateral = order_corners(approximation.reshape(4, 2))
                break
        if quadrilateral is None:
            continue

        area = abs(float(cv2.contourArea(quadrilateral)))
        area_ratio = area / frame_area
        if not 0.30 <= area_ratio <= 0.82:
            continue

        top_width = float(np.linalg.norm(quadrilateral[1] - quadrilateral[0]))
        bottom_width = float(np.linalg.norm(quadrilateral[2] - quadrilateral[3]))
        left_height = float(np.linalg.norm(quadrilateral[3] - quadrilateral[0]))
        right_height = float(np.linalg.norm(quadrilateral[2] - quadrilateral[1]))
        mean_width = 0.5 * (top_width + bottom_width)
        mean_height = 0.5 * (left_height + right_height)
        if mean_width < width * 0.48 or mean_height < height * 0.50:
            continue

        aspect_ratio = mean_width / max(mean_height, 1.0)
        if not 0.85 <= aspect_ratio <= 1.85:
            continue

        center = np.mean(quadrilateral, axis=0)
        center_error = (
            abs(float(center[0]) - width * 0.5) / width +
            abs(float(center[1]) - height * 0.5) / height
        )
        ratio_error = abs(math.log(max(aspect_ratio, 1e-6) / expected_ratio))
        score = area_ratio - 0.18 * center_error - 0.12 * ratio_error
        candidates.append((score, quadrilateral))

    if not candidates:
        raise ValueError("暗区法没有找到可信的内屏四边形")
    return max(candidates, key=lambda item: item[0])[1]


def _detect_screen_corners_once(
    frame: np.ndarray,
    prefer_dark_outline: bool,
) -> np.ndarray:
    """利用屏幕四周长边自动检测屏幕四角。

    这里只把霍夫变换用于寻找屏幕外框，不用它检测上下参考线。
    参考线采用后面的亮度行投影算法，因此不会因为线条太粗而漏检。
    """

    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    if prefer_dark_outline:
        # 暗区法速度更快，作为霍夫边界无法闭合时的兜底。
        try:
            return detect_dark_screen_corners(gray)
        except ValueError:
            pass

    # 根据图像中位亮度自动设置 Canny 阈值，适应不同曝光。
    median = float(np.median(gray))
    lower = int(max(15, 0.45 * median))
    upper = int(min(220, max(lower + 25, 1.35 * median)))
    edges = cv2.Canny(gray, lower, upper)

    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 360.0,
        threshold=max(45, min(height, width) // 8),
        minLineLength=max(120, int(min(height, width) * 0.27)),
        maxLineGap=max(20, int(min(height, width) * 0.08)),
    )
    if lines is None:
        raise ValueError("没有检测到足够长的屏幕边界")

    horizontal: list[tuple[float, float, tuple[int, int, int, int]]] = []
    vertical: list[tuple[float, float, tuple[int, int, int, int]]] = []
    for raw in lines[:, 0]:
        x1, y1, x2, y2 = [int(value) for value in raw]
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        angle = abs(math.degrees(math.atan2(dy, dx)))
        angle = min(angle, abs(180.0 - angle))
        midpoint_x = 0.5 * (x1 + x2)
        midpoint_y = 0.5 * (y1 + y2)
        segment = (x1, y1, x2, y2)
        if angle <= 14.0:
            horizontal.append((length, midpoint_y, segment))
        elif angle >= 72.0:
            vertical.append((length, midpoint_x, segment))

    if len(horizontal) < 2 or len(vertical) < 2:
        raise ValueError("没有找到完整的屏幕横边和竖边")

    # 左边界优先选择靠近画面内部且较长的竖线，避开机壳外轮廓。
    left_candidates = [
        item for item in vertical
        if width * 0.07 <= item[1] <= width * 0.42
    ]
    right_candidates = [
        item for item in vertical
        if width * 0.58 <= item[1] <= width * 0.98
    ]
    if not left_candidates or not right_candidates:
        raise ValueError("无法确定屏幕左右边界")

    def left_edge_polarity(segment: tuple[int, int, int, int]) -> float:
        """判断左边界是否满足“左侧亮机壳、右侧暗屏幕”。"""

        x1, y1, x2, y2 = [float(value) for value in segment]
        sample_count = 56
        offset = max(6, int(round(width * 0.012)))
        radius = 2
        left_values: list[float] = []
        right_values: list[float] = []
        for factor in np.linspace(0.08, 0.92, sample_count):
            x = int(round(x1 + factor * (x2 - x1)))
            y = int(round(y1 + factor * (y2 - y1)))
            if not (radius <= y < height - radius):
                continue
            left_x = x - offset
            right_x = x + offset
            if not (radius <= left_x < width - radius):
                continue
            if not (radius <= right_x < width - radius):
                continue
            left_patch = gray[
                y - radius:y + radius + 1,
                left_x - radius:left_x + radius + 1,
            ]
            right_patch = gray[
                y - radius:y + radius + 1,
                right_x - radius:right_x + radius + 1,
            ]
            left_values.append(float(np.median(left_patch)))
            right_values.append(float(np.median(right_patch)))
        if not left_values:
            return -255.0
        return float(np.median(left_values) - np.median(right_values))

    # 外壳斜边常比内屏边界更长，不能只按线长选择。
    inner_left_candidates = [
        item for item in left_candidates
        if left_edge_polarity(item[2]) > 10.0
    ]
    left_pool = inner_left_candidates or left_candidates
    left = max(left_pool, key=lambda item: item[0] + 0.18 * item[1])
    right = max(right_candidates, key=lambda item: item[0] - 0.05 * item[1])
    left_x = left[1]
    right_x = right[1]
    if right_x - left_x < width * 0.45:
        raise ValueError("检测到的屏幕宽度过小")

    # 水平边必须大部分位于左右屏幕边界之间。
    def overlaps_screen(segment: tuple[int, int, int, int]) -> bool:
        x1, _, x2, _ = segment
        segment_left = min(x1, x2)
        segment_right = max(x1, x2)
        overlap = min(segment_right, right_x) - max(segment_left, left_x)
        return overlap >= width * 0.22

    top_candidates = [
        item for item in horizontal
        if height * 0.035 <= item[1] <= height * 0.34
        and overlaps_screen(item[2])
    ]
    bottom_candidates = [
        item for item in horizontal
        if height * 0.62 <= item[1] <= height * 0.93
        and overlaps_screen(item[2])
    ]
    if not top_candidates or not bottom_candidates:
        raise ValueError("无法确定屏幕上下边界")

    # 顶边取最靠上的长线，底边取 93% 高度以内最靠下的长线。
    top = min(top_candidates, key=lambda item: item[1] - 0.001 * item[0])
    bottom = max(bottom_candidates, key=lambda item: item[1] + 0.001 * item[0])

    left_line = line_equation(left[2])
    right_line = line_equation(right[2])
    top_line = line_equation(top[2])
    bottom_line = line_equation(bottom[2])
    corners = order_corners(np.asarray([
        line_intersection(left_line, top_line),
        line_intersection(right_line, top_line),
        line_intersection(right_line, bottom_line),
        line_intersection(left_line, bottom_line),
    ]))

    # 基本几何检查，防止把机壳边缘误当成屏幕。
    area = abs(float(cv2.contourArea(corners)))
    frame_area = float(height * width)
    if not frame_area * 0.25 <= area <= frame_area * 0.88:
        raise ValueError(f"屏幕四边形面积异常：{area:.0f}")
    return corners


def detect_screen_corners(frame: np.ndarray) -> np.ndarray:
    """优先按真实外框直线定位内屏，失败时再使用暗区轮廓。

    粗亮参考线紧贴屏幕边缘时，暗区轮廓可能在亮线处提前结束并裁掉
    一部分参考线；霍夫外框不会受到这个问题影响。
    """

    try:
        return _detect_screen_corners_once(frame, prefer_dark_outline=False)
    except ValueError:
        return _detect_screen_corners_once(frame, prefer_dark_outline=True)


def rectify_screen(
    frame: np.ndarray,
    corners: np.ndarray,
    size: tuple[int, int],
) -> np.ndarray:
    """把倾斜屏幕矫正为固定大小的正视图。"""

    width, height = size
    destination = np.asarray([
        [0, 0],
        [width - 1, 0],
        [width - 1, height - 1],
        [0, height - 1],
    ], dtype=np.float32)
    transform = cv2.getPerspectiveTransform(order_corners(corners), destination)
    return cv2.warpPerspective(frame, transform, (width, height))


def smooth_profile(values: np.ndarray, size: int) -> np.ndarray:
    """对一维曲线做高斯平滑。"""

    size = max(3, int(size) | 1)
    return cv2.GaussianBlur(
        values.astype(np.float32).reshape(-1, 1),
        (1, size),
        0,
    ).reshape(-1)


def top_fraction_mean(values: np.ndarray, fraction: float) -> np.ndarray:
    """每一行只统计最亮的一部分像素，减小暗网格的影响。"""

    count = max(1, int(round(values.shape[1] * fraction)))
    selected = np.partition(values, -count, axis=1)[:, -count:]
    return np.mean(selected, axis=1)


def detect_band(
    activity: np.ndarray,
    start: int,
    stop: int,
) -> tuple[int, int, float, float]:
    """在指定纵向范围内寻找一条粗亮线及其上下边界。"""

    region = activity[start:stop]
    if region.size < 5:
        raise ValueError("参考线搜索范围太小")
    peak = start + int(np.argmax(region))
    baseline = float(np.percentile(region, 35.0))
    peak_value = float(activity[peak])
    contrast = peak_value - baseline
    if contrast < 7.0:
        raise ValueError("参考线与背景的亮度差太小")

    threshold = baseline + 0.34 * contrast
    lower = peak
    upper = peak
    while lower > start and activity[lower - 1] >= threshold:
        lower -= 1
    while upper + 1 < stop and activity[upper + 1] >= threshold:
        upper += 1

    rows = np.arange(lower, upper + 1, dtype=np.float32)
    weights = np.maximum(activity[lower:upper + 1] - baseline, 0.0)
    center = (
        float(np.sum(rows * weights) / np.sum(weights))
        if float(np.sum(weights)) > 0.0
        else float(peak)
    )
    return lower, upper, center, contrast


def longest_run(binary: np.ndarray) -> tuple[int, int] | None:
    """返回一维布尔数组中最长的连续真值区间。"""

    padded = np.pad(binary.astype(np.int8), (1, 1))
    changes = np.diff(padded)
    starts = np.flatnonzero(changes == 1)
    stops = np.flatnonzero(changes == -1)
    if starts.size == 0:
        return None
    lengths = stops - starts
    index = int(np.argmax(lengths))
    return int(starts[index]), int(stops[index])


def fit_reference_curve(
    score: np.ndarray,
    rough_band: tuple[int, int],
    rough_center_y: float,
    left_x: float,
    right_x: float,
    curve_center_x: float,
    curve_scale_x: float,
) -> tuple[tuple[float, float, float], tuple[int, int], float, float]:
    """逐列定位粗亮线中心，并做带离群点剔除的二次曲线拟合。

    返回值依次为：曲线系数、覆盖整条曲线的纵向包络、拟合残差、
    有效列覆盖率。逐列搜索使用局部连续亮带，而不是单个最亮像素，
    因此参考线较粗、局部过曝或穿过网格时仍能得到稳定中心。
    """

    height, width = score.shape
    rough_low, rough_high = rough_band
    x_start = max(1, int(math.floor(left_x)))
    x_stop = min(width - 2, int(math.ceil(right_x)))
    if x_stop - x_start < width * 0.25:
        raise ValueError("参考线曲线拟合的水平范围过小")

    rough_thickness = max(1, rough_high - rough_low + 1)
    vertical_padding = max(
        12,
        int(round(height * 0.055)),
        int(round(rough_thickness * 0.9)),
    )
    search_start = max(1, rough_low - vertical_padding)
    search_stop = min(height - 2, rough_high + vertical_padding)
    if search_stop - search_start < 12:
        raise ValueError("参考线曲线拟合的纵向范围过小")

    # 小范围二维平滑只用于参考线定位；原图和后续拐点检测不受影响。
    roi = score[search_start:search_stop + 1, x_start:x_stop + 1]
    smoothed = cv2.GaussianBlur(roi.astype(np.float32), (5, 3), 0)
    candidates: list[tuple[float, float, float, float, float, float, float]] = []

    for local_x in range(smoothed.shape[1]):
        column = smoothed[:, local_x]
        baseline = float(np.percentile(column, 28.0))
        peak_index = int(np.argmax(column))
        peak_value = float(column[peak_index])
        contrast = peak_value - baseline
        if contrast < 5.0:
            continue

        # 取包含峰值的连续亮带，避免同列的网格或波形参与中心计算。
        threshold = baseline + 0.38 * contrast
        low = peak_index
        high = peak_index
        while low > 0 and float(column[low - 1]) >= threshold:
            low -= 1
        while high + 1 < column.size and float(column[high + 1]) >= threshold:
            high += 1

        thickness = high - low + 1
        clipped_low = low == 0
        clipped_high = high == column.size - 1
        if clipped_low and clipped_high:
            continue
        if thickness < 2 or thickness > max(10, int(round(height * 0.11))):
            continue

        rows = np.arange(low, high + 1, dtype=np.float32)
        weights = np.maximum(column[low:high + 1] - baseline, 0.0)
        weight_sum = float(np.sum(weights))
        if weight_sum <= 0.0:
            continue
        center_y = search_start + float(np.sum(rows * weights) / weight_sum)
        candidates.append((
            float(x_start + local_x),
            center_y,
            float(search_start + low),
            float(search_start + high),
            contrast,
            float(thickness),
            float(clipped_low),
            float(clipped_high),
        ))

    minimum_columns = max(24, int(round((x_stop - x_start + 1) * 0.32)))
    if len(candidates) < minimum_columns:
        raise ValueError("参考线有效采样列不足")

    samples = np.asarray(candidates, dtype=np.float64)
    # 两端没有参考线时，普通波形也可能形成局部峰；参考线覆盖大多数列且更亮，
    # 先按整体峰值和对比度中位数剔除这些弱候选。
    contrast_limit = max(6.0, float(np.median(samples[:, 4])) * 0.42)
    strong = samples[:, 4] >= contrast_limit
    if int(np.count_nonzero(strong)) >= minimum_columns:
        samples = samples[strong]

    normalized_x = (samples[:, 0] - curve_center_x) / max(curve_scale_x, 1.0)
    clipped_low_fraction = float(np.mean(samples[:, 6]))
    clipped_high_fraction = float(np.mean(samples[:, 7]))
    if clipped_high_fraction >= 0.35:
        # 下参考线常紧贴内屏下边缘。此时拟合朝屏幕内部的上边缘，
        # 最后再用全局行投影中心恢复其真实纵向位置。
        y_values = samples[:, 2]
    elif clipped_low_fraction >= 0.35:
        y_values = samples[:, 3]
    else:
        y_values = samples[:, 1]
    contrast_values = samples[:, 4]
    weights = np.sqrt(np.clip(
        contrast_values / max(float(np.median(contrast_values)), 1.0),
        0.35,
        3.0,
    ))
    keep = np.ones(samples.shape[0], dtype=bool)
    coefficients = np.asarray([0.0, 0.0, float(np.median(y_values))])

    # 反复拟合并按 MAD 剔除离群列，可抵抗网格交点、反光和波形交叉。
    for _ in range(6):
        if int(np.count_nonzero(keep)) < minimum_columns:
            break
        coefficients = np.polyfit(
            normalized_x[keep], y_values[keep], 2, w=weights[keep])
        residuals = y_values - np.polyval(coefficients, normalized_x)
        residual_center = float(np.median(residuals[keep]))
        mad = float(np.median(np.abs(residuals[keep] - residual_center)))
        residual_limit = max(1.5, 3.5 * 1.4826 * mad)
        new_keep = np.abs(residuals - residual_center) <= residual_limit
        if int(np.count_nonzero(new_keep)) < minimum_columns:
            break
        if np.array_equal(new_keep, keep):
            keep = new_keep
            break
        keep = new_keep

    if int(np.count_nonzero(keep)) < minimum_columns:
        raise ValueError("参考线曲线拟合后的有效采样列不足")

    # 极端二次项通常来自局部反光，而不是 CRT 的正常几何弯曲；此时退回直线。
    if abs(float(coefficients[0])) > height * 0.10:
        linear = np.polyfit(
            normalized_x[keep], y_values[keep], 1, w=weights[keep])
        coefficients = np.asarray([0.0, float(linear[0]), float(linear[1])])

    # 边缘拟合只负责斜率和曲率；纵向绝对位置仍以全宽行投影为准。
    # 这一步也可消除粗亮线局部过曝造成的固定中心偏差。
    sample_curve_y = np.polyval(coefficients, normalized_x[keep])
    coefficients[2] += float(rough_center_y) - float(np.median(sample_curve_y))

    fitted_y = np.polyval(coefficients, normalized_x[keep])
    vertical_offset = float(rough_center_y) - float(np.median(sample_curve_y))
    absolute_residuals = np.abs((y_values[keep] + vertical_offset) - fitted_y)
    fit_residual = float(np.median(absolute_residuals))
    typical_thickness = float(np.percentile(samples[keep, 5], 75.0))

    curve_x = np.linspace(left_x, right_x, max(32, x_stop - x_start + 1))
    curve_normalized_x = (
        (curve_x - curve_center_x) / max(curve_scale_x, 1.0)
    )
    curve_y = np.polyval(coefficients, curve_normalized_x)
    envelope_margin = max(
        3.0,
        0.55 * typical_thickness + 2.0,
        3.0 * fit_residual + 1.0,
    )
    band = (
        max(0, int(math.floor(float(np.min(curve_y)) - envelope_margin))),
        min(height - 1, int(math.ceil(float(np.max(curve_y)) + envelope_margin))),
    )
    coverage = float(np.count_nonzero(keep)) / max(1.0, right_x - left_x + 1.0)
    return (
        tuple(float(value) for value in coefficients),
        band,
        fit_residual,
        coverage,
    )


def detect_reference_lines(screen: np.ndarray) -> ReferenceLines:
    """检测上、下参考线。

    先用行亮度投影找到上下粗带，再逐列提取中心并拟合二次曲线。
    这样既不怕参考线过粗，也能补偿模拟示波器的倾斜和桶形失真。
    """

    blue, green, red = cv2.split(screen.astype(np.int16))
    brightness = np.max(screen, axis=2).astype(np.float32)
    green_excess = np.clip(green - np.maximum(blue, red), 0, 255).astype(np.float32)

    # 白绿色参考线可能不是纯绿色，因此同时使用亮度和绿色占优量。
    score = 0.72 * brightness + 0.85 * green_excess
    height, width = score.shape
    x_margin = max(4, int(round(width * 0.04)))
    row_activity = top_fraction_mean(score[:, x_margin:width - x_margin], 0.28)
    row_activity = smooth_profile(row_activity, max(5, height // 70))

    search = min(0.45, max(0.25, REFERENCE_SEARCH_FRACTION))
    top_start = max(1, int(height * 0.02))
    top_stop = max(top_start + 5, int(height * search))
    bottom_start = min(height - 6, int(height * (1.0 - search)))
    bottom_stop = min(height - 1, int(height * 0.98))

    top_low, top_high, top_y, top_contrast = detect_band(
        row_activity, top_start, top_stop)
    bottom_low, bottom_high, bottom_y, bottom_contrast = detect_band(
        row_activity, bottom_start, bottom_stop)

    if bottom_y - top_y < height * 0.48:
        raise ValueError("上下参考线距离过近")

    def horizontal_span(low: int, high: int) -> tuple[int, int]:
        band = score[low:high + 1]
        # 只使用该带内较亮像素，并对列覆盖率做闭运算补齐小缺口。
        threshold = max(
            float(np.percentile(band, 70.0)),
            float(np.percentile(score, 91.0)),
        )
        coverage = np.mean(band >= threshold, axis=0)
        active = (coverage >= 0.10).astype(np.uint8) * 255
        close_width = max(9, width // 35)
        active = cv2.morphologyEx(
            active.reshape(1, -1),
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (close_width, 1)),
        ).reshape(-1) > 0
        active[:x_margin] = False
        active[width - x_margin:] = False
        run = longest_run(active)
        if run is None or run[1] - run[0] < width * 0.30:
            raise ValueError("参考线横向长度不足")
        return run

    top_left, top_right = horizontal_span(top_low, top_high)
    bottom_left, bottom_right = horizontal_span(bottom_low, bottom_high)
    # 两条水平线都由同一个 X 信号扫出，本应具有相同的左右范围。
    # CRT 几何失真会让倾斜线的一端落出检测带，因此取二者联合范围更可靠。
    left_x = float(min(top_left, bottom_left))
    right_x = float(max(top_right - 1, bottom_right - 1))
    horizontal_width = right_x - left_x
    if horizontal_width < width * 0.38:
        raise ValueError("上下参考线的水平范围过小")

    curve_center_x = 0.5 * (left_x + right_x)
    curve_scale_x = max(1.0, 0.5 * horizontal_width)
    top_curve, top_band, top_residual, top_coverage = fit_reference_curve(
        score,
        (top_low, top_high),
        top_y,
        left_x,
        right_x,
        curve_center_x,
        curve_scale_x,
    )
    bottom_curve, bottom_band, bottom_residual, bottom_coverage = fit_reference_curve(
        score,
        (bottom_low, bottom_high),
        bottom_y,
        left_x,
        right_x,
        curve_center_x,
        curve_scale_x,
    )
    top_y = float(top_curve[2])
    bottom_y = float(bottom_curve[2])
    if bottom_y - top_y < height * 0.48:
        raise ValueError("拟合后的上下参考线距离过近")

    contrast_score = min(1.0, min(top_contrast, bottom_contrast) / 45.0)
    span_score = min(1.0, horizontal_width / (width * 0.65))
    separation_score = min(1.0, (bottom_y - top_y) / (height * 0.75))
    curve_score = min(
        1.0,
        min(top_coverage, bottom_coverage) / 0.72,
    ) * math.exp(-max(top_residual, bottom_residual) / 5.0)
    confidence = (
        0.38 * contrast_score +
        0.22 * span_score +
        0.16 * separation_score +
        0.24 * curve_score
    )

    return ReferenceLines(
        top_y=top_y,
        bottom_y=bottom_y,
        top_curve=top_curve,
        bottom_curve=bottom_curve,
        curve_center_x=curve_center_x,
        curve_scale_x=curve_scale_x,
        top_band=top_band,
        bottom_band=bottom_band,
        left_x=left_x,
        right_x=right_x,
        confidence=float(np.clip(confidence, 0.0, 1.0)),
    )


def build_side_trace_score(
    screen: np.ndarray,
    y_start: int,
    y_stop: int,
    side_bands: tuple[tuple[int, int], tuple[int, int]],
) -> tuple[np.ndarray, np.ndarray]:
    """只增强左右窄带中的绿色亮轨迹。

    旧算法会在整幅图上做大核形态学运算。这里改为对两个小区域使用
    9x9 均值背景差，中央波形和中央反光都不会参与计算。
    """

    height, width = screen.shape[:2]
    score = np.zeros((height, width), np.float32)
    green_excess = np.zeros((height, width), np.uint8)

    for x_start, x_stop in side_bands:
        crop = screen[y_start:y_stop, x_start:x_stop]
        blue, green, red = cv2.split(crop)
        green_i16 = green.astype(np.int16)
        local_excess = np.clip(
            green_i16 - np.maximum(blue, red).astype(np.int16), 0, 255
        ).astype(np.uint8)

        # 小核均值滤波估计局部背景，绿色细亮轨迹保留为正差值。
        background = cv2.boxFilter(
            green, cv2.CV_8U, (9, 9), normalize=True)
        detail = cv2.subtract(green, background).astype(np.float32)
        local_score = (
            0.72 * detail +
            0.45 * local_excess.astype(np.float32)
        )
        local_score = cv2.GaussianBlur(local_score, (3, 3), 0)
        score[y_start:y_stop, x_start:x_stop] = local_score
        green_excess[y_start:y_stop, x_start:x_stop] = local_excess

    return score, green_excess


def estimate_waveform_edges(
    screen: np.ndarray,
    y_start: int,
    y_stop: int,
) -> tuple[float, float, np.ndarray, np.ndarray]:
    """从当前帧自动估计正弦波的左右极值线。

    真正的正弦极值在多处 Y 位置形成短小的竖直回转段。先在左右半屏做绿色
    局部高通，再用竖向开运算累加这些重复回转段；人物反光虽然面积大，但不
    会在同一 X 坐标形成多次细窄回转，因此不会主导列峰值。

    返回 ``(left_x, right_x, score, green_excess)``。后两个数组会由正式提点
    流程复用，避免为了动态定位重复计算整幅图的局部高通。
    """

    height, width = screen.shape[:2]
    center_x = width // 2
    # The camera is fixed. Ignore the outer 7.5% on both sides and search only
    # the requested middle 85% of the rectified oscilloscope image.
    outer_margin = max(10, int(round(
        width * (1.0 - HORIZONTAL_ROI_FRACTION) * 0.5)))
    center_gap = max(12, int(round(width * 0.035)))
    broad_bands = (
        (outer_margin, center_x - center_gap),
        (center_x + center_gap, width - outer_margin),
    )
    if min(stop - start for start, stop in broad_bands) < 40:
        raise ValueError("动态极值搜索区域太窄")

    score, green_excess = build_side_trace_score(
        screen,
        y_start,
        y_stop,
        broad_bands,
    )
    broad_scores = np.concatenate([
        score[y_start:y_stop, start:stop].reshape(-1)
        for start, stop in broad_bands
    ])
    broad_green = np.concatenate([
        green_excess[y_start:y_stop, start:stop].reshape(-1)
        for start, stop in broad_bands
    ])
    score_threshold = max(
        3.0,
        float(np.percentile(broad_scores, SIDE_SCORE_PERCENTILE)),
    )
    green_threshold = max(
        4.0,
        float(np.percentile(broad_green, SIDE_GREEN_PERCENTILE)),
    )

    broad_mask = np.zeros((height, width), np.uint8)
    for start, stop in broad_bands:
        local_score = score[y_start:y_stop, start:stop]
        local_green = green_excess[y_start:y_stop, start:stop]
        broad_mask[y_start:y_stop, start:stop] = np.where(
            (local_score >= score_threshold)
            & (local_green >= green_threshold),
            255,
            0,
        ).astype(np.uint8)
    broad_mask = cv2.morphologyEx(
        broad_mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )

    vertical_size = max(7, int(round((y_stop - y_start) * 0.021)) | 1)
    vertical_turns = cv2.morphologyEx(
        broad_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, vertical_size)),
    )
    weighted_turns = np.where(vertical_turns > 0, score, 0.0)
    column_profile = np.sum(weighted_turns, axis=0).astype(np.float32)
    smooth_width = max(9, int(round(width * 0.033)) | 1)
    column_profile = cv2.GaussianBlur(
        column_profile.reshape(1, -1),
        (smooth_width, 1),
        0,
    ).reshape(-1)

    def strongest_column(start: int, stop: int) -> tuple[float, float]:
        region = column_profile[start:stop]
        if region.size == 0:
            return 0.0, 0.0
        local_index = int(np.argmax(region))
        return float(start + local_index), float(region[local_index])

    left_x, left_strength = strongest_column(*broad_bands[0])
    right_x, right_strength = strongest_column(*broad_bands[1])
    if min(left_strength, right_strength) <= 0.0:
        raise ValueError("当前帧无法定位正弦波左右极值")
    if right_x - left_x < width * 0.38:
        raise ValueError("当前帧检测到的正弦波水平幅度过小")

    return left_x, right_x, score, green_excess


def find_profile_peaks(
    profile: np.ndarray,
    start: int,
    stop: int,
    minimum_distance: int,
    maximum_count: int,
) -> list[tuple[int, float]]:
    """从纵向亮度曲线中寻找少量互相分离的峰值。"""

    region = profile[start:stop].astype(np.float32)
    if region.size < 5 or float(np.max(region)) <= 0.0:
        return []

    # 平滑只在一维数组上进行，计算量远低于逐行拟合完整轨迹。
    smooth_size = max(5, int(round(region.size * 0.018)) | 1)
    smoothed = smooth_profile(region, smooth_size)
    positive = smoothed[smoothed > 0.0]
    if positive.size < 3:
        return []

    low_level = float(np.percentile(positive, 35.0))
    high_level = float(np.percentile(positive, 92.0))
    threshold = low_level + 0.24 * max(0.0, high_level - low_level)

    # 先找三点局部极大值，再按强度做纵向非极大值抑制。
    local_maximum = np.zeros(region.size, dtype=bool)
    local_maximum[1:-1] = (
        (smoothed[1:-1] >= smoothed[:-2]) &
        (smoothed[1:-1] >= smoothed[2:]) &
        (smoothed[1:-1] >= threshold)
    )
    candidate_indices = np.flatnonzero(local_maximum)
    if candidate_indices.size == 0:
        candidate_indices = np.asarray([int(np.argmax(smoothed))], np.int32)

    ordered = sorted(
        candidate_indices.tolist(),
        key=lambda index: float(smoothed[index]),
        reverse=True,
    )
    selected: list[int] = []
    for index in ordered:
        if all(abs(index - previous) >= minimum_distance for previous in selected):
            selected.append(index)
            if len(selected) >= maximum_count:
                break

    return sorted(
        [(start + index, float(smoothed[index])) for index in selected],
        key=lambda item: item[0],
    )


def localize_turning_point(
    score: np.ndarray,
    mask: np.ndarray,
    peak_y: int,
    x_start: int,
    x_stop: int,
    side: str,
    vertical_radius: int,
) -> tuple[float, float, float] | None:
    """在一个侧边亮峰附近定位真正的左/右极值点。"""

    height = score.shape[0]
    y_start = max(0, peak_y - vertical_radius)
    y_stop = min(height, peak_y + vertical_radius + 1)
    local_mask = mask[y_start:y_stop, x_start:x_stop] > 0
    rows, columns = np.nonzero(local_mask)
    if rows.size < 3:
        return None

    values = score[y_start:y_stop, x_start:x_stop][rows, columns]
    # 只使用局部较亮像素，避免稀疏噪点把拐点拉向中间。
    brightness_limit = float(np.percentile(values, 45.0))
    bright = values >= brightness_limit
    rows = rows[bright]
    columns = columns[bright]
    values = values[bright]
    if rows.size < 2:
        return None

    global_x = columns.astype(np.float32) + float(x_start)
    global_y = rows.astype(np.float32) + float(y_start)
    edge_quantile = 18.0 if side == "left" else 82.0
    edge_x = float(np.percentile(global_x, edge_quantile))
    edge_margin = max(2.0, (x_stop - x_start) * 0.035)
    if side == "left":
        edge_pixels = global_x <= edge_x + edge_margin
    else:
        edge_pixels = global_x >= edge_x - edge_margin

    global_x = global_x[edge_pixels]
    global_y = global_y[edge_pixels]
    values = values[edge_pixels]
    if global_x.size == 0:
        return None

    weights = np.maximum(values - float(np.min(values)) + 1.0, 1.0)
    weight_sum = float(np.sum(weights))
    x_px = float(np.sum(global_x * weights) / weight_sum)
    y_px = float(np.sum(global_y * weights) / weight_sum)
    strength = float(np.max(values))
    return x_px, y_px, strength


def select_alternating_edge_points(
    candidates: list[tuple[float, float, float]],
    left_x: float,
    right_x: float,
) -> list[tuple[float, float, float]]:
    """选择左右极值交替、点数最多且总强度最高的候选序列。

    X 轴为正弦波时，相邻真实拐点一定在左右两侧交替出现。动态规划允许从
    任意一侧开始，也不假定最终点数，因此能删除靠近边界的局部强反光，而
    不需要为 0.1/0.5/2 ms 三档分别写死点数。
    """

    ordered = sorted(candidates, key=lambda item: item[1], reverse=True)
    if len(ordered) < 2:
        return ordered

    sides = [
        0 if abs(item[0] - left_x) <= abs(item[0] - right_x) else 1
        for item in ordered
    ]
    # 每个状态保存：序列长度、累计强度、候选下标路径。
    states: list[tuple[int, float, list[int]]] = []
    for index, candidate in enumerate(ordered):
        best = (1, float(candidate[2]), [index])
        for previous in range(index):
            if sides[previous] == sides[index]:
                continue
            previous_state = states[previous]
            proposal = (
                previous_state[0] + 1,
                previous_state[1] + float(candidate[2]),
                previous_state[2] + [index],
            )
            if proposal[:2] > best[:2]:
                best = proposal
        states.append(best)

    winner = max(states, key=lambda state: (state[0], state[1]))
    return [ordered[index] for index in winner[2]]


def extract_waveform_points(
    screen: np.ndarray,
    references: ReferenceLines,
    maximum_points: int,
) -> tuple[list[WavePoint], np.ndarray, float, float]:
    """只提取左右侧高亮拐点，不拟合中间的完整波形。"""

    height, width = screen.shape[:2]
    # 固定机位下不再先寻找并屏蔽两条参考亮线。直接根据一次性标定曲线确定
    # -2 V 到 +2 V 的有效锯齿高度，只排除边缘少量像素。
    boundary_x = (width * 0.08, width * 0.92)
    top_limit = min(references.top_y_at(x_px) for x_px in boundary_x)
    bottom_limit = max(references.bottom_y_at(x_px) for x_px in boundary_x)
    edge_margin = max(2, int(round(FIXED_TRACE_EDGE_MARGIN * height / DEFAULT_SCREEN_SIZE[1])))
    full_y_start = max(1, int(math.floor(top_limit)) + edge_margin)
    full_y_stop = min(height - 1, int(math.ceil(bottom_limit)) - edge_margin)
    if not 0.0 < POINT_SEARCH_CENTER_FRACTION <= 1.0:
        raise ValueError("拐点中心搜索比例必须在 0～1 之间")
    full_y_span = full_y_stop - full_y_start
    search_y_span = max(1, int(round(
        full_y_span * POINT_SEARCH_CENTER_FRACTION)))
    trim_top = (full_y_span - search_y_span) // 2
    y_start = full_y_start + trim_top
    y_stop = y_start + search_y_span
    detected_left_x, detected_right_x, score, green_excess = (
        estimate_waveform_edges(screen, y_start, y_stop)
    )
    line_span = detected_right_x - detected_left_x
    side_width = max(24, int(round(line_span * SIDE_SEARCH_FRACTION)))
    left_start = max(1, int(round(detected_left_x - line_span * 0.025)))
    left_stop = min(width - 1, int(round(detected_left_x + side_width)))
    right_start = max(1, int(round(detected_right_x - side_width)))
    right_stop = min(width - 1, int(round(detected_right_x + line_span * 0.025)))
    if y_stop - y_start < 40 or min(left_stop - left_start, right_stop - right_start) < 20:
        raise ValueError("固定标定后的波形区域太小")

    side_bands = ((left_start, left_stop), (right_start, right_stop))

    # 动态定位完成后，阈值只统计左右窄带；中央反光不再进入正式候选集合。
    side_score_values = np.concatenate([
        score[y_start:y_stop, left_start:left_stop].reshape(-1),
        score[y_start:y_stop, right_start:right_stop].reshape(-1),
    ])
    side_green_values = np.concatenate([
        green_excess[y_start:y_stop, left_start:left_stop].reshape(-1),
        green_excess[y_start:y_stop, right_start:right_stop].reshape(-1),
    ])
    score_threshold = max(
        3.0,
        float(np.percentile(side_score_values, SIDE_SCORE_PERCENTILE)),
    )
    green_threshold = max(
        4.0,
        float(np.percentile(side_green_values, SIDE_GREEN_PERCENTILE)),
    )

    trace_mask = np.zeros((height, width), np.uint8)
    for band_start, band_stop in side_bands:
        band_score = score[y_start:y_stop, band_start:band_stop]
        band_green = green_excess[y_start:y_stop, band_start:band_stop]
        trace_mask[y_start:y_stop, band_start:band_stop] = np.where(
            (band_score >= score_threshold) & (band_green >= green_threshold),
            255,
            0,
        ).astype(np.uint8)

    trace_mask = cv2.morphologyEx(
        trace_mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )
    # 删除贯穿较长距离的网格线，但保留短小的侧边拐点亮斑。
    long_horizontal = cv2.morphologyEx(
        trace_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (max(38, width // 8), 1)),
    )
    long_vertical = cv2.morphologyEx(
        trace_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (1, max(34, height // 8))),
    )
    long_structures = cv2.bitwise_or(long_horizontal, long_vertical)
    # 粗亮线主体被开运算识别后，再轻微扩张以连同端点光晕一起删除。
    # 真实拐点的水平长度远小于上面的长核，不会进入 long_structures。
    long_structures = cv2.dilate(
        long_structures,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (max(5, width // 80) | 1, max(3, height // 170) | 1),
        ),
    )
    trace_mask = cv2.subtract(trace_mask, long_structures)

    def side_profile(band_start: int, band_stop: int, edge_x: float) -> np.ndarray:
        band_score = score[:, band_start:band_stop]
        band_mask = trace_mask[:, band_start:band_stop] > 0
        columns = np.arange(band_start, band_stop, dtype=np.float32)
        distance = np.abs(columns - float(edge_x))
        edge_weight = np.clip(1.0 - distance / max(side_width, 1), 0.18, 1.0)
        weighted = np.where(band_mask, band_score * edge_weight[None, :], 0.0)
        # 每行只平均最亮的几个像素，粗细变化不会明显改变峰值位置。
        fraction = min(0.16, max(0.04, 8.0 / max(1, band_stop - band_start)))
        return top_fraction_mean(weighted, fraction)

    left_profile = side_profile(left_start, left_stop, detected_left_x)
    right_profile = side_profile(right_start, right_stop, detected_right_x)
    # 最坏情况 0.1 ms x 100 kHz 约有 10 个周期，即每侧约 10 个拐点。
    # 6% 的纵向间距可保留这些峰，同时继续抑制同一粗亮斑的重复峰。
    same_side_gap = max(8, int(round((y_stop - y_start) * 0.06)))
    per_side_limit = max(2, (max(1, int(maximum_points)) + 1) // 2 + 1)
    left_peaks = find_profile_peaks(
        left_profile, y_start, y_stop, same_side_gap, per_side_limit)
    right_peaks = find_profile_peaks(
        right_profile, y_start, y_stop, same_side_gap, per_side_limit)

    vertical_radius = max(6, int(round((y_stop - y_start) * 0.035)))
    turning_points: list[tuple[float, float, float]] = []
    for peak_y, _ in left_peaks:
        point = localize_turning_point(
            score, trace_mask, peak_y, left_start, left_stop,
            "left", vertical_radius)
        if point is not None:
            turning_points.append(point)
    for peak_y, _ in right_peaks:
        point = localize_turning_point(
            score, trace_mask, peak_y, right_start, right_stop,
            "right", vertical_radius)
        if point is not None:
            turning_points.append(point)

    # 大面积反光可能比 CRT 轨迹更亮，但不会贴合本帧自动估计的两条极值线。
    edge_tolerance = max(
        7.0,
        line_span * TURNING_POINT_EDGE_TOLERANCE_FRACTION,
    )
    turning_points = [
        candidate
        for candidate in turning_points
        if min(
            abs(candidate[0] - detected_left_x),
            abs(candidate[0] - detected_right_x),
        ) <= edge_tolerance
    ]

    # 同一纵向位置只允许一个拐点，抑制网格交点形成的左右重复检测。
    global_gap = max(7, int(round((y_stop - y_start) * 0.035)))
    selected: list[tuple[float, float, float]] = []
    for candidate in sorted(turning_points, key=lambda item: item[2], reverse=True):
        if all(abs(candidate[1] - previous[1]) >= global_gap for previous in selected):
            selected.append(candidate)
    point_limit = max(1, int(maximum_points))
    selected = sorted(selected[:point_limit], key=lambda item: item[1], reverse=True)
    selected = select_alternating_edge_points(
        selected,
        detected_left_x,
        detected_right_x,
    )

    points: list[WavePoint] = []

    # FPGA 的有效锯齿扫描从 -2 V 开始并逐渐升到 +2 V：
    # 屏幕下方对应扫描起点，屏幕上方对应扫描终点。
    # 每个点使用固定标定曲线在自身 X 位置处的上下边界，不从当前图像检测。
    # 这样既不依赖参考亮线，又保留 CRT 几何失真的一次性补偿。
    for x_px, y_px, strength in selected:
        x_normalized = (
            2.0 * (x_px - detected_left_x) /
            max(1.0, detected_right_x - detected_left_x) - 1.0
        )
        local_top_y = references.top_y_at(x_px)
        local_bottom_y = references.bottom_y_at(x_px)
        y_normalized = (
            1.0 - 2.0 * (y_px - local_top_y) /
            max(1.0, local_bottom_y - local_top_y)
        )
        points.append(WavePoint(
            x_px=x_px,
            y_px=y_px,
            x_normalized=float(np.clip(x_normalized, -1.25, 1.25)),
            y_normalized=float(np.clip(y_normalized, -1.1, 1.1)),
            y_volts=float(np.clip(2.0 * y_normalized, -2.2, 2.2)),
            time_normalized=float(np.clip((y_normalized + 1.0) * 0.5, 0.0, 1.0)),
            strength=float(np.clip(strength / 180.0, 0.0, 1.0)),
        ))

    # 局部曲线归一化后再按扫描时间排序，避免较强几何失真改变点的先后次序。
    points.sort(key=lambda point: point.time_normalized)

    if len(points) < min(2, point_limit):
        raise ValueError(f"只提取到 {len(points)} 个侧边拐点")
    return points, trace_mask, detected_left_x, detected_right_x


# ==================== 相位间隔计算（稳健版） ====================

def compute_same_side_period_samples(
    points: list[WavePoint],
) -> list[tuple[float, float]]:
    """计算左侧到左侧、右侧到右侧的完整周期。

    返回 ``(归一化完整周期, Y 像素周期)``。同侧做差可以自然抵消左右侧之间
    固定的纵向偏移，不再把一短一长的两种半周期混在一起。
    """

    side_points: dict[int, list[WavePoint]] = {0: [], 1: []}
    for point in points:
        side = 0 if point.x_normalized < 0.0 else 1
        side_points[side].append(point)

    samples: list[tuple[float, float]] = []
    for same_side_points in side_points.values():
        ordered = sorted(
            same_side_points,
            key=lambda point: point.time_normalized,
        )
        for first, second in zip(ordered, ordered[1:]):
            pixel_period = abs(second.y_px - first.y_px)
            if pixel_period <= 0.0:
                continue
            normalized_period = pixel_period / FREQUENCY_RAMP_HEIGHT_PX
            samples.append((normalized_period, pixel_period))
    return samples


def compute_phase_intervals(points: list[WavePoint]) -> list[float]:
    """返回同侧到同侧的归一化完整周期，保留原函数名以兼容旧调用。"""

    return [sample[0] for sample in compute_same_side_period_samples(points)]


def select_standard_period_samples(
    samples: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """保留占多数的标准周期，剔除漏点形成的二倍及以上长间距。"""

    if len(samples) < 2:
        return []

    pixel_periods = np.asarray([sample[1] for sample in samples], np.float64)
    standard_center = float(np.median(pixel_periods))
    if standard_center <= 0.0:
        return []

    long_period_limit = standard_center * LONG_PERIOD_RATIO_MIN
    lower = standard_center * (1.0 - STANDARD_PERIOD_TOLERANCE)
    upper = standard_center * (1.0 + STANDARD_PERIOD_TOLERANCE)
    return [
        sample
        for sample in samples
        if sample[1] < long_period_limit and lower <= sample[1] <= upper
    ]


def compute_robust_phase_interval(
    points: list[WavePoint],
    ramp_duration_us: float = EFFECTIVE_RAMP_DURATION_US,
) -> tuple[float, float, int, float]:
    """
    左右侧分别计算同侧到同侧的完整周期，再保留占多数的标准周期簇。
    漏检同侧点产生的二倍及以上长间距不参与频率计算。
    返回：(完整周期归一化间隔, 标准差, 有效周期数, 估计频率Hz)
    """
    samples = compute_same_side_period_samples(points)
    standard_samples = select_standard_period_samples(samples)
    if len(standard_samples) < 2:
        return 0.0, 0.0, 0, 0.0

    normalized_periods = np.asarray(
        [sample[0] for sample in standard_samples],
        np.float64,
    )
    avg_interval = float(np.median(normalized_periods))
    std_interval = float(np.std(normalized_periods))
    valid_count = len(standard_samples)

    # 归一化间隔是一个完整周期，频率公式不再乘 2。
    if ramp_duration_us <= 0.0:
        raise ValueError("锯齿持续时间必须大于 0")
    ramp_duration_sec = ramp_duration_us / 1_000_000.0
    period_sec = avg_interval * ramp_duration_sec
    freq_hz = 1.0 / period_sec if period_sec > 0.0 else 0.0

    return avg_interval, std_interval, valid_count, freq_hz


# ================================================================

def prepare_display_background(screen: np.ndarray) -> np.ndarray:
    """生成仅供人眼观察的去条纹背景，不参与任何测量。

    先根据每一行的中位亮度估计横向扫描带，再做轻微空间平滑。波形只占
    一行中的少量像素，因此中位数不会把绿色波形本身当成扫描带消除。
    """

    if screen.ndim != 3 or screen.shape[2] != 3:
        raise ValueError("结果预览要求 BGR 彩色图像")

    gray = cv2.cvtColor(screen, cv2.COLOR_BGR2GRAY)
    row_level = np.median(gray, axis=1).astype(np.float32).reshape(-1, 1)
    smooth_kernel = max(9, int(round(screen.shape[0] * 0.055)) | 1)
    smooth_row_level = cv2.GaussianBlur(
        row_level, (1, smooth_kernel), 0).reshape(-1)
    stripe_offset = row_level.reshape(-1) - smooth_row_level

    corrected = (
        screen.astype(np.float32)
        - 0.82 * stripe_offset[:, None, None]
    )
    corrected = np.clip(corrected, 0, 255).astype(np.uint8)

    # 纵向平滑进一步压低一两像素宽的扫描线，同时保留较粗的绿色轨迹。
    softened = cv2.GaussianBlur(corrected, (3, 5), 0)
    return cv2.addWeighted(corrected, 0.38, softened, 0.62, -4.0)


def draw_labeled_point(
    canvas: np.ndarray,
    center: tuple[int, int],
    label: str,
    image_width: int,
    image_top: int,
    image_bottom: int,
) -> None:
    """绘制在绿色波形和扫描线背景上仍清楚可见的拐点标记。"""

    # 黑色阴影、白色外圈和红色中心形成三层对比，亮背景和暗背景都能看清。
    cv2.circle(canvas, center, 12, (0, 0, 0), -1, cv2.LINE_AA)
    cv2.circle(canvas, center, 9, (255, 255, 255), -1, cv2.LINE_AA)
    cv2.circle(canvas, center, 5, (0, 45, 255), -1, cv2.LINE_AA)

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.52
    thickness = 2
    (text_width, text_height), baseline = cv2.getTextSize(
        label, font, font_scale, thickness)

    # 左侧拐点的编号放在右边，右侧拐点的编号放在左边，避免贴出窗口。
    if center[0] < image_width // 2:
        text_x = center[0] + 16
    else:
        text_x = center[0] - 16 - text_width
    text_x = int(np.clip(text_x, 5, max(5, image_width - text_width - 6)))
    text_y = int(np.clip(
        center[1] + text_height // 2,
        image_top + text_height + 6,
        image_bottom - baseline - 6,
    ))

    box_left = text_x - 4
    box_top = text_y - text_height - 4
    box_right = text_x + text_width + 4
    box_bottom = text_y + baseline + 4
    cv2.rectangle(
        canvas, (box_left, box_top), (box_right, box_bottom),
        (10, 13, 16), -1, cv2.LINE_AA)
    cv2.rectangle(
        canvas, (box_left, box_top), (box_right, box_bottom),
        (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(
        canvas, label, (text_x, text_y), font, font_scale,
        (255, 255, 255), thickness, cv2.LINE_AA)


def draw_result(
    screen: np.ndarray,
    references: ReferenceLines,
    points: list[WavePoint],
    avg_interval: float,
    interval_std: float,
    valid_count: int,
    freq_hz: float,
) -> np.ndarray:
    """生成信息栏与波形分离的清晰结果面板。"""

    preview = prepare_display_background(screen)
    image_height, image_width = preview.shape[:2]
    header_height = DISPLAY_HEADER_HEIGHT
    canvas = np.full(
        (image_height + header_height, image_width, 3),
        (18, 22, 26),
        np.uint8,
    )
    canvas[header_height:, :] = preview
    cv2.line(
        canvas, (0, header_height - 1), (image_width - 1, header_height - 1),
        (80, 92, 102), 1, cv2.LINE_AA)

    # 所有状态文字都放在独立实色信息栏中，不再覆盖波形。
    wide_layout = image_width >= 520
    title_text = (
        f"DETECTED  {len(points)}  TURNING POINTS"
        if wide_layout else f"POINTS  {len(points)}"
    )
    cv2.putText(
        canvas,
        title_text,
        (14, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (245, 248, 250),
        2,
        cv2.LINE_AA,
    )
    calibration_text = "AUTO X-EDGE"
    (cal_width, _), _ = cv2.getTextSize(
        calibration_text, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
    cv2.putText(
        canvas,
        calibration_text,
        (max(14, image_width - cal_width - 14), 27),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (80, 220, 255),
        1,
        cv2.LINE_AA,
    )

    if valid_count > 0:
        frequency_label = "FREQ" if valid_count >= 3 else "FREQ EST"
        frequency_text = (
            f"{frequency_label} {freq_hz / 1000.0:.3f} kHz"
            if freq_hz >= 1000.0 else f"{frequency_label} {freq_hz:.1f} Hz"
        )
        if wide_layout:
            metric_text = (
                f"PERIOD {avg_interval:.4f}    STD {interval_std:.4f}    N {valid_count}"
            )
            metric_x = 220
            metric_scale = 0.47
        else:
            metric_text = (
                f"T {avg_interval:.3f}  S {interval_std:.3f}  N {valid_count}"
            )
            metric_x = 142
            metric_scale = 0.38
        cv2.putText(
            canvas, frequency_text, (14, 61),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60 if wide_layout else 0.48,
            (0, 230, 255), 2,
            cv2.LINE_AA)
        cv2.putText(
            canvas, metric_text, (metric_x, 60),
            cv2.FONT_HERSHEY_SIMPLEX, metric_scale, (190, 205, 215), 1,
            cv2.LINE_AA)
    else:
        cv2.putText(
            canvas, "WAITING FOR ENOUGH POINTS", (14, 61),
            cv2.FONT_HERSHEY_SIMPLEX, 0.56, (80, 180, 255), 2,
            cv2.LINE_AA)

    # 点按实际锯齿扫描时间从下到上编号；P1 是最早出现的最下方拐点。
    for index, point in enumerate(points, start=1):
        center = (
            int(round(point.x_px)),
            int(round(point.y_px)) + header_height,
        )
        draw_labeled_point(
            canvas,
            center,
            f"P{index}",
            image_width,
            header_height,
            header_height + image_height,
        )

    return canvas


def process_frame(
    frame: np.ndarray,
    screen_size: tuple[int, int],
    maximum_points: int,
    manual_corners: np.ndarray | None,
    ramp_duration_us: float = EFFECTIVE_RAMP_DURATION_US,
    render_overlay: bool = True,
) -> ProcessResult:
    """按固定机位标定处理一帧，并计算稳健的相位间隔和频率。"""

    corners = manual_corners if manual_corners is not None else get_fixed_screen_corners(frame)
    rectified = rectify_screen(frame, corners, screen_size)
    references = get_fixed_reference_calibration(screen_size)
    points, trace_mask, detected_left_x, detected_right_x = (
        extract_waveform_points(rectified, references, maximum_points)
    )
    # 记录本帧实际极值线，供 CSV 归一化结果和后续调试读取。曲线标尺的
    # center/scale 保持固定，因为它们描述的是 CRT 几何而不是波形水平位置。
    references = replace(
        references,
        left_x=detected_left_x,
        right_x=detected_right_x,
    )

    # 计算稳健相位间隔和频率
    avg_interval, std_interval, valid_count, freq_hz = compute_robust_phase_interval(
        points, ramp_duration_us)

    # 实时主循环需要先完成当前帧识别，再用多帧平均背景绘制一次结果，因此可
    # 跳过这里的首次绘制，避免在树莓派上每帧重复做两遍预览去条纹。
    overlay = (
        draw_result(
            rectified,
            references,
            points,
            avg_interval,
            std_interval,
            valid_count,
            freq_hz,
        )
        if render_overlay else rectified
    )
    return ProcessResult(
        corners,
        rectified,
        trace_mask,
        overlay,
        points,
        references,
        avg_interval,
        std_interval,
        valid_count,
        freq_hz,
    )


def draw_corners(frame: np.ndarray, corners: np.ndarray) -> np.ndarray:
    """在原图上绘制检测到的屏幕区域。"""

    output = frame.copy()
    cv2.polylines(output, [corners.round().astype(np.int32)], True,
                  (0, 0, 255), 3, cv2.LINE_AA)
    return output


def save_result(output_dir: Path, frame: np.ndarray, result: ProcessResult) -> None:
    """保存图片和 CSV，并在 CSV 中添加平均间隔和频率信息。"""

    output_dir.mkdir(parents=True, exist_ok=True)
    write_image(output_dir / "screen_detection.png", draw_corners(frame, result.corners))
    write_image(output_dir / "rectified.png", result.rectified)
    write_image(output_dir / "trace_mask.png", result.trace_mask)
    write_image(output_dir / "points_overlay.png", result.overlay)

    # CSV 保存原始点数据 + 统计信息
    with (output_dir / "points.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "序号", "X像素", "Y像素", "X归一化", "Y归一化",
            "Y电压", "时间归一化", "置信度",
        ])
        for index, point in enumerate(result.points):
            writer.writerow([
                index,
                f"{point.x_px:.3f}",
                f"{point.y_px:.3f}",
                f"{point.x_normalized:.6f}",
                f"{point.y_normalized:.6f}",
                f"{point.y_volts:.6f}",
                f"{point.time_normalized:.6f}",
                f"{point.strength:.6f}",
            ])
        # 额外写入统计行
        writer.writerow([])
        writer.writerow(["稳健完整周期归一化间隔", f"{result.avg_phase_interval:.6f}"])
        writer.writerow(["标准差", f"{result.phase_interval_std:.6f}"])
        writer.writerow(["有效间隔数", f"{result.valid_interval_count}"])
        writer.writerow(["估计频率 (Hz)", f"{result.frequency_hz:.1f}"])


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="单文件 OpenCV 示波器侧边拐点提取（固定机位版）")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("opencv_output"),
        help="结果保存目录",
    )
    parser.add_argument("--max-points", type=int, default=DEFAULT_MAX_POINTS)
    parser.add_argument("--width", type=int, default=DEFAULT_SCREEN_SIZE[0])
    parser.add_argument("--height", type=int, default=DEFAULT_SCREEN_SIZE[1])
    parser.add_argument(
        "--corners",
        type=float,
        nargs=8,
        metavar=("TL_X", "TL_Y", "TR_X", "TR_Y", "BR_X", "BR_Y", "BL_X", "BL_Y"),
        help="临时覆盖文件顶部的固定屏幕四角：左上、右上、右下、左下",
    )
    parser.add_argument(
        "--ramp-us",
        type=float,
        choices=RAMP_DURATION_CHOICES_US,
        default=EFFECTIVE_RAMP_DURATION_US,
        help="FPGA 有效锯齿持续时间：100、500 或 2000 微秒",
    )
    parser.add_argument("--no-gui", action="store_true", help="不打开调试窗口")
    parser.add_argument(
        "--camera-width", type=int, default=FIXED_CALIBRATION_FRAME_SIZE[0])
    parser.add_argument(
        "--camera-height", type=int, default=FIXED_CALIBRATION_FRAME_SIZE[1])
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument(
        "--save-every",
        type=int,
        default=0,
        help="每隔多少帧保存调试图，0 表示只在退出时保存",
    )
    parser.add_argument("--exposure", type=float, help="固定曝光值，具体范围由摄像头决定")
    parser.add_argument("--gain", type=float, help="固定增益值")
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    screen_size = (max(320, args.width), max(240, args.height))
    manual_corners = (
        order_corners(np.asarray(args.corners, np.float32).reshape(4, 2))
        if args.corners is not None else None
    )

    # 直接打开默认摄像头
    capture = cv2.VideoCapture(0)
    if not capture.isOpened():
        raise RuntimeError("无法打开摄像头 0")

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.camera_width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.camera_height)
    capture.set(cv2.CAP_PROP_FPS, args.camera_fps)
    if args.exposure is not None:
        capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        capture.set(cv2.CAP_PROP_EXPOSURE, args.exposure)
    if args.gain is not None:
        capture.set(cv2.CAP_PROP_GAIN, args.gain)

    last_result: ProcessResult | None = None
    last_frame: np.ndarray | None = None
    result_saved = False
    frame_index = 0
    display_accumulator: np.ndarray | None = None
    temporal_period_filter = TemporalPeriodFilter()

    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("摄像头没有返回图像")

            try:
                result = process_frame(
                    frame,
                    screen_size,
                    args.max_points,
                    manual_corners,
                    args.ramp_us,
                    False,
                )
                stable_period, stable_frequency_hz = temporal_period_filter.update(
                    result.avg_phase_interval,
                    result.valid_interval_count,
                    args.ramp_us,
                )
                if stable_frequency_hz > 0.0:
                    result = replace(
                        result,
                        avg_phase_interval=stable_period,
                        frequency_hz=stable_frequency_hz,
                    )
                last_result = result
                last_frame = frame
                status_frame = frame if args.no_gui else draw_corners(frame, result.corners)

                # 识别始终使用当前原始帧；多帧平均只替换给人看的背景。
                # 这样可以压低 CRT 扫描带，又不会让历史帧影响拐点坐标和频率。
                rectified_float = result.rectified.astype(np.float32)
                if (
                    display_accumulator is None
                    or display_accumulator.shape != rectified_float.shape
                ):
                    display_accumulator = rectified_float.copy()
                else:
                    cv2.accumulateWeighted(
                        rectified_float,
                        display_accumulator,
                        DISPLAY_TEMPORAL_ALPHA,
                    )
                display_screen = cv2.convertScaleAbs(display_accumulator)
                result.overlay = draw_result(
                    display_screen,
                    result.references,
                    result.points,
                    result.avg_phase_interval,
                    result.phase_interval_std,
                    result.valid_interval_count,
                    result.frequency_hz,
                )

                # 打印信息，包含平均间隔和频率
                if frame_index % 15 == 0:
                    print(
                        f"frame={frame_index} points={len(result.points)} "
                        f"full_period={result.avg_phase_interval:.4f} "
                        f"freq={result.frequency_hz:.0f}Hz  n={result.valid_interval_count}",
                        end="\r"
                    )
            except Exception as error:
                status_frame = frame.copy()
                cv2.putText(status_frame, f"ERROR: {error}", (20, 38),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2,
                            cv2.LINE_AA)
                print(f"frame={frame_index} error={error}")
                result = None

            periodic_save = args.save_every > 0 and frame_index % max(1, args.save_every) == 0
            if result is not None and periodic_save:
                save_result(args.output_dir, frame, result)
                result_saved = True

            if not args.no_gui:
                cv2.imshow("1-original-screen", status_frame)
                if result is not None:
                    cv2.imshow("2-rectified", result.rectified)
                    cv2.imshow("3-trace-mask", result.trace_mask)
                    cv2.imshow("4-turning-points", result.overlay)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s") and last_result is not None:
                    save_result(args.output_dir, frame, last_result)
                    result_saved = True

            frame_index += 1
    except KeyboardInterrupt:
        pass
    finally:
        capture.release()
        cv2.destroyAllWindows()

    if last_result is None:
        return 1
    if args.no_gui and last_frame is not None:
        save_result(args.output_dir, last_frame, last_result)
        result_saved = True
    if result_saved:
        print(f"\n结果已保存到：{args.output_dir.resolve()}")
    else:
        print("\n运行结束；按 S 可在调试界面中保存结果")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
