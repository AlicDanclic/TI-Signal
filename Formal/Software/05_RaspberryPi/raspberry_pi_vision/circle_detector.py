# -*- coding: utf-8 -*-
"""圆形检测与质量评估模块

用于第五问自动锁圆功能，检测李萨如图形是否为圆形，
并评估圆度质量以判断是否达到锁定条件。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class CircleQuality:
    """圆形质量评估结果"""
    is_circle: bool           # 是否检测为圆形
    circularity: float        # 圆度 (0.0-1.0)
    quality_score: int        # 综合质量分数 (0-100)
    radius_ratio: float       # 长短轴比 (接近1.0为圆)
    center_x: float           # 圆心X坐标（归一化）
    center_y: float           # 圆心Y坐标（归一化）
    coverage: float           # 轨迹覆盖率
    symmetry: float           # 对称性评分
    reason: str               # 判断原因


class CircleDetector:
    """圆形检测器

    检测李萨如图形是否为圆形，评估圆度质量。
    当两个正弦波频率相同且相位差为90度时，形成圆形。
    """

    def __init__(self, config: dict[str, Any]) -> None:
        circle_cfg = config.get("circle_detection", {})

        # 圆度阈值
        self.min_circularity = float(circle_cfg.get("min_circularity", 0.75))
        self.good_circularity = float(circle_cfg.get("good_circularity", 0.85))

        # 轴比阈值（长轴/短轴应接近1.0）
        self.max_axis_ratio = float(circle_cfg.get("max_axis_ratio", 1.25))
        self.good_axis_ratio = float(circle_cfg.get("good_axis_ratio", 1.10))

        # 覆盖率阈值（轨迹应覆盖完整圆周）
        self.min_coverage = float(circle_cfg.get("min_coverage", 0.70))
        self.good_coverage = float(circle_cfg.get("good_coverage", 0.85))

        # 对称性阈值
        self.min_symmetry = float(circle_cfg.get("min_symmetry", 0.65))

        # 质量评分阈值
        self.lock_quality_threshold = int(circle_cfg.get("lock_quality_threshold", 75))

    def detect_circle(self, mask: np.ndarray) -> CircleQuality:
        """检测掩膜中的圆形并评估质量

        Args:
            mask: 轨迹掩膜 (H x W 的二值图像)

        Returns:
            CircleQuality: 圆形质量评估结果
        """
        if mask.size == 0 or np.count_nonzero(mask) < 50:
            return CircleQuality(
                False, 0.0, 0, 0.0, 0.5, 0.5, 0.0, 0.0,
                "insufficient trace pixels"
            )

        height, width = mask.shape[:2]

        # 提取轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        if not contours:
            return CircleQuality(
                False, 0.0, 0, 0.0, 0.5, 0.5, 0.0, 0.0,
                "no contours found"
            )

        # 使用最大轮廓
        main_contour = max(contours, key=cv2.contourArea)

        if len(main_contour) < 10:
            return CircleQuality(
                False, 0.0, 0, 0.0, 0.5, 0.5, 0.0, 0.0,
                "contour too small"
            )

        # 拟合椭圆（需要至少5个点）
        if len(main_contour) < 5:
            return CircleQuality(
                False, 0.0, 0, 0.0, 0.5, 0.5, 0.0, 0.0,
                "insufficient points for ellipse fit"
            )

        try:
            ellipse = cv2.fitEllipse(main_contour)
            center, axes, angle = ellipse
            major_axis = max(axes)
            minor_axis = min(axes)
        except cv2.error:
            return CircleQuality(
                False, 0.0, 0, 0.0, 0.5, 0.5, 0.0, 0.0,
                "ellipse fit failed"
            )

        # 计算轴比
        if minor_axis > 0:
            axis_ratio = major_axis / minor_axis
        else:
            axis_ratio = 10.0

        # 计算圆度（使用周长和面积）
        perimeter = cv2.arcLength(main_contour, True)
        area = cv2.contourArea(main_contour)

        if perimeter > 0:
            # 圆形的圆度 = 4π * 面积 / 周长^2，完美圆形为1.0
            circularity = (4.0 * np.pi * area) / (perimeter * perimeter)
            circularity = min(1.0, circularity)
        else:
            circularity = 0.0

        # 归一化圆心坐标
        center_x_norm = center[0] / max(width, 1.0)
        center_y_norm = center[1] / max(height, 1.0)

        # 计算覆盖率（轨迹点在拟合椭圆附近的比例）
        coverage = self._calculate_coverage(main_contour, ellipse)

        # 计算对称性
        symmetry = self._calculate_symmetry(mask, center)

        # 综合评分
        quality_score = self._calculate_quality_score(
            circularity, axis_ratio, coverage, symmetry
        )

        # 判断是否为圆形
        is_circle = (
            circularity >= self.min_circularity and
            axis_ratio <= self.max_axis_ratio and
            coverage >= self.min_coverage and
            symmetry >= self.min_symmetry
        )

        # 生成原因说明
        if not is_circle:
            reasons = []
            if circularity < self.min_circularity:
                reasons.append(f"circularity {circularity:.3f} < {self.min_circularity}")
            if axis_ratio > self.max_axis_ratio:
                reasons.append(f"axis_ratio {axis_ratio:.3f} > {self.max_axis_ratio}")
            if coverage < self.min_coverage:
                reasons.append(f"coverage {coverage:.3f} < {self.min_coverage}")
            if symmetry < self.min_symmetry:
                reasons.append(f"symmetry {symmetry:.3f} < {self.min_symmetry}")
            reason = "; ".join(reasons)
        else:
            reason = "circle detected"

        return CircleQuality(
            is_circle=is_circle,
            circularity=float(circularity),
            quality_score=quality_score,
            radius_ratio=float(axis_ratio),
            center_x=float(center_x_norm),
            center_y=float(center_y_norm),
            coverage=float(coverage),
            symmetry=float(symmetry),
            reason=reason
        )

    def _calculate_coverage(
        self, contour: np.ndarray, ellipse: tuple
    ) -> float:
        """计算轨迹在拟合椭圆上的覆盖率"""
        center, axes, angle = ellipse
        cx, cy = center
        a, b = axes[0] / 2.0, axes[1] / 2.0

        if a <= 0 or b <= 0:
            return 0.0

        # 将轮廓点转换到椭圆坐标系
        angle_rad = np.deg2rad(angle)
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)

        points = contour.reshape(-1, 2).astype(np.float64)
        dx = points[:, 0] - cx
        dy = points[:, 1] - cy

        # 旋转到椭圆主轴
        x_rot = dx * cos_a + dy * sin_a
        y_rot = -dx * sin_a + dy * cos_a

        # 计算点到椭圆的归一化距离
        distances = np.sqrt((x_rot / a) ** 2 + (y_rot / b) ** 2)

        # 在椭圆附近的点（距离在0.8到1.2之间）
        on_ellipse = np.sum((distances >= 0.8) & (distances <= 1.2))

        return float(on_ellipse) / max(len(points), 1.0)

    def _calculate_symmetry(
        self, mask: np.ndarray, center: tuple[float, float]
    ) -> float:
        """计算图形的对称性"""
        height, width = mask.shape[:2]
        cx, cy = int(center[0]), int(center[1])

        # 确保中心在图像内
        if not (0 <= cx < width and 0 <= cy < height):
            return 0.0

        # 计算四个象限的对称性
        radius = min(cx, cy, width - cx, height - cy)
        if radius < 10:
            return 0.0

        # 水平对称性
        left_region = mask[max(0, cy - radius):min(height, cy + radius),
                           max(0, cx - radius):cx]
        right_region = mask[max(0, cy - radius):min(height, cy + radius),
                            cx:min(width, cx + radius)]
        right_flipped = cv2.flip(right_region, 1)

        # 匹配左右区域
        min_width = min(left_region.shape[1], right_flipped.shape[1])
        if min_width > 0:
            h_match = np.sum(
                left_region[:, -min_width:] == right_flipped[:, :min_width]
            )
            h_total = left_region[:, -min_width:].size
            h_symmetry = h_match / max(h_total, 1.0)
        else:
            h_symmetry = 0.0

        # 垂直对称性
        top_region = mask[max(0, cy - radius):cy,
                          max(0, cx - radius):min(width, cx + radius)]
        bottom_region = mask[cy:min(height, cy + radius),
                             max(0, cx - radius):min(width, cx + radius)]
        bottom_flipped = cv2.flip(bottom_region, 0)

        min_height = min(top_region.shape[0], bottom_flipped.shape[0])
        if min_height > 0:
            v_match = np.sum(
                top_region[-min_height:, :] == bottom_flipped[:min_height, :]
            )
            v_total = top_region[-min_height:, :].size
            v_symmetry = v_match / max(v_total, 1.0)
        else:
            v_symmetry = 0.0

        # 综合对称性
        return float((h_symmetry + v_symmetry) / 2.0)

    def _calculate_quality_score(
        self,
        circularity: float,
        axis_ratio: float,
        coverage: float,
        symmetry: float
    ) -> int:
        """计算综合质量分数 (0-100)"""
        # 圆度评分 (40分)
        circ_score = circularity * 40.0

        # 轴比评分 (25分)
        # 完美圆形轴比为1.0，超过阈值线性下降
        if axis_ratio <= 1.0:
            axis_score = 25.0
        elif axis_ratio <= self.good_axis_ratio:
            axis_score = 25.0 * (self.good_axis_ratio - axis_ratio) / (
                self.good_axis_ratio - 1.0
            )
        else:
            axis_score = max(0.0, 25.0 * (self.max_axis_ratio - axis_ratio) / (
                self.max_axis_ratio - self.good_axis_ratio
            ))

        # 覆盖率评分 (20分)
        coverage_score = coverage * 20.0

        # 对称性评分 (15分)
        symmetry_score = symmetry * 15.0

        total = circ_score + axis_score + coverage_score + symmetry_score
        return int(np.clip(total, 0, 100))
