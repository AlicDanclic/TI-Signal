# -*- coding: utf-8 -*-
"""频率扫描模块

实现100Hz步进的频率扫描，用于第五问自动锁圆功能。
在粗测频率基础上进行精细扫频，寻找最佳圆形。
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any

import numpy as np

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class ScanResult:
    """扫描结果"""
    frequency_hz: float       # 最佳频率
    quality_score: int        # 质量分数
    tuning_word: int          # DDS调谐字
    scan_count: int           # 扫描次数
    found_circle: bool        # 是否找到圆形
    reason: str               # 结果说明


class FrequencyScanner:
    """频率扫描器

    从粗测频率开始，以100Hz为步进进行扫频，
    寻找能够形成最佳圆形的频率点。
    """

    def __init__(self, config: dict[str, Any]) -> None:
        scan_cfg = config.get("frequency_scan", {})

        # 扫描参数
        self.step_hz = float(scan_cfg.get("step_hz", 100.0))  # 步进频率
        self.initial_range_hz = float(scan_cfg.get("initial_range_hz", 500.0))
        self.extended_range_hz = float(scan_cfg.get("extended_range_hz", 1000.0))
        self.max_scan_points = int(scan_cfg.get("max_scan_points", 20))

        # 质量阈值
        self.min_quality_for_lock = int(scan_cfg.get("min_quality_for_lock", 75))
        self.good_quality_threshold = int(scan_cfg.get("good_quality_threshold", 85))

        # DDS参数
        self.dds_clock_hz = float(scan_cfg.get("dds_clock_hz", 50_000_000.0))

        # 扫描策略
        self.scan_strategy = scan_cfg.get("scan_strategy", "bidirectional")

    def plan_scan_points(
        self,
        coarse_frequency_hz: float,
        uncertainty_hz: float = 500.0
    ) -> list[float]:
        """规划扫描点序列

        Args:
            coarse_frequency_hz: 粗测频率
            uncertainty_hz: 不确定度

        Returns:
            扫描频率点列表（已按100Hz对齐）
        """
        # 确保粗测频率对齐到100Hz
        center_freq = self._align_to_grid(coarse_frequency_hz)

        # 初始扫描范围
        scan_range = max(uncertainty_hz, self.initial_range_hz)
        half_range = scan_range / 2.0

        # 生成扫描点
        points = []

        if self.scan_strategy == "bidirectional":
            # 双向扫描：中心 -> +step -> -step -> +2*step -> -2*step ...
            points.append(center_freq)

            offset = self.step_hz
            while offset <= half_range and len(points) < self.max_scan_points:
                if center_freq + offset <= 100_000.0:
                    points.append(center_freq + offset)
                if center_freq - offset >= 1000.0 and len(points) < self.max_scan_points:
                    points.append(center_freq - offset)
                offset += self.step_hz

        elif self.scan_strategy == "sweep_up":
            # 向上扫描
            freq = center_freq - half_range
            while freq <= center_freq + half_range and len(points) < self.max_scan_points:
                if 1000.0 <= freq <= 100_000.0:
                    points.append(freq)
                freq += self.step_hz

        else:  # "fine_grid"
            # 密集网格
            freq = center_freq - half_range
            while freq <= center_freq + half_range and len(points) < self.max_scan_points:
                if 1000.0 <= freq <= 100_000.0:
                    points.append(freq)
                freq += self.step_hz

        # 确保所有点都对齐到100Hz网格
        points = [self._align_to_grid(f) for f in points]

        # 去重并排序（如果需要）
        points = sorted(set(points))

        LOGGER.info(
            f"Planned {len(points)} scan points around {center_freq:.1f} Hz "
            f"(range: ±{half_range:.1f} Hz, step: {self.step_hz:.1f} Hz)"
        )

        return points

    def _align_to_grid(self, frequency_hz: float) -> float:
        """将频率对齐到100Hz网格

        Args:
            frequency_hz: 输入频率

        Returns:
            对齐后的频率（100Hz的整数倍）
        """
        return round(frequency_hz / 100.0) * 100.0

    def frequency_to_tuning_word(self, frequency_hz: float) -> int:
        """将频率转换为DDS调谐字

        Args:
            frequency_hz: 目标频率

        Returns:
            32位DDS调谐字
        """
        if frequency_hz <= 0.0 or self.dds_clock_hz <= 0.0:
            return 0

        # DDS调谐字 = freq * 2^32 / clock
        tuning_word = int(round(frequency_hz * (2**32) / self.dds_clock_hz))

        # 限制在有效范围内
        tuning_word = max(1, min(0xFFFFFFFF, tuning_word))

        return tuning_word

    def tuning_word_to_frequency(self, tuning_word: int) -> float:
        """将DDS调谐字转换回频率

        Args:
            tuning_word: 32位DDS调谐字

        Returns:
            频率 (Hz)
        """
        if tuning_word <= 0 or self.dds_clock_hz <= 0.0:
            return 0.0

        # freq = tuning_word * clock / 2^32
        frequency_hz = float(tuning_word) * self.dds_clock_hz / (2**32)

        return frequency_hz

    def select_best_frequency(
        self,
        scan_results: list[tuple[float, int, bool]]
    ) -> ScanResult:
        """从扫描结果中选择最佳频率

        Args:
            scan_results: 扫描结果列表 [(frequency_hz, quality_score, is_circle), ...]

        Returns:
            ScanResult: 最佳扫描结果
        """
        if not scan_results:
            return ScanResult(
                0.0, 0, 0, 0, False,
                "no scan results available"
            )

        # 筛选出检测到圆形的结果
        circle_results = [
            (freq, quality, is_circle)
            for freq, quality, is_circle in scan_results
            if is_circle
        ]

        if circle_results:
            # 选择质量最高的
            best = max(circle_results, key=lambda x: x[1])
            best_freq, best_quality, _ = best

            tuning_word = self.frequency_to_tuning_word(best_freq)

            return ScanResult(
                frequency_hz=best_freq,
                quality_score=best_quality,
                tuning_word=tuning_word,
                scan_count=len(scan_results),
                found_circle=True,
                reason=f"best quality: {best_quality}"
            )
        else:
            # 没有找到圆形，返回质量最高的点
            best = max(scan_results, key=lambda x: x[1])
            best_freq, best_quality, _ = best

            tuning_word = self.frequency_to_tuning_word(best_freq)

            return ScanResult(
                frequency_hz=best_freq,
                quality_score=best_quality,
                tuning_word=tuning_word,
                scan_count=len(scan_results),
                found_circle=False,
                reason=f"no circle found, best quality: {best_quality}"
            )

    def should_extend_scan(
        self,
        current_results: list[tuple[float, int, bool]],
        scan_points_completed: int
    ) -> bool:
        """判断是否需要扩展扫描范围

        Args:
            current_results: 当前扫描结果
            scan_points_completed: 已完成的扫描点数

        Returns:
            是否需要扩展扫描
        """
        # 如果已经找到高质量圆形，不需要扩展
        for _, quality, is_circle in current_results:
            if is_circle and quality >= self.good_quality_threshold:
                return False

        # 如果已经扫描了足够多的点，不再扩展
        if scan_points_completed >= self.max_scan_points:
            return False

        # 如果没有找到任何圆形，可以考虑扩展
        has_circle = any(is_circle for _, _, is_circle in current_results)

        if not has_circle and scan_points_completed >= 10:
            return True

        return False

    def generate_extended_points(
        self,
        original_center_hz: float,
        existing_points: list[float]
    ) -> list[float]:
        """生成扩展扫描点

        Args:
            original_center_hz: 原始中心频率
            existing_points: 已扫描的频率点

        Returns:
            新的扫描点列表
        """
        existing_set = set(existing_points)
        new_points = []

        center = self._align_to_grid(original_center_hz)
        half_range = self.extended_range_hz / 2.0

        # 生成更大范围的点
        freq = center - half_range
        while freq <= center + half_range:
            freq_aligned = self._align_to_grid(freq)
            if (1000.0 <= freq_aligned <= 100_000.0 and
                freq_aligned not in existing_set):
                new_points.append(freq_aligned)
            freq += self.step_hz

        LOGGER.info(
            f"Extended scan: generated {len(new_points)} additional points "
            f"(range: ±{half_range:.1f} Hz)"
        )

        return new_points
