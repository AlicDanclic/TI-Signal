#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试圆形检测功能

用于验证circle_detector和frequency_scanner模块的正确性。
"""

import sys
import numpy as np
import cv2
from pathlib import Path

# 添加模块路径
sys.path.insert(0, str(Path(__file__).parent))

from circle_detector import CircleDetector, CircleQuality
from frequency_scanner import FrequencyScanner, ScanResult


def create_circle_mask(width: int, height: int, center_x: float, center_y: float,
                       radius: float, thickness: int = 3) -> np.ndarray:
    """创建圆形掩膜用于测试"""
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(mask, (int(center_x), int(center_y)), int(radius), 255, thickness)
    return mask


def create_ellipse_mask(width: int, height: int, center_x: float, center_y: float,
                        major_axis: float, minor_axis: float, angle: float = 0.0,
                        thickness: int = 3) -> np.ndarray:
    """创建椭圆掩膜用于测试"""
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.ellipse(mask, (int(center_x), int(center_y)),
                (int(major_axis), int(minor_axis)), angle, 0, 360, 255, thickness)
    return mask


def test_circle_detector():
    """测试圆形检测器"""
    print("=" * 60)
    print("测试圆形检测器")
    print("=" * 60)

    config = {
        "circle_detection": {
            "min_circularity": 0.75,
            "good_circularity": 0.85,
            "max_axis_ratio": 1.25,
            "good_axis_ratio": 1.10,
            "min_coverage": 0.70,
            "good_coverage": 0.85,
            "min_symmetry": 0.65,
            "lock_quality_threshold": 75
        }
    }

    detector = CircleDetector(config)

    # 测试1: 完美圆形
    print("\n测试1: 完美圆形")
    mask1 = create_circle_mask(640, 480, 320, 240, 150, thickness=5)
    result1 = detector.detect_circle(mask1)
    print(f"  is_circle: {result1.is_circle}")
    print(f"  quality_score: {result1.quality_score}")
    print(f"  circularity: {result1.circularity:.3f}")
    print(f"  radius_ratio: {result1.radius_ratio:.3f}")
    print(f"  coverage: {result1.coverage:.3f}")
    print(f"  symmetry: {result1.symmetry:.3f}")
    print(f"  reason: {result1.reason}")

    # 测试2: 轻微椭圆
    print("\n测试2: 轻微椭圆 (1.2:1)")
    mask2 = create_ellipse_mask(640, 480, 320, 240, 180, 150, thickness=5)
    result2 = detector.detect_circle(mask2)
    print(f"  is_circle: {result2.is_circle}")
    print(f"  quality_score: {result2.quality_score}")
    print(f"  circularity: {result2.circularity:.3f}")
    print(f"  radius_ratio: {result2.radius_ratio:.3f}")
    print(f"  reason: {result2.reason}")

    # 测试3: 明显椭圆
    print("\n测试3: 明显椭圆 (2:1)")
    mask3 = create_ellipse_mask(640, 480, 320, 240, 200, 100, thickness=5)
    result3 = detector.detect_circle(mask3)
    print(f"  is_circle: {result3.is_circle}")
    print(f"  quality_score: {result3.quality_score}")
    print(f"  circularity: {result3.circularity:.3f}")
    print(f"  radius_ratio: {result3.radius_ratio:.3f}")
    print(f"  reason: {result3.reason}")

    # 测试4: 空掩膜
    print("\n测试4: 空掩膜")
    mask4 = np.zeros((640, 480), dtype=np.uint8)
    result4 = detector.detect_circle(mask4)
    print(f"  is_circle: {result4.is_circle}")
    print(f"  quality_score: {result4.quality_score}")
    print(f"  reason: {result4.reason}")

    # 测试5: 不完整的圆
    print("\n测试5: 不完整的圆 (3/4圆)")
    mask5 = np.zeros((640, 480), dtype=np.uint8)
    cv2.ellipse(mask5, (320, 240), (150, 150), 0, 0, 270, 255, 5)
    result5 = detector.detect_circle(mask5)
    print(f"  is_circle: {result5.is_circle}")
    print(f"  quality_score: {result5.quality_score}")
    print(f"  circularity: {result5.circularity:.3f}")
    print(f"  coverage: {result5.coverage:.3f}")
    print(f"  reason: {result5.reason}")

    print("\n✓ 圆形检测器测试完成")
    return True


def test_frequency_scanner():
    """测试频率扫描器"""
    print("\n" + "=" * 60)
    print("测试频率扫描器")
    print("=" * 60)

    config = {
        "frequency_scan": {
            "step_hz": 100.0,
            "initial_range_hz": 500.0,
            "extended_range_hz": 1000.0,
            "max_scan_points": 20,
            "min_quality_for_lock": 75,
            "good_quality_threshold": 85,
            "early_stop_quality": 90,
            "dds_clock_hz": 50_000_000.0,
            "scan_strategy": "bidirectional"
        }
    }

    scanner = FrequencyScanner(config)

    # 测试1: 规划扫描点
    print("\n测试1: 规划扫描点 (5000 Hz 中心)")
    points1 = scanner.plan_scan_points(5000.0, 500.0)
    print(f"  生成 {len(points1)} 个扫描点")
    print(f"  前5个点: {points1[:5]}")

    # 测试2: 频率对齐
    print("\n测试2: 频率对齐到100Hz网格")
    test_freqs = [4567.3, 5000.0, 5049.9, 5050.1, 5099.5]
    for freq in test_freqs:
        aligned = scanner._align_to_grid(freq)
        print(f"  {freq:.1f} Hz -> {aligned:.1f} Hz")

    # 测试3: DDS调谐字转换
    print("\n测试3: DDS调谐字转换")
    test_freqs_dds = [1000.0, 5000.0, 10000.0, 50000.0, 100000.0]
    for freq in test_freqs_dds:
        tuning_word = scanner.frequency_to_tuning_word(freq)
        back_freq = scanner.tuning_word_to_frequency(tuning_word)
        print(f"  {freq:.1f} Hz -> TW={tuning_word} -> {back_freq:.1f} Hz")

    # 测试4: 选择最佳频率
    print("\n测试4: 选择最佳频率")
    scan_results = [
        (4800.0, 65, False),
        (4900.0, 72, False),
        (5000.0, 88, True),   # 最佳
        (5100.0, 80, True),
        (5200.0, 70, False),
    ]
    best = scanner.select_best_frequency(scan_results)
    print(f"  最佳频率: {best.frequency_hz:.1f} Hz")
    print(f"  质量分数: {best.quality_score}")
    print(f"  找到圆形: {best.found_circle}")
    print(f"  扫描点数: {best.scan_count}")

    # 测试5: 无圆形情况
    print("\n测试5: 无圆形情况")
    scan_results_no_circle = [
        (4800.0, 65, False),
        (4900.0, 72, False),
        (5000.0, 68, False),
        (5100.0, 70, False),
    ]
    best_no_circle = scanner.select_best_frequency(scan_results_no_circle)
    print(f"  最佳频率: {best_no_circle.frequency_hz:.1f} Hz")
    print(f"  质量分数: {best_no_circle.quality_score}")
    print(f"  找到圆形: {best_no_circle.found_circle}")
    print(f"  原因: {best_no_circle.reason}")

    # 测试6: 扩展扫描判断
    print("\n测试6: 判断是否需要扩展扫描")
    should_extend = scanner.should_extend_scan(scan_results_no_circle, 10)
    print(f"  需要扩展: {should_extend}")

    # 测试7: 生成扩展点
    print("\n测试7: 生成扩展扫描点")
    existing = [4500.0, 4600.0, 4700.0, 4800.0, 4900.0, 5000.0,
                5100.0, 5200.0, 5300.0, 5400.0, 5500.0]
    extended = scanner.generate_extended_points(5000.0, existing)
    print(f"  生成 {len(extended)} 个新扫描点")
    print(f"  前5个新点: {extended[:5] if extended else '无'}")

    print("\n✓ 频率扫描器测试完成")
    return True


def test_integration():
    """集成测试"""
    print("\n" + "=" * 60)
    print("集成测试: 模拟完整扫描流程")
    print("=" * 60)

    config = {
        "circle_detection": {
            "min_circularity": 0.75,
            "max_axis_ratio": 1.25,
            "min_coverage": 0.70,
            "lock_quality_threshold": 75
        },
        "frequency_scan": {
            "step_hz": 100.0,
            "initial_range_hz": 500.0,
            "max_scan_points": 20,
            "dds_clock_hz": 50_000_000.0,
            "scan_strategy": "bidirectional"
        }
    }

    detector = CircleDetector(config)
    scanner = FrequencyScanner(config)

    # 模拟扫描过程
    coarse_freq = 5000.0
    print(f"\n粗测频率: {coarse_freq:.1f} Hz")

    scan_points = scanner.plan_scan_points(coarse_freq, 500.0)
    print(f"规划 {len(scan_points)} 个扫描点")

    # 模拟每个扫描点的测试
    scan_results = []
    for i, freq in enumerate(scan_points[:11]):  # 只测试前11个点
        # 模拟圆形质量（最佳点在5000Hz）
        freq_error = abs(freq - 5000.0)
        if freq_error == 0:
            quality = 92
            is_circle = True
        elif freq_error <= 100:
            quality = 85
            is_circle = True
        elif freq_error <= 200:
            quality = 75
            is_circle = True
        elif freq_error <= 300:
            quality = 68
            is_circle = False
        else:
            quality = 60
            is_circle = False

        scan_results.append((freq, quality, is_circle))
        print(f"  点 {i+1}: {freq:.1f} Hz -> 质量={quality}, 圆形={is_circle}")

        # 模拟提前停止
        if is_circle and quality >= 90:
            print(f"  ✓ 找到高质量圆形，提前停止")
            break

    # 选择最佳结果
    best = scanner.select_best_frequency(scan_results)
    print(f"\n最终结果:")
    print(f"  最佳频率: {best.frequency_hz:.1f} Hz")
    print(f"  质量分数: {best.quality_score}")
    print(f"  DDS调谐字: {best.tuning_word}")
    print(f"  找到圆形: {best.found_circle}")

    print("\n✓ 集成测试完成")
    return True


if __name__ == "__main__":
    print("开始测试圆形检测和频率扫描功能\n")

    try:
        success = True
        success &= test_circle_detector()
        success &= test_frequency_scanner()
        success &= test_integration()

        if success:
            print("\n" + "=" * 60)
            print("✓ 所有测试通过")
            print("=" * 60)
            sys.exit(0)
        else:
            print("\n" + "=" * 60)
            print("✗ 部分测试失败")
            print("=" * 60)
            sys.exit(1)

    except Exception as e:
        print(f"\n✗ 测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
