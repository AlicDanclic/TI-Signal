import cv2
import numpy as np

from opencv_main import (
    DISPLAY_HEADER_HEIGHT,
    FREQUENCY_RAMP_HEIGHT_PX,
    TemporalPeriodFilter,
    WavePoint,
    compute_robust_phase_interval,
    draw_result,
    estimate_waveform_edges,
    get_fixed_reference_calibration,
    prepare_display_background,
    select_alternating_edge_points,
)


def make_period_point(side: str, time_normalized: float) -> WavePoint:
    """构造一个左/右侧拐点，Y 像素与锯齿时间保持线性。"""

    is_left = side == "left"
    return WavePoint(
        x_px=95.0 if is_left else 575.0,
        y_px=time_normalized * FREQUENCY_RAMP_HEIGHT_PX,
        x_normalized=-1.0 if is_left else 1.0,
        y_normalized=1.0 - 2.0 * time_normalized,
        y_volts=2.0 - 4.0 * time_normalized,
        time_normalized=time_normalized,
        strength=1.0,
    )


def make_striped_screen() -> np.ndarray:
    """构造类似摄像头拍摄模拟示波器时产生的横向扫描带。"""

    screen = np.zeros((240, 320, 3), np.uint8)
    for row in range(screen.shape[0]):
        level = 78 if (row // 3) % 2 == 0 else 24
        screen[row, :] = (level // 2, level, level // 3)
    cv2.line(screen, (50, 210), (270, 30), (25, 235, 65), 7, cv2.LINE_AA)
    return screen


def test_display_background_reduces_horizontal_banding() -> None:
    screen = make_striped_screen()
    preview = prepare_display_background(screen)

    before = cv2.cvtColor(screen, cv2.COLOR_BGR2GRAY).mean(axis=1)
    after = cv2.cvtColor(preview, cv2.COLOR_BGR2GRAY).mean(axis=1)

    assert preview.shape == screen.shape
    assert float(np.std(after)) < float(np.std(before)) * 0.55


def test_result_panel_keeps_header_off_waveform_and_marks_points() -> None:
    screen = make_striped_screen()
    original = screen.copy()
    point = WavePoint(
        x_px=52.0,
        y_px=120.0,
        x_normalized=-1.0,
        y_normalized=0.0,
        y_volts=0.0,
        time_normalized=0.5,
        strength=1.0,
    )
    references = get_fixed_reference_calibration((320, 240))

    panel = draw_result(
        screen,
        references,
        [point],
        avg_interval=0.05,
        interval_std=0.002,
        valid_count=4,
        freq_hz=10_000.0,
    )

    # 绘图不能改动识别使用的原图，信息栏必须占用新增区域。
    assert np.array_equal(screen, original)
    assert panel.shape == (240 + DISPLAY_HEADER_HEIGHT, 320, 3)

    # 红色中心应位于加上信息栏偏移后的坐标，而不是覆盖原波形坐标。
    center = panel[120 + DISPLAY_HEADER_HEIGHT, 52]
    assert int(center[2]) > 220
    assert int(center[2]) > int(center[1]) * 3


def test_edge_points_choose_stronger_alternating_sequence() -> None:
    # 第四个候选是靠近左边界的弱反光。它与第三个点同侧，必须被排除。
    candidates = [
        (91.0, 442.0, 0.12),
        (547.0, 333.0, 0.08),
        (104.0, 192.0, 0.12),
        (109.0, 140.0, 0.06),
        (548.0, 108.0, 0.12),
    ]

    selected = select_alternating_edge_points(candidates, 98.0, 542.0)

    assert [(round(x), round(y)) for x, y, _ in selected] == [
        (91, 442),
        (547, 333),
        (104, 192),
        (548, 108),
    ]


def test_waveform_edges_follow_current_horizontal_position() -> None:
    screen = np.full((512, 640, 3), (35, 70, 48), np.uint8)
    trace_color = (20, 245, 45)
    for row in (145, 340):
        cv2.line(screen, (146, row - 12), (146, row + 12),
                 trace_color, 5, cv2.LINE_AA)
    for row in (210, 430):
        cv2.line(screen, (592, row - 12), (592, row + 12),
                 trace_color, 5, cv2.LINE_AA)

    # 模拟面积很大但颜色不鲜明的人物反光。
    cv2.rectangle(screen, (350, 100), (480, 410),
                  (115, 145, 130), -1)

    left_x, right_x, _, _ = estimate_waveform_edges(screen, 75, 490)

    assert abs(left_x - 146.0) <= 4.0
    assert abs(right_x - 592.0) <= 4.0


def test_frequency_uses_same_side_full_periods() -> None:
    # 左右侧相差使相邻半周期交替为 0.07/0.13，但同侧完整周期始终为 0.20。
    points = [
        make_period_point("left", value)
        for value in (0.00, 0.20, 0.40, 0.60)
    ]
    points += [
        make_period_point("right", value)
        for value in (0.07, 0.27, 0.47, 0.67)
    ]

    period, period_std, valid_count, frequency_hz = (
        compute_robust_phase_interval(points, 500.0)
    )

    assert abs(period - 0.20) < 1e-9
    assert period_std < 1e-9
    assert valid_count == 6
    assert abs(frequency_hz - 10_000.0) < 1e-6


def test_frequency_rejects_missed_point_double_period() -> None:
    # 左侧从 0.20 直接跳到 0.60，产生一个 2x 长周期；它不能拉低频率。
    points = [
        make_period_point("left", value)
        for value in (0.00, 0.20, 0.60, 0.80)
    ]
    points += [
        make_period_point("right", value)
        for value in (0.07, 0.27, 0.47, 0.67)
    ]

    period, _, valid_count, frequency_hz = compute_robust_phase_interval(
        points,
        500.0,
    )

    assert abs(period - 0.20) < 1e-9
    assert valid_count == 5
    assert abs(frequency_hz - 10_000.0) < 1e-6


def test_temporal_period_filter_rejects_one_frame_jump() -> None:
    period_filter = TemporalPeriodFilter(window_size=5)
    periods = (0.200, 0.202, 0.350, 0.198, 0.200)
    outputs = [
        period_filter.update(period, valid_count=4, ramp_duration_us=500.0)
        for period in periods
    ]

    # 0.350 是单帧异常，5 帧中位数仍应锁定在 0.200，即 10 kHz。
    stable_period, stable_frequency_hz = outputs[-1]
    assert abs(stable_period - 0.200) < 1e-9
    assert abs(stable_frequency_hz - 10_000.0) < 1e-6

    # 真实新频率连续占据窗口多数后必须切换，不能永久锁死旧值。
    for _ in range(3):
        stable_period, stable_frequency_hz = period_filter.update(
            0.100,
            valid_count=4,
            ramp_duration_us=500.0,
        )
    assert abs(stable_period - 0.100) < 1e-9
    assert abs(stable_frequency_hz - 20_000.0) < 1e-6
