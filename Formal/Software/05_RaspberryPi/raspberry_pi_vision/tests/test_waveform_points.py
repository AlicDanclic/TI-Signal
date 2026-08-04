import cv2
import numpy as np
import pytest

from vision import WaveformPointExtractor


def make_scope_frame(*, include_reference: bool = True,
                     include_trace: bool = True
                     ) -> tuple[np.ndarray, list[tuple[int, int]]]:
    image = np.full((640, 800, 3), (55, 95, 65), np.uint8)
    for x in range(80, 721, 80):
        cv2.line(image, (x, 25), (x, 615), (30, 48, 35), 2)
    for y in range(60, 581, 65):
        cv2.line(image, (45, y), (755, y), (30, 48, 35), 2)

    trace_color = (30, 245, 65)
    if include_reference:
        cv2.line(image, (145, 62), (710, 62), trace_color, 18,
                 cv2.LINE_AA)
        cv2.line(image, (145, 578), (710, 578), trace_color, 18,
                 cv2.LINE_AA)

    expected = []
    if include_trace:
        rows = list(range(115, 535, 28))
        for index, row in enumerate(rows):
            column = 180 if index % 2 == 0 else 675
            expected.append((column, row))
            cv2.line(image, (column - 16, row), (column + 16, row),
                     trace_color, 7, cv2.LINE_AA)
    image = cv2.GaussianBlur(image, (3, 3), 0)
    return image, expected


def test_reference_lines_and_sparse_points_are_extracted() -> None:
    image, expected = make_scope_frame()
    extractor = WaveformPointExtractor({
        "vision": {"point_extraction": {"minimum_points": 6}}
    })
    result = extractor.extract(image)

    assert abs(result.calibration.top_y - 62) < 8
    assert abs(result.calibration.bottom_y - 578) < 8
    assert abs(result.calibration.left_x - 145) < 25
    assert abs(result.calibration.right_x - 710) < 25
    assert len(result.points) >= len(expected) - 3

    extracted_rows = np.asarray([point.y_px for point in result.points])
    for _, expected_row in expected:
        assert np.min(np.abs(extracted_rows - expected_row)) <= 5

    positive = sum(point.x_normalized > 0.5 for point in result.points)
    negative = sum(point.x_normalized < -0.5 for point in result.points)
    assert positive >= 5
    assert negative >= 5


def test_point_limit_keeps_vertical_coverage() -> None:
    image, _ = make_scope_frame()
    extractor = WaveformPointExtractor()
    result = extractor.extract(image, maximum_points=8)

    assert len(result.points) == 8
    rows = [point.y_px for point in result.points]
    assert min(rows) < 150
    assert max(rows) > 500


def test_very_small_point_limit_is_supported() -> None:
    image, _ = make_scope_frame()
    extractor = WaveformPointExtractor()
    result = extractor.extract(image, maximum_points=3)

    assert len(result.points) == 3
    assert [point.time_normalized for point in result.points] == sorted(
        point.time_normalized for point in result.points)


def test_grid_and_reference_lines_alone_are_not_waveform_points() -> None:
    image, _ = make_scope_frame(include_trace=False)
    extractor = WaveformPointExtractor()

    with pytest.raises(ValueError, match="waveform points"):
        extractor.extract(image)


def test_missing_reference_lines_cannot_create_voltage_calibration() -> None:
    image, _ = make_scope_frame(include_reference=False)
    extractor = WaveformPointExtractor()

    with pytest.raises(ValueError, match="reference"):
        extractor.extract(image)
