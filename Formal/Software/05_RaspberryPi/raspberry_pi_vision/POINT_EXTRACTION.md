# Task5 waveform point extraction

`extract_points.py` processes one oscilloscope photograph or camera frame. It:

1. Detects the green CRT screen and applies perspective correction.
2. Detects the thick upper and lower Task5 reference lines.
3. Calibrates them as +2 V and -2 V and uses their horizontal span for X
   normalization.
4. Masks both reference bands.
5. Extracts a limited number of bright pulse-ramp sample groups.
6. Writes CSV, JSON, a binary mask, and annotated debug images.

Offline image:

```bash
python3 extract_points.py --image scope.jpg --output-dir point_output
```

Raspberry Pi camera:

```bash
python3 extract_points.py --camera 0 --output-dir point_output
```

For a fixed camera, manual corners are more stable than detecting the screen on
every run. The order is top-left, top-right, bottom-right, bottom-left:

```bash
python3 extract_points.py --image scope.jpg \
  --corners 403 256 1326 300 1207 911 436 895 \
  --output-dir point_output
```

Outputs:

- `points.csv`: sparse point coordinates and calibrated Y voltage.
- `points.json`: points, reference calibration, and detected screen corners.
- `points_overlay.png`: numbered points over the rectified screen.
- `trace_mask.png`: reference-line-free trace mask.
- `screen_detection.png`: detected screen quadrilateral.

The most useful columns are:

- `x_normalized`: X coordinate mapped to approximately -1..+1.
- `y_normalized`: Y coordinate mapped to +1 at the upper line and -1 at the
  lower line.
- `y_volts`: Y coordinate mapped to +2 V..-2 V.
- `time_normalized`: ramp time, from 0 at -2 V to 1 at +2 V.
- `strength`: relative confidence of the selected visible sample group.

Lock camera exposure and focus during measurement. The thick reference lines
are caused by long CRT dwell time and camera integration; their center
positions remain suitable for calibration even when their visible bands are
wide.

The two reference lines exist only when the adjustable ramp width is below the
fixed 10 ms repetition period. At the default full 10 ms width there is no idle
interval, so the extractor intentionally reports that calibration lines are
missing. Shorten the ramp once with K5 before point extraction.
