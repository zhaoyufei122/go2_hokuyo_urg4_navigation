import math

from go2_base_nav.scan_filter import filter_ranges


def test_out_of_range_readings_become_inf():
    ranges = [0.007, 0.019, 0.06, 0.5, 4.0, 4.2, math.inf, math.nan]
    filtered = filter_ranges(ranges, min_range=0.06, max_range=4.0)

    assert filtered[0] == math.inf  # URG error code leaked as 0.007 m
    assert filtered[1] == math.inf  # 0.019 m fake reading
    assert filtered[2] == math.inf  # boundary: exactly min_range is dropped
    assert filtered[3] == 0.5
    assert filtered[5] == math.inf  # beyond max range
    assert filtered[6] == math.inf  # no-return stays inf
    assert math.isnan(filtered[7])  # NaN stays NaN


def test_valid_ranges_pass_through_unchanged():
    ranges = [0.5, 1.23, 3.99]
    assert filter_ranges(ranges, 0.06, 4.0) == ranges


def test_front_sector_masked_when_gripped():
    # 360° 扫描、1° 步进：正前方 ±45° 全遮蔽，其余保留
    n = 360
    ranges = [1.0] * n
    filtered = filter_ranges(
        ranges, 0.06, 4.0,
        angle_min=-math.pi, angle_increment=math.radians(1.0),
        mask_center_rad=0.0, mask_half_rad=math.radians(45.0))
    assert filtered[180] == math.inf      # 正前方（0°）
    assert filtered[180 + 44] == math.inf # +44° 在扇区内
    assert filtered[180 + 45] == math.inf # 边界含
    assert filtered[180 + 46] == 1.0      # 扇区外保留
    assert filtered[0] == 1.0             # 正后方保留


def test_no_mask_by_default():
    ranges = [1.0] * 10
    assert filter_ranges(ranges, 0.06, 4.0) == ranges


def test_grip_watch_sector_min_range():
    from go2_base_nav.grip_watch import sector_min_range
    n = 360
    ranges = [math.inf] * n
    ranges[180] = 0.4     # 正前方 0.4m（walker 在位）
    ranges[200] = 0.3     # +20° 处
    # ±20° 扇区：最小 0.4（200 在 +20° 边界外）
    assert abs(sector_min_range(ranges, -math.pi, math.radians(1.0),
                                math.radians(20)) - 0.4) < 1e-9
    # ±25° 扇区：最小 0.3
    assert abs(sector_min_range(ranges, -math.pi, math.radians(1.0),
                                math.radians(25)) - 0.3) < 1e-9


def test_grip_watch_sector_min_range_filters_body_noise():
    """/scan_raw 含 ~0.015m 机身自身噪声读数（2026-07-18 实测），
    min_range=0.06 必须滤掉，否则永远误判 HOLDING。"""
    from go2_base_nav.grip_watch import sector_min_range
    n = 360
    ranges = [math.inf] * n
    ranges[180] = 0.015   # 机身噪声（正前方）
    ranges[190] = 0.8     # walker 真读数
    # 噪声被滤 → 最小是 0.8；若没滤会返回 0.015
    assert abs(sector_min_range(ranges, -math.pi, math.radians(1.0),
                                math.radians(20)) - 0.8) < 1e-9
    # 只有噪声 → inf（判丢失）
    ranges[190] = math.inf
    assert math.isinf(sector_min_range(ranges, -math.pi, math.radians(1.0),
                                       math.radians(20)))
