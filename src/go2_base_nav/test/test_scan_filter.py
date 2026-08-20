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
