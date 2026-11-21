"""Distance-per-swing measurement from actual head pose, including braking."""


def cumulative_strokes(angles, threshold=10.):
    """Yield directional excursions that clear a hysteresis band, at each sample."""
    if threshold <= 0:
        raise ValueError("Stroke threshold must be positive")
    iterator = iter(angles)
    first = next(iterator, None)
    if first is None:
        return
    low = high = first
    direction = count = 0
    yield count
    for angle in iterator:
        low = min(low, angle)
        high = max(high, angle)
        if direction <= 0 and angle - low >= threshold:
            count += 1
            direction = 1
            high = angle
        elif direction >= 0 and high - angle >= threshold:
            count += 1
            direction = -1
            low = angle
        yield count


def stroke_count(angles, threshold=10.):
    return max(cumulative_strokes(angles, threshold), default=0)
