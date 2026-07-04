#! /usr/bin/python3
import numpy as np


EPSILON = 1e-4
MAX_MONOTONIC_CORRECTION = 0.8
MIN_CELL_SAMPLES = 5
MAX_CELL_STD = 0.8


def select_target_column(data, preferred, fallback, logger):
    if preferred in data.columns:
        return preferred
    logger.warning(
        f"{preferred} is not available. Falling back to {fallback}; pitch compensation will not be used."
    )
    return fallback


def reject_outliers(data, column_thresholds, logger):
    filtered = data.copy()
    for column, threshold in column_thresholds.items():
        if threshold <= 0:
            continue
        mean = filtered[column].mean()
        std = filtered[column].std()
        if not np.isfinite(std) or std <= 0.0:
            logger.warning(f"Skip outlier filtering for {column}: invalid std={std}")
            continue
        before = len(filtered)
        filtered = filtered[np.abs(filtered[column] - mean) <= std * threshold]
        removed = before - len(filtered)
        logger.info(f"Filtered {removed} samples from {column} with {threshold} sigma threshold")
    return filtered


def normalize_columns(data, columns):
    normalized = data.copy()
    stats = {}
    for column in columns:
        mean = normalized[column].mean()
        std = normalized[column].std()
        if not np.isfinite(std) or std <= 0.0:
            raise ValueError(f"Cannot normalize {column}: invalid std={std}")
        normalized[column] = (normalized[column] - mean) / std
        stats[column] = (mean, std)
    return normalized, stats


def denormalize(value, stats, column):
    mean, std = stats[column]
    return value * std + mean


def validate_map(velocity_index, pedal_index, value_map, increasing, map_name):
    errors = []
    if np.any(~np.isfinite(velocity_index)) or np.any(~np.isfinite(pedal_index)):
        errors.append(f"{map_name}: index contains NaN or Inf")
    if np.any(np.diff(velocity_index) <= 0.0):
        errors.append(f"{map_name}: velocity index is not strictly increasing")
    if np.any(np.diff(pedal_index) <= 0.0):
        errors.append(f"{map_name}: pedal index is not strictly increasing")
    if np.any(~np.isfinite(value_map)):
        errors.append(f"{map_name}: map contains NaN or Inf")

    for pedal_idx in range(1, value_map.shape[0]):
        previous = value_map[pedal_idx - 1]
        current = value_map[pedal_idx]
        invalid = current <= previous if increasing else current >= previous
        for velocity_idx in np.where(invalid)[0]:
            relation = ">" if increasing else "<"
            errors.append(
                f"{map_name}: invalid monotonicity at pedal[{pedal_idx - 1}->{pedal_idx}] "
                f"velocity[{velocity_idx}]={velocity_index[velocity_idx]:.3f}; "
                f"{current[velocity_idx]:.6f} should be {relation} {previous[velocity_idx]:.6f}"
            )
    return errors


def _isotonic_increasing(values):
    levels = []
    weights = []
    starts = []
    ends = []
    for idx, value in enumerate(values):
        levels.append(float(value))
        weights.append(1.0)
        starts.append(idx)
        ends.append(idx)
        while len(levels) >= 2 and levels[-2] > levels[-1]:
            total_weight = weights[-2] + weights[-1]
            merged_level = (levels[-2] * weights[-2] + levels[-1] * weights[-1]) / total_weight
            levels[-2] = merged_level
            weights[-2] = total_weight
            ends[-2] = ends[-1]
            levels.pop()
            weights.pop()
            starts.pop()
            ends.pop()

    result = np.empty_like(values, dtype=float)
    for level, start, end in zip(levels, starts, ends):
        result[start : end + 1] = level
    return result


def enforce_monotonic_columns(value_map, increasing, epsilon=EPSILON):
    corrected = value_map.copy()
    for velocity_idx in range(corrected.shape[1]):
        column = corrected[:, velocity_idx]
        if increasing:
            column = _isotonic_increasing(column)
            for pedal_idx in range(1, len(column)):
                column[pedal_idx] = max(column[pedal_idx], column[pedal_idx - 1] + epsilon)
        else:
            column = -_isotonic_increasing(-column)
            for pedal_idx in range(1, len(column)):
                column[pedal_idx] = min(column[pedal_idx], column[pedal_idx - 1] - epsilon)
        corrected[:, velocity_idx] = column
    return corrected


def report_coverage(data, velocity_col, pedal_col, target_col, velocity_range, pedal_range, logger):
    count_map = np.zeros((len(pedal_range), len(velocity_range)), dtype=int)
    sample_map = [[[] for _ in velocity_range] for _ in pedal_range]

    velocity_half_span = np.diff(velocity_range).mean() / 2.0
    pedal_half_span = np.diff(pedal_range).mean() / 2.0

    for _, row in data.iterrows():
        velocity_idx = np.argmin(np.abs(velocity_range - row[velocity_col]))
        pedal_idx = np.argmin(np.abs(pedal_range - row[pedal_col]))
        if (
            abs(velocity_range[velocity_idx] - row[velocity_col]) <= velocity_half_span
            and abs(pedal_range[pedal_idx] - row[pedal_col]) <= pedal_half_span
        ):
            count_map[pedal_idx, velocity_idx] += 1
            sample_map[pedal_idx][velocity_idx].append(row[target_col])

    sparse_cells = np.argwhere(count_map < MIN_CELL_SAMPLES)
    high_std_cells = []
    for pedal_idx in range(len(pedal_range)):
        for velocity_idx in range(len(velocity_range)):
            samples = sample_map[pedal_idx][velocity_idx]
            if len(samples) >= MIN_CELL_SAMPLES and np.std(samples) > MAX_CELL_STD:
                high_std_cells.append((pedal_idx, velocity_idx, float(np.std(samples))))

    logger.info(
        f"Coverage: min={count_map.min()}, max={count_map.max()}, "
        f"sparse_cells={len(sparse_cells)}/{count_map.size}, high_std_cells={len(high_std_cells)}"
    )
    for pedal_idx, velocity_idx in sparse_cells[:10]:
        logger.warning(
            f"Sparse cell: pedal={pedal_range[pedal_idx]:.2f}, "
            f"velocity={velocity_range[velocity_idx]:.2f}, count={count_map[pedal_idx, velocity_idx]}"
        )
    for pedal_idx, velocity_idx, std in high_std_cells[:10]:
        logger.warning(
            f"High variance cell: pedal={pedal_range[pedal_idx]:.2f}, "
            f"velocity={velocity_range[velocity_idx]:.2f}, std={std:.3f}"
        )
    return count_map


def save_checked_map(filename, velocity_range, pedal_range, value_map, increasing, map_name, logger):
    velocity_range = np.asarray(velocity_range, dtype=float)
    pedal_range = np.asarray(pedal_range, dtype=float)
    value_map = np.asarray(value_map, dtype=float)

    raw_errors = validate_map(velocity_range, pedal_range, value_map, increasing, map_name)
    if raw_errors:
        logger.warning(f"{map_name} failed validation before monotonic correction.")
        for error in raw_errors[:20]:
            logger.warning(error)
        corrected_map = enforce_monotonic_columns(value_map, increasing)
        max_correction = float(np.max(np.abs(corrected_map - value_map)))
        logger.warning(f"{map_name} max monotonic correction: {max_correction:.6f} m/s^2")
        if max_correction > MAX_MONOTONIC_CORRECTION:
            raise ValueError(
                f"{map_name} requires too much monotonic correction ({max_correction:.3f}). "
                "Refusing to overwrite the map."
            )
        value_map = corrected_map

    final_errors = validate_map(velocity_range, pedal_range, value_map, increasing, map_name)
    if final_errors:
        raise ValueError("\n".join(final_errors[:20]))

    headers = [""] + ["{:.2f}".format(v) for v in velocity_range]
    output = np.column_stack((pedal_range, value_map))
    np.savetxt(filename, output, delimiter=",", header=",".join(headers), comments="")
    logger.info(f"Saved checked {map_name} to {filename}")
    return value_map
