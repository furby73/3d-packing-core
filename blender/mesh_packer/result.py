"""Validate the packer's versioned output before changing a Blender scene."""

import json
import math


def _numbers(value, count):
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"Expected {count} matrix/coordinate values")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError("Non-finite or invalid placement coordinates")
    return value


def load_result(path, input_count, dimensions, gap):
    with open(path, encoding="utf-8") as file:
        data = json.load(file)
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError("Unsupported packer result format; update the solver and extension together")
    for field, expected in zip(("width", "length", "height", "gap"), (*dimensions, gap)):
        value = data.get(field)
        if type(value) not in (int, float) or not math.isfinite(value) or not math.isclose(value, expected, rel_tol=1e-8, abs_tol=1e-12):
            raise ValueError(f"Packer returned an unexpected {field}")
    count = data.get("bin_count")
    if type(count) is not int or not 1 <= count <= input_count:
        raise ValueError("Invalid container count")
    bins = data.get("bins")
    if not isinstance(bins, list) or len(bins) != count:
        raise ValueError("Incomplete container data")
    offsets = {}
    for container in bins:
        if not isinstance(container, dict):
            raise ValueError("Invalid container data")
        index = container.get("id")
        if type(index) is not int or not 0 <= index < count or index in offsets:
            raise ValueError("Invalid or duplicate container ID")
        offsets[index] = _numbers(container.get("display_offset"), 3)
    placements = data.get("placements")
    if not isinstance(placements, list) or len(placements) != input_count:
        raise ValueError("Packer returned the wrong number of placements")
    ordered = [None] * input_count
    for item in placements:
        if not isinstance(item, dict):
            raise ValueError("Invalid placement")
        index, container = item.get("input_index"), item.get("bin")
        if type(index) is not int or not 0 <= index < input_count or ordered[index] is not None:
            raise ValueError("Invalid or duplicate input index")
        if type(container) is not int or container not in offsets:
            raise ValueError("Unknown container in placement")
        matrix = list(_numbers(item.get("matrix"), 12))
        for axis, offset in enumerate(offsets[container]):
            matrix[axis * 4 + 3] += offset
        ordered[index] = (container, matrix)
    if {container for container, _ in ordered} != set(offsets):
        raise ValueError("Result contains an empty container")
    return ordered, offsets
