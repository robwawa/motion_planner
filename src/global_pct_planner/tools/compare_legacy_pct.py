#!/usr/bin/env python3
"""Compare a legacy Python tomogram with a native ``.pctm`` map.

The legacy exporter stored ``data`` as float16.  The native format deliberately
stores float32, so a serialized float16 map is checked with a quantization
tolerance while a float32 CPU reference is checked at the 1e-4 acceptance
tolerance.  In both cases NaN placement and the traversability classification
are required to match exactly.
"""

import argparse
import pickle
import struct
import sys

import numpy as np


MAGIC = b"GPCTM01\0"
HEADER = struct.Struct("<8s6I5dQ")
CHANNELS = 5


def load_legacy(path):
    with open(path, "rb") as stream:
        payload = pickle.load(stream)
    raw = np.asarray(payload["data"])
    if raw.ndim != 4 or raw.shape[0] != CHANNELS:
        raise ValueError("legacy tomogram data must have shape [5, layer, row, col]")
    data = np.asarray(raw, dtype=np.float32)
    center = np.asarray(payload["center"], dtype=np.float64).reshape(-1)
    if center.size < 2:
        raise ValueError("legacy tomogram center must contain x and y")
    return {
        "data": data,
        "source_dtype": raw.dtype,
        "resolution": float(payload["resolution"]),
        "center": center[:2],
        "slice_h0": float(payload["slice_h0"]),
        "slice_dh": float(payload["slice_dh"]),
    }


def load_pctm(path):
    with open(path, "rb") as stream:
        header_bytes = stream.read(HEADER.size)
        if len(header_bytes) != HEADER.size:
            raise ValueError("truncated .pctm header")
        (magic, version, endian, layers, rows, cols, channels, resolution,
         center_x, center_y, slice_h0, slice_dh, count) = HEADER.unpack(header_bytes)
        if (magic != MAGIC or version != 1 or endian != 0x01020304 or
                channels != CHANNELS or count != layers * rows * cols):
            raise ValueError("unsupported or corrupt .pctm header")
        arrays = []
        for _ in range(CHANNELS):
            values = np.frombuffer(stream.read(count * 4), dtype="<f4")
            if values.size != count:
                raise ValueError("truncated .pctm payload")
            arrays.append(values.reshape((layers, rows, cols)))
    return {
        "data": np.stack(arrays, axis=0),
        "resolution": resolution,
        "center": np.array([center_x, center_y], dtype=np.float64),
        "slice_h0": slice_h0,
        "slice_dh": slice_dh,
    }


def compare(args):
    legacy = load_legacy(args.legacy)
    pctm = load_pctm(args.pctm)
    old = legacy["data"]
    new = pctm["data"]
    if old.shape != new.shape:
        raise AssertionError("dimensions differ: {} != {}".format(old.shape, new.shape))

    metadata_errors = {
        "resolution": abs(legacy["resolution"] - pctm["resolution"]),
        "center_x": abs(legacy["center"][0] - pctm["center"][0]),
        "center_y": abs(legacy["center"][1] - pctm["center"][1]),
        "slice_h0": abs(legacy["slice_h0"] - pctm["slice_h0"]),
        "slice_dh": abs(legacy["slice_dh"] - pctm["slice_dh"]),
    }
    bad_metadata = {key: value for key, value in metadata_errors.items()
                    if value > args.metadata_tolerance}
    if bad_metadata:
        raise AssertionError("metadata differs: {}".format(bad_metadata))

    if not np.array_equal(np.isfinite(old), np.isfinite(new)):
        raise AssertionError("finite-value layout differs")
    if not np.array_equal(np.isnan(old), np.isnan(new)):
        raise AssertionError("NaN layout differs")

    diffs = []
    for channel in range(CHANNELS):
        mask = np.isfinite(old[channel]) & np.isfinite(new[channel])
        diff = np.abs(old[channel][mask] - new[channel][mask])
        diffs.append(float(diff.max()) if diff.size else 0.0)

    threshold = args.threshold
    old_free = np.isfinite(old[0]) & (old[0] <= threshold)
    new_free = np.isfinite(new[0]) & (new[0] <= threshold)
    classification_diff = int(np.count_nonzero(old_free != new_free))
    tolerance = args.tolerance
    if tolerance is None:
        tolerance = 2e-2 if np.dtype(legacy["source_dtype"]) == np.float16 else 1e-4
    print("shape={}".format(old.shape))
    print("source_dtype={}".format(legacy["source_dtype"]))
    print("metadata_max_error={:.9g}".format(max(metadata_errors.values())))
    print("channel_max_error=" + ", ".join("{:.9g}".format(value) for value in diffs))
    print("numeric_tolerance={:.9g}".format(tolerance))
    print("traversability_classification_diff={}".format(classification_diff))
    if max(diffs) > tolerance:
        raise AssertionError("numeric error exceeds tolerance")
    if classification_diff != 0:
        raise AssertionError("traversability classification differs")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("legacy", help="legacy .pickle tomogram")
    parser.add_argument("pctm", help="native .pctm tomogram")
    parser.add_argument("--threshold", type=float, default=49.0,
                        help="traversability free-space threshold (default: 49)")
    parser.add_argument("--tolerance", type=float, default=None,
                        help="maximum finite float error; auto-selects float16/float32 default")
    parser.add_argument("--metadata-tolerance", type=float, default=1e-6)
    args = parser.parse_args()
    try:
        return compare(args)
    except (AssertionError, OSError, KeyError, ValueError, pickle.UnpicklingError) as error:
        print("comparison failed: {}".format(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
