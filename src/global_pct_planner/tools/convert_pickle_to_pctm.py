#!/usr/bin/env python3
"""Convert a legacy pct_planner pickle tomogram to the native .pctm format.

The converter is intentionally the only Python component in the new package.
The C++ planner and tomography nodes read .pctm directly at runtime.
"""

import argparse
import os
import pickle
import struct

import numpy as np


MAGIC = b"GPCTM01\0"
VERSION = 1
ENDIAN_MARKER = 0x01020304
CHANNELS = 5
HEADER = struct.Struct("<8s6I5dQ")


def convert(input_path, output_path):
    with open(input_path, "rb") as stream:
        legacy = pickle.load(stream)
    data = np.asarray(legacy["data"], dtype=np.float32)
    if data.ndim != 4 or data.shape[0] != CHANNELS:
        raise ValueError("legacy tomogram data must have shape [5, layer, row, col]")
    layers, rows, cols = (int(value) for value in data.shape[1:])
    resolution = float(legacy["resolution"])
    center = np.asarray(legacy["center"], dtype=np.float64).reshape(-1)
    if center.size < 2 or resolution <= 0.0:
        raise ValueError("legacy tomogram metadata is invalid")
    slice_h0 = float(legacy["slice_h0"])
    slice_dh = float(legacy["slice_dh"])
    count = layers * rows * cols
    header = HEADER.pack(
        MAGIC,
        VERSION,
        ENDIAN_MARKER,
        layers,
        rows,
        cols,
        CHANNELS,
        resolution,
        float(center[0]),
        float(center[1]),
        slice_h0,
        slice_dh,
        count,
    )
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    temporary_path = output_path + ".tmp"
    with open(temporary_path, "wb") as stream:
        stream.write(header)
        for channel in data:
            stream.write(np.ascontiguousarray(channel, dtype="<f4").tobytes())
    os.replace(temporary_path, output_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="legacy .pickle tomogram")
    parser.add_argument("output", nargs="?", help="output .pctm path")
    args = parser.parse_args()
    output = args.output or os.path.splitext(args.input)[0] + ".pctm"
    convert(args.input, output)
    print("converted {} -> {}".format(args.input, output))


if __name__ == "__main__":
    main()
