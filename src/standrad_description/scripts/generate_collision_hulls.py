#!/usr/bin/env python3
"""Generate compact bounding-box STL files for collision geometry."""

import argparse
from pathlib import Path
import struct

import numpy as np


def read_binary_stl(path):
    with path.open("rb") as stream:
        stream.seek(80)
        triangle_count = struct.unpack("<I", stream.read(4))[0]
        records = np.fromfile(
            stream,
            dtype=np.dtype(
                [
                    ("normal", "<f4", (3,)),
                    ("vertices", "<f4", (3, 3)),
                    ("attribute", "<u2"),
                ]
            ),
            count=triangle_count,
        )
    if len(records) != triangle_count:
        raise ValueError(f"Invalid binary STL: {path}")
    return records["vertices"].reshape(-1, 3)


def write_binary_stl(path, vertices, faces):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(b"Convex collision hull".ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(faces)))
        for face in faces:
            triangle = vertices[face].astype(np.float32)
            normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
            length = np.linalg.norm(normal)
            if length:
                normal /= length
            stream.write(struct.pack("<12fH", *normal, *triangle.reshape(-1), 0))


def generate_hull(source, destination):
    source_vertices = read_binary_stl(source)
    low = source_vertices.min(axis=0)
    high = source_vertices.max(axis=0)
    vertices = np.array(
        [
            [low[0], low[1], low[2]], [high[0], low[1], low[2]],
            [high[0], high[1], low[2]], [low[0], high[1], low[2]],
            [low[0], low[1], high[2]], [high[0], low[1], high[2]],
            [high[0], high[1], high[2]], [low[0], high[1], high[2]],
        ],
        dtype=np.float32,
    )
    faces = np.array(
        [
            [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
            [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
            [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7],
        ],
        dtype=np.int32,
    )
    write_binary_stl(destination, vertices, faces)
    print(f"{source.name}: {len(source_vertices) // 3} faces -> {len(faces)} faces")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()

    for source in sorted(args.source.glob("*.STL")):
        generate_hull(source, args.destination / source.name)


if __name__ == "__main__":
    main()
