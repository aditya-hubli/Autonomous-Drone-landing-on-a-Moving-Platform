#!/usr/bin/env python3
"""Generate 4 ArUco marker images and a combined platform top texture.

Markers: IDs 0-3 from DICT_4X4_50, placed in a 2x2 grid on a green background.
Output: individual marker PNGs + combined platform_top.png
"""

import os
import sys

import cv2
import numpy as np

# Output directory
TEXTURE_DIR = os.path.join(
    os.path.dirname(__file__), '..', 'src', 'mars_simulation',
    'models', 'landing_platform', 'materials', 'textures')

MARKER_IDS = [0, 1, 2, 3]
MARKER_PX = 200       # individual marker size in pixels
BORDER_PX = 30        # white border around each marker
TEXTURE_PX = 1024     # combined texture resolution (square)
DICT = cv2.aruco.DICT_4X4_50


def generate_single_marker(marker_id, size_px):
    """Generate a single ArUco marker image with a white border."""
    aruco_dict = cv2.aruco.Dictionary_get(DICT)
    marker = cv2.aruco.drawMarker(aruco_dict, marker_id, size_px)
    # Add white border
    bordered = cv2.copyMakeBorder(
        marker, BORDER_PX, BORDER_PX, BORDER_PX, BORDER_PX,
        cv2.BORDER_CONSTANT, value=255)
    return bordered


def generate_platform_texture():
    """Create a combined texture: green background with 4 ArUco markers at corners."""
    tex = np.zeros((TEXTURE_PX, TEXTURE_PX, 3), dtype=np.uint8)
    # Green background (BGR)
    tex[:, :] = (0, 200, 0)

    marker_total = MARKER_PX + 2 * BORDER_PX  # size with border
    margin = 80  # pixels from edge of texture

    # Corner positions (top-left of each marker region)
    positions = [
        (margin, margin),                                          # top-left
        (TEXTURE_PX - margin - marker_total, margin),             # top-right
        (margin, TEXTURE_PX - margin - marker_total),             # bottom-left
        (TEXTURE_PX - margin - marker_total,
         TEXTURE_PX - margin - marker_total),                     # bottom-right
    ]

    for i, (px, py) in enumerate(positions):
        marker_gray = generate_single_marker(MARKER_IDS[i], MARKER_PX)
        marker_bgr = cv2.cvtColor(marker_gray, cv2.COLOR_GRAY2BGR)
        tex[py:py + marker_total, px:px + marker_total] = marker_bgr

    return tex


def main():
    os.makedirs(TEXTURE_DIR, exist_ok=True)

    # Generate individual markers
    for mid in MARKER_IDS:
        marker = generate_single_marker(mid, MARKER_PX)
        path = os.path.join(TEXTURE_DIR, f'aruco_{mid}.png')
        cv2.imwrite(path, marker)
        print(f'  Saved {path}')

    # Generate combined platform top texture
    texture = generate_platform_texture()
    combined_path = os.path.join(TEXTURE_DIR, 'platform_top.png')
    cv2.imwrite(combined_path, texture)
    print(f'  Saved {combined_path}')

    print('Done. 4 individual markers + 1 combined texture generated.')


if __name__ == '__main__':
    main()
