"""Unit tests for include path correctness in camera_based_lidar sources."""

import os
import re

SOURCE_DIR = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', 'src')
)


def _resolve(filename):
    return os.path.join(SOURCE_DIR, filename)


SOURCES = ['camera_based_lidar.cpp', 'camera_based_lidar_main.cpp']


def test_no_ros_racer_vision_include():
    """No file should include via ros_racer_vision/ path."""
    for src in SOURCES:
        with open(_resolve(src)) as f:
            content = f.read()
        assert 'ros_racer_vision' not in content, \
            f'{src} still uses ros_racer_vision/ include path'


def test_local_include_exists():
    """Files should include camera_based_lidar.hpp directly."""
    for src in SOURCES:
        with open(_resolve(src)) as f:
            content = f.read()
        assert re.search(
            r'#include\s+"camera_based_lidar\.hpp"', content,
        ), f'{src} does not have direct include of camera_based_lidar.hpp'
