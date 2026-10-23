"""Unit tests for ag_labs_cpp build integration."""

import os
import xml.etree.ElementTree as ET
import re

PACKAGE_DIR = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..')
)


def _resolve(*parts):
    return os.path.join(PACKAGE_DIR, *parts)


def test_package_xml_has_image_transport():
    """package.xml should contain image_transport dependency."""
    tree = ET.parse(_resolve('package.xml'))
    root = tree.getroot()
    deps = {dep.text for dep in root.findall('depend')}
    assert 'image_transport' in deps, \
        'Missing <depend>image_transport</depend>'


def test_cmakelists_has_camera_based_lidar_node():
    """CMakeLists.txt should define camera_based_lidar_node executable."""
    with open(_resolve('CMakeLists.txt')) as f:
        content = f.read()
    assert re.search(
        r'add_executable\(camera_based_lidar_node',
        content,
    ), 'Missing add_executable(camera_based_lidar_node ...)'


def test_cmakelists_has_camera_based_lidar_in_install():
    """CMakeLists.txt should install camera_based_lidar_node target."""
    with open(_resolve('CMakeLists.txt')) as f:
        content = f.read()
    assert 'camera_based_lidar_node' in content, \
        'camera_based_lidar_node not found in install(TARGETS ...)'


def test_cmakelists_has_image_transport_find_package():
    """CMakeLists.txt should find_package(image_transport)."""
    with open(_resolve('CMakeLists.txt')) as f:
        content = f.read()
    assert 'find_package(image_transport' in content, \
        'Missing find_package(image_transport ...)'
