# SPDX-License-Identifier: Apache-2.0
"""Print a portable URDF with ROS package:// mesh URIs; no ROS import needed."""

from pathlib import Path
import argparse
import sys
import xml.etree.ElementTree as ET


def convert(path, package):
    tree = ET.parse(path)
    for mesh in tree.findall('.//mesh'):
        filename = mesh.get('filename')
        if '://' not in filename:
            mesh.set('filename', f'package://{package}/{Path(filename).as_posix()}')
    return ET.tostring(tree.getroot(), encoding='unicode')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('--package', default='parol6_description')
    args = parser.parse_args()
    sys.stdout.write(convert(args.urdf, args.package)+'\n')
