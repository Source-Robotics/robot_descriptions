"""Export a product README and local previews for a hardware repository.

Models remain in robot_descriptions. Relative documentation/model links point
back to a selected central revision; preview images are copied unchanged.
"""
import argparse
from pathlib import Path
import re
import shutil
from urllib.parse import quote, unquote, urlsplit
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PRODUCTS = ('par6', 'parol6', 'msg', 'ssg48', 'raft')
REPOSITORY = 'https://github.com/Source-Robotics/robot_descriptions'


def export(product, destination, revision):
    if product not in PRODUCTS:
        raise ValueError(f'Unknown product: {product}')
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', revision):
        raise ValueError('Use a commit SHA, tag, or simple branch name')
    destination = Path(destination).resolve()
    if (destination/'package.xml').exists():
        raise ValueError('Choose a documentation directory, not an installed ROS package')
    source = ROOT/f'guides/{product}.md'
    content = source.read_text(encoding='utf-8')
    images = {}

    def rewrite(match):
        prefix, href, suffix = match.groups()
        if urlsplit(href).scheme or href.startswith('#'):
            return match.group(0)
        parts = href.split('#', 1)
        target = (source.parent/unquote(parts[0])).resolve()
        if not target.is_relative_to(ROOT) or not target.exists():
            raise ValueError(f'Missing or external guide target: {href}')
        if prefix.startswith('!['):
            relative = Path('previews')/target.name
            if relative in images and images[relative] != target:
                raise ValueError(f'Preview filename collision: {relative}')
            images[relative] = target
            url = relative.as_posix()
        else:
            route = 'tree' if target.is_dir() else 'blob'
            relative = quote(target.relative_to(ROOT).as_posix(), safe='/')
            url = f'{REPOSITORY}/{route}/{revision}/{relative}'
        if len(parts) > 1:
            url += '#'+parts[1]
        return prefix+url+suffix

    content = re.sub(r'(!?\[[^\]]*\]\()([^\)]+)(\))', rewrite, content)
    version = ET.parse(ROOT/'urdf/par6_description/package.xml').findtext('version')
    content += f'''
## Maintained model source

This folder contains the product guide and preview images. Download the runtime
model packages using the links above and extract them into your working directory.
All model files are maintained in [{REPOSITORY.rsplit('/',1)[1]}]({REPOSITORY});
this hardware repository has no model submodule to initialize.

Current tested model release: **[v{version}]({REPOSITORY}/releases/tag/v{version})**.
See the [shared product guide]({REPOSITORY}/blob/{revision}/guides/{product}.md)
for the corresponding central documentation.
'''
    destination.mkdir(parents=True, exist_ok=True)
    for relative, original in images.items():
        target = destination/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(original, target)
    (destination/'README.md').write_text(content, encoding='utf-8', newline='\n')
    return {'product': product, 'destination': str(destination), 'preview_count': len(images),
            'files': ['README.md', *[p.as_posix() for p in images]]}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--product', required=True, choices=PRODUCTS)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--revision', required=True, help='Central commit/tag used for documentation links')
    args = parser.parse_args()
    print(export(args.product, args.output, args.revision))
