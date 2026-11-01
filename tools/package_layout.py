"""Shared documentation/preview directories and portable-package link checks."""
import json
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit


def documentation(package):
    directory=Path(package)/'docs'
    directory.mkdir(parents=True,exist_ok=True)
    return directory


def previews(package):
    directory=Path(package)/'previews'
    directory.mkdir(parents=True,exist_ok=True)
    return directory


def write_index(package):
    directory=documentation(package)
    lines=['# Package documentation', '', '[Back to the model instructions](../README.md)', '']
    for path in sorted(directory.glob('*.md')):
        if path.name=='README.md':continue
        title=next((s.lstrip('# ').strip() for s in path.read_text(encoding='utf-8').splitlines()
                    if s.startswith('# ')),path.stem)
        lines.append(f'- [{title}]({path.name})')
    if (directory/'kinematics_review').is_dir():
        lines += ['', '[Dimensioned SVG/PNG sheets and numeric data](kinematics_review/)']
    (directory/'README.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


def verify(package):
    """Check the browsable package and its catalog links after relocation."""
    package=Path(package).resolve()
    assert {p.name for p in package.glob('*.md')}=={'README.md'},package
    assert not list(package.glob('*.png')),package
    assert not (package/'kinematics_review').exists(),package
    assert (package/'docs/README.md').is_file(),package
    for path in [package/'README.md',*(package/'docs').rglob('*.md')]:
        for href in re.findall(r'\]\(([^)]+)\)',path.read_text(encoding='utf-8')):
            if urlsplit(href).scheme or href.startswith('#'):continue
            target=(path.parent/unquote(href.split('#',1)[0])).resolve()
            assert target.is_relative_to(package) and target.exists(),(path,href)
    data=json.loads((package/'sim2bot.robot.json').read_text(encoding='utf-8'))
    for item in [data,*data['variants']]:
        assert (package/item['entry']).is_file(),item
        if item.get('preview'):
            image=package/item['preview']
            assert image.parent==package/'previews' and image.is_file(),image
