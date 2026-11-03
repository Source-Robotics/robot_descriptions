"""Shared documentation/preview directories and portable-package link checks."""
import json
from pathlib import Path
import re
import shutil
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
    source=Path(__file__).resolve().parents[1]
    data=json.loads((Path(package)/'sim2bot.robot.json').read_text(encoding='utf-8'))
    colors=(source/'docs/COLORS.md').read_text(encoding='utf-8')
    base=f'https://github.com/Source-Robotics/robot_descriptions/blob/v{data["packageVersion"]}/'
    colors=colors.replace('](../source/',']('+base+'source/').replace('](maintenance/BUILDING.md)',']('+base+'docs/maintenance/BUILDING.md)')
    colors=colors.replace('](images/color_examples/','](../previews/color_examples/')
    examples=previews(package)/'color_examples'
    examples.mkdir(parents=True,exist_ok=True)
    for path in (source/'docs/images/color_examples').iterdir():
        if path.suffix in {'.png','.json'}:
            shutil.copyfile(path,examples/path.name)
    (directory/'COLORS.md').write_text(colors,encoding='utf-8')
    shutil.copyfile(source/'source/appearance/colors.json',directory/'colors.json')
    has_msg=Path(package).name in {'par6','parol6','msg','par6_description','parol6_description','msg_description'}
    if has_msg:
        camera=(source/'docs/MSG_CAMERA.md').read_text(encoding='utf-8')
        camera=camera.replace('](images/','](../previews/')
        (directory/'MSG_CAMERA.md').write_text(camera,encoding='utf-8')
        for name in ['msg_camera_solid.png','msg_camera_soft_tpu.png']:
            shutil.copyfile(source/'docs/images'/name,previews(package)/name)
    front=Path(package)/'README.md'
    text=front.read_text(encoding='utf-8')
    text=text.replace('[Changing colors](docs/COLORS.md): material names, editable palettes and examples.',
                      '[Changing colors](docs/COLORS.md): shared palette, part overrides and illustrated examples.')
    if '[Changing colors](docs/COLORS.md)' not in text:
        text+='\n[Changing colors](docs/COLORS.md): shared palette, part overrides and illustrated examples.\n'
    if has_msg and '[MSG built-in camera](docs/MSG_CAMERA.md)' not in text:
        text+='\n[MSG built-in camera](docs/MSG_CAMERA.md): simulated views with both jaw options and a rendering example.\n'
    front.write_text(text,encoding='utf-8')
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
