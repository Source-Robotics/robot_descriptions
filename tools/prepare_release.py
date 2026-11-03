"""Validate release versions and prepare checksummed GitHub release assets."""

from pathlib import Path
import argparse
import hashlib
import json
import shutil
import subprocess
import xml.etree.ElementTree as ET
import zipfile

ROOT=Path(__file__).resolve().parents[1]
PACKAGES={'par6_mujoco.zip':'par6','par6_urdf.zip':'par6_description',
          'parol6_mujoco.zip':'parol6','parol6_urdf.zip':'parol6_description',
          'msg_mujoco.zip':'msg','msg_urdf.zip':'msg_description',
          'ssg48_mujoco.zip':'ssg48','ssg48_urdf.zip':'ssg48_description'}
MOVEIT_PACKAGES={'par6_moveit_config.zip':'par6_moveit_config',
                'parol6_moveit_config.zip':'parol6_moveit_config'}


def prepare(root=ROOT,tag=None):
    root=Path(root)
    version=ET.parse(root/'urdf/par6_description/package.xml').findtext('version')
    expected=f'v{version}'
    if tag is not None and tag!=expected:
        raise ValueError(f'Release tag {tag!r} does not match package version {expected!r}')
    notes=root/f'releases/{expected}.md'
    if not notes.is_file():raise ValueError(f'Missing release notes: {notes}')
    files=[]
    for filename,package in PACKAGES.items():
        path=root/'dist'/filename
        with zipfile.ZipFile(path) as archive:
            if archive.testzip() is not None:raise ValueError(f'Archive CRC failure: {filename}')
            metadata=json.loads(archive.read(f'{package}/sim2bot.robot.json'))
            if metadata['packageVersion']!=version:
                raise ValueError(f'Archive version mismatch: {filename}')
        files.append(path)
    for filename,package in MOVEIT_PACKAGES.items():
        path=root/'dist'/filename
        with zipfile.ZipFile(path) as archive:
            if archive.testzip() is not None:raise ValueError(f'Archive CRC failure: {filename}')
            actual=ET.fromstring(archive.read(f'{package}/package.xml')).findtext('version')
            if actual!=version:raise ValueError(f'Archive version mismatch: {filename}')
        files.append(path)
    booklet=root/'output/pdf/robot_kinematics_review.pdf'
    if not booklet.read_bytes().startswith(b'%PDF-'):raise ValueError('Invalid review booklet')
    pdf=root/'dist'/booklet.name
    shutil.copyfile(booklet,pdf);files.append(pdf)
    result=subprocess.run(['git','-c',f'safe.directory={root.resolve().as_posix()}',
                           '-C',str(root),'rev-parse','HEAD'],capture_output=True,text=True)
    manifest={'version':version,'tag':expected,
              'source_commit':result.stdout.strip() if result.returncode==0 else None,
              'assets':{p.name:{'bytes':p.stat().st_size,
                                'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
                        for p in files}}
    path=root/'dist/release_manifest.json'
    path.write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8',newline='\n')
    files.append(path)
    checksums=''.join(f'{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.name}\n' for p in files)
    (root/'dist/SHA256SUMS').write_text(checksums,encoding='ascii',newline='\n')
    print(f'Prepared {expected}: eight model ZIPs, two MoveIt ZIPs, review PDF, manifest and SHA256SUMS.',flush=True)
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag',help='Require the pushed tag to match the package version')
    args=parser.parse_args();prepare(tag=args.tag)
